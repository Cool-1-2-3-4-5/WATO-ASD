#include "planner_node.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

PlannerNode::PlannerNode() : Node("planner"), planner_(this->get_logger()) {
  robot::PlannerConfig config;
  config.lethal_cost = this->declare_parameter<int>("lethal_cost", config.lethal_cost);
  config.cost_weight = this->declare_parameter<double>("cost_weight", config.cost_weight);
  config.allow_unknown = this->declare_parameter<bool>("allow_unknown", config.allow_unknown);
  config.goal_search_radius =
    this->declare_parameter<double>("goal_search_radius", config.goal_search_radius);
  planner_.configure(config);

  goal_tolerance_ = this->declare_parameter<double>("goal_tolerance", 0.5);
  replan_period_ = this->declare_parameter<double>("replan_period", 1.0);
  goal_timeout_ = this->declare_parameter<double>("goal_timeout", 180.0);

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", rclcpp::QoS(1).transient_local().reliable(),
    std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(250), std::bind(&PlannerNode::timerCallback, this));

  goal_start_time_ = this->now();
  last_plan_time_ = this->now();
  RCLCPP_INFO(this->get_logger(), "Planner ready, waiting for a goal on /goal_point");
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  map_ = msg;
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planPath();  // the world changed, so the old path may no longer be valid
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  if (map_ && !msg->header.frame_id.empty() && msg->header.frame_id != map_->header.frame_id) {
    RCLCPP_WARN(this->get_logger(), "Goal frame '%s' differs from map frame '%s'; using it as-is",
                msg->header.frame_id.c_str(), map_->header.frame_id.c_str());
  }
  goal_ = {msg->point.x, msg->point.y};
  planned_goal_ = goal_;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  goal_start_time_ = this->now();
  RCLCPP_INFO(this->get_logger(), "New goal (%.2f, %.2f)", goal_.x, goal_.y);
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_position_ = {msg->pose.pose.position.x, msg->pose.pose.position.y};
  have_odom_ = true;
}

void PlannerNode::timerCallback() {
  if (state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    return;
  }
  if (have_odom_ && distanceToGoal() < goal_tolerance_) {
    finishGoal("Goal reached");
    return;
  }
  if ((this->now() - goal_start_time_).seconds() > goal_timeout_) {
    finishGoal("Goal timed out");
    return;
  }
  if ((this->now() - last_plan_time_).seconds() >= replan_period_) {
    planPath();
  }
}

// Distance to where we are actually heading: the requested goal, or the nearest free cell to it
// if the goal was clicked inside an obstacle.
double PlannerNode::distanceToGoal() const {
  return std::hypot(planned_goal_.x - robot_position_.x, planned_goal_.y - robot_position_.y);
}

void PlannerNode::finishGoal(const std::string& reason) {
  RCLCPP_INFO(this->get_logger(), "%s (distance %.2fm), waiting for next goal",
              reason.c_str(), distanceToGoal());
  state_ = State::WAITING_FOR_GOAL;
  publishEmptyPath();
}

void PlannerNode::publishEmptyPath() {
  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = map_ ? map_->header.frame_id : "sim_world";
  path_pub_->publish(path);
}

void PlannerNode::planPath() {
  last_plan_time_ = this->now();
  if (!map_ || !have_odom_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Cannot plan yet: map=%d odom=%d", map_ != nullptr, have_odom_);
    return;
  }

  std::vector<robot::Point2D> points;
  std::string error;
  if (!planner_.plan(*map_, robot_position_, goal_, points, error)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Planning failed: %s. Stopping and retrying.", error.c_str());
    publishEmptyPath();
    return;
  }

  planned_goal_ = points.back();

  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = map_->header.frame_id;
  path.poses.reserve(points.size());
  for (const auto& p : points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = p.x;
    pose.pose.position.y = p.y;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  path_pub_->publish(path);
  RCLCPP_DEBUG(this->get_logger(), "Published path with %zu poses", path.poses.size());
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
