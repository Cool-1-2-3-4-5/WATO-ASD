#include "control_node.hpp"

using namespace std::chrono_literals;

ControlNode::ControlNode() : Node("control_node") {
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, std::bind(&ControlNode::path_callback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&ControlNode::odom_callback, this, std::placeholders::_1));

  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  timer_ = this->create_wall_timer(
    100ms, std::bind(&ControlNode::control_loop, this));
}

void ControlNode::path_callback(nav_msgs::msg::Path::SharedPtr msg) {
  path_msg_ = msg;
}

void ControlNode::odom_callback(nav_msgs::msg::Odometry::SharedPtr msg) {
  odom_msg_ = msg;
}

void ControlNode::control_loop() {
  if (!path_msg_ || !odom_msg_) {
    return;
  }

  int path_size = path_msg_->poses.size();
  if (path_size == 0) {
    return;
  }

  double robot_x = odom_msg_->pose.pose.position.x;
  double robot_y = odom_msg_->pose.pose.position.y;
  
  double qw = odom_msg_->pose.pose.orientation.w;
  double qx = odom_msg_->pose.pose.orientation.x;
  double qy = odom_msg_->pose.pose.orientation.y;
  double qz = odom_msg_->pose.pose.orientation.z;
  
  double yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

  double lookahead = 1.0;
  double speed = 0.5;
  
  double target_x = path_msg_->poses[path_size - 1].pose.position.x;
  double target_y = path_msg_->poses[path_size - 1].pose.position.y;

  for (int i = 0; i < path_size; i++) {
    double px = path_msg_->poses[i].pose.position.x;
    double py = path_msg_->poses[i].pose.position.y;
    double dx = px - robot_x;
    double dy = py - robot_y;
    double dist = std::sqrt((dx * dx) + (dy * dy));
    
    if (dist >= lookahead) {
      target_x = px;
      target_y = py;
      break;
    }
  }

  double final_x = path_msg_->poses[path_size - 1].pose.position.x;
  double final_y = path_msg_->poses[path_size - 1].pose.position.y;
  
  double end_dx = final_x - robot_x;
  double end_dy = final_y - robot_y;
  double dist_to_goal = std::sqrt((end_dx * end_dx) + (end_dy * end_dy));

  geometry_msgs::msg::Twist cmd;

  if (dist_to_goal < 0.1) {
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
  } else {
    double tx = target_x - robot_x;
    double ty = target_y - robot_y;
    
    double local_y = -std::sin(yaw) * tx + std::cos(yaw) * ty;
    double dist_to_target = std::sqrt((tx * tx) + (ty * ty));
    
    cmd.linear.x = speed;
    cmd.angular.z = speed * (2.0 * local_y) / (dist_to_target * dist_to_target);
  }

  cmd_pub_->publish(cmd);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<ControlNode> node = std::make_shared<ControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}