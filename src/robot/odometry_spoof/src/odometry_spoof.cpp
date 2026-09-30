#include <chrono>
#include <memory>

#include "odometry_spoof.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

OdometrySpoofNode::OdometrySpoofNode() : Node("odometry_spoof") {
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom/filtered", 10);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&OdometrySpoofNode::timerCallback, this)
  );
}

void OdometrySpoofNode::timerCallback() {
  const std::string target_frame = "robot/chassis/lidar";
  const std::string source_frame = "sim_world";

  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_->lookupTransform(
      source_frame,
      target_frame,
      tf2::TimePointZero
    );
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Could not transform %s to %s: %s",
                source_frame.c_str(), target_frame.c_str(), ex.what());
    return;
  }

  nav_msgs::msg::Odometry odom_msg;

  odom_msg.header.stamp = transform_stamped.header.stamp;
  odom_msg.header.frame_id = source_frame;
  odom_msg.child_frame_id  = target_frame;

  odom_msg.pose.pose.position.x = transform_stamped.transform.translation.x;
  odom_msg.pose.pose.position.y = transform_stamped.transform.translation.y;
  odom_msg.pose.pose.position.z = transform_stamped.transform.translation.z;
  odom_msg.pose.pose.orientation = transform_stamped.transform.rotation;

  if (has_last_transform_)
  {
    rclcpp::Time current_time = transform_stamped.header.stamp;
    double dt = (current_time - last_time_).seconds();

    if (dt > 0.0)
    {
      double dx = odom_msg.pose.pose.position.x - last_position_.x();
      double dy = odom_msg.pose.pose.position.y - last_position_.y();
      double dz = odom_msg.pose.pose.position.z - last_position_.z();

      odom_msg.twist.twist.linear.x = dx / dt;
      odom_msg.twist.twist.linear.y = dy / dt;
      odom_msg.twist.twist.linear.z = dz / dt;

      tf2::Quaternion q_last(last_orientation_.x(),
                             last_orientation_.y(),
                             last_orientation_.z(),
                             last_orientation_.w());

      tf2::Quaternion q_current(transform_stamped.transform.rotation.x,
                                transform_stamped.transform.rotation.y,
                                transform_stamped.transform.rotation.z,
                                transform_stamped.transform.rotation.w);

      tf2::Quaternion q_diff = q_last.inverse() * q_current;

      double roll_diff, pitch_diff, yaw_diff;
      tf2::Matrix3x3(q_diff).getRPY(roll_diff, pitch_diff, yaw_diff);

      odom_msg.twist.twist.angular.x = roll_diff  / dt;
      odom_msg.twist.twist.angular.y = pitch_diff / dt;
      odom_msg.twist.twist.angular.z = yaw_diff   / dt;
    }
    else
    {
      odom_msg.twist.twist.linear.x  = 0.0;
      odom_msg.twist.twist.linear.y  = 0.0;
      odom_msg.twist.twist.linear.z  = 0.0;
      odom_msg.twist.twist.angular.x = 0.0;
      odom_msg.twist.twist.angular.y = 0.0;
      odom_msg.twist.twist.angular.z = 0.0;
    }
  }
  else
  {
    odom_msg.twist.twist.linear.x  = 0.0;
    odom_msg.twist.twist.linear.y  = 0.0;
    odom_msg.twist.twist.linear.z  = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = 0.0;

    has_last_transform_ = true;
  }

  odom_pub_->publish(odom_msg);

  last_time_ = transform_stamped.header.stamp;
  last_position_.setValue(
    odom_msg.pose.pose.position.x,
    odom_msg.pose.pose.position.y,
    odom_msg.pose.pose.position.z
  );
  last_orientation_.setValue(
    odom_msg.pose.pose.orientation.x,
    odom_msg.pose.pose.orientation.y,
    odom_msg.pose.pose.orientation.z,
    odom_msg.pose.pose.orientation.w
  );
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometrySpoofNode>());
  rclcpp::shutdown();
  return 0;
}
