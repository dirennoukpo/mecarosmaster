#pragma once

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace straight_line_pid
{

class StraightLinePidNode : public rclcpp::Node
{
public:
  explicit StraightLinePidNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ROS
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_sub_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  // callbacks
  void cmdVelCallback(
    const geometry_msgs::msg::TwistStamped::SharedPtr msg);

  void imuCallback(
    const sensor_msgs::msg::Imu::SharedPtr msg);

  void controlLoop();

  // utils
  double quaternionToYaw(
    const geometry_msgs::msg::Quaternion & q) const;

  double normalizeAngle(double angle) const;

  double clamp(double value, double min, double max) const;

  void stopRobot();

  // parameters
  double kp_;
  double ki_;
  double kd_;

  double max_wz_;

  double control_frequency_;

  double timeout_;

  // state
  double current_yaw_;

  double target_yaw_;

  double linear_x_;

  double integral_;

  double previous_error_;

  bool moving_;

  bool imu_received_;

  rclcpp::Time last_cmd_time_;

  rclcpp::Time last_control_time_;
};

} // namespace straight_line_pid