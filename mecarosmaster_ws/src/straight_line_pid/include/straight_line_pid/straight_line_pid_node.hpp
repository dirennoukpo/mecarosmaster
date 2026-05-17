#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <control_toolbox/pid.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <memory>

namespace straight_line_pid
{

class StraightLinePidNode : public rclcpp::Node
{
public:
  explicit StraightLinePidNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void controlLoop();
  double quaternionToYaw(const geometry_msgs::msg::Quaternion & q) const;
  void resetPid();
  void applyGainsFromParams();

  control_toolbox::Pid pid_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr         imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr        odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr      cmd_in_sub_;
  // Publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr         cmd_out_pub_;
  rclcpp::TimerBase::SharedPtr                                    timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  double current_yaw_{0.0};
  double odom_yaw_{0.0};
  double target_yaw_{0.0};
  double linear_x_{0.0};
  double incoming_angular_z_{0.0};
  bool   is_moving_{false};
  bool   imu_received_{false};
  rclcpp::Time last_time_;
  double control_frequency_{50.0};
};

}  // namespace straight_line_pid