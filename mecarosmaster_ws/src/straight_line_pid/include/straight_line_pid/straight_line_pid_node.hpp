#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <control_toolbox/pid.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace straight_line_pid
{

class StraightLinePidNode : public rclcpp::Node
{
public:
  explicit StraightLinePidNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ── Callbacks ──────────────────────────────────────────────────────────────
  void cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void controlLoop();

  // ── Helpers ────────────────────────────────────────────────────────────────
  void   applyGainsFromParams();
  void   resetPid();
  void   emergencyStop();
  double quaternionToYaw(const geometry_msgs::msg::Quaternion & q) const;

  // ── ROS interfaces ─────────────────────────────────────────────────────────
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_in_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr            imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr          odom_sub_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr    cmd_out_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  // ── PID ────────────────────────────────────────────────────────────────────
  control_toolbox::Pid pid_;

  // ── State ──────────────────────────────────────────────────────────────────
  double control_frequency_{50.0};

  // Durée max sans réception de cmd_vel avant arrêt d'urgence (secondes)
  double cmd_vel_timeout_{0.5};

  double current_yaw_{0.0};
  double target_yaw_{0.0};
  double odom_yaw_{0.0};

  double linear_x_{0.0};
  double incoming_angular_z_{0.0};

  bool is_moving_{false};
  bool imu_received_{false};

  rclcpp::Time last_time_;

  // Horodatage de la dernière commande reçue — watchdog
  rclcpp::Time last_cmd_time_;
  bool cmd_received_{false};
};

}  // namespace straight_line_pid