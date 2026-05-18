// #pragma once

// #include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/twist_stamped.hpp>
// #include <geometry_msgs/msg/quaternion.hpp>
// #include <sensor_msgs/msg/imu.hpp>
// #include <nav_msgs/msg/odometry.hpp>
// #include <control_toolbox/pid.hpp>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Matrix3x3.h>

// namespace straight_line_pid
// {

// class StraightLinePidNode : public rclcpp::Node
// {
// public:
//   explicit StraightLinePidNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

// private:
//   // ── Callbacks ──────────────────────────────────────────────────────────────
//   void cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
//   void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
//   void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
//   void controlLoop();

//   // ── Helpers ────────────────────────────────────────────────────────────────
//   void   applyGainsFromParams();
//   void   resetPid();          // reset intégrateur + last_time_ + first_iteration_
//   void   emergencyStop();
//   double quaternionToYaw(const geometry_msgs::msg::Quaternion & q) const;

//   // ── ROS interfaces ─────────────────────────────────────────────────────────
//   rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_in_sub_;
//   rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr            imu_sub_;
//   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr          odom_sub_;
//   rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr    cmd_out_pub_;

//   rclcpp::TimerBase::SharedPtr timer_;
//   rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

//   // ── PID ────────────────────────────────────────────────────────────────────
//   control_toolbox::Pid pid_;

//   // ── State ──────────────────────────────────────────────────────────────────
//   double control_frequency_{50.0};
//   double cmd_vel_timeout_{0.5};

//   double current_yaw_{0.0};
//   double target_yaw_{0.0};
//   double odom_yaw_{0.0};

//   double linear_x_{0.0};
//   double incoming_angular_z_{0.0};

//   bool is_moving_{false};
//   bool imu_received_{false};

//   // Watchdog
//   rclcpp::Time last_cmd_time_;
//   bool cmd_received_{false};

//   // Evite un spike D/I au premier tick après resetPid()
//   bool first_iteration_{true};

//   rclcpp::Time last_time_;
// };

// }  // namespace straight_line_pid

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace straight_line_pid
{

// ── PID maison — état 100 % visible et contrôlable ───────────────────────────
struct PidState
{
  double kp{0.0};
  double ki{0.0};
  double kd{0.0};
  double i_clamp{0.5};

  double integral{0.0};
  double prev_error{0.0};
  bool   initialized{false};   // false = premier tick, on ne calcule pas D

  void reset()
  {
    integral    = 0.0;
    prev_error  = 0.0;
    initialized = false;
  }

  // Retourne la commande angulaire, dt en secondes
  double compute(double error, double dt)
  {
    // Terme P
    const double p_term = kp * error;

    // Terme I avec anti-windup par clamping
    integral += error * dt;
    integral  = std::clamp(integral, -i_clamp, i_clamp);
    const double i_term = ki * integral;

    // Terme D : sauté au premier appel pour éviter le spike initial
    double d_term = 0.0;
    if (initialized) {
      d_term = kd * (error - prev_error) / dt;
    }
    initialized = true;
    prev_error  = error;

    return p_term + i_term + d_term;
  }
};

// ─────────────────────────────────────────────────────────────────────────────

class StraightLinePidNode : public rclcpp::Node
{
public:
  explicit StraightLinePidNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void controlLoop();

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
  PidState pid_;

  // ── State ──────────────────────────────────────────────────────────────────
  double control_frequency_{50.0};
  double cmd_vel_timeout_{0.5};

  double current_yaw_{0.0};
  double target_yaw_{0.0};
  double odom_yaw_{0.0};

  double linear_x_{0.0};

  bool is_moving_{false};
  bool imu_received_{false};

  rclcpp::Time last_time_;
  rclcpp::Time last_cmd_time_;
  bool cmd_received_{false};
};

}  // namespace straight_line_pid