#include "straight_line_pid/straight_line_pid_node.hpp"

#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

namespace straight_line_pid
{

StraightLinePidNode::StraightLinePidNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("straight_line_pid", options)
{
  this->declare_parameter("control_frequency", 50.0);
  this->declare_parameter("pid.p",       0.0);
  this->declare_parameter("pid.i",       0.0);
  this->declare_parameter("pid.d",       0.0);
  this->declare_parameter("pid.i_clamp", 0.5);

  control_frequency_ = this->get_parameter("control_frequency").as_double();
  applyGainsFromParams();

  // Live gain tuning without restart
  param_cb_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & p : params) {
        if (p.get_name().rfind("pid.", 0) == 0) {
          applyGainsFromParams();
          RCLCPP_INFO(this->get_logger(), "PID gains updated.");
          break;
        }
      }
      return result;
    });

  // Subscribe to the standard /cmd_vel topic.
  // The launch file remaps it from whatever the upstream node publishes.
  // This means no "phantom" topic: teleop, Nav2, etc. all work unchanged.
  cmd_in_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel_in", 10,
    std::bind(&StraightLinePidNode::cmdVelCallback, this, std::placeholders::_1));

  // IMU: BEST_EFFORT to match driver QoS
  rclcpp::QoS imu_qos(10);
  imu_qos.best_effort();
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/mecarosmaster/imu/data", imu_qos,
    std::bind(&StraightLinePidNode::imuCallback, this, std::placeholders::_1));

  // Odometry: available as backup / future use
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/diff_drive_controller/odom", 10,
    std::bind(&StraightLinePidNode::odomCallback, this, std::placeholders::_1));

  // Publish corrected cmd_vel directly to the controller
  cmd_out_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "cmd_vel_out", 10);

  const auto period_ms = std::chrono::milliseconds(
    static_cast<int>(1000.0 / control_frequency_));

  timer_ = this->create_wall_timer(
    period_ms,
    std::bind(&StraightLinePidNode::controlLoop, this));

  last_time_ = this->now();

  RCLCPP_INFO(this->get_logger(),
    "StraightLinePidNode ready at %.1f Hz — intercepts /cmd_vel, corrects yaw.",
    control_frequency_);
}

void StraightLinePidNode::applyGainsFromParams()
{
  const double p       = this->get_parameter("pid.p").as_double();
  const double i       = this->get_parameter("pid.i").as_double();
  const double d       = this->get_parameter("pid.d").as_double();
  const double i_clamp = this->get_parameter("pid.i_clamp").as_double();
  pid_.initPid(p, i, d, i_clamp, -i_clamp, true);
}

void StraightLinePidNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  current_yaw_ = quaternionToYaw(msg->orientation);
  imu_received_ = true;
}

void StraightLinePidNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Store odom yaw as a sanity cross-check (available for future use)
  odom_yaw_ = quaternionToYaw(msg->pose.pose.orientation);
}

void StraightLinePidNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  const double new_linear_x = msg->linear.x;
  // Pass through any intentional rotation as-is (e.g. turning commands)
  incoming_angular_z_ = msg->angular.z;

  // Transition: idle -> moving straight
  if (!is_moving_ && std::abs(new_linear_x) > 1e-3
      && std::abs(msg->angular.z) < 1e-3)
  {
    if (!imu_received_) {
      RCLCPP_WARN(this->get_logger(),
        "No IMU data yet — cannot lock yaw. Forwarding raw command.");
      cmd_out_pub_->publish(*msg);
      return;
    }
    target_yaw_ = current_yaw_;
    RCLCPP_INFO(this->get_logger(),
      "Heading locked: %.4f rad (%.1f deg)", target_yaw_, target_yaw_ * 180.0 / M_PI);
    resetPid();
    is_moving_ = true;
  }

  // If angular.z is non-zero, the user wants to turn: pass through, no PID
  if (std::abs(msg->angular.z) > 1e-3) {
    is_moving_ = false;
    cmd_out_pub_->publish(*msg);
    return;
  }

  // Transition: moving -> stop
  if (is_moving_ && std::abs(new_linear_x) < 1e-3) {
    RCLCPP_INFO(this->get_logger(), "Stop — releasing heading lock.");
    is_moving_ = false;
    cmd_out_pub_->publish(geometry_msgs::msg::Twist{});
  }

  linear_x_ = new_linear_x;
}

void StraightLinePidNode::controlLoop()
{
  if (!is_moving_ || !imu_received_) return;

  const rclcpp::Time now = this->now();
  const rclcpp::Duration dt = now - last_time_;
  last_time_ = now;

  if (dt.seconds() <= 0.0) return;

  double error = target_yaw_ - current_yaw_;
  while (error >  M_PI) error -= 2.0 * M_PI;
  while (error < -M_PI) error += 2.0 * M_PI;

  const double angular_z = pid_.computeCommand(error, dt.nanoseconds());

  geometry_msgs::msg::Twist twist;
  twist.linear.x  = linear_x_;
  twist.angular.z = angular_z;
  cmd_out_pub_->publish(twist);

  RCLCPP_DEBUG(this->get_logger(),
    "yaw=%.4f target=%.4f err=%.4f angular_z=%.4f",
    current_yaw_, target_yaw_, error, angular_z);
}

double StraightLinePidNode::quaternionToYaw(
  const geometry_msgs::msg::Quaternion & q) const
{
  tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
  tf2::Matrix3x3 m(tf_q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

void StraightLinePidNode::resetPid()
{
  pid_.reset();
  last_time_ = this->now();
}

}  // namespace straight_line_pid

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<straight_line_pid::StraightLinePidNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}