#include "straight_line_pid/straight_line_pid_node.hpp"

#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

namespace straight_line_pid
{

StraightLinePidNode::StraightLinePidNode(
  const rclcpp::NodeOptions & options)
: Node("straight_line_pid", options)
{
  // paramètres
  declare_parameter("kp", 3.0);
  declare_parameter("ki", 0.0);
  declare_parameter("kd", 0.2);

  declare_parameter("max_wz", 1.0);

  declare_parameter("control_frequency", 50.0);

  declare_parameter("timeout", 0.5);

  kp_ = get_parameter("kp").as_double();
  ki_ = get_parameter("ki").as_double();
  kd_ = get_parameter("kd").as_double();

  max_wz_ = get_parameter("max_wz").as_double();

  control_frequency_ =
    get_parameter("control_frequency").as_double();

  timeout_ =
    get_parameter("timeout").as_double();

  // état
  current_yaw_ = 0.0;
  target_yaw_ = 0.0;
  linear_x_ = 0.0;

  integral_ = 0.0;
  previous_error_ = 0.0;

  moving_ = false;
  imu_received_ = false;

  last_cmd_time_ = now();
  last_control_time_ = now();

  // subscriber cmd
  cmd_sub_ =
    create_subscription<geometry_msgs::msg::TwistStamped>(
      "cmd_vel_in",
      rclcpp::SensorDataQoS(),
      std::bind(
        &StraightLinePidNode::cmdVelCallback,
        this,
        std::placeholders::_1));

  // subscriber imu
  rclcpp::QoS imu_qos(10);
  imu_qos.best_effort();

  imu_sub_ =
    create_subscription<sensor_msgs::msg::Imu>(
      "/mecarosmaster/imu/data",
      imu_qos,
      std::bind(
        &StraightLinePidNode::imuCallback,
        this,
        std::placeholders::_1));

  // publisher
  cmd_pub_ =
    create_publisher<geometry_msgs::msg::TwistStamped>(
      "cmd_vel_out",
      10);

  // timer
  timer_ =
    create_wall_timer(
      std::chrono::milliseconds(
        static_cast<int>(1000.0 / control_frequency_)),
      std::bind(
        &StraightLinePidNode::controlLoop,
        this));

  RCLCPP_INFO(
    get_logger(),
    "Straight Line PID Started");
}

double StraightLinePidNode::clamp(
  double value,
  double min,
  double max) const
{
  return std::max(min, std::min(value, max));
}

double StraightLinePidNode::normalizeAngle(double angle) const
{
  while (angle > M_PI)
    angle -= 2.0 * M_PI;

  while (angle < -M_PI)
    angle += 2.0 * M_PI;

  return angle;
}

double StraightLinePidNode::quaternionToYaw(
  const geometry_msgs::msg::Quaternion & q) const
{
  tf2::Quaternion quat(q.x, q.y, q.z, q.w);

  tf2::Matrix3x3 m(quat);

  double roll, pitch, yaw;

  m.getRPY(roll, pitch, yaw);

  return yaw;
}

void StraightLinePidNode::imuCallback(
  const sensor_msgs::msg::Imu::SharedPtr msg)
{
  current_yaw_ =
    quaternionToYaw(msg->orientation);

  imu_received_ = true;
}

void StraightLinePidNode::cmdVelCallback(
  const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  last_cmd_time_ = now();

  const double lx = msg->twist.linear.x;
  const double az = msg->twist.angular.z;

  // rotation pure
  if (std::abs(az) > 0.001)
  {
    moving_ = false;

    cmd_pub_->publish(*msg);

    return;
  }

  // arrêt
  if (std::abs(lx) < 0.001)
  {
    stopRobot();

    return;
  }

  // nouveau départ
  if (!moving_)
  {
    target_yaw_ = current_yaw_;

    integral_ = 0.0;

    previous_error_ = 0.0;

    moving_ = true;

    RCLCPP_INFO(
      get_logger(),
      "Heading locked: %.2f deg",
      target_yaw_ * 180.0 / M_PI);
  }

  linear_x_ = lx;
}

void StraightLinePidNode::stopRobot()
{
  moving_ = false;

  linear_x_ = 0.0;

  integral_ = 0.0;

  previous_error_ = 0.0;

  geometry_msgs::msg::TwistStamped out;

  out.header.stamp = now();

  cmd_pub_->publish(out);
}

void StraightLinePidNode::controlLoop()
{
  if (!moving_)
    return;

  if (!imu_received_)
    return;

  const auto now_time = now();

  // timeout
  if ((now_time - last_cmd_time_).seconds() > timeout_)
  {
    stopRobot();

    return;
  }

  // dt
  double dt =
    (now_time - last_control_time_).seconds();

  dt = clamp(dt, 0.001, 0.1);

  last_control_time_ = now_time;

  // erreur
  double error =
    normalizeAngle(target_yaw_ - current_yaw_);

  // PID
  integral_ += error * dt;

  double derivative =
    (error - previous_error_) / dt;

  previous_error_ = error;

  double wz =
      kp_ * error
    + ki_ * integral_
    + kd_ * derivative;

  // saturation
  wz = clamp(wz, -max_wz_, max_wz_);

  // publication
  geometry_msgs::msg::TwistStamped out;

  out.header.stamp = now_time;

  out.twist.linear.x = linear_x_;

  out.twist.angular.z = wz;

  cmd_pub_->publish(out);

  RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    500,
    "err=%.3f  wz=%.3f",
    error,
    wz);
}

} // namespace straight_line_pid

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<
      straight_line_pid::StraightLinePidNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}