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
  this->declare_parameter("cmd_vel_timeout",   0.5);   // secondes
  this->declare_parameter("pid.p",       0.0);
  this->declare_parameter("pid.i",       0.00);
  this->declare_parameter("pid.d",       0.0);
  this->declare_parameter("pid.i_clamp", 0.5);

  control_frequency_ = this->get_parameter("control_frequency").as_double();
  cmd_vel_timeout_   = this->get_parameter("cmd_vel_timeout").as_double();
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

  // INPUT : TwistStamped — remappé depuis /cmd_vel
  cmd_in_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "cmd_vel_in", 10,
    std::bind(&StraightLinePidNode::cmdVelCallback, this, std::placeholders::_1));

  // IMU : BEST_EFFORT pour correspondre au QoS du driver
  rclcpp::QoS imu_qos(10);
  imu_qos.best_effort();
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/mecarosmaster/imu/data", imu_qos,
    std::bind(&StraightLinePidNode::imuCallback, this, std::placeholders::_1));

  // Odométrie : vérification croisée / usage futur
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/diff_drive_controller/odom", 10,
    std::bind(&StraightLinePidNode::odomCallback, this, std::placeholders::_1));

  // OUTPUT : TwistStamped — remappé vers /diff_drive_controller/cmd_vel
  cmd_out_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    "cmd_vel_out", 10);

  const auto period_ms = std::chrono::milliseconds(
    static_cast<int>(1000.0 / control_frequency_));

  timer_ = this->create_wall_timer(
    period_ms,
    std::bind(&StraightLinePidNode::controlLoop, this));

  last_time_    = this->now();
  last_cmd_time_ = this->now();

  RCLCPP_INFO(this->get_logger(),
    "StraightLinePidNode ready at %.1f Hz — timeout watchdog: %.2f s.",
    control_frequency_, cmd_vel_timeout_);
}

// ── Helpers ──────────────────────────────────────────────────────────────────

void StraightLinePidNode::applyGainsFromParams()
{
  const double p       = this->get_parameter("pid.p").as_double();
  const double i       = this->get_parameter("pid.i").as_double();
  const double d       = this->get_parameter("pid.d").as_double();
  const double i_clamp = this->get_parameter("pid.i_clamp").as_double();
  pid_.initPid(p, i, d, i_clamp, -i_clamp, true);
}

void StraightLinePidNode::emergencyStop()
{
  geometry_msgs::msg::TwistStamped stop;
  stop.header.stamp    = this->now();
  stop.header.frame_id = "base_link";
  // Tous les champs twist restent à zéro — robot à l'arrêt
  cmd_out_pub_->publish(stop);

  is_moving_ = false;
  linear_x_  = 0.0;
  pid_.reset();

  RCLCPP_WARN(this->get_logger(),
    "Timeout cmd_vel (%.2f s) — arrêt d'urgence.", cmd_vel_timeout_);
}

// ── Callbacks ────────────────────────────────────────────────────────────────

void StraightLinePidNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  current_yaw_ = quaternionToYaw(msg->orientation);
  imu_received_ = true;
}

void StraightLinePidNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  odom_yaw_ = quaternionToYaw(msg->pose.pose.orientation);
}

void StraightLinePidNode::cmdVelCallback(
  const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  // ── Mise à jour du watchdog ───────────────────────────────────────────────
  last_cmd_time_ = this->now();
  cmd_received_  = true;

  const double new_linear_x  = msg->twist.linear.x;
  const double new_angular_z = msg->twist.angular.z;
  incoming_angular_z_        = new_angular_z;

  // ── Transition : idle -> avance en ligne droite ───────────────────────────
  if (!is_moving_ && std::abs(new_linear_x) > 1e-3
      && std::abs(new_angular_z) < 1e-3)
  {
    if (!imu_received_) {
      RCLCPP_WARN(this->get_logger(),
        "Pas de données IMU — impossible de verrouiller le cap. "
        "Transmission brute de la commande.");
      cmd_out_pub_->publish(*msg);
      return;
    }
    target_yaw_ = current_yaw_;
    RCLCPP_INFO(this->get_logger(),
      "Cap verrouillé : %.4f rad (%.1f deg)",
      target_yaw_, target_yaw_ * 180.0 / M_PI);
    resetPid();
    is_moving_ = true;
  }

  // ── Commande de rotation : passage direct, PID désactivé ─────────────────
  if (std::abs(new_angular_z) > 1e-3) {
    is_moving_ = false;
    cmd_out_pub_->publish(*msg);
    return;
  }

  // ── Transition : mouvement -> arrêt explicite (linear.x == 0) ────────────
  if (is_moving_ && std::abs(new_linear_x) < 1e-3) {
    RCLCPP_INFO(this->get_logger(), "Commande d'arrêt reçue — relâchement du cap.");
    is_moving_ = false;
    geometry_msgs::msg::TwistStamped stop;
    stop.header.stamp    = this->now();
    stop.header.frame_id = msg->header.frame_id;
    cmd_out_pub_->publish(stop);
  }

  linear_x_ = new_linear_x;
}

// ── Boucle de contrôle ───────────────────────────────────────────────────────

void StraightLinePidNode::controlLoop()
{
  // ── Watchdog : aucune commande reçue depuis cmd_vel_timeout_ secondes ─────
  if (cmd_received_) {
    const double age = (this->now() - last_cmd_time_).seconds();
    if (age > cmd_vel_timeout_) {
      if (is_moving_) {
        emergencyStop();
      }
      return;   // Ne publie rien tant que les commandes ne reviennent pas
    }
  }

  if (!is_moving_ || !imu_received_) return;

  const rclcpp::Time now = this->now();
  const rclcpp::Duration dt = now - last_time_;
  last_time_ = now;

  if (dt.seconds() <= 0.0) return;

  // Erreur angulaire ramenée dans [-π, π]
  double error = target_yaw_ - current_yaw_;
  while (error >  M_PI) error -= 2.0 * M_PI;
  while (error < -M_PI) error += 2.0 * M_PI;

  const double angular_z = pid_.computeCommand(error, dt.nanoseconds());

  geometry_msgs::msg::TwistStamped twist;
  twist.header.stamp    = now;
  twist.header.frame_id = "base_link";
  twist.twist.linear.x  = linear_x_;
  twist.twist.angular.z = angular_z;
  cmd_out_pub_->publish(twist);

  RCLCPP_DEBUG(this->get_logger(),
    "yaw=%.4f target=%.4f err=%.4f angular_z=%.4f",
    current_yaw_, target_yaw_, error, angular_z);
}

// ── Utilitaires ──────────────────────────────────────────────────────────────

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

// ── main ─────────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<straight_line_pid::StraightLinePidNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}