// #include "straight_line_pid/straight_line_pid_node.hpp"

// #include <cmath>
// #include <chrono>

// using namespace std::chrono_literals;

// namespace straight_line_pid
// {

// StraightLinePidNode::StraightLinePidNode(const rclcpp::NodeOptions & options)
// : rclcpp::Node("straight_line_pid", options)
// {
//   this->declare_parameter("control_frequency", 50.0);
//   this->declare_parameter("cmd_vel_timeout",   0.5);
//   this->declare_parameter("pid.p",       0.0);
//   this->declare_parameter("pid.i",       0.0);
//   this->declare_parameter("pid.d",       0.0);
//   this->declare_parameter("pid.i_clamp", 0.5);

//   control_frequency_ = this->get_parameter("control_frequency").as_double();
//   cmd_vel_timeout_   = this->get_parameter("cmd_vel_timeout").as_double();
//   applyGainsFromParams();

//   // Live gain tuning : on réinitialise TOUJOURS l'état interne du PID
//   // pour éviter l'accumulation d'erreur I/D des sessions précédentes
//   param_cb_ = this->add_on_set_parameters_callback(
//     [this](const std::vector<rclcpp::Parameter> & params) {
//       rcl_interfaces::msg::SetParametersResult result;
//       result.successful = true;
//       for (const auto & p : params) {
//         if (p.get_name().rfind("pid.", 0) == 0) {
//           applyGainsFromParams();  // nouveaux gains
//           resetPid();              // état interne propre (I=0, D=0, last_time frais)
//           RCLCPP_INFO(this->get_logger(),
//             "PID gains updated & state cleared (p=%.3f i=%.3f d=%.3f).",
//             this->get_parameter("pid.p").as_double(),
//             this->get_parameter("pid.i").as_double(),
//             this->get_parameter("pid.d").as_double());
//           break;
//         }
//       }
//       return result;
//     });

//   // INPUT : TwistStamped
//   cmd_in_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
//     "cmd_vel_in", 10,
//     std::bind(&StraightLinePidNode::cmdVelCallback, this, std::placeholders::_1));

//   // IMU : BEST_EFFORT pour correspondre au QoS du driver
//   rclcpp::QoS imu_qos(10);
//   imu_qos.best_effort();
//   imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
//     "/mecarosmaster/imu/data", imu_qos,
//     std::bind(&StraightLinePidNode::imuCallback, this, std::placeholders::_1));

//   // Odométrie
//   odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
//     "/diff_drive_controller/odom", 10,
//     std::bind(&StraightLinePidNode::odomCallback, this, std::placeholders::_1));

//   // OUTPUT : TwistStamped
//   cmd_out_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
//     "cmd_vel_out", 10);

//   const auto period_ms = std::chrono::milliseconds(
//     static_cast<int>(1000.0 / control_frequency_));

//   timer_ = this->create_wall_timer(
//     period_ms,
//     std::bind(&StraightLinePidNode::controlLoop, this));

//   last_time_     = this->now();
//   last_cmd_time_ = this->now();

//   RCLCPP_INFO(this->get_logger(),
//     "StraightLinePidNode ready at %.1f Hz — timeout watchdog: %.2f s.",
//     control_frequency_, cmd_vel_timeout_);
// }

// // ── Helpers ──────────────────────────────────────────────────────────────────

// void StraightLinePidNode::applyGainsFromParams()
// {
//   const double p       = this->get_parameter("pid.p").as_double();
//   const double i       = this->get_parameter("pid.i").as_double();
//   const double d       = this->get_parameter("pid.d").as_double();
//   const double i_clamp = this->get_parameter("pid.i_clamp").as_double();
//   pid_.initPid(p, i, d, i_clamp, -i_clamp, true);
// }

// void StraightLinePidNode::resetPid()
// {
//   // pid_.reset() remet à zéro l'intégrateur, l'erreur précédente et le terme D
//   pid_.reset();
//   // Rafraîchir last_time_ pour que le premier dt soit propre
//   last_time_       = this->now();
//   // Signaler à controlLoop() de sauter le premier tick (dt invalide)
//   first_iteration_ = true;
// }

// void StraightLinePidNode::emergencyStop()
// {
//   geometry_msgs::msg::TwistStamped stop;
//   stop.header.stamp    = this->now();
//   stop.header.frame_id = "base_link";
//   cmd_out_pub_->publish(stop);

//   is_moving_       = false;
//   linear_x_        = 0.0;
//   // Nettoyage complet : garantit qu'aucun résidu I/D ne subsiste
//   resetPid();

//   RCLCPP_WARN(this->get_logger(),
//     "Timeout cmd_vel (%.2f s) — arrêt d'urgence.", cmd_vel_timeout_);
// }

// // ── Callbacks ────────────────────────────────────────────────────────────────

// void StraightLinePidNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
// {
//   current_yaw_  = quaternionToYaw(msg->orientation);
//   imu_received_ = true;
// }

// void StraightLinePidNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
// {
//   odom_yaw_ = quaternionToYaw(msg->pose.pose.orientation);
// }

// void StraightLinePidNode::cmdVelCallback(
//   const geometry_msgs::msg::TwistStamped::SharedPtr msg)
// {
//   last_cmd_time_ = this->now();
//   cmd_received_  = true;

//   const double new_linear_x  = msg->twist.linear.x;
//   const double new_angular_z = msg->twist.angular.z;
//   incoming_angular_z_        = new_angular_z;

//   // ── idle -> ligne droite ──────────────────────────────────────────────────
//   if (!is_moving_ && std::abs(new_linear_x) > 1e-3
//       && std::abs(new_angular_z) < 1e-3)
//   {
//     if (!imu_received_) {
//       RCLCPP_WARN(this->get_logger(),
//         "Pas de données IMU — transmission brute.");
//       cmd_out_pub_->publish(*msg);
//       return;
//     }
//     target_yaw_ = current_yaw_;
//     RCLCPP_INFO(this->get_logger(),
//       "Cap verrouillé : %.4f rad (%.1f deg)",
//       target_yaw_, target_yaw_ * 180.0 / M_PI);
//     // Reset complet à chaque nouveau départ : efface toute accumulation
//     resetPid();
//     is_moving_ = true;
//   }

//   // ── rotation : passage direct, PID désactivé ─────────────────────────────
//   if (std::abs(new_angular_z) > 1e-3) {
//     if (is_moving_) {
//       RCLCPP_INFO(this->get_logger(), "Rotation détectée — relâchement du cap.");
//       resetPid();   // nettoie l'état pour la prochaine ligne droite
//       is_moving_ = false;
//     }
//     cmd_out_pub_->publish(*msg);
//     return;
//   }

//   // ── arrêt explicite (linear.x == 0) ──────────────────────────────────────
//   if (is_moving_ && std::abs(new_linear_x) < 1e-3) {
//     RCLCPP_INFO(this->get_logger(), "Arrêt — relâchement du cap.");
//     is_moving_ = false;
//     resetPid();   // nettoie pour la prochaine session
//     geometry_msgs::msg::TwistStamped stop;
//     stop.header.stamp    = this->now();
//     stop.header.frame_id = msg->header.frame_id;
//     cmd_out_pub_->publish(stop);
//   }

//   linear_x_ = new_linear_x;
// }

// // ── Boucle de contrôle ───────────────────────────────────────────────────────

// void StraightLinePidNode::controlLoop()
// {
//   // ── Watchdog ──────────────────────────────────────────────────────────────
//   if (cmd_received_) {
//     const double age = (this->now() - last_cmd_time_).seconds();
//     if (age > cmd_vel_timeout_) {
//       if (is_moving_) {
//         emergencyStop();
//       }
//       return;
//     }
//   }

//   if (!is_moving_ || !imu_received_) return;

//   const rclcpp::Time now = this->now();
//   const rclcpp::Duration dt = now - last_time_;
//   last_time_ = now;

//   // ── Premier tick après un reset : dt potentiellement invalide ────────────
//   // On saute ce tick pour éviter un spike D et ne pas injecter un gros dt
//   // dans l'intégrateur I avec une erreur initiale non nulle.
//   if (first_iteration_) {
//     first_iteration_ = false;
//     return;
//   }

//   if (dt.seconds() <= 0.0) return;

//   // Erreur angulaire dans [-π, π]
//   double error = target_yaw_ - current_yaw_;
//   while (error >  M_PI) error -= 2.0 * M_PI;
//   while (error < -M_PI) error += 2.0 * M_PI;

//   const double angular_z = pid_.computeCommand(error, dt.nanoseconds());

//   geometry_msgs::msg::TwistStamped twist;
//   twist.header.stamp    = now;
//   twist.header.frame_id = "base_link";
//   twist.twist.linear.x  = linear_x_;
//   twist.twist.angular.z = angular_z;
//   cmd_out_pub_->publish(twist);

//   RCLCPP_DEBUG(this->get_logger(),
//     "yaw=%.4f target=%.4f err=%.4f angular_z=%.4f",
//     current_yaw_, target_yaw_, error, angular_z);
// }

// // ── Utilitaires ──────────────────────────────────────────────────────────────

// double StraightLinePidNode::quaternionToYaw(
//   const geometry_msgs::msg::Quaternion & q) const
// {
//   tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
//   tf2::Matrix3x3 m(tf_q);
//   double roll, pitch, yaw;
//   m.getRPY(roll, pitch, yaw);
//   return yaw;
// }

// }  // namespace straight_line_pid

// // ── main ─────────────────────────────────────────────────────────────────────

// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<straight_line_pid::StraightLinePidNode>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }


#include "straight_line_pid/straight_line_pid_node.hpp"

#include <cmath>
#include <chrono>
#include <algorithm>   // std::clamp

using namespace std::chrono_literals;

namespace straight_line_pid
{

StraightLinePidNode::StraightLinePidNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("straight_line_pid", options)
{
  this->declare_parameter("control_frequency", 50.0);
  this->declare_parameter("cmd_vel_timeout",   0.5);
  this->declare_parameter("pid.p",       0.0);
  this->declare_parameter("pid.i",       0.0);
  this->declare_parameter("pid.d",       0.0);
  this->declare_parameter("pid.i_clamp", 0.5);

  control_frequency_ = this->get_parameter("control_frequency").as_double();
  cmd_vel_timeout_   = this->get_parameter("cmd_vel_timeout").as_double();
  applyGainsFromParams();

  param_cb_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & p : params) {
        if (p.get_name().rfind("pid.", 0) == 0) {
          applyGainsFromParams();
          resetPid();   // efface integral, prev_error, initialized
          RCLCPP_INFO(this->get_logger(),
            "Gains mis à jour et état PID effacé  "
            "[p=%.4f  i=%.4f  d=%.4f  i_clamp=%.4f]",
            pid_.kp, pid_.ki, pid_.kd, pid_.i_clamp);
          break;
        }
      }
      return result;
    });

  cmd_in_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "cmd_vel_in", 10,
    std::bind(&StraightLinePidNode::cmdVelCallback, this, std::placeholders::_1));

  rclcpp::QoS imu_qos(10);
  imu_qos.best_effort();
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/mecarosmaster/imu/data", imu_qos,
    std::bind(&StraightLinePidNode::imuCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/diff_drive_controller/odom", 10,
    std::bind(&StraightLinePidNode::odomCallback, this, std::placeholders::_1));

  cmd_out_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    "cmd_vel_out", 10);

  const auto period_ms = std::chrono::milliseconds(
    static_cast<int>(1000.0 / control_frequency_));

  timer_ = this->create_wall_timer(
    period_ms, std::bind(&StraightLinePidNode::controlLoop, this));

  last_time_     = this->now();
  last_cmd_time_ = this->now();

  RCLCPP_INFO(this->get_logger(),
    "StraightLinePidNode prêt à %.1f Hz — watchdog: %.2f s.",
    control_frequency_, cmd_vel_timeout_);
}

// ── Helpers ──────────────────────────────────────────────────────────────────

void StraightLinePidNode::applyGainsFromParams()
{
  pid_.kp      = this->get_parameter("pid.p").as_double();
  pid_.ki      = this->get_parameter("pid.i").as_double();
  pid_.kd      = this->get_parameter("pid.d").as_double();
  pid_.i_clamp = this->get_parameter("pid.i_clamp").as_double();
  // Ne touche PAS à pid_.integral / pid_.prev_error / pid_.initialized :
  // c'est resetPid() qui s'en charge explicitement
}

void StraightLinePidNode::resetPid()
{
  // Remet integral=0, prev_error=0, initialized=false
  // → le prochain compute() ne calculera pas de terme D
  // → aucun résidu des sessions précédentes
  pid_.reset();
  last_time_ = this->now();
  RCLCPP_DEBUG(this->get_logger(), "PID state cleared.");
}

void StraightLinePidNode::emergencyStop()
{
  geometry_msgs::msg::TwistStamped stop;
  stop.header.stamp    = this->now();
  stop.header.frame_id = "base_link";
  cmd_out_pub_->publish(stop);

  is_moving_ = false;
  linear_x_  = 0.0;
  resetPid();

  RCLCPP_WARN(this->get_logger(),
    "Watchdog — aucune commande depuis %.2f s : arrêt d'urgence.",
    cmd_vel_timeout_);
}

// ── Callbacks ────────────────────────────────────────────────────────────────

void StraightLinePidNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  current_yaw_  = quaternionToYaw(msg->orientation);
  imu_received_ = true;
}

void StraightLinePidNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  odom_yaw_ = quaternionToYaw(msg->pose.pose.orientation);
}

void StraightLinePidNode::cmdVelCallback(
  const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  last_cmd_time_ = this->now();
  cmd_received_  = true;

  const double lx = msg->twist.linear.x;
  const double az = msg->twist.angular.z;

  // ── rotation explicite : pass-through direct, PID nettoyé ────────────────
  if (std::abs(az) > 1e-3) {
    if (is_moving_) {
      RCLCPP_INFO(this->get_logger(), "Rotation — cap relâché, PID effacé.");
      is_moving_ = false;
      resetPid();
    }
    cmd_out_pub_->publish(*msg);
    return;
  }

  // ── arrêt explicite ───────────────────────────────────────────────────────
  if (std::abs(lx) < 1e-3) {
    if (is_moving_) {
      RCLCPP_INFO(this->get_logger(), "Arrêt — cap relâché, PID effacé.");
      is_moving_ = false;
      resetPid();
      geometry_msgs::msg::TwistStamped stop;
      stop.header.stamp    = this->now();
      stop.header.frame_id = msg->header.frame_id;
      cmd_out_pub_->publish(stop);
    }
    linear_x_ = 0.0;
    return;
  }

  // ── idle -> ligne droite ──────────────────────────────────────────────────
  if (!is_moving_) {
    if (!imu_received_) {
      RCLCPP_WARN(this->get_logger(), "Pas d'IMU — transmission brute.");
      cmd_out_pub_->publish(*msg);
      return;
    }
    target_yaw_ = current_yaw_;
    resetPid();   // ← état propre garanti avant le premier tick
    is_moving_  = true;
    RCLCPP_INFO(this->get_logger(),
      "Cap verrouillé : %.4f rad (%.1f°)",
      target_yaw_, target_yaw_ * 180.0 / M_PI);
  }

  linear_x_ = lx;
}

// ── Boucle de contrôle ───────────────────────────────────────────────────────

void StraightLinePidNode::controlLoop()
{
  // Watchdog
  if (cmd_received_) {
    const double age = (this->now() - last_cmd_time_).seconds();
    if (age > cmd_vel_timeout_) {
      if (is_moving_) emergencyStop();
      return;
    }
  }

  if (!is_moving_ || !imu_received_) return;

  const rclcpp::Time now = this->now();
  const double dt = (now - last_time_).seconds();
  last_time_ = now;

  if (dt <= 0.0) return;

  // Erreur angulaire dans [-π, π]
  double error = target_yaw_ - current_yaw_;
  while (error >  M_PI) error -= 2.0 * M_PI;
  while (error < -M_PI) error += 2.0 * M_PI;

  // Calcul PID transparent : P + I (clampé) + D (sauté au 1er tick)
  const double angular_z = pid_.compute(error, dt);

  geometry_msgs::msg::TwistStamped twist;
  twist.header.stamp    = now;
  twist.header.frame_id = "base_link";
  twist.twist.linear.x  = linear_x_;
  twist.twist.angular.z = angular_z;
  cmd_out_pub_->publish(twist);

  RCLCPP_INFO(this->get_logger(),
    "yaw=%.4f  target=%.4f  err=%.4f  I=%.4f  az=%.4f",
    current_yaw_, target_yaw_, error, pid_.integral, angular_z);
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