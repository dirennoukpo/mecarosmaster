/*
** straight_line_pid_node.hpp for mecarosmaster [SSH: ROSMASTER-YAHBOOM]
**
** Made by dirennoukpo
** Login   <diren.noukpo@epitech.eu>
**
** Started on  Mon May 18 22:59:58 2026 dirennoukpo
** Last update Wed May 19 12:40:42 2026 dirennoukpo
**
** CHANGELOG v3 :
**   - Correction du chargement YAML (ParameterDescriptor sans default hardcodé)
**   - Watchdog timeout réduit à 0.3s (arrêt quasi-immédiat après silence)
**   - Anti-reset-intempestif : stop_ticks_ (N msgs lx≈0 consécutifs requis)
**   - stop() avec raison loggée
*/

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace straight_line_pid
{

class StraightLinePidNode : public rclcpp::Node
{
public:
  explicit StraightLinePidNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void controlLoop();

  void   pollParams();
  void   stop(const std::string & reason);
  void   resetIntegral();
  double quaternionToYaw(const geometry_msgs::msg::Quaternion & q) const;
  double normalizeAngle(double angle) const;

  // ── ROS interfaces ──────────────────────────────────────────────────────────
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_in_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr            imu_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr    cmd_out_pub_;
  rclcpp::TimerBase::SharedPtr                                       timer_;

  // ── Géométrie du robot ──────────────────────────────────────────────────────
  static constexpr double WHEEL_RADIUS     = 0.045;  // [m]
  static constexpr double WHEEL_SEPARATION = 0.169;  // [m]

  // ── Paramètres de contrôle ──────────────────────────────────────────────────
  // Ces valeurs ne sont PAS des defaults hardcodés : elles sont écrasées
  // par le YAML via declare_parameter() avec ParameterDescriptor (pas de valeur).
  // Elles servent uniquement à initialiser les membres avant pollParams().
  double kp_{1.5};
  double ki_{0.3};
  double ki_max_{0.2};
  double max_wz_{0.5};
  double deadband_{0.017};
  double cmd_vel_timeout_{0.3};   // [s] — arrêt rapide si plus de commande
  double control_frequency_{50.0};

  // Nombre de messages lx≈0 consécutifs avant d'accepter un arrêt explicite.
  // Protège contre les lx=0 parasites de la téléop (latence clavier, etc.).
  // À 50 Hz : 5 ticks = 100 ms de silence "lx=0" avant arrêt.
  int stop_ticks_threshold_{5};

  // Snapshots pour détection de changements live
  double last_kp_{-1}, last_ki_{-1}, last_ki_max_{-1}, last_max_{-1}, last_db_{-1};

  // ── État du contrôleur PI ───────────────────────────────────────────────────
  double integral_{0.0};
  rclcpp::Time last_control_time_;
  bool   first_control_{true};

  // ── État général ────────────────────────────────────────────────────────────
  double current_yaw_{0.0};
  double target_yaw_{0.0};
  double linear_x_{0.0};

  bool is_moving_{false};
  bool imu_received_{false};
  bool cmd_received_{false};

  // Compteur anti-reset-intempestif :
  // incrémenté à chaque callback avec lx≈0 pendant is_moving_=true.
  // Réinitialisé à 0 dès qu'un lx != 0 arrive.
  // L'arrêt n'est déclenché que quand stop_ticks_ >= stop_ticks_threshold_.
  int stop_ticks_{0};

  rclcpp::Time last_cmd_time_;
};

}  // namespace straight_line_pid