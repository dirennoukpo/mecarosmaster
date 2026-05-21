/*
** straight_line_pid_node.hpp for mecarosmaster [SSH: ROSMASTER-YAHBOOM]
**
** Made by dirennoukpo
** Login   <diren.noukpo@epitech.eu>
**
** Started on  Mon May 18 22:59:58 2026 dirennoukpo
** Last update Thu May 20 09:55:38 2026 dirennoukpo
**
** CHANGELOG v5 :
**   - Correction du commentaire sur dynamic_typing (voir .cpp v5).
**
** CHANGELOG v4 :
**   - Suppression des initialiseurs membres hardcodés (kp_, deadband_, etc.)
**     Ces valeurs étaient TROMPEUSES : le commentaire disait "elles sont
**     écrasées par le YAML" mais elles servaient de silencieux fallback si
**     le YAML était absent ou ignoré. Désormais le constructeur lève une
**     exception explicite si un paramètre est manquant — plus de fallback caché.
**   - Les membres PID sont déclarés sans valeur d'initialisation.
**     Ils sont OBLIGATOIREMENT remplis par get_parameter() dans le constructeur.
**   - stop_ticks_threshold_ sans valeur d'initialisation pour la même raison.
**   - Mise à jour du changelog (v3 → v4).
**
** CHANGELOG v3 :
**   - Anti-reset-intempestif : stop_ticks_ (N msgs lx≈0 consécutifs requis)
**   - Watchdog timeout configurable via YAML
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
  // Ces valeurs SONT les mêmes defaults que declare_parameter<T>(name, default)
  // dans le constructeur. Elles ne servent que si le nœud est lancé via
  // `ros2 run` sans YAML (cas de debug). En utilisation normale via le launch,
  // le YAML pid_params.yaml les écrase car il a priorité sur les defaults
  // (ordre Humble : default < YAML < CLI override).
  // Le launch v4+ ne passe PLUS de bloc override → YAML = niveau le plus haut.
  double kp_{1.5};
  double ki_{0.3};
  double ki_max_{0.2};
  double max_wz_{0.5};
  double deadband_{0.010};     // ← valeur YAML : 0.010 (pas 0.017)
  double cmd_vel_timeout_{0.5}; // ← valeur YAML : 0.5s
  double control_frequency_{50.0};

  // Nombre de messages lx≈0 consécutifs avant d'accepter un arrêt.
  // À 50 Hz : 5 ticks = 100 ms de silence avant arrêt.
  int stop_ticks_threshold_{5};

  // Snapshots pour détection de changements live (pollParams).
  // Initialisés à -1 pour signaler "pas encore chargé" au premier appel.
  double last_kp_{-1.0}, last_ki_{-1.0}, last_ki_max_{-1.0},
         last_max_{-1.0}, last_db_{-1.0};

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

  // Compteur anti-reset-intempestif.
  // Incrémenté à chaque callback avec lx≈0 pendant is_moving_=true.
  // Réinitialisé à 0 dès qu'un lx != 0 arrive.
  // L'arrêt n'est déclenché que quand stop_ticks_ >= stop_ticks_threshold_.
  int stop_ticks_{0};

  rclcpp::Time last_cmd_time_;
};

}  // namespace straight_line_pid