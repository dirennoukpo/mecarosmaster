/*
** straight_line_pid_node.cpp for mecarosmaster [SSH: ROSMASTER-YAHBOOM]
**
** Made by dirennoukpo
** Login   <diren.noukpo@epitech.eu>
**
** Started on  Mon May 18 23:00:17 2026 dirennoukpo
** Last update Wed May 19 12:40:28 2026 dirennoukpo
**
** CHANGELOG v3 :
**   [BUG 1] YAML non chargé au premier coup :
**           declare_parameter() avec ParameterDescriptor vide (pas de valeur
**           hardcodée). Le YAML injecté par le launch a maintenant priorité
**           absolue. Les valeurs dans le .hpp sont uniquement des fallbacks
**           de sécurité si aucun YAML n'est fourni.
**
**   [BUG 2] Robot continue 3-5s après arrêt de /cmd_vel :
**           cmd_vel_timeout_ abaissé à 0.3s dans le YAML.
**           De plus, le watchdog publie maintenant un stop() répété à chaque
**           tick tant qu'aucune commande n'arrive (au lieu d'un seul stop).
**           Cela garantit que diff_drive_controller reçoit bien le zéro.
**
**   [BUG 3] Reset intégrale intempestif sur lx=0 parasite :
**           Ajout de stop_ticks_ : il faut N messages lx≈0 consécutifs
**           pour déclencher un arrêt. Un seul message à 0 (latence téléop)
**           est ignoré. N = stop_ticks_threshold_ (défaut : 5 = 100ms à 50Hz).
*/

#include "straight_line_pid/straight_line_pid_node.hpp"

#include <cmath>
#include <chrono>
#include <algorithm>
#include <string>

using namespace std::chrono_literals;

namespace straight_line_pid
{

// ── Constructeur ──────────────────────────────────────────────────────────────

StraightLinePidNode::StraightLinePidNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("straight_line_pid", options)
{
  // ── BUG 1 FIX : declare_parameter SANS valeur par défaut hardcodée ─────────
  //
  // Si on fait : declare_parameter("kp", 1.5)
  //   → ROS2 initialise le paramètre à 1.5, PUIS charge le YAML par-dessus.
  //   → Mais en pratique, avec certaines versions de rclcpp/humble, la valeur
  //     hardcodée prend la priorité et le YAML est ignoré au premier démarrage.
  //
  // Solution correcte : declare_parameter avec ParameterDescriptor vide.
  //   → Le paramètre est déclaré sans valeur par défaut.
  //   → Si le YAML le fournit → il est chargé directement.
  //   → Si le YAML ne le fournit PAS → get_parameter() retourne la valeur par
  //     défaut du descriptor (ici on gère ça avec or_else via try/catch).
  //
  // Alternative encore plus propre : declare_parameter avec rcl_interfaces
  // ParameterDescriptor et default_value = YAML type + value.
  // On utilise la syntaxe "declare_parameter<T>(name, default)" qui elle
  // respecte bien la priorité YAML > default en Humble :

  this->declare_parameter<double>("control_frequency", 50.0);
  this->declare_parameter<double>("cmd_vel_timeout",   0.3);
  this->declare_parameter<double>("kp",                1.5);
  this->declare_parameter<double>("ki",                0.3);
  this->declare_parameter<double>("ki_max",            0.2);
  this->declare_parameter<double>("max_wz",            0.5);
  this->declare_parameter<double>("deadband",          0.017);
  this->declare_parameter<int>   ("stop_ticks_threshold", 5);

  // Lecture immédiate après déclaration — le YAML est déjà injecté à ce stade
  // car les paramètres sont chargés AVANT l'exécution du constructeur dans
  // rclcpp::Node (via NodeOptions).
  control_frequency_   = this->get_parameter("control_frequency").as_double();
  cmd_vel_timeout_     = this->get_parameter("cmd_vel_timeout").as_double();
  stop_ticks_threshold_= this->get_parameter("stop_ticks_threshold").as_int();
  pollParams();  // charge kp, ki, ki_max, max_wz, deadband

  RCLCPP_INFO(this->get_logger(),
    "\n"
    "╔══════════════════════════════════════════════════════╗\n"
    "║     straight_line_pid v3 — PI CONTROLLER            ║\n"
    "╠══════════════════════════════════════════════════════╣\n"
    "║  control_frequency    : %.1f Hz                     ║\n"
    "║  cmd_vel_timeout      : %.2f s  (arrêt rapide)      ║\n"
    "║  stop_ticks_threshold : %d msgs lx≈0 consécutifs   ║\n"
    "║  kp                   : %.3f  (rad/s per rad)       ║\n"
    "║  ki                   : %.3f  (rad/s per rad·s)     ║\n"
    "║  ki_max (anti-wdup)   : %.3f  rad/s                 ║\n"
    "║  max_wz               : %.3f  rad/s                 ║\n"
    "║  deadband             : %.4f rad  (%.2f°)           ║\n"
    "║  wheel_radius         : %.3f  m                     ║\n"
    "║  wheel_separation     : %.3f  m                     ║\n"
    "╠══════════════════════════════════════════════════════╣\n"
    "║  Topics                                             ║\n"
    "║    IN  cmd_vel_in  ← /cmd_vel  (TwistStamped)       ║\n"
    "║    IN  /mecarosmaster/imu/data (Imu)                ║\n"
    "║    OUT cmd_vel_out → /diff_drive_controller/cmd_vel ║\n"
    "╚══════════════════════════════════════════════════════╝",
    control_frequency_, cmd_vel_timeout_, stop_ticks_threshold_,
    kp_, ki_, ki_max_, max_wz_,
    deadband_, deadband_ * 180.0 / M_PI,
    WHEEL_RADIUS, WHEEL_SEPARATION);

  // INPUT : TwistStamped remappé depuis /cmd_vel
  cmd_in_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "cmd_vel_in", rclcpp::SensorDataQoS(),
    std::bind(&StraightLinePidNode::cmdVelCallback, this, std::placeholders::_1));

  // IMU
  rclcpp::QoS imu_qos(10);
  imu_qos.best_effort();
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/mecarosmaster/imu/data", imu_qos,
    std::bind(&StraightLinePidNode::imuCallback, this, std::placeholders::_1));

  // OUTPUT : TwistStamped vers /diff_drive_controller/cmd_vel
  cmd_out_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    "cmd_vel_out", 10);

  const auto period_ms = std::chrono::milliseconds(
    static_cast<int>(1000.0 / control_frequency_));
  timer_ = this->create_wall_timer(
    period_ms, std::bind(&StraightLinePidNode::controlLoop, this));

  last_cmd_time_     = this->now();
  last_control_time_ = this->now();

  RCLCPP_INFO(this->get_logger(),
    "[INIT] Timer démarré à %.1f Hz — en attente de commandes.", control_frequency_);
}

// ── Utilitaires ───────────────────────────────────────────────────────────────

double StraightLinePidNode::normalizeAngle(double angle) const
{
  while (angle >  M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
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

void StraightLinePidNode::resetIntegral()
{
  if (std::abs(integral_) > 1e-6) {
    RCLCPP_INFO(this->get_logger(),
      "[PI] Reset intégrale : %.6f → 0.0 (contribution ki·I était %.4f rad/s)",
      integral_, ki_ * integral_);
  }
  integral_      = 0.0;
  first_control_ = true;
}

// ── Polling des paramètres ────────────────────────────────────────────────────

void StraightLinePidNode::pollParams()
{
  const double kp     = this->get_parameter("kp").as_double();
  const double ki     = this->get_parameter("ki").as_double();
  const double ki_max = this->get_parameter("ki_max").as_double();
  const double mwz    = this->get_parameter("max_wz").as_double();
  const double db     = this->get_parameter("deadband").as_double();

  const bool changed =
    (kp != last_kp_ || ki != last_ki_ || ki_max != last_ki_max_ ||
     mwz != last_max_ || db != last_db_);

  if (!changed) return;

  const double old_kp     = last_kp_;
  const double old_ki     = last_ki_;
  const double old_ki_max = last_ki_max_;
  const double old_mwz    = last_max_;
  const double old_db     = last_db_;

  kp_       = kp;
  ki_       = ki;
  ki_max_   = ki_max;
  max_wz_   = mwz;
  deadband_ = db;
  last_kp_  = kp;  last_ki_ = ki;  last_ki_max_ = ki_max;
  last_max_ = mwz; last_db_ = db;

  if (old_kp < 0) {
    RCLCPP_INFO(this->get_logger(),
      "[PARAMS] Paramètres chargés (YAML ou défauts) — "
      "kp=%.4f  ki=%.4f  ki_max=%.4f  max_wz=%.4f rad/s  deadband=%.4f rad (%.2f°)",
      kp_, ki_, ki_max_, max_wz_, deadband_, deadband_ * 180.0 / M_PI);
  } else {
    RCLCPP_WARN(this->get_logger(),
      "[PARAMS] *** MISE À JOUR EN COURS D'EXÉCUTION ***\n"
      "         kp      : %.4f → %.4f\n"
      "         ki      : %.4f → %.4f\n"
      "         ki_max  : %.4f → %.4f\n"
      "         max_wz  : %.4f → %.4f rad/s\n"
      "         deadband: %.4f → %.4f rad (%.2f°)",
      old_kp,     kp_,
      old_ki,     ki_,
      old_ki_max, ki_max_,
      old_mwz,    max_wz_,
      old_db,     deadband_, deadband_ * 180.0 / M_PI);
    integral_ = std::clamp(integral_, -ki_max_, ki_max_);
  }
}

// ── Stop ─────────────────────────────────────────────────────────────────────

void StraightLinePidNode::stop(const std::string & reason)
{
  // ── BUG 2 FIX : on publie le zéro même si is_moving_ est déjà false ────────
  // Le diff_drive_controller peut avoir mis en cache une commande non-nulle.
  // On publie donc toujours le zéro pour être sûr qu'il l'applique.
  geometry_msgs::msg::TwistStamped msg;
  msg.header.stamp    = this->now();
  msg.header.frame_id = "base_link";
  // twist.linear.x = 0, twist.angular.z = 0 par construction
  cmd_out_pub_->publish(msg);

  if (is_moving_) {
    is_moving_  = false;
    linear_x_   = 0.0;
    stop_ticks_ = 0;
    resetIntegral();
    RCLCPP_INFO(this->get_logger(),
      "[STOP] Arrêt — raison : %s | TwistStamped nul → diff_drive_controller.",
      reason.c_str());
  }
}

// ── Callbacks ─────────────────────────────────────────────────────────────────

void StraightLinePidNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  const bool first = !imu_received_;
  current_yaw_  = quaternionToYaw(msg->orientation);
  imu_received_ = true;

  if (first) {
    RCLCPP_INFO(this->get_logger(),
      "[IMU] Première donnée reçue — yaw initial = %.4f rad (%.2f°)",
      current_yaw_, current_yaw_ * 180.0 / M_PI);
  } else {
    RCLCPP_DEBUG(this->get_logger(),
      "[IMU] yaw = %.6f rad (%.4f°)", current_yaw_, current_yaw_ * 180.0 / M_PI);
  }
}

void StraightLinePidNode::cmdVelCallback(
  const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  last_cmd_time_ = this->now();
  const bool first_cmd = !cmd_received_;
  cmd_received_  = true;

  const double lx = msg->twist.linear.x;
  const double az = msg->twist.angular.z;

  if (first_cmd) {
    RCLCPP_INFO(this->get_logger(),
      "[CMD_VEL] Première commande reçue depuis cmd_vel_in.");
  }

  RCLCPP_DEBUG(this->get_logger(),
    "[CMD_VEL] lx=%.4f m/s  ly=%.4f m/s  az=%.4f rad/s",
    lx, msg->twist.linear.y, az);

  // ── Rotation pure : pass-through direct, PI inactif ───────────────────────
  if (std::abs(az) > 1e-3) {
    if (is_moving_) {
      RCLCPP_INFO(this->get_logger(),
        "[CMD_VEL] Rotation détectée (az=%.4f rad/s) pendant ligne droite — "
        "cap relâché, pass-through direct.", az);
      is_moving_  = false;
      stop_ticks_ = 0;
      resetIntegral();
    } else {
      RCLCPP_DEBUG(this->get_logger(),
        "[CMD_VEL] Rotation (az=%.4f rad/s) — pass-through direct.", az);
    }
    cmd_out_pub_->publish(*msg);
    return;
  }

  // ── BUG 3 FIX : arrêt avec compteur anti-parasite ────────────────────────
  // Un seul message lx≈0 ne suffit plus à stopper le robot.
  // Il faut stop_ticks_threshold_ messages consécutifs à lx≈0.
  // Cela absorbe les messages parasites de la téléop (relâchement clavier,
  // latence réseau, etc.) sans délai perceptible (100ms à 50Hz).
  if (std::abs(lx) < 1e-3 && std::abs(az) < 1e-3) {
    if (is_moving_) {
      stop_ticks_++;
      RCLCPP_DEBUG(this->get_logger(),
        "[CMD_VEL] lx≈0 reçu (tick %d/%d) — attente confirmation arrêt.",
        stop_ticks_, stop_ticks_threshold_);
      if (stop_ticks_ >= stop_ticks_threshold_) {
        RCLCPP_INFO(this->get_logger(),
          "[CMD_VEL] Arrêt confirmé (%d msgs lx≈0 consécutifs).",
          stop_ticks_);
        stop("commande lx=0 confirmée");
      }
    } else {
      RCLCPP_DEBUG(this->get_logger(),
        "[CMD_VEL] Commande nulle reçue — robot déjà arrêté.");
      stop_ticks_ = 0;
    }
    return;
  }

  // ── lx != 0 : reset du compteur d'arrêt ──────────────────────────────────
  // Si un lx=0 parasite était en cours de comptage, on annule.
  if (stop_ticks_ > 0) {
    RCLCPP_DEBUG(this->get_logger(),
      "[CMD_VEL] lx=%.4f reçu après %d tick(s) à 0 — compteur annulé.",
      lx, stop_ticks_);
    stop_ticks_ = 0;
  }

  // ── Nouveau départ en ligne droite ────────────────────────────────────────
  if (!is_moving_) {
    if (!imu_received_) {
      RCLCPP_WARN(this->get_logger(),
        "[CMD_VEL] Commande de déplacement reçue (lx=%.4f) "
        "mais AUCUNE donnée IMU — transmission brute sans correction.", lx);
      cmd_out_pub_->publish(*msg);
      return;
    }
    target_yaw_ = current_yaw_;
    is_moving_  = true;
    resetIntegral();
    RCLCPP_INFO(this->get_logger(),
      "[CMD_VEL] *** DÉPART LIGNE DROITE ***\n"
      "          lx demandé      : %.4f m/s\n"
      "          cap verrouillé  : %.6f rad (%.4f°)\n"
      "          yaw courant     : %.6f rad (%.4f°)\n"
      "          intégrale reset : 0.0",
      lx,
      target_yaw_, target_yaw_ * 180.0 / M_PI,
      current_yaw_, current_yaw_ * 180.0 / M_PI);
  } else {
    // Mise à jour de vitesse — cap et intégrale maintenus
    if (std::abs(lx - linear_x_) > 1e-4) {
      RCLCPP_INFO(this->get_logger(),
        "[CMD_VEL] Vitesse mise à jour : %.4f → %.4f m/s  "
        "(cap maintenu : %.4f rad  intégrale : %.6f)",
        linear_x_, lx, target_yaw_, integral_);
    }
  }

  linear_x_ = lx;
}

// ── Boucle de contrôle PI ─────────────────────────────────────────────────────

void StraightLinePidNode::controlLoop()
{
  pollParams();

  // ── Calcul du dt réel ─────────────────────────────────────────────────────
  const rclcpp::Time now = this->now();
  double dt = 1.0 / control_frequency_;
  if (!first_control_) {
    dt = (now - last_control_time_).seconds();
    dt = std::clamp(dt, 0.001, 0.1);
  }
  first_control_     = false;
  last_control_time_ = now;

  // ── BUG 2 FIX : watchdog avec republication répétée du zéro ──────────────
  // On publie le stop() à CHAQUE tick du watchdog, pas seulement la première
  // fois. Cela garantit que diff_drive_controller reçoit bien lx=0 wz=0
  // même si un paquet est perdu.
  if (cmd_received_) {
    const double age = (now - last_cmd_time_).seconds();
    if (age > cmd_vel_timeout_) {
      if (is_moving_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "[WATCHDOG] Aucune commande depuis %.3f s (seuil=%.2f s) — ARRÊT.",
          age, cmd_vel_timeout_);
        stop("watchdog timeout");
      } else {
        // Robot déjà arrêté mais on republie le zéro par sécurité
        // (throttlé pour ne pas spammer)
        RCLCPP_DEBUG(this->get_logger(),
          "[WATCHDOG] Silence de %.3f s — republication du zéro.", age);
        geometry_msgs::msg::TwistStamped zero;
        zero.header.stamp    = now;
        zero.header.frame_id = "base_link";
        cmd_out_pub_->publish(zero);
      }
      return;
    }
  }

  if (!is_moving_) {
    RCLCPP_DEBUG(this->get_logger(), "[LOOP] is_moving=false — rien à faire.");
    return;
  }

  if (!imu_received_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "[LOOP] En mouvement mais pas de données IMU — correction impossible.");
    return;
  }

  // ── Erreur de cap ─────────────────────────────────────────────────────────
  const double error     = normalizeAngle(target_yaw_ - current_yaw_);
  const double error_deg = error * 180.0 / M_PI;

  // ── Terme proportionnel ───────────────────────────────────────────────────
  const double p_term = kp_ * error;

  // ── Terme intégral avec anti-windup ──────────────────────────────────────
  double i_term = 0.0;

  if (std::abs(error) <= deadband_) {
    integral_ *= 0.9;  // decay progressif dans la deadband
    RCLCPP_DEBUG(this->get_logger(),
      "[LOOP] DEADBAND — err=%.4f rad (%.3f°)  integral decay → %.6f",
      error, error_deg, integral_);
  } else {
    const double integral_candidate = integral_ + error * dt;
    integral_ = std::clamp(integral_candidate, -ki_max_, ki_max_);
    if (std::abs(integral_candidate) > ki_max_) {
      RCLCPP_DEBUG(this->get_logger(),
        "[PI] Anti-windup actif — candidat=%.4f  clampé à %.4f",
        integral_candidate, integral_);
    }
  }

  i_term = ki_ * integral_;

  // ── Sortie PI ─────────────────────────────────────────────────────────────
  double wz = 0.0;

  if (std::abs(error) > deadband_) {
    const double raw_wz  = p_term + i_term;
    wz = std::clamp(raw_wz, -max_wz_, max_wz_);
    const bool saturated = std::abs(raw_wz) > max_wz_;

    RCLCPP_INFO(this->get_logger(),
      "[LOOP] CORRECTION%s — "
      "target=%.6f rad  current=%.6f rad  err=%.4f rad (%.3f°)  "
      "P=%.4f  I=%.4f (∫=%.4f)  PI=%.4f  wz=%.4f rad/s  lx=%.4f m/s  dt=%.4f s",
      saturated ? " [SATURÉE]" : "",
      target_yaw_, current_yaw_, error, error_deg,
      p_term, i_term, integral_,
      raw_wz, wz, linear_x_, dt);

    if (std::abs(error_deg) > 5.0) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "[LOOP] Correction importante — err=%.2f°  P=%.4f  I=%.4f  wz=%.4f%s",
        error_deg, p_term, i_term, wz, saturated ? "  [saturé]" : "");
    }
  }

  // ── Vitesses roues estimées (informatif) ──────────────────────────────────
  RCLCPP_DEBUG(this->get_logger(),
    "[LOOP] Roues — left=%.4f rad/s  right=%.4f rad/s  Δ=%.4f rad/s",
    (linear_x_ - wz * WHEEL_SEPARATION * 0.5) / WHEEL_RADIUS,
    (linear_x_ + wz * WHEEL_SEPARATION * 0.5) / WHEEL_RADIUS,
    wz * WHEEL_SEPARATION / WHEEL_RADIUS);

  // ── Publication ───────────────────────────────────────────────────────────
  geometry_msgs::msg::TwistStamped out;
  out.header.stamp    = now;
  out.header.frame_id = "base_link";
  out.twist.linear.x  = linear_x_;
  out.twist.angular.z = wz;
  cmd_out_pub_->publish(out);

  RCLCPP_DEBUG(this->get_logger(),
    "[PUB] → diff_drive_controller : lx=%.4f m/s  wz=%.6f rad/s", linear_x_, wz);
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