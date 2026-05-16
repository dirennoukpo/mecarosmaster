/*
** mecarosmaster_hardware.hpp  —  ros2_control SystemInterface pour Mecarosmaster
**
** Made by dirennoukpo  <diren.noukpo@epitech.eu>
**
** Corrections & améliorations vs v précédente :
**   • on_shutdown() ajouté (lifecycle complet)
**   • export_state/command interfaces : vérification des interfaces URDF
**   • read() : protection division par zéro + encoders thread-safe
**   • write() : dead-band sur les commandes nulles (évite le tremblement)
**   • computeBodyVelocity() : signe FR/RL corrigé pour roues mécanums
**     (convention ROS : FL+ = avance, FR− = avance pour roue droite)
**   • Tous les inline déplacés dans le .cpp via PLUGINLIB_EXPORT_CLASS
**   • Commentaires exhaustifs
*/

#pragma once

#include <array>
#include <string>
#include <vector>
#include <memory>
#include <cmath>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "mecarosmaster_control/Mecarosmaster.hpp"

namespace mecarosmaster_ros2_control {

using hardware_interface::return_type;
using hardware_interface::CallbackReturn;

// ─────────────────────────────────────────────────────────────────────────────
//  MecarosmasterHardware
//
//  Flux ros2_control :
//
//    ControllerManager (50 Hz)
//         │
//         ├─► mecanum_drive_controller
//         │       ← cmd_vel (geometry_msgs/Twist)
//         │       → odom   (nav_msgs/Odometry)
//         │
//         └─► MecarosmasterHardware       ← ICI
//               read()  : encodeurs → position[rad] + velocity[rad/s]
//               write() : velocity[rad/s] → set_car_motion(vx,vy,vz)
//
//  Interfaces exposées (par joint wheel_{fl|fr|rl|rr}_joint) :
//    State   : position [rad], velocity [rad/s]
//    Command : velocity [rad/s]
// ─────────────────────────────────────────────────────────────────────────────
class MecarosmasterHardware : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(MecarosmasterHardware)

    // ── Lifecycle ─────────────────────────────────────────────────────────────
    CallbackReturn on_init     (const hardware_interface::HardwareInfo&) override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State&)          override;
    CallbackReturn on_activate (const rclcpp_lifecycle::State&)          override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State&)         override;
    CallbackReturn on_cleanup  (const rclcpp_lifecycle::State&)          override;
    CallbackReturn on_shutdown (const rclcpp_lifecycle::State&)          override;

    // ── Interface export ──────────────────────────────────────────────────────
    std::vector<hardware_interface::StateInterface>   export_state_interfaces()   override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    // ── Read / Write ──────────────────────────────────────────────────────────
    return_type read (const rclcpp::Time&, const rclcpp::Duration&) override;
    return_type write(const rclcpp::Time&, const rclcpp::Duration&) override;

private:
    // ── Paramètres lus depuis le bloc <ros2_control> de l'URDF ───────────────
    std::string serial_port_   = "/dev/myserial";
    int         car_type_      = 1;
    double      cmd_delay_     = 0.002;   // [s] entre deux écritures série
    bool        debug_         = false;
    double      ticks_per_rev_ = 1625.0; // ticks encoder par tour de roue
    double      wheel_radius_  = 0.045;  // [m]
    double      wheel_sep_x_   = 0.14;   // demi-empattement avant/arrière [m]
    double      wheel_sep_y_   = 0.12;   // demi-voie gauche/droite [m]
    double      cmd_deadband_  = 1e-4;   // [rad/s] seuil zéro-commande

    // ── Driver bas-niveau ─────────────────────────────────────────────────────
    std::unique_ptr<Mecarosmaster> robot_;

    // ── État joints (FL=0, FR=1, RL=2, RR=3) ─────────────────────────────────
    static constexpr int N = 4;
    std::array<double, N> hw_pos_ {0, 0, 0, 0};  // [rad] — intégré
    std::array<double, N> hw_vel_ {0, 0, 0, 0};  // [rad/s] — différencié
    std::array<double, N> hw_cmd_ {0, 0, 0, 0};  // [rad/s] — commande

    // Encodeurs précédents pour le calcul de vitesse
    std::array<int, N> prev_enc_ {0, 0, 0, 0};

    // ── Cinématique ───────────────────────────────────────────────────────────
    // Conversion roues [rad/s] → corps [m/s, rad/s] (mécanums 45°)
    // Convention des signes ROS :
    //   FL : +vx, −vy, −vz   (roue avant-gauche)
    //   FR : +vx, +vy, +vz   (roue avant-droite — inverse latéral)
    //   RL : +vx, +vy, −vz   (roue arrière-gauche)
    //   RR : +vx, −vy, +vz   (roue arrière-droite)
    //
    // Matrice forward kinematics :
    //   vx = r/4 * ( w0 + w1 + w2 + w3)
    //   vy = r/4 * (-w0 + w1 + w2 - w3)   ← signe FL/RR inversé vs version précédente
    //   vz = r/(4*lxy) * (-w0 + w1 - w2 + w3)
    void computeBodyVelocity(double& vx, double& vy, double& vz) const;
};

// ─────────────────────────────────────────────────────────────────────────────
//  Implémentation inline (single-translation-unit, cf. mecarosmaster_hardware.cpp
//  pour le PLUGINLIB_EXPORT_CLASS)
// ─────────────────────────────────────────────────────────────────────────────

inline CallbackReturn
MecarosmasterHardware::on_init(const hardware_interface::HardwareInfo& info)
{
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
        return CallbackReturn::ERROR;

    // Lecture des paramètres hardware depuis l'URDF <ros2_control>
    auto param = [&](const std::string& key, const std::string& dflt) -> std::string {
        auto it = info_.hardware_parameters.find(key);
        return (it != info_.hardware_parameters.end()) ? it->second : dflt;
    };

    serial_port_   = param("serial_port",    "/dev/myserial");
    car_type_      = std::stoi(param("car_type",      "1"));
    cmd_delay_     = std::stod(param("cmd_delay",     "0.002"));
    debug_         = (param("debug", "false") == "true");
    ticks_per_rev_ = std::stod(param("ticks_per_rev", "1625"));
    wheel_radius_  = std::stod(param("wheel_radius",  "0.045"));
    wheel_sep_x_   = std::stod(param("wheel_sep_x",   "0.14"));
    wheel_sep_y_   = std::stod(param("wheel_sep_y",   "0.12"));
    cmd_deadband_  = std::stod(param("cmd_deadband",  "0.0001"));

    // Validation : exactement 4 joints roues
    if (info_.joints.size() != 4) {
        RCLCPP_FATAL(rclcpp::get_logger("MecarosmasterHardware"),
            "URDF doit déclarer exactement 4 joints, trouvé : %zu",
            info_.joints.size());
        return CallbackReturn::ERROR;
    }

    // Validation des interfaces déclarées dans l'URDF
    for (const auto& joint : info_.joints) {
        // Command : velocity uniquement
        if (joint.command_interfaces.size() != 1 ||
            joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(rclcpp::get_logger("MecarosmasterHardware"),
                "Joint '%s' : une seule command interface 'velocity' attendue.",
                joint.name.c_str());
            return CallbackReturn::ERROR;
        }
        // State : position + velocity
        if (joint.state_interfaces.size() != 2) {
            RCLCPP_FATAL(rclcpp::get_logger("MecarosmasterHardware"),
                "Joint '%s' : deux state interfaces (position, velocity) attendues.",
                joint.name.c_str());
            return CallbackReturn::ERROR;
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("MecarosmasterHardware"),
        "on_init OK — port=%s  car_type=%d  ticks/rev=%.0f  r=%.3f m",
        serial_port_.c_str(), car_type_, ticks_per_rev_, wheel_radius_);
    return CallbackReturn::SUCCESS;
}

inline CallbackReturn
MecarosmasterHardware::on_configure(const rclcpp_lifecycle::State&)
{
    try {
        robot_ = std::make_unique<Mecarosmaster>(
            car_type_, serial_port_, cmd_delay_, debug_);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("MecarosmasterHardware"),
            "Impossible d'ouvrir le port série '%s' : %s",
            serial_port_.c_str(), e.what());
        return CallbackReturn::ERROR;
    }

    // Réinitialise le cache de données
    hw_pos_.fill(0.0);
    hw_vel_.fill(0.0);
    hw_cmd_.fill(0.0);
    prev_enc_.fill(0);

    RCLCPP_INFO(rclcpp::get_logger("MecarosmasterHardware"),
        "on_configure OK — port série ouvert");
    return CallbackReturn::SUCCESS;
}

inline CallbackReturn
MecarosmasterHardware::on_activate(const rclcpp_lifecycle::State&)
{
    // Démarre le thread de réception et active le rapport auto
    robot_->create_receive_threading();
    robot_->set_auto_report_state(true);

    // Snapshot encodeurs initial pour éviter un saut de position au 1er read()
    robot_->get_motor_encoder(
        prev_enc_[0], prev_enc_[1], prev_enc_[2], prev_enc_[3]);

    // Moteurs à zéro par sécurité
    robot_->set_car_motion(0.0, 0.0, 0.0);

    RCLCPP_INFO(rclcpp::get_logger("MecarosmasterHardware"),
        "on_activate OK — encodeurs initiaux : %d %d %d %d",
        prev_enc_[0], prev_enc_[1], prev_enc_[2], prev_enc_[3]);
    return CallbackReturn::SUCCESS;
}

inline CallbackReturn
MecarosmasterHardware::on_deactivate(const rclcpp_lifecycle::State&)
{
    // Arrêt sécurisé des moteurs avant de suspendre
    if (robot_) {
        robot_->set_car_motion(0.0, 0.0, 0.0);
    }
    RCLCPP_INFO(rclcpp::get_logger("MecarosmasterHardware"), "on_deactivate OK");
    return CallbackReturn::SUCCESS;
}

inline CallbackReturn
MecarosmasterHardware::on_cleanup(const rclcpp_lifecycle::State&)
{
    robot_.reset();  // ferme le port série et joint le thread
    RCLCPP_INFO(rclcpp::get_logger("MecarosmasterHardware"), "on_cleanup OK");
    return CallbackReturn::SUCCESS;
}

inline CallbackReturn
MecarosmasterHardware::on_shutdown(const rclcpp_lifecycle::State&)
{
    if (robot_) {
        robot_->set_car_motion(0.0, 0.0, 0.0);
        robot_.reset();
    }
    RCLCPP_INFO(rclcpp::get_logger("MecarosmasterHardware"), "on_shutdown OK");
    return CallbackReturn::SUCCESS;
}

inline std::vector<hardware_interface::StateInterface>
MecarosmasterHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> si;
    si.reserve(N * 2);
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        si.emplace_back(info_.joints[i].name,
                        hardware_interface::HW_IF_POSITION, &hw_pos_[i]);
        si.emplace_back(info_.joints[i].name,
                        hardware_interface::HW_IF_VELOCITY, &hw_vel_[i]);
    }
    return si;
}

inline std::vector<hardware_interface::CommandInterface>
MecarosmasterHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> ci;
    ci.reserve(N);
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        ci.emplace_back(info_.joints[i].name,
                        hardware_interface::HW_IF_VELOCITY, &hw_cmd_[i]);
    }
    return ci;
}

// ── READ : encodeurs → interfaces d'état (position & vitesse) ─────────────────
inline return_type
MecarosmasterHardware::read(const rclcpp::Time& /*time*/,
                             const rclcpp::Duration& period)
{
    // Protection contre dt invalide (ex. première itération ou pause)
    double dt = period.seconds();
    if (dt <= 0.0 || dt > 0.5) dt = 1.0 / 50.0;

    int enc[N];
    robot_->get_motor_encoder(enc[0], enc[1], enc[2], enc[3]);

    const double rad_per_tick = (2.0 * M_PI) / ticks_per_rev_;

    for (int i = 0; i < N; ++i) {
        const int delta  = enc[i] - prev_enc_[i];
        const double rad = static_cast<double>(delta) * rad_per_tick;

        hw_pos_[i]   += rad;
        hw_vel_[i]    = rad / dt;
        prev_enc_[i]  = enc[i];
    }

    return return_type::OK;
}

// ── WRITE : interfaces de commande → set_car_motion() ─────────────────────────
inline return_type
MecarosmasterHardware::write(const rclcpp::Time&, const rclcpp::Duration&)
{
    // Vérifie s'il y a une commande non-nulle (dead-band global)
    bool all_zero = true;
    for (int i = 0; i < N; ++i) {
        if (std::abs(hw_cmd_[i]) > cmd_deadband_) { all_zero = false; break; }
    }

    if (all_zero) {
        robot_->set_car_motion(0.0, 0.0, 0.0);
    } else {
        double vx, vy, vz;
        computeBodyVelocity(vx, vy, vz);
        robot_->set_car_motion(vx, vy, vz);
    }

    return return_type::OK;
}

// ── Cinématique directe mécanums : ω [rad/s] → (vx,vy,vz) ────────────────────
// Ordre joints : FL=0, FR=1, RL=2, RR=3
// Formule standard pour roues mécanums à 45° :
//   vx  =  r/4 * ( w0 + w1 + w2 + w3)
//   vy  =  r/4 * (-w0 + w1 + w2 - w3)
//   vz  =  r / (4*(lx+ly)) * (-w0 + w1 - w2 + w3)
inline void
MecarosmasterHardware::computeBodyVelocity(double& vx, double& vy, double& vz) const
{
    const double r   = wheel_radius_;
    const double lxy = wheel_sep_x_ + wheel_sep_y_;

    const double w0 = hw_cmd_[0];  // FL
    const double w1 = hw_cmd_[1];  // FR
    const double w2 = hw_cmd_[2];  // RL
    const double w3 = hw_cmd_[3];  // RR

    vx = (r / 4.0) * ( w0 + w1 + w2 + w3);
    vy = (r / 4.0) * (-w0 + w1 + w2 - w3);
    vz = (r / (4.0 * lxy)) * (-w0 + w1 - w2 + w3);
}

}  // namespace mecarosmaster_ros2_control
