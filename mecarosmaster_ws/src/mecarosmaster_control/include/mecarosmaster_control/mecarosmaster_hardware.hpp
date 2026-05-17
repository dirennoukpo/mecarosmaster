/*
** mecarosmaster_hardware.hpp  —  ros2_control SystemInterface pour Mecarosmaster
** Made by dirennoukpo  <diren.noukpo@epitech.eu>
**
** Corrections v3 :
**   • Ordre joints aligné avec l'URDF réel Yahboom X3 :
**       [0]=front_left_joint  [1]=front_right_joint
**       [2]=back_left_joint   [3]=back_right_joint
**   • Dimensions géométriques corrigées depuis l'URDF :
**       wheel_sep_x = 0.08 m (demi-empattement)
**       wheel_sep_y = 0.0845 m (demi-voie)
**   • on_shutdown() ajouté
**   • read() : protection dt invalide
**   • write() : dead-band + arrêt propre si commandes nulles
**   • Validation interfaces URDF dans on_init()
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

class MecarosmasterHardware : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(MecarosmasterHardware)

    CallbackReturn on_init     (const hardware_interface::HardwareInfo&) override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State&)          override;
    CallbackReturn on_activate (const rclcpp_lifecycle::State&)          override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State&)         override;
    CallbackReturn on_cleanup  (const rclcpp_lifecycle::State&)          override;
    CallbackReturn on_shutdown (const rclcpp_lifecycle::State&)          override;

    std::vector<hardware_interface::StateInterface>   export_state_interfaces()   override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    return_type read (const rclcpp::Time&, const rclcpp::Duration&) override;
    return_type write(const rclcpp::Time&, const rclcpp::Duration&) override;

private:
    // ── Paramètres URDF ───────────────────────────────────────────────────────
    std::string serial_port_   = "/dev/myserial";
    int         car_type_      = 1;
    double      cmd_delay_     = 0.002;
    bool        debug_         = false;
    double      ticks_per_rev_ = 1625.0;
    double      wheel_radius_  = 0.045;   // [m]
    // Demi-dimensions depuis l'URDF Yahboom X3 :
    //   front_left  x= 0.08  y= 0.084492
    //   back_left   x=-0.08  y= 0.084492
    //   → demi-empattement = 0.08 m, demi-voie = 0.0845 m
    double      wheel_sep_x_   = 0.08;    // demi-empattement [m]
    double      wheel_sep_y_   = 0.0845;  // demi-voie [m]
    double      cmd_deadband_  = 1e-4;    // [rad/s]

    std::unique_ptr<Mecarosmaster> robot_;

    // Ordre : [0]=FL [1]=FR [2]=RL [3]=RR
    // (front_left_joint, front_right_joint, back_left_joint, back_right_joint)
    static constexpr int N = 4;
    std::array<double, N> hw_pos_ {0, 0, 0, 0};
    std::array<double, N> hw_vel_ {0, 0, 0, 0};
    std::array<double, N> hw_cmd_ {0, 0, 0, 0};
    std::array<int,    N> prev_enc_ {0, 0, 0, 0};

    // Cinématique directe mécanums :
    //   vx = r/4 * ( w0 + w1 + w2 + w3)
    //   vy = r/4 * (-w0 + w1 + w2 - w3)
    //   vz = r/(4*(lx+ly)) * (-w0 + w1 - w2 + w3)
    // w0=FL w1=FR w2=RL w3=RR, convention ROS (X=avant, Y=gauche)
    void computeBodyVelocity(double& vx, double& vy, double& vz) const;
};

// ─────────────────────────────────────────────────────────────────────────────

inline CallbackReturn
MecarosmasterHardware::on_init(const hardware_interface::HardwareInfo& info)
{
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
        return CallbackReturn::ERROR;

    auto param = [&](const std::string& key, const std::string& dflt) {
        auto it = info_.hardware_parameters.find(key);
        return (it != info_.hardware_parameters.end()) ? it->second : dflt;
    };

    serial_port_   = param("serial_port",    "/dev/myserial");
    car_type_      = std::stoi(param("car_type",      "1"));
    cmd_delay_     = std::stod(param("cmd_delay",     "0.002"));
    debug_         = (param("debug", "false") == "true");
    ticks_per_rev_ = std::stod(param("ticks_per_rev", "1625"));
    wheel_radius_  = std::stod(param("wheel_radius",  "0.045"));
    wheel_sep_x_   = std::stod(param("wheel_sep_x",   "0.08"));
    wheel_sep_y_   = std::stod(param("wheel_sep_y",   "0.0845"));
    cmd_deadband_  = std::stod(param("cmd_deadband",  "0.0001"));

    if (info_.joints.size() != 4) {
        RCLCPP_FATAL(rclcpp::get_logger("MecarosmasterHardware"),
            "4 joints attendus, trouvé %zu", info_.joints.size());
        return CallbackReturn::ERROR;
    }

    // Vérification des interfaces déclarées pour chaque joint
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        const auto& j = info_.joints[i];
        if (j.command_interfaces.size() != 1 ||
            j.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(rclcpp::get_logger("MecarosmasterHardware"),
                "Joint '%s' : une command interface 'velocity' attendue",
                j.name.c_str());
            return CallbackReturn::ERROR;
        }
        if (j.state_interfaces.size() != 2) {
            RCLCPP_FATAL(rclcpp::get_logger("MecarosmasterHardware"),
                "Joint '%s' : deux state interfaces (position+velocity) attendues",
                j.name.c_str());
            return CallbackReturn::ERROR;
        }
        RCLCPP_INFO(rclcpp::get_logger("MecarosmasterHardware"),
            "  joint[%zu] = '%s'", i, j.name.c_str());
    }

    RCLCPP_INFO(rclcpp::get_logger("MecarosmasterHardware"),
        "on_init OK — port=%s  car_type=%d  ticks/rev=%.0f  r=%.3f m  "
        "sep_x=%.3f m  sep_y=%.3f m",
        serial_port_.c_str(), car_type_, ticks_per_rev_, wheel_radius_,
        wheel_sep_x_, wheel_sep_y_);
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
            "Impossible d'ouvrir '%s' : %s", serial_port_.c_str(), e.what());
        return CallbackReturn::ERROR;
    }
    hw_pos_.fill(0.0); hw_vel_.fill(0.0); hw_cmd_.fill(0.0); prev_enc_.fill(0);
    RCLCPP_INFO(rclcpp::get_logger("MecarosmasterHardware"), "on_configure OK");
    return CallbackReturn::SUCCESS;
}

inline CallbackReturn
MecarosmasterHardware::on_activate(const rclcpp_lifecycle::State&)
{
    robot_->create_receive_threading();
    robot_->set_auto_report_state(true);
    robot_->get_motor_encoder(prev_enc_[0], prev_enc_[1], prev_enc_[2], prev_enc_[3]);
    robot_->set_car_motion(0.0, 0.0, 0.0);
    RCLCPP_INFO(rclcpp::get_logger("MecarosmasterHardware"),
        "on_activate OK — encodeurs : %d %d %d %d",
        prev_enc_[0], prev_enc_[1], prev_enc_[2], prev_enc_[3]);
    return CallbackReturn::SUCCESS;
}

inline CallbackReturn
MecarosmasterHardware::on_deactivate(const rclcpp_lifecycle::State&)
{
    if (robot_) robot_->set_car_motion(0.0, 0.0, 0.0);
    RCLCPP_INFO(rclcpp::get_logger("MecarosmasterHardware"), "on_deactivate OK");
    return CallbackReturn::SUCCESS;
}

inline CallbackReturn
MecarosmasterHardware::on_cleanup(const rclcpp_lifecycle::State&)
{
    robot_.reset();
    RCLCPP_INFO(rclcpp::get_logger("MecarosmasterHardware"), "on_cleanup OK");
    return CallbackReturn::SUCCESS;
}

inline CallbackReturn
MecarosmasterHardware::on_shutdown(const rclcpp_lifecycle::State&)
{
    if (robot_) { robot_->set_car_motion(0.0, 0.0, 0.0); robot_.reset(); }
    RCLCPP_INFO(rclcpp::get_logger("MecarosmasterHardware"), "on_shutdown OK");
    return CallbackReturn::SUCCESS;
}

inline std::vector<hardware_interface::StateInterface>
MecarosmasterHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> si;
    si.reserve(N * 2);
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        si.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_pos_[i]);
        si.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_vel_[i]);
    }
    return si;
}

inline std::vector<hardware_interface::CommandInterface>
MecarosmasterHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> ci;
    ci.reserve(N);
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        ci.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_cmd_[i]);
    }
    return ci;
}

inline return_type
MecarosmasterHardware::read(const rclcpp::Time&, const rclcpp::Duration& period)
{
    double dt = period.seconds();
    if (dt <= 0.0 || dt > 0.5) dt = 1.0 / 50.0;

    int enc[N];
    robot_->get_motor_encoder(enc[0], enc[1], enc[2], enc[3]);

    const double rad_per_tick = (2.0 * M_PI) / ticks_per_rev_;
    for (int i = 0; i < N; ++i) {
        const double drad = static_cast<double>(enc[i] - prev_enc_[i]) * rad_per_tick;
        hw_pos_[i]   += drad;
        hw_vel_[i]    = drad / dt;
        prev_enc_[i]  = enc[i];
    }
    return return_type::OK;
}

inline return_type
MecarosmasterHardware::write(const rclcpp::Time&, const rclcpp::Duration&)
{
    bool all_zero = true;
    for (int i = 0; i < N; ++i)
        if (std::abs(hw_cmd_[i]) > cmd_deadband_) { all_zero = false; break; }

    if (all_zero) {
        robot_->set_car_motion(0.0, 0.0, 0.0);
    } else {
        double vx, vy, vz;
        computeBodyVelocity(vx, vy, vz);
        robot_->set_car_motion(vx, vy, vz);
    }
    return return_type::OK;
}

inline void
MecarosmasterHardware::computeBodyVelocity(double& vx, double& vy, double& vz) const
{
    // Cinématique directe mécanums (roues 45°, convention ROS REP-103)
    // [0]=FL [1]=FR [2]=RL [3]=RR (RL = back_left, RR = back_right)
    const double r   = wheel_radius_;
    const double lxy = wheel_sep_x_ + wheel_sep_y_;   // L = lx + ly

    const double w0 = hw_cmd_[0];  // front_left
    const double w1 = hw_cmd_[1];  // front_right
    const double w2 = hw_cmd_[2];  // back_left
    const double w3 = hw_cmd_[3];  // back_right

    vx = (r / 4.0) * ( w0 + w1 + w2 + w3);
    vy = (r / 4.0) * (-w0 + w1 + w2 - w3);   // vy positif = gauche (ROS REP-103)
    vz = (r / (4.0 * lxy)) * (-w0 + w1 - w2 + w3);
}

}  // namespace mecarosmaster_ros2_control
