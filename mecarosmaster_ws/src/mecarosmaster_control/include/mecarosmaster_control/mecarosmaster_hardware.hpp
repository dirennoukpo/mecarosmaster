// /*
// ** mecarosmaster_hardware.hpp  —  ros2_control SystemInterface pour Mecarosmaster
// ** Made by dirennoukpo  <diren.noukpo@epitech.eu>
// **
// ** v5 : publication IMU / batterie / encodeurs intégrée dans le plugin
// **      → un seul port série ouvert même en mode use_ros2_control:=true
// **      → un rclcpp::Node interne publie sur les mêmes topics que le node autonome
// */

// #pragma once

// #include <array>
// #include <string>
// #include <vector>
// #include <memory>
// #include <cmath>
// #include <limits>

// #include "hardware_interface/system_interface.hpp"
// #include "hardware_interface/handle.hpp"
// #include "hardware_interface/hardware_info.hpp"
// #include "hardware_interface/types/hardware_interface_type_values.hpp"
// #include "rclcpp/rclcpp.hpp"
// #include "rclcpp_lifecycle/state.hpp"

// // Messages publiés
// #include "sensor_msgs/msg/imu.hpp"
// #include "sensor_msgs/msg/magnetic_field.hpp"
// #include "sensor_msgs/msg/battery_state.hpp"
// #include "geometry_msgs/msg/vector3_stamped.hpp"
// #include "std_msgs/msg/int32_multi_array.hpp"

// #include "tf2/LinearMath/Quaternion.h"

// #include "mecarosmaster_control/Mecarosmaster.hpp"

// namespace mecarosmaster_ros2_control {

// using hardware_interface::return_type;
// using hardware_interface::CallbackReturn;

// class MecarosmasterHardware : public hardware_interface::SystemInterface
// {
// public:
//     RCLCPP_SHARED_PTR_DEFINITIONS(MecarosmasterHardware)

//     CallbackReturn on_init      (const hardware_interface::HardwareInfo&) override;
//     CallbackReturn on_configure (const rclcpp_lifecycle::State&)          override;
//     CallbackReturn on_activate  (const rclcpp_lifecycle::State&)          override;
//     CallbackReturn on_deactivate(const rclcpp_lifecycle::State&)          override;
//     CallbackReturn on_cleanup   (const rclcpp_lifecycle::State&)          override;
//     CallbackReturn on_shutdown  (const rclcpp_lifecycle::State&)          override;

//     std::vector<hardware_interface::StateInterface>   export_state_interfaces()   override;
//     std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

//     return_type read (const rclcpp::Time&, const rclcpp::Duration&) override;
//     return_type write(const rclcpp::Time&, const rclcpp::Duration&) override;

// private:
//     // ── Paramètres hardware ───────────────────────────────────────────────────
//     std::string serial_port_   = "/dev/myserial";
//     int         car_type_      = 1;
//     double      cmd_delay_     = 0.002;
//     bool        debug_         = false;
//     double      ticks_per_rev_ = 1625.0;
//     double      wheel_radius_  = 0.045;
//     double      wheel_sep_y_   = 0.169;
//     double      cmd_deadband_  = 1e-4;
//     double      publish_rate_  = 50.0;   // Hz pour le timer IMU/batterie
//     std::string imu_frame_     = "imu_link";

//     std::unique_ptr<Mecarosmaster> robot_;

//     // ── Joints [0]=FL [1]=FR [2]=BL [3]=BR ───────────────────────────────────
//     static constexpr int N = 4;
//     std::array<double, N> hw_pos_  {0, 0, 0, 0};
//     std::array<double, N> hw_vel_  {0, 0, 0, 0};
//     std::array<double, N> hw_cmd_  {0, 0, 0, 0};
//     std::array<int,    N> prev_enc_{0, 0, 0, 0};

//     // ── Node interne pour publier IMU / batterie / encodeurs ──────────────────
//     // Le plugin ros2_control n'est pas un rclcpp::Node, mais on peut en créer
//     // un interne pour publisher des topics sans ouvrir un second port série.
//     rclcpp::Node::SharedPtr sensor_node_;
//     rclcpp::TimerBase::SharedPtr sensor_timer_;

//     rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr              pub_imu_;
//     rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr    pub_mag_;
//     rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_rpy_;
//     rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr     pub_battery_;
//     rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr     pub_enc_;

//     void publishSensors();
// };

// // ══════════════════════════════════════════════════════════════════════════════
// //  on_init
// // ══════════════════════════════════════════════════════════════════════════════
// inline CallbackReturn
// MecarosmasterHardware::on_init(const hardware_interface::HardwareInfo& info)
// {
//     if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
//         return CallbackReturn::ERROR;

//     auto param = [&](const std::string& key, const std::string& dflt) {
//         auto it = info_.hardware_parameters.find(key);
//         return (it != info_.hardware_parameters.end()) ? it->second : dflt;
//     };

//     serial_port_   = param("serial_port",    "/dev/myserial");
//     car_type_      = std::stoi(param("car_type",      "1"));
//     cmd_delay_     = std::stod(param("cmd_delay",     "0.002"));
//     debug_         = (param("debug", "false") == "true");
//     ticks_per_rev_ = std::stod(param("ticks_per_rev", "1625"));
//     wheel_radius_  = std::stod(param("wheel_radius",  "0.045"));
//     wheel_sep_y_   = std::stod(param("wheel_sep_y",   "0.169"));
//     cmd_deadband_  = std::stod(param("cmd_deadband",  "0.0001"));
//     publish_rate_  = std::stod(param("publish_rate",  "50.0"));
//     imu_frame_     = param("imu_frame", "imu_link");

//     if (info_.joints.size() != 4) {
//         RCLCPP_FATAL(rclcpp::get_logger("MecarosmasterHardware"),
//             "4 joints attendus, trouvé %zu", info_.joints.size());
//         return CallbackReturn::ERROR;
//     }

//     for (size_t i = 0; i < info_.joints.size(); ++i) {
//         const auto& j = info_.joints[i];
//         if (j.command_interfaces.size() != 1 ||
//             j.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
//         {
//             RCLCPP_FATAL(rclcpp::get_logger("MecarosmasterHardware"),
//                 "Joint '%s' : une command interface 'velocity' attendue", j.name.c_str());
//             return CallbackReturn::ERROR;
//         }
//         if (j.state_interfaces.size() != 2) {
//             RCLCPP_FATAL(rclcpp::get_logger("MecarosmasterHardware"),
//                 "Joint '%s' : deux state interfaces attendues", j.name.c_str());
//             return CallbackReturn::ERROR;
//         }
//         RCLCPP_INFO(rclcpp::get_logger("MecarosmasterHardware"),
//             "  joint[%zu] = '%s'", i, j.name.c_str());
//     }

//     RCLCPP_INFO(rclcpp::get_logger("MecarosmasterHardware"),
//         "on_init OK — port=%s  car_type=%d  ticks/rev=%.0f  r=%.3f m  sep_y=%.3f m",
//         serial_port_.c_str(), car_type_, ticks_per_rev_, wheel_radius_, wheel_sep_y_);
//     return CallbackReturn::SUCCESS;
// }

// // ══════════════════════════════════════════════════════════════════════════════
// //  on_configure  — ouvre le port série + crée le node interne de capteurs
// // ══════════════════════════════════════════════════════════════════════════════
// inline CallbackReturn
// MecarosmasterHardware::on_configure(const rclcpp_lifecycle::State&)
// {
//     // ── Ouvrir le port série ──────────────────────────────────────────────────
//     try {
//         robot_ = std::make_unique<Mecarosmaster>(
//             car_type_, serial_port_, cmd_delay_, debug_);
//     } catch (const std::exception& e) {
//         RCLCPP_FATAL(rclcpp::get_logger("MecarosmasterHardware"),
//             "Impossible d'ouvrir '%s' : %s", serial_port_.c_str(), e.what());
//         return CallbackReturn::ERROR;
//     }

//     hw_pos_.fill(0.0); hw_vel_.fill(0.0);
//     hw_cmd_.fill(0.0); prev_enc_.fill(0);

//     // ── Créer un node ROS interne pour les capteurs ───────────────────────────
//     // Ce node vit dans le même processus que le controller_manager.
//     // Il utilise la même connexion série via robot_ → aucun second port ouvert.
//     rclcpp::NodeOptions opts;
//     opts.automatically_declare_parameters_from_overrides(true);
//     sensor_node_ = rclcpp::Node::make_shared("mecarosmaster_sensors", opts);

//     const auto qos = rclcpp::SensorDataQoS();
//     pub_imu_     = sensor_node_->create_publisher<sensor_msgs::msg::Imu>(
//                        "mecarosmaster/imu/data",  qos);
//     pub_mag_     = sensor_node_->create_publisher<sensor_msgs::msg::MagneticField>(
//                        "mecarosmaster/imu/mag",   qos);
//     pub_rpy_     = sensor_node_->create_publisher<geometry_msgs::msg::Vector3Stamped>(
//                        "mecarosmaster/imu/rpy",   qos);
//     pub_battery_ = sensor_node_->create_publisher<sensor_msgs::msg::BatteryState>(
//                        "mecarosmaster/battery",   10);
//     pub_enc_     = sensor_node_->create_publisher<std_msgs::msg::Int32MultiArray>(
//                        "mecarosmaster/encoders",  qos);

//     RCLCPP_INFO(rclcpp::get_logger("MecarosmasterHardware"),
//         "on_configure OK — node capteurs créé");
//     return CallbackReturn::SUCCESS;
// }

// // ══════════════════════════════════════════════════════════════════════════════
// //  on_activate  — démarre le thread série + le timer de publication capteurs
// // ══════════════════════════════════════════════════════════════════════════════
// inline CallbackReturn
// MecarosmasterHardware::on_activate(const rclcpp_lifecycle::State&)
// {
//     robot_->create_receive_threading();
//     robot_->set_auto_report_state(true);
//     robot_->get_motor_encoder(prev_enc_[0], prev_enc_[1], prev_enc_[2], prev_enc_[3]);
//     robot_->set_car_motion(0.0, 0.0, 0.0);

//     // Timer qui publie IMU/batterie/encodeurs à publish_rate_ Hz
//     const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
//         std::chrono::duration<double>(1.0 / publish_rate_));
//     sensor_timer_ = sensor_node_->create_wall_timer(
//         period, [this]() { publishSensors(); });

//     // Ajouter le node interne à l'executor du processus courant
//     // (le controller_manager utilise un MultiThreadedExecutor)
//     rclcpp::spin_some(sensor_node_);

//     RCLCPP_INFO(rclcpp::get_logger("MecarosmasterHardware"),
//         "on_activate OK — encodeurs : %d %d %d %d — timer capteurs %.0f Hz",
//         prev_enc_[0], prev_enc_[1], prev_enc_[2], prev_enc_[3], publish_rate_);
//     return CallbackReturn::SUCCESS;
// }

// // ══════════════════════════════════════════════════════════════════════════════
// //  on_deactivate / on_cleanup / on_shutdown
// // ══════════════════════════════════════════════════════════════════════════════
// inline CallbackReturn
// MecarosmasterHardware::on_deactivate(const rclcpp_lifecycle::State&)
// {
//     if (sensor_timer_) { sensor_timer_->cancel(); sensor_timer_.reset(); }
//     if (robot_) robot_->set_car_motion(0.0, 0.0, 0.0);
//     RCLCPP_INFO(rclcpp::get_logger("MecarosmasterHardware"), "on_deactivate OK");
//     return CallbackReturn::SUCCESS;
// }

// inline CallbackReturn
// MecarosmasterHardware::on_cleanup(const rclcpp_lifecycle::State&)
// {
//     sensor_node_.reset();
//     robot_.reset();
//     RCLCPP_INFO(rclcpp::get_logger("MecarosmasterHardware"), "on_cleanup OK");
//     return CallbackReturn::SUCCESS;
// }

// inline CallbackReturn
// MecarosmasterHardware::on_shutdown(const rclcpp_lifecycle::State&)
// {
//     if (sensor_timer_) { sensor_timer_->cancel(); sensor_timer_.reset(); }
//     if (robot_) { robot_->set_car_motion(0.0, 0.0, 0.0); robot_.reset(); }
//     sensor_node_.reset();
//     RCLCPP_INFO(rclcpp::get_logger("MecarosmasterHardware"), "on_shutdown OK");
//     return CallbackReturn::SUCCESS;
// }

// // ══════════════════════════════════════════════════════════════════════════════
// //  export_state_interfaces / export_command_interfaces
// // ══════════════════════════════════════════════════════════════════════════════
// inline std::vector<hardware_interface::StateInterface>
// MecarosmasterHardware::export_state_interfaces()
// {
//     std::vector<hardware_interface::StateInterface> si;
//     si.reserve(N * 2);
//     for (size_t i = 0; i < info_.joints.size(); ++i) {
//         si.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_pos_[i]);
//         si.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_vel_[i]);
//     }
//     return si;
// }

// inline std::vector<hardware_interface::CommandInterface>
// MecarosmasterHardware::export_command_interfaces()
// {
//     std::vector<hardware_interface::CommandInterface> ci;
//     ci.reserve(N);
//     for (size_t i = 0; i < info_.joints.size(); ++i) {
//         ci.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_cmd_[i]);
//     }
//     return ci;
// }

// // ══════════════════════════════════════════════════════════════════════════════
// //  read  — encodeurs → position / vitesse joints
// // ══════════════════════════════════════════════════════════════════════════════
// inline return_type
// MecarosmasterHardware::read(const rclcpp::Time&, const rclcpp::Duration& period)
// {
//     double dt = period.seconds();
//     if (dt <= 0.0 || dt > 0.5) dt = 1.0 / 50.0;

//     int enc[N];
//     robot_->get_motor_encoder(enc[0], enc[1], enc[2], enc[3]);

//     const double rad_per_tick = (2.0 * M_PI) / ticks_per_rev_;
//     for (int i = 0; i < N; ++i) {
//         const double drad = static_cast<double>(enc[i] - prev_enc_[i]) * rad_per_tick;
//         hw_pos_[i]   += drad;
//         hw_vel_[i]    = drad / dt;
//         prev_enc_[i]  = enc[i];
//     }

//     // Spin le node capteurs pour que son timer soit exécuté
//     rclcpp::spin_some(sensor_node_);

//     return return_type::OK;
// }

// // ══════════════════════════════════════════════════════════════════════════════
// //  write  — diff drive : rad/s joints → set_car_motion(vx, 0, wz)
// // ══════════════════════════════════════════════════════════════════════════════
// inline return_type
// MecarosmasterHardware::write(const rclcpp::Time&, const rclcpp::Duration&)
// {
//     const double w_left  = (hw_cmd_[0] + hw_cmd_[2]) * 0.5;  // FL + BL
//     const double w_right = (hw_cmd_[1] + hw_cmd_[3]) * 0.5;  // FR + BR

//     if (std::abs(w_left) <= cmd_deadband_ && std::abs(w_right) <= cmd_deadband_) {
//         robot_->set_car_motion(0.0, 0.0, 0.0);
//         return return_type::OK;
//     }

//     const double vx = wheel_radius_ * (w_right + w_left) * 0.5;
//     const double wz = wheel_radius_ * (w_right - w_left) / wheel_sep_y_;

//     robot_->set_car_motion(vx, 0.0, wz);

//     if (debug_) {
//         RCLCPP_DEBUG(rclcpp::get_logger("MecarosmasterHardware"),
//             "write: wL=%.3f wR=%.3f → vx=%.3f wz=%.3f",
//             w_left, w_right, vx, wz);
//     }
//     return return_type::OK;
// }

// // ══════════════════════════════════════════════════════════════════════════════
// //  publishSensors  — appelé par le timer interne à publish_rate_ Hz
// // ══════════════════════════════════════════════════════════════════════════════
// inline void
// MecarosmasterHardware::publishSensors()
// {
//     if (!robot_) return;

//     const auto stamp = sensor_node_->now();
//     constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

//     // ── IMU ───────────────────────────────────────────────────────────────────
//     {
//         double ax, ay, az, gx, gy, gz, roll, pitch, yaw;
//         robot_->get_accelerometer_data(ax, ay, az);
//         robot_->get_gyroscope_data(gx, gy, gz);
//         robot_->get_imu_attitude_data(roll, pitch, yaw, false);

//         sensor_msgs::msg::Imu msg;
//         msg.header.stamp    = stamp;
//         msg.header.frame_id = imu_frame_;

//         tf2::Quaternion q;
//         q.setRPY(roll, pitch, yaw);
//         msg.orientation.x = q.x();
//         msg.orientation.y = q.y();
//         msg.orientation.z = q.z();
//         msg.orientation.w = q.w();
//         msg.orientation_covariance    = {1.2e-3,0,0, 0,1.2e-3,0, 0,0,1.2e-3};
//         msg.linear_acceleration.x     = ax;
//         msg.linear_acceleration.y     = ay;
//         msg.linear_acceleration.z     = az;
//         msg.linear_acceleration_covariance = {1e-2,0,0, 0,1e-2,0, 0,0,1e-2};
//         msg.angular_velocity.x        = gx;
//         msg.angular_velocity.y        = gy;
//         msg.angular_velocity.z        = gz;
//         msg.angular_velocity_covariance = {1e-4,0,0, 0,1e-4,0, 0,0,1e-4};

//         pub_imu_->publish(msg);

//         // RPY
//         double roll_d, pitch_d, yaw_d;
//         robot_->get_imu_attitude_data(roll_d, pitch_d, yaw_d, true);
//         geometry_msgs::msg::Vector3Stamped rpy;
//         rpy.header = msg.header;
//         rpy.vector.x = roll_d;
//         rpy.vector.y = pitch_d;
//         rpy.vector.z = yaw_d;
//         pub_rpy_->publish(rpy);
//     }

//     // ── Magnétomètre ──────────────────────────────────────────────────────────
//     {
//         double mx, my, mz;
//         robot_->get_magnetometer_data(mx, my, mz);

//         sensor_msgs::msg::MagneticField msg;
//         msg.header.stamp    = stamp;
//         msg.header.frame_id = imu_frame_;
//         msg.magnetic_field.x = mx;
//         msg.magnetic_field.y = my;
//         msg.magnetic_field.z = mz;
//         msg.magnetic_field_covariance = {1e-4,0,0, 0,1e-4,0, 0,0,1e-4};
//         pub_mag_->publish(msg);
//     }

//     // ── Batterie ──────────────────────────────────────────────────────────────
//     {
//         sensor_msgs::msg::BatteryState msg;
//         msg.header.stamp = stamp;
//         msg.voltage      = static_cast<float>(robot_->get_battery_voltage());
//         msg.present      = true;
//         msg.current      = kNaN;
//         msg.charge       = kNaN;
//         msg.capacity     = kNaN;
//         msg.design_capacity = kNaN;
//         msg.percentage   = kNaN;
//         msg.power_supply_status     =
//             sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
//         msg.power_supply_health     =
//             sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
//         msg.power_supply_technology =
//             sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
//         pub_battery_->publish(msg);
//     }

//     // ── Encodeurs bruts ───────────────────────────────────────────────────────
//     {
//         std_msgs::msg::Int32MultiArray msg;
//         msg.data = {prev_enc_[0], prev_enc_[1], prev_enc_[2], prev_enc_[3]};
//         pub_enc_->publish(msg);
//     }
// }

// }  // namespace mecarosmaster_ros2_control



/*
** mecarosmaster_hardware.hpp  —  ros2_control SystemInterface pour Mecarosmaster
** Made by dirennoukpo  <diren.noukpo@epitech.eu>
**
** v6 : suppression de la dépendance tf2 → quaternion RPY calculé inline
**      (évite l'erreur "tf2/LinearMath/Quaternion.h: No such file or directory")
*/

#pragma once

#include <array>
#include <string>
#include <vector>
#include <memory>
#include <cmath>
#include <limits>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

#include "mecarosmaster_control/Mecarosmaster.hpp"

namespace mecarosmaster_ros2_control {

using hardware_interface::return_type;
using hardware_interface::CallbackReturn;

// ── Quaternion RPY sans tf2 ───────────────────────────────────────────────────
struct Quat { double x, y, z, w; };
inline Quat rpy_to_quat(double roll, double pitch, double yaw)
{
    const double cr = std::cos(roll  * 0.5);
    const double sr = std::sin(roll  * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    const double cy = std::cos(yaw   * 0.5);
    const double sy = std::sin(yaw   * 0.5);
    return {
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy,
        cr*cp*cy + sr*sp*sy
    };
}

// ─────────────────────────────────────────────────────────────────────────────

class MecarosmasterHardware : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(MecarosmasterHardware)

    CallbackReturn on_init      (const hardware_interface::HardwareInfo&) override;
    CallbackReturn on_configure (const rclcpp_lifecycle::State&)          override;
    CallbackReturn on_activate  (const rclcpp_lifecycle::State&)          override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State&)          override;
    CallbackReturn on_cleanup   (const rclcpp_lifecycle::State&)          override;
    CallbackReturn on_shutdown  (const rclcpp_lifecycle::State&)          override;

    std::vector<hardware_interface::StateInterface>   export_state_interfaces()   override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    return_type read (const rclcpp::Time&, const rclcpp::Duration&) override;
    return_type write(const rclcpp::Time&, const rclcpp::Duration&) override;

private:
    std::string serial_port_   = "/dev/myserial";
    int         car_type_      = 1;
    double      cmd_delay_     = 0.002;
    bool        debug_         = false;
    double      ticks_per_rev_ = 1625.0;
    double      wheel_radius_  = 0.045;
    double      wheel_sep_y_   = 0.169;
    double      cmd_deadband_  = 1e-4;
    double      publish_rate_  = 50.0;
    std::string imu_frame_     = "imu_link";

    std::unique_ptr<Mecarosmaster> robot_;

    static constexpr int N = 4;
    std::array<double, N> hw_pos_  {0, 0, 0, 0};
    std::array<double, N> hw_vel_  {0, 0, 0, 0};
    std::array<double, N> hw_cmd_  {0, 0, 0, 0};
    std::array<int,    N> prev_enc_{0, 0, 0, 0};

    rclcpp::Node::SharedPtr  sensor_node_;
    rclcpp::TimerBase::SharedPtr sensor_timer_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr              pub_imu_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr    pub_mag_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_rpy_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr     pub_battery_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr     pub_enc_;

    void publishSensors();
};

// ══════════════════════════════════════════════════════════════════════════════

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
    wheel_sep_y_   = std::stod(param("wheel_sep_y",   "0.169"));
    cmd_deadband_  = std::stod(param("cmd_deadband",  "0.0001"));
    publish_rate_  = std::stod(param("publish_rate",  "50.0"));
    imu_frame_     = param("imu_frame", "imu_link");

    if (info_.joints.size() != 4) {
        RCLCPP_FATAL(rclcpp::get_logger("MecarosmasterHardware"),
            "4 joints attendus, trouvé %zu", info_.joints.size());
        return CallbackReturn::ERROR;
    }
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        const auto& j = info_.joints[i];
        if (j.command_interfaces.size() != 1 ||
            j.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(rclcpp::get_logger("MecarosmasterHardware"),
                "Joint '%s' : command interface 'velocity' attendue", j.name.c_str());
            return CallbackReturn::ERROR;
        }
        if (j.state_interfaces.size() != 2) {
            RCLCPP_FATAL(rclcpp::get_logger("MecarosmasterHardware"),
                "Joint '%s' : 2 state interfaces attendues", j.name.c_str());
            return CallbackReturn::ERROR;
        }
        RCLCPP_INFO(rclcpp::get_logger("MecarosmasterHardware"),
            "  joint[%zu] = '%s'", i, j.name.c_str());
    }
    RCLCPP_INFO(rclcpp::get_logger("MecarosmasterHardware"),
        "on_init OK — port=%s  car_type=%d  ticks/rev=%.0f  r=%.3f m  sep_y=%.3f m",
        serial_port_.c_str(), car_type_, ticks_per_rev_, wheel_radius_, wheel_sep_y_);
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
    hw_pos_.fill(0.0); hw_vel_.fill(0.0);
    hw_cmd_.fill(0.0); prev_enc_.fill(0);

    // Node interne pour les capteurs (même processus, même port série)
    rclcpp::NodeOptions opts;
    opts.automatically_declare_parameters_from_overrides(true);
    sensor_node_ = rclcpp::Node::make_shared("mecarosmaster_sensors", opts);

    const auto qos = rclcpp::SensorDataQoS();
    pub_imu_     = sensor_node_->create_publisher<sensor_msgs::msg::Imu>(
                       "mecarosmaster/imu/data",  qos);
    pub_mag_     = sensor_node_->create_publisher<sensor_msgs::msg::MagneticField>(
                       "mecarosmaster/imu/mag",   qos);
    pub_rpy_     = sensor_node_->create_publisher<geometry_msgs::msg::Vector3Stamped>(
                       "mecarosmaster/imu/rpy",   qos);
    pub_battery_ = sensor_node_->create_publisher<sensor_msgs::msg::BatteryState>(
                       "mecarosmaster/battery",   10);
    pub_enc_     = sensor_node_->create_publisher<std_msgs::msg::Int32MultiArray>(
                       "mecarosmaster/encoders",  qos);

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

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_));
    sensor_timer_ = sensor_node_->create_wall_timer(
        period, [this]() { publishSensors(); });

    RCLCPP_INFO(rclcpp::get_logger("MecarosmasterHardware"),
        "on_activate OK — encodeurs : %d %d %d %d",
        prev_enc_[0], prev_enc_[1], prev_enc_[2], prev_enc_[3]);
    return CallbackReturn::SUCCESS;
}

inline CallbackReturn
MecarosmasterHardware::on_deactivate(const rclcpp_lifecycle::State&)
{
    if (sensor_timer_) { sensor_timer_->cancel(); sensor_timer_.reset(); }
    if (robot_) robot_->set_car_motion(0.0, 0.0, 0.0);
    RCLCPP_INFO(rclcpp::get_logger("MecarosmasterHardware"), "on_deactivate OK");
    return CallbackReturn::SUCCESS;
}

inline CallbackReturn
MecarosmasterHardware::on_cleanup(const rclcpp_lifecycle::State&)
{
    sensor_node_.reset();
    robot_.reset();
    RCLCPP_INFO(rclcpp::get_logger("MecarosmasterHardware"), "on_cleanup OK");
    return CallbackReturn::SUCCESS;
}

inline CallbackReturn
MecarosmasterHardware::on_shutdown(const rclcpp_lifecycle::State&)
{
    if (sensor_timer_) { sensor_timer_->cancel(); sensor_timer_.reset(); }
    if (robot_) { robot_->set_car_motion(0.0, 0.0, 0.0); robot_.reset(); }
    sensor_node_.reset();
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
    for (size_t i = 0; i < info_.joints.size(); ++i)
        ci.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_cmd_[i]);
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
        hw_pos_[i]  += drad;
        hw_vel_[i]   = drad / dt;
        prev_enc_[i] = enc[i];
    }

    rclcpp::spin_some(sensor_node_);
    return return_type::OK;
}

inline return_type
MecarosmasterHardware::write(const rclcpp::Time&, const rclcpp::Duration&)
{
    const double w_left  = (hw_cmd_[0] + hw_cmd_[2]) * 0.5;
    const double w_right = (hw_cmd_[1] + hw_cmd_[3]) * 0.5;

    if (std::abs(w_left) <= cmd_deadband_ && std::abs(w_right) <= cmd_deadband_) {
        robot_->set_car_motion(0.0, 0.0, 0.0);
        return return_type::OK;
    }

    const double vx = wheel_radius_ * (w_right + w_left) * 0.5;
    const double wz = wheel_radius_ * (w_right - w_left) / wheel_sep_y_;
    robot_->set_car_motion(vx, 0.0, wz);

    if (debug_)
        RCLCPP_DEBUG(rclcpp::get_logger("MecarosmasterHardware"),
            "write: wL=%.3f wR=%.3f → vx=%.3f wz=%.3f", w_left, w_right, vx, wz);

    return return_type::OK;
}

inline void
MecarosmasterHardware::publishSensors()
{
    if (!robot_) return;
    const auto stamp = sensor_node_->now();
    constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

    // ── IMU ───────────────────────────────────────────────────────────────────
    {
        double ax, ay, az, gx, gy, gz, roll, pitch, yaw;
        robot_->get_accelerometer_data(ax, ay, az);
        robot_->get_gyroscope_data(gx, gy, gz);
        robot_->get_imu_attitude_data(roll, pitch, yaw, false);

        const auto q = rpy_to_quat(roll, pitch, yaw);

        sensor_msgs::msg::Imu msg;
        msg.header.stamp    = stamp;
        msg.header.frame_id = imu_frame_;
        msg.orientation.x   = q.x;
        msg.orientation.y   = q.y;
        msg.orientation.z   = q.z;
        msg.orientation.w   = q.w;
        msg.orientation_covariance         = {1.2e-3,0,0, 0,1.2e-3,0, 0,0,1.2e-3};
        msg.linear_acceleration.x          = ax;
        msg.linear_acceleration.y          = ay;
        msg.linear_acceleration.z          = az;
        msg.linear_acceleration_covariance = {1e-2,0,0, 0,1e-2,0, 0,0,1e-2};
        msg.angular_velocity.x             = gx;
        msg.angular_velocity.y             = gy;
        msg.angular_velocity.z             = gz;
        msg.angular_velocity_covariance    = {1e-4,0,0, 0,1e-4,0, 0,0,1e-4};
        pub_imu_->publish(msg);

        // RPY en degrés
        double rd, pd, yd;
        robot_->get_imu_attitude_data(rd, pd, yd, true);
        geometry_msgs::msg::Vector3Stamped rpy;
        rpy.header   = msg.header;
        rpy.vector.x = rd;
        rpy.vector.y = pd;
        rpy.vector.z = yd;
        pub_rpy_->publish(rpy);
    }

    // ── Magnétomètre ──────────────────────────────────────────────────────────
    {
        double mx, my, mz;
        robot_->get_magnetometer_data(mx, my, mz);
        sensor_msgs::msg::MagneticField msg;
        msg.header.stamp    = stamp;
        msg.header.frame_id = imu_frame_;
        msg.magnetic_field.x = mx;
        msg.magnetic_field.y = my;
        msg.magnetic_field.z = mz;
        msg.magnetic_field_covariance = {1e-4,0,0, 0,1e-4,0, 0,0,1e-4};
        pub_mag_->publish(msg);
    }

    // ── Batterie ──────────────────────────────────────────────────────────────
    {
        sensor_msgs::msg::BatteryState msg;
        msg.header.stamp    = stamp;
        msg.voltage         = static_cast<float>(robot_->get_battery_voltage());
        msg.present         = true;
        msg.current         = kNaN;
        msg.charge          = kNaN;
        msg.capacity        = kNaN;
        msg.design_capacity = kNaN;
        msg.percentage      = kNaN;
        msg.power_supply_status     =
            sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
        msg.power_supply_health     =
            sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
        msg.power_supply_technology =
            sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
        pub_battery_->publish(msg);
    }

    // ── Encodeurs bruts ───────────────────────────────────────────────────────
    {
        std_msgs::msg::Int32MultiArray msg;
        msg.data = {prev_enc_[0], prev_enc_[1], prev_enc_[2], prev_enc_[3]};
        pub_enc_->publish(msg);
    }
}

}  // namespace mecarosmaster_ros2_control