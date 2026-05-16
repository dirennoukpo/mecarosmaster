/*
** mecarosmaster_node.cpp  —  ROS 2 Humble  —  node autonome (sans ros2_control)
**
** Made by dirennoukpo  <diren.noukpo@epitech.eu>
**
** Corrections vs version précédente :
**   • cmd_vel topic renommé "cmd_vel" (sans namespace) pour compatibilité
**     Navigation Stack et teleop_twist_keyboard
**   • joint_states publié sur "/joint_states" (requis par robot_state_publisher)
**   • odom      publié sur "/odom"
**   • Watchdog thread-safe (std::atomic)
**   • Intégration d'odométrie : utilise les données motion du robot (pas DR pur)
**   • publishJointStates : vitesses calculées par différenciation des encodeurs
**   • Suppression de la dépendance tf2_geometry_msgs (non utilisée)
**   • RCLCPP_INFO_ONCE au lieu de double RCLCPP_INFO dans le constructeur
*/

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "mecarosmaster_control/Mecarosmaster.hpp"

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <chrono>
#include <mutex>
#include <atomic>
#include <array>

using namespace std::chrono_literals;

// ─────────────────────────────────────────────────────────────────────────────
//  Paramètres du node
// ─────────────────────────────────────────────────────────────────────────────
struct NodeParams {
    std::string serial_port     = "/dev/myserial";
    int         car_type        = 1;
    double      cmd_delay       = 0.002;
    bool        debug           = false;
    double      publish_rate    = 50.0;
    bool        publish_tf      = true;
    std::string odom_frame      = "odom";
    std::string base_frame      = "base_link";
    std::string imu_frame       = "imu_link";
    double      gyro_cov        = 1e-4;
    double      accel_cov       = 1e-2;
    double      mag_cov         = 1e-4;
    bool        arm_enabled     = true;
    double      cmd_vel_timeout = 0.5;
    double      ticks_per_rev   = 1625.0;
    double      wheel_radius    = 0.045;
};

// ─────────────────────────────────────────────────────────────────────────────
//  Intégrateur de pose (dead-reckoning)
// ─────────────────────────────────────────────────────────────────────────────
struct OdomPose {
    double x{0}, y{0}, theta{0};

    void integrate(double vx, double vy, double vz, double dt) noexcept
    {
        const double c = std::cos(theta);
        const double s = std::sin(theta);
        x     += (vx * c - vy * s) * dt;
        y     += (vx * s + vy * c) * dt;
        theta += vz * dt;
    }
};

// ─────────────────────────────────────────────────────────────────────────────
//  MecarosmasterNode
// ─────────────────────────────────────────────────────────────────────────────
class MecarosmasterNode : public rclcpp::Node
{
public:
    explicit MecarosmasterNode(
        const rclcpp::NodeOptions& opts = rclcpp::NodeOptions())
    : Node("mecarosmaster_node", opts)
    {
        // ── Paramètres ────────────────────────────────────────────────────────
        declare_parameter("serial_port",     params_.serial_port);
        declare_parameter("car_type",        params_.car_type);
        declare_parameter("cmd_delay",       params_.cmd_delay);
        declare_parameter("debug",           params_.debug);
        declare_parameter("publish_rate",    params_.publish_rate);
        declare_parameter("publish_tf",      params_.publish_tf);
        declare_parameter("odom_frame",      params_.odom_frame);
        declare_parameter("base_frame",      params_.base_frame);
        declare_parameter("imu_frame",       params_.imu_frame);
        declare_parameter("gyro_cov",        params_.gyro_cov);
        declare_parameter("accel_cov",       params_.accel_cov);
        declare_parameter("mag_cov",         params_.mag_cov);
        declare_parameter("arm_enabled",     params_.arm_enabled);
        declare_parameter("cmd_vel_timeout", params_.cmd_vel_timeout);
        declare_parameter("ticks_per_rev",   params_.ticks_per_rev);
        declare_parameter("wheel_radius",    params_.wheel_radius);

        get_parameter("serial_port",     params_.serial_port);
        get_parameter("car_type",        params_.car_type);
        get_parameter("cmd_delay",       params_.cmd_delay);
        get_parameter("debug",           params_.debug);
        get_parameter("publish_rate",    params_.publish_rate);
        get_parameter("publish_tf",      params_.publish_tf);
        get_parameter("odom_frame",      params_.odom_frame);
        get_parameter("base_frame",      params_.base_frame);
        get_parameter("imu_frame",       params_.imu_frame);
        get_parameter("gyro_cov",        params_.gyro_cov);
        get_parameter("accel_cov",       params_.accel_cov);
        get_parameter("mag_cov",         params_.mag_cov);
        get_parameter("arm_enabled",     params_.arm_enabled);
        get_parameter("cmd_vel_timeout", params_.cmd_vel_timeout);
        get_parameter("ticks_per_rev",   params_.ticks_per_rev);
        get_parameter("wheel_radius",    params_.wheel_radius);

        // ── Driver ────────────────────────────────────────────────────────────
        try {
            robot_ = std::make_unique<Mecarosmaster>(
                params_.car_type, params_.serial_port,
                params_.cmd_delay, params_.debug);
        } catch (const std::exception& e) {
            RCLCPP_FATAL(get_logger(),
                "Impossible d'ouvrir Mecarosmaster : %s", e.what());
            throw;
        }

        robot_->create_receive_threading();
        robot_->set_auto_report_state(true);
        robot_->set_uart_servo_ctrl_enable(params_.arm_enabled);

        // Snapshot encodeurs initial
        robot_->get_motor_encoder(
            prev_enc_[0], prev_enc_[1], prev_enc_[2], prev_enc_[3]);

        // ── TF broadcaster ────────────────────────────────────────────────────
        if (params_.publish_tf) {
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }

        // ── Publishers ────────────────────────────────────────────────────────
        const auto sensor_qos = rclcpp::SensorDataQoS();

        pub_imu_     = create_publisher<sensor_msgs::msg::Imu>(
                       "mecarosmaster/imu/data",    sensor_qos);
        pub_rpy_     = create_publisher<geometry_msgs::msg::Vector3Stamped>(
                       "mecarosmaster/imu/rpy",     sensor_qos);
        pub_mag_     = create_publisher<sensor_msgs::msg::MagneticField>(
                       "mecarosmaster/imu/mag",     sensor_qos);
        pub_odom_    = create_publisher<nav_msgs::msg::Odometry>(
                       "odom",                      sensor_qos);
        pub_battery_ = create_publisher<sensor_msgs::msg::BatteryState>(
                       "mecarosmaster/battery",     10);
        pub_enc_     = create_publisher<std_msgs::msg::Int32MultiArray>(
                       "mecarosmaster/encoders",    sensor_qos);
        pub_joint_   = create_publisher<sensor_msgs::msg::JointState>(
                       "joint_states",              sensor_qos);  // topic standard RSP
        pub_vel_     = create_publisher<geometry_msgs::msg::TwistStamped>(
                       "mecarosmaster/velocity",    sensor_qos);

        // ── Subscribers ───────────────────────────────────────────────────────

        // cmd_vel — commande principale de mouvement (compatible Nav2 / teleop)
        sub_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", rclcpp::SensorDataQoS(),
            [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
                last_cmd_vel_time_.store(
                    now().nanoseconds(), std::memory_order_relaxed);
                motor_stopped_.store(false, std::memory_order_relaxed);
                robot_->set_car_motion(
                    msg->linear.x, msg->linear.y, msg->angular.z);
            });

        // Vitesses moteurs brutes [−100..100] × 4
        sub_motors_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            "mecarosmaster/motors/cmd", 10,
            [this](std_msgs::msg::Float32MultiArray::ConstSharedPtr msg) {
                if (msg->data.size() < 4) {
                    RCLCPP_WARN(get_logger(),
                        "motors/cmd : 4 valeurs attendues, %zu reçues",
                        msg->data.size());
                    return;
                }
                robot_->set_motor(
                    msg->data[0], msg->data[1],
                    msg->data[2], msg->data[3]);
            });

        // Servos PWM [0..180] × 4
        sub_pwm_servos_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            "mecarosmaster/pwm_servos/cmd", 10,
            [this](std_msgs::msg::Float32MultiArray::ConstSharedPtr msg) {
                if (msg->data.size() < 4) {
                    RCLCPP_WARN(get_logger(),
                        "pwm_servos/cmd : 4 valeurs attendues, %zu reçues",
                        msg->data.size());
                    return;
                }
                robot_->set_pwm_servo_all(
                    static_cast<int>(msg->data[0]),
                    static_cast<int>(msg->data[1]),
                    static_cast<int>(msg->data[2]),
                    static_cast<int>(msg->data[3]));
            });

        // Couleur LED (ColorRGBA, valeurs [0..1])
        sub_leds_ = create_subscription<std_msgs::msg::ColorRGBA>(
            "mecarosmaster/leds/color", 10,
            [this](std_msgs::msg::ColorRGBA::ConstSharedPtr msg) {
                robot_->set_colorful_lamps(
                    0,
                    static_cast<int>(std::clamp(msg->r, 0.f, 1.f) * 255),
                    static_cast<int>(std::clamp(msg->g, 0.f, 1.f) * 255),
                    static_cast<int>(std::clamp(msg->b, 0.f, 1.f) * 255));
            });

        // Bras 6-DOF — JointTrajectory (positions en degrés)
        sub_arm_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "mecarosmaster/arm/joint_cmd", 10,
            [this](trajectory_msgs::msg::JointTrajectory::ConstSharedPtr msg) {
                if (msg->points.empty()) return;
                const auto& pt = msg->points.front();
                if (pt.positions.size() < 6) {
                    RCLCPP_WARN(get_logger(),
                        "arm/joint_cmd : 6 positions attendues, %zu reçues",
                        pt.positions.size());
                    return;
                }
                int run_time = static_cast<int>(
                    rclcpp::Duration(pt.time_from_start).seconds() * 1000.0);
                run_time = std::clamp(run_time, 0, 2000);

                std::vector<int> angles(6);
                for (int i = 0; i < 6; ++i)
                    angles[i] = static_cast<int>(pt.positions[i]);
                robot_->set_uart_servo_angle_array(angles, run_time);
            });

        // Direction Ackermann [−45..45] degrés
        sub_akm_ = create_subscription<std_msgs::msg::Int32>(
            "mecarosmaster/akm/steering", 10,
            [this](std_msgs::msg::Int32::ConstSharedPtr msg) {
                robot_->set_akm_steering_angle(msg->data, true);
            });

        // Activation/désactivation du bras
        sub_arm_enable_ = create_subscription<std_msgs::msg::Bool>(
            "mecarosmaster/arm/enable", 10,
            [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
                robot_->set_uart_servo_ctrl_enable(msg->data);
                params_.arm_enabled = msg->data;
                RCLCPP_INFO(get_logger(),
                    "Bras %s", msg->data ? "ACTIVÉ" : "DÉSACTIVÉ");
            });

        // ── Services ──────────────────────────────────────────────────────────
        auto make_trigger = [&](const std::string& name, auto cb) {
            return create_service<std_srvs::srv::Trigger>(name, cb);
        };

        srv_reset_flash_ = make_trigger("mecarosmaster/reset_flash",
            [this](std_srvs::srv::Trigger::Request::ConstSharedPtr,
                   std_srvs::srv::Trigger::Response::SharedPtr res) {
                robot_->reset_flash_value();
                res->success = true; res->message = "Flash reset envoyé";
            });

        srv_reset_car_ = make_trigger("mecarosmaster/reset_car",
            [this](std_srvs::srv::Trigger::Request::ConstSharedPtr,
                   std_srvs::srv::Trigger::Response::SharedPtr res) {
                robot_->reset_car_state();
                res->success = true; res->message = "État robot réinitialisé";
            });

        srv_beep_ = make_trigger("mecarosmaster/beep",
            [this](std_srvs::srv::Trigger::Request::ConstSharedPtr,
                   std_srvs::srv::Trigger::Response::SharedPtr res) {
                robot_->set_beep(200);
                res->success = true; res->message = "Bip 200 ms";
            });

        srv_stop_ = make_trigger("mecarosmaster/stop",
            [this](std_srvs::srv::Trigger::Request::ConstSharedPtr,
                   std_srvs::srv::Trigger::Response::SharedPtr res) {
                stopMotors();
                res->success = true; res->message = "Moteurs arrêtés";
            });

        srv_clear_odom_ = make_trigger("mecarosmaster/clear_odom",
            [this](std_srvs::srv::Trigger::Request::ConstSharedPtr,
                   std_srvs::srv::Trigger::Response::SharedPtr res) {
                std::lock_guard<std::mutex> lk(pose_mutex_);
                pose_ = OdomPose{};
                res->success = true; res->message = "Odométrie remise à zéro";
            });

        // ── Callback paramètres dynamiques ────────────────────────────────────
        param_cb_handle_ = add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& p) {
                return onSetParameters(p);
            });

        // ── Timer principal ───────────────────────────────────────────────────
        last_publish_time_ = now();
        const auto period  = std::chrono::duration<double>(
            1.0 / std::max(params_.publish_rate, 1.0));
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            [this]() { timerCallback(); });

        RCLCPP_INFO(get_logger(),
            "mecarosmaster_node prêt\n"
            "  port=%s  car_type=%d  taux=%.0f Hz\n"
            "  publish_tf=%s  cmd_vel_timeout=%.2f s",
            params_.serial_port.c_str(), params_.car_type,
            params_.publish_rate,
            params_.publish_tf ? "oui" : "non",
            params_.cmd_vel_timeout);
    }

private:
    // ── Timer ─────────────────────────────────────────────────────────────────
    void timerCallback()
    {
        const auto stamp = now();

        double dt = (stamp - last_publish_time_).seconds();
        last_publish_time_ = stamp;
        if (dt <= 0.0 || dt > 1.0) dt = 1.0 / params_.publish_rate;

        // Watchdog cmd_vel
        checkCmdVelTimeout(stamp);

        // Vitesses depuis le robot (rapport automatique)
        double vx = 0, vy = 0, vz = 0;
        robot_->get_motion_data(vx, vy, vz);

        // Intégration de pose
        {
            std::lock_guard<std::mutex> lk(pose_mutex_);
            pose_.integrate(vx, vy, vz, dt);
        }

        // Publications
        publishImu(stamp);
        publishRpy(stamp);
        publishMag(stamp);
        publishOdom(stamp, vx, vy, vz);
        publishBattery(stamp);
        publishEncoders(stamp);
        publishJointStates(stamp, dt);
        publishVelocity(stamp, vx, vy, vz);

        if (params_.publish_tf && tf_broadcaster_) {
            broadcastOdomTf(stamp);
        }
    }

    // ── Watchdog ──────────────────────────────────────────────────────────────
    void checkCmdVelTimeout(const rclcpp::Time& now_t)
    {
        if (params_.cmd_vel_timeout <= 0.0) return;
        if (motor_stopped_.load(std::memory_order_relaxed)) return;

        const int64_t last_ns =
            last_cmd_vel_time_.load(std::memory_order_relaxed);
        const double elapsed =
            (now_t.nanoseconds() - last_ns) * 1e-9;

        if (elapsed > params_.cmd_vel_timeout) {
            stopMotors();
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "cmd_vel timeout (%.2f s) — moteurs arrêtés", elapsed);
        }
    }

    void stopMotors()
    {
        robot_->set_car_motion(0.0, 0.0, 0.0);
        motor_stopped_.store(true, std::memory_order_relaxed);
    }

    // ── sensor_msgs/Imu ───────────────────────────────────────────────────────
    void publishImu(const rclcpp::Time& stamp)
    {
        double ax, ay, az, gx, gy, gz, roll, pitch, yaw;
        robot_->get_accelerometer_data(ax, ay, az);
        robot_->get_gyroscope_data(gx, gy, gz);
        robot_->get_imu_attitude_data(roll, pitch, yaw, false);

        sensor_msgs::msg::Imu msg;
        msg.header.stamp    = stamp;
        msg.header.frame_id = params_.imu_frame;

        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        msg.orientation.x = q.x();
        msg.orientation.y = q.y();
        msg.orientation.z = q.z();
        msg.orientation.w = q.w();
        msg.orientation_covariance.fill(0.0);
        msg.orientation_covariance[0] = 1.2e-3;
        msg.orientation_covariance[4] = 1.2e-3;
        msg.orientation_covariance[8] = 1.2e-3;

        msg.linear_acceleration.x = ax;
        msg.linear_acceleration.y = ay;
        msg.linear_acceleration.z = az;
        msg.linear_acceleration_covariance.fill(0.0);
        msg.linear_acceleration_covariance[0] =
        msg.linear_acceleration_covariance[4] =
        msg.linear_acceleration_covariance[8] = params_.accel_cov;

        msg.angular_velocity.x = gx;
        msg.angular_velocity.y = gy;
        msg.angular_velocity.z = gz;
        msg.angular_velocity_covariance.fill(0.0);
        msg.angular_velocity_covariance[0] =
        msg.angular_velocity_covariance[4] =
        msg.angular_velocity_covariance[8] = params_.gyro_cov;

        pub_imu_->publish(msg);
    }

    // ── RPY degrés ────────────────────────────────────────────────────────────
    void publishRpy(const rclcpp::Time& stamp)
    {
        double roll, pitch, yaw;
        robot_->get_imu_attitude_data(roll, pitch, yaw, true);

        geometry_msgs::msg::Vector3Stamped msg;
        msg.header.stamp    = stamp;
        msg.header.frame_id = params_.imu_frame;
        msg.vector.x = roll;
        msg.vector.y = pitch;
        msg.vector.z = yaw;
        pub_rpy_->publish(msg);
    }

    // ── sensor_msgs/MagneticField ─────────────────────────────────────────────
    void publishMag(const rclcpp::Time& stamp)
    {
        double mx, my, mz;
        robot_->get_magnetometer_data(mx, my, mz);

        sensor_msgs::msg::MagneticField msg;
        msg.header.stamp    = stamp;
        msg.header.frame_id = params_.imu_frame;
        msg.magnetic_field.x = mx;
        msg.magnetic_field.y = my;
        msg.magnetic_field.z = mz;
        msg.magnetic_field_covariance.fill(0.0);
        msg.magnetic_field_covariance[0] =
        msg.magnetic_field_covariance[4] =
        msg.magnetic_field_covariance[8] = params_.mag_cov;
        pub_mag_->publish(msg);
    }

    // ── nav_msgs/Odometry ─────────────────────────────────────────────────────
    void publishOdom(const rclcpp::Time& stamp,
                     double vx, double vy, double vz)
    {
        OdomPose p;
        {
            std::lock_guard<std::mutex> lk(pose_mutex_);
            p = pose_;
        }
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, p.theta);

        nav_msgs::msg::Odometry msg;
        msg.header.stamp    = stamp;
        msg.header.frame_id = params_.odom_frame;
        msg.child_frame_id  = params_.base_frame;

        msg.pose.pose.position.x    = p.x;
        msg.pose.pose.position.y    = p.y;
        msg.pose.pose.position.z    = 0.0;
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();
        msg.pose.pose.orientation.w = q.w();
        msg.pose.covariance.fill(0.0);
        msg.pose.covariance[0]  = 1e-2;
        msg.pose.covariance[7]  = 1e-2;
        msg.pose.covariance[35] = 1e-2;

        msg.twist.twist.linear.x  = vx;
        msg.twist.twist.linear.y  = vy;
        msg.twist.twist.angular.z = vz;
        msg.twist.covariance.fill(0.0);
        msg.twist.covariance[0]  = 1e-3;
        msg.twist.covariance[7]  = 1e-3;
        msg.twist.covariance[35] = 1e-3;

        pub_odom_->publish(msg);
    }

    // ── TF2 odom → base_link ──────────────────────────────────────────────────
    void broadcastOdomTf(const rclcpp::Time& stamp)
    {
        OdomPose p;
        {
            std::lock_guard<std::mutex> lk(pose_mutex_);
            p = pose_;
        }
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, p.theta);

        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp    = stamp;
        tf.header.frame_id = params_.odom_frame;
        tf.child_frame_id  = params_.base_frame;
        tf.transform.translation.x = p.x;
        tf.transform.translation.y = p.y;
        tf.transform.translation.z = 0.0;
        tf.transform.rotation.x    = q.x();
        tf.transform.rotation.y    = q.y();
        tf.transform.rotation.z    = q.z();
        tf.transform.rotation.w    = q.w();

        tf_broadcaster_->sendTransform(tf);
    }

    // ── sensor_msgs/BatteryState ──────────────────────────────────────────────
    void publishBattery(const rclcpp::Time& stamp)
    {
        constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

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

    // ── std_msgs/Int32MultiArray — compteurs encodeurs bruts ──────────────────
    void publishEncoders(const rclcpp::Time& /*stamp*/)
    {
        int m1, m2, m3, m4;
        robot_->get_motor_encoder(m1, m2, m3, m4);

        std_msgs::msg::Int32MultiArray msg;
        msg.data = {m1, m2, m3, m4};
        pub_enc_->publish(msg);
    }

    // ── sensor_msgs/JointState — positions et vitesses roues ─────────────────
    // Publie sur "/joint_states" — topic standard consommé par robot_state_publisher
    void publishJointStates(const rclcpp::Time& stamp, double dt)
    {
        int enc[4];
        robot_->get_motor_encoder(enc[0], enc[1], enc[2], enc[3]);

        const double rad_per_tick =
            (2.0 * M_PI) / std::max(params_.ticks_per_rev, 1.0);

        sensor_msgs::msg::JointState msg;
        msg.header.stamp = stamp;
        msg.name     = {"wheel_fl_joint", "wheel_fr_joint",
                        "wheel_rl_joint", "wheel_rr_joint"};
        msg.position.resize(4);
        msg.velocity.resize(4);

        for (int i = 0; i < 4; ++i) {
            const double drad =
                static_cast<double>(enc[i] - prev_enc_[i]) * rad_per_tick;
            joint_pos_[i] += drad;
            msg.position[i] = joint_pos_[i];
            msg.velocity[i] = (dt > 0.0) ? drad / dt : 0.0;
            prev_enc_[i] = enc[i];
        }
        msg.effort = {};
        pub_joint_->publish(msg);
    }

    // ── geometry_msgs/TwistStamped ────────────────────────────────────────────
    void publishVelocity(const rclcpp::Time& stamp,
                         double vx, double vy, double vz)
    {
        geometry_msgs::msg::TwistStamped msg;
        msg.header.stamp    = stamp;
        msg.header.frame_id = params_.base_frame;
        msg.twist.linear.x  = vx;
        msg.twist.linear.y  = vy;
        msg.twist.angular.z = vz;
        pub_vel_->publish(msg);
    }

    // ── Callback paramètres dynamiques ────────────────────────────────────────
    rcl_interfaces::msg::SetParametersResult
    onSetParameters(const std::vector<rclcpp::Parameter>& params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto& p : params) {
            const auto& name = p.get_name();
            if      (name == "publish_tf")      params_.publish_tf      = p.as_bool();
            else if (name == "cmd_vel_timeout") params_.cmd_vel_timeout = p.as_double();
            else if (name == "gyro_cov")        params_.gyro_cov        = p.as_double();
            else if (name == "accel_cov")       params_.accel_cov       = p.as_double();
            else if (name == "mag_cov")         params_.mag_cov         = p.as_double();
            else if (name == "arm_enabled") {
                params_.arm_enabled = p.as_bool();
                robot_->set_uart_servo_ctrl_enable(params_.arm_enabled);
            }
        }
        return result;
    }

    // ── Membres ───────────────────────────────────────────────────────────────
    NodeParams                     params_;
    std::unique_ptr<Mecarosmaster> robot_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    OdomPose         pose_;
    std::mutex       pose_mutex_;
    rclcpp::Time     last_publish_time_;

    // Watchdog thread-safe (évite mutex dans les callbacks subscriber)
    std::atomic<int64_t> last_cmd_vel_time_{0};
    std::atomic<bool>    motor_stopped_{true};

    // Encodeurs précédents + position angulaire intégrée des joints
    std::array<int, 4>    prev_enc_  {0, 0, 0, 0};
    std::array<double, 4> joint_pos_ {0.0, 0.0, 0.0, 0.0};

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr              pub_imu_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_rpy_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr    pub_mag_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr            pub_odom_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr     pub_battery_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr     pub_enc_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr       pub_joint_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr   pub_vel_;

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr             sub_cmd_vel_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr      sub_motors_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr      sub_pwm_servos_;
    rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr              sub_leds_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr sub_arm_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr                  sub_akm_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                   sub_arm_enable_;

    // Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_flash_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_car_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_beep_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_stop_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_clear_odom_;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// ─────────────────────────────────────────────────────────────────────────────
//  main
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<MecarosmasterNode>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("main"),
            "Exception non gérée : %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
