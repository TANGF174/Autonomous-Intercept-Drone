#include "uav_target_sim.hpp"

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include "px4_msgs/msg/vehicle_rates_setpoint.hpp"

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <algorithm>
#include <random>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;


UavTargetControl::UavTargetControl() : Node("uav_target_sim")
{
    // --- 参数声明与解析 ---
    this->declare_parameter<std::string>("motion_mode", "linear");
    this->declare_parameter<double>("max_range", 10.0);
    this->declare_parameter<double>("linear_path_length", 20.0);
    this->declare_parameter<double>("linear_max_speed", 3.0);
    this->declare_parameter<double>("linear_acceleration", 2.0);
    this->declare_parameter<double>("staging_position_tolerance", 0.35);
    this->declare_parameter<double>("staging_velocity_tolerance", 0.25);
    this->declare_parameter<double>("staging_stable_duration", 1.0);

    std::string mode_str = this->get_parameter("motion_mode").as_string();
    max_range_ = static_cast<float>(this->get_parameter("max_range").as_double());
    linear_path_length_ = std::max(0.2f, static_cast<float>(
        this->get_parameter("linear_path_length").as_double()));
    linear_half_length_ = linear_path_length_ * 0.5f;
    linear_max_speed_ = std::max(0.1f, static_cast<float>(
        this->get_parameter("linear_max_speed").as_double()));
    linear_acceleration_ = std::max(0.1f, static_cast<float>(
        this->get_parameter("linear_acceleration").as_double()));
    staging_position_tolerance_ = std::max(0.05f, static_cast<float>(
        this->get_parameter("staging_position_tolerance").as_double()));
    staging_velocity_tolerance_ = std::max(0.05f, static_cast<float>(
        this->get_parameter("staging_velocity_tolerance").as_double()));
    staging_stable_duration_ = std::max(dt_, static_cast<float>(
        this->get_parameter("staging_stable_duration").as_double()));

    if (mode_str == "linear")           motion_mode_ = MotionMode::LINEAR;
    else if (mode_str == "circle")      motion_mode_ = MotionMode::CIRCLE;
    else if (mode_str == "sinusoidal")  motion_mode_ = MotionMode::SINUSOIDAL;
    else if (mode_str == "random_walk") motion_mode_ = MotionMode::RANDOM_WALK;
    else {
        RCLCPP_WARN(this->get_logger(),
            "Unknown motion_mode '%s', defaulting to 'linear'", mode_str.c_str());
        motion_mode_ = MotionMode::LINEAR;
    }

    RCLCPP_INFO(this->get_logger(), "=== Motion mode: %s | max_range: %.1fm ===",
        mode_str.c_str(), max_range_);
    if (motion_mode_ == MotionMode::LINEAR) {
        RCLCPP_INFO(this->get_logger(),
            "Linear path: length=%.1fm, max_speed=%.1fm/s, acceleration=%.1fm/s^2, fixed_yaw=0",
            linear_path_length_, linear_max_speed_, linear_acceleration_);
    }

    // --- 发布者配置 ---
    offboard_control_mode_publisher_ =
        this->create_publisher<OffboardControlMode>("/px4_2/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ =
        this->create_publisher<TrajectorySetpoint>("/px4_2/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher_ =
        this->create_publisher<VehicleCommand>("/px4_2/fmu/in/vehicle_command", 10);

    // --- 订阅者配置 ---
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    vehicle_rates_setpoint_subscription_ = this->create_subscription<VehicleRatesSetpoint>(
        "/px4_2/fmu/out/vehicle_rates_setpoint", qos,
        [this](const VehicleRatesSetpoint::SharedPtr msg) { (void)msg; });

    global_position_sub_ = this->create_subscription<SensorGps>(
        "/px4_2/fmu/out/vehicle_gps_position", qos,
        std::bind(&UavTargetControl::global_position_callback, this, std::placeholders::_1));

    local_position_sub_ = this->create_subscription<VehicleLocalPosition>(
        "/px4_2/fmu/out/vehicle_local_position", qos,
        std::bind(&UavTargetControl::local_position_callback, this, std::placeholders::_1));

    // --- 状态变量初始化 ---
    offboard_setpoint_counter_ = 0;
    theta_ = 0.0f;
    angular_speed_ = 0.05f;

    pos_x_ = 5.0f;
    pos_y_ = 0.0f;
    pos_z_ = -5.0f;
    vel_x_ = 0.0f;
    vel_y_ = 0.0f;
    sim_time_ = 0.0f;
    linear_position_ = 0.0f;
    linear_velocity_ = 0.0f;
    linear_acceleration_cmd_ = 0.0f;
    linear_leg_start_ = 0.0f;
    linear_leg_end_ = linear_half_length_;
    linear_leg_elapsed_ = 0.0f;
    staging_stable_ticks_ = 0;
    motion_started_ = false;

    if (motion_mode_ == MotionMode::SINUSOIDAL) {
        vel_x_ = base_speed_;
    }

    // --- 定时器 ---
    timer_ = this->create_wall_timer(100ms, [this]() {
        publish_offboard_control_mode();

        if (offboard_setpoint_counter_ < 15) {
            publish_trajectory_setpoint(linear_center_x_, linear_center_y_, pos_z_);
            offboard_setpoint_counter_++;
        }
        else if (offboard_setpoint_counter_ == 15) {
            RCLCPP_INFO(this->get_logger(), "Switching to Offboard and Arming...");
            this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            this->arm();
            offboard_setpoint_counter_++;
        }
        else {
            if (!motion_started_) {
                // 在路径中心完成起飞和就位，速度、加速度及 yaw 指令均为 0。
                publish_trajectory_setpoint(linear_center_x_, linear_center_y_, pos_z_);

                if (update_staging_stability()) {
                    motion_started_ = true;
                    linear_leg_start_ = 0.0f;
                    linear_leg_end_ = linear_half_length_;
                    linear_leg_elapsed_ = 0.0f;
                    RCLCPP_INFO(this->get_logger(),
                        "Target stable; starting motion with fixed yaw=0");
                } else {
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "Waiting for stabilization: pos=(%.2f, %.2f, %.2f), "
                        "vel=(%.2f, %.2f, %.2f)",
                        local_x_, local_y_, local_z_, local_vx_, local_vy_, local_vz_);
                }
            } else {
                execute_motion();
            }
        }
    });
}

void UavTargetControl::arm() {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Arm command sent");
}

void UavTargetControl::publish_offboard_control_mode() {
    OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

void UavTargetControl::publish_trajectory_setpoint(
    float x, float y, float z,
    float vx, float vy, float vz,
    float ax, float ay, float az) {
    TrajectorySetpoint msg{};
    msg.position = {x, y, z};
    msg.velocity = {vx, vy, vz};
    msg.acceleration = {ax, ay, az};
    msg.yaw = 0.0f;
    msg.yawspeed = 0.0f;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

// ============================================================
//  边界约束：超出 max_range_ 时施加回复力
//  目标靠近边界会被平滑拉回，保证不飞出拦截机视野
// ============================================================
void UavTargetControl::apply_boundary() {
    float dx = pos_x_ - boundary_center_x_;
    float dy = pos_y_ - boundary_center_y_;
    float dist = std::sqrt(dx * dx + dy * dy);

    if (dist > max_range_ * 0.8f && dist > 0.01f) {
        // 从80%处开始施加渐进回复力
        float overshoot = (dist - max_range_ * 0.8f) / (max_range_ * 0.2f);
        float restore = 3.0f * overshoot * overshoot;
        float nx = dx / dist;
        float ny = dy / dist;
        vel_x_ -= restore * nx * dt_;
        vel_y_ -= restore * ny * dt_;
    }
}

// ============================================================
//  运动模式调度
// ============================================================
void UavTargetControl::execute_motion() {
    switch (motion_mode_) {
        case MotionMode::LINEAR:       move_linear();       break;
        case MotionMode::CIRCLE:       move_in_circle();    break;
        case MotionMode::SINUSOIDAL:   move_sinusoidal();   break;
        case MotionMode::RANDOM_WALK:  move_random_walk();  break;
    }
}

// ============================================================
//  1. 有限直线平滑往返
//     位置始终沿 PX4 局部 NED X 轴变化，yaw 始终固定为 0。
//     NED X 对应 Gazebo ENU Y，因此目标在拦截机相机中横向运动。
//     距离足够时使用梯形速度曲线；距离较短时自动退化为三角速度曲线。
// ============================================================
void UavTargetControl::move_linear() {
    const float delta = linear_leg_end_ - linear_leg_start_;
    const float direction = delta >= 0.0f ? 1.0f : -1.0f;
    const float distance = std::abs(delta);

    float peak_speed = linear_max_speed_;
    float accel_time = peak_speed / linear_acceleration_;
    float accel_distance =
        0.5f * linear_acceleration_ * accel_time * accel_time;
    float cruise_time = 0.0f;

    // 路径太短、来不及达到最大速度时，使用三角速度曲线。
    if (2.0f * accel_distance > distance) {
        peak_speed = std::sqrt(distance * linear_acceleration_);
        accel_time = peak_speed / linear_acceleration_;
        accel_distance =
            0.5f * linear_acceleration_ * accel_time * accel_time;
    } else {
        const float cruise_distance = distance - 2.0f * accel_distance;
        cruise_time = cruise_distance / peak_speed;
    }

    const float total_time = 2.0f * accel_time + cruise_time;
    linear_leg_elapsed_ += dt_;
    const float t = std::min(linear_leg_elapsed_, total_time);

    float travelled = 0.0f;
    float speed = 0.0f;
    float acceleration = 0.0f;

    if (t < accel_time) {
        // 匀加速：s=1/2*a*t^2，v=a*t。
        travelled = 0.5f * linear_acceleration_ * t * t;
        speed = linear_acceleration_ * t;
        acceleration = linear_acceleration_;
    } else if (t < accel_time + cruise_time) {
        // 匀速：先加上加速段距离，再加 v*t。
        const float cruise_elapsed = t - accel_time;
        travelled = accel_distance + peak_speed * cruise_elapsed;
        speed = peak_speed;
        acceleration = 0.0f;
    } else if (t < total_time) {
        // 匀减速：s=v0*t-1/2*a*t^2，v=v0-a*t。
        const float decel_elapsed = t - accel_time - cruise_time;
        travelled = accel_distance + peak_speed * cruise_time
            + peak_speed * decel_elapsed
            - 0.5f * linear_acceleration_ * decel_elapsed * decel_elapsed;
        speed = peak_speed - linear_acceleration_ * decel_elapsed;
        acceleration = -linear_acceleration_;
    } else {
        // 精确落在端点，并将速度、加速度归零后再开始反向段。
        travelled = distance;
        speed = 0.0f;
        acceleration = 0.0f;
    }

    linear_position_ = linear_leg_start_ + direction * travelled;
    linear_velocity_ = direction * speed;
    linear_acceleration_cmd_ = direction * acceleration;

    pos_x_ = linear_center_x_ + linear_position_;
    pos_y_ = linear_center_y_;

    publish_trajectory_setpoint(
        pos_x_, pos_y_, pos_z_,
        linear_velocity_, 0.0f, 0.0f,
        linear_acceleration_cmd_, 0.0f, 0.0f);

    if (linear_leg_elapsed_ >= total_time) {
        linear_leg_start_ = linear_leg_end_;
        linear_leg_end_ = linear_leg_end_ > 0.0f
            ? -linear_half_length_
            : linear_half_length_;
        linear_leg_elapsed_ = 0.0f;

        RCLCPP_INFO(this->get_logger(),
            "Reached endpoint x=%.2f; next endpoint x=%.2f",
            linear_leg_start_, linear_leg_end_);
    }
}

// ============================================================
//  2. 匀速圆周运动 (原有, 天然有界无需约束)
// ============================================================
void UavTargetControl::move_in_circle() {
    theta_ += angular_speed_;
    if (theta_ > 2 * M_PI) theta_ -= 2 * M_PI;

    float x = radius_ * std::cos(theta_);
    float y = radius_ * std::sin(theta_);

    TrajectorySetpoint msg{};
    msg.position = {x, y, -5.0};
    msg.yaw = theta_;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

// ============================================================
//  3. 正弦机动: a_T = A*sin(ωt), A=0.5m/s², ω=0.5rad/s
//     前向匀速 + 侧向正弦加速度 + 边界约束
// ============================================================
void UavTargetControl::move_sinusoidal() {
    sim_time_ += dt_;

    float a_y = sin_amplitude_ * std::sin(sin_omega_ * sim_time_);
    vel_y_ += a_y * dt_;

    apply_boundary();

    pos_x_ += vel_x_ * dt_;
    pos_y_ += vel_y_ * dt_;

    TrajectorySetpoint msg{};
    msg.position = {pos_x_, pos_y_, pos_z_};
    msg.yaw = std::atan2(vel_y_, vel_x_);
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

// ============================================================
//  4. 随机游走: v_T(t+Δt) = v_T(t) + N(0, σ_v), σ_v=0.2m/s
//     无规律逃逸 + 边界约束
// ============================================================
void UavTargetControl::move_random_walk() {
    vel_x_ += sigma_v_ * normal_dist_(rng_);
    vel_y_ += sigma_v_ * normal_dist_(rng_);

    float speed = std::sqrt(vel_x_ * vel_x_ + vel_y_ * vel_y_);
    if (speed > max_speed_) {
        vel_x_ *= max_speed_ / speed;
        vel_y_ *= max_speed_ / speed;
    }

    apply_boundary();

    pos_x_ += vel_x_ * dt_;
    pos_y_ += vel_y_ * dt_;

    TrajectorySetpoint msg{};
    msg.position = {pos_x_, pos_y_, pos_z_};
    msg.yaw = std::atan2(vel_y_, vel_x_);
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

// ============================================================
//  Vehicle command & GPS callback
// ============================================================
void UavTargetControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
    VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 3;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publisher_->publish(msg);
}

void UavTargetControl::global_position_callback(const px4_msgs::msg::SensorGps::SharedPtr msg)
{
    (void)msg;
    if (!msg) {
        RCLCPP_WARN(this->get_logger(), "Received null GPS message");
        return;
    }
}

void UavTargetControl::local_position_callback(
    const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
    if (!msg) {
        return;
    }

    local_position_valid_ =
        msg->xy_valid && msg->z_valid && msg->v_xy_valid && msg->v_z_valid;
    local_x_ = msg->x;
    local_y_ = msg->y;
    local_z_ = msg->z;
    local_vx_ = msg->vx;
    local_vy_ = msg->vy;
    local_vz_ = msg->vz;
}

bool UavTargetControl::update_staging_stability()
{
    if (!local_position_valid_) {
        staging_stable_ticks_ = 0;
        return false;
    }

    const float dx = local_x_ - linear_center_x_;
    const float dy = local_y_ - linear_center_y_;
    const float dz = local_z_ - pos_z_;
    const float position_error = std::sqrt(dx * dx + dy * dy + dz * dz);
    const float speed = std::sqrt(
        local_vx_ * local_vx_ + local_vy_ * local_vy_ + local_vz_ * local_vz_);

    if (position_error <= staging_position_tolerance_ &&
        speed <= staging_velocity_tolerance_) {
        staging_stable_ticks_++;
    } else {
        staging_stable_ticks_ = 0;
    }

    const int required_ticks = std::max(
        1, static_cast<int>(std::ceil(staging_stable_duration_ / dt_)));
    return staging_stable_ticks_ >= required_ticks;
}
