//
// Created by verse on 24-10-16.
//

#ifndef UAV_TARGET_SIM_H
#define UAV_TARGET_SIM_H

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <cmath>
#include <random>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace GeographicLib;

enum class MotionMode {
    LINEAR,         // 有限直线平滑往返
    CIRCLE,         // 匀速圆周运动
    SINUSOIDAL,     // 正弦机动 (周期性规避)
    RANDOM_WALK,    // 随机游走 (无规律逃逸)
};

class UavTargetControl : public rclcpp::Node
{
public:
    UavTargetControl();

    void arm();
    void disarm();

private:
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

    rclcpp::Subscription<VehicleRatesSetpoint>::SharedPtr vehicle_rates_setpoint_subscription_;
    rclcpp::Subscription<SensorGps>::SharedPtr global_position_sub_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_position_sub_;

    std::vector<double> enu_xyz = {0, 0, 0};
    std::vector<double> init_enu_xyz = {0, 0, 0};

    MotionMode motion_mode_ = MotionMode::LINEAR;

    // ---- 有限直线平滑往返参数 ----
    float linear_path_length_ = 20.0f;   // 两端点之间的总长度 (m)
    float linear_half_length_ = 10.0f;   // 路径中心到单侧端点的距离 (m)
    float linear_max_speed_ = 3.0f;      // 最大巡航速度 (m/s)
    float linear_acceleration_ = 2.0f;   // 加速和减速绝对值 (m/s^2)
    float linear_center_x_ = 0.0f;
    float linear_center_y_ = 0.0f;

    float linear_position_ = 0.0f;          // 相对路径中心的 NED X 位置 (m)
    float linear_velocity_ = 0.0f;          // 当前规划速度 (m/s)
    float linear_acceleration_cmd_ = 0.0f;  // 当前规划加速度 (m/s^2)
    float linear_leg_start_ = 0.0f;         // 当前直线段的起点
    float linear_leg_end_ = 10.0f;          // 当前直线段的终点
    float linear_leg_elapsed_ = 0.0f;       // 当前直线段已运行时间 (s)

    // ---- 起飞和就位稳定判定 ----
    float staging_position_tolerance_ = 0.35f; // 三维位置误差阈值 (m)
    float staging_velocity_tolerance_ = 0.25f; // 三维速度阈值 (m/s)
    float staging_stable_duration_ = 1.0f;     // 需要连续稳定的时间 (s)
    int staging_stable_ticks_ = 0;
    bool motion_started_ = false;

    float local_x_ = 0.0f;
    float local_y_ = 0.0f;
    float local_z_ = 0.0f;
    float local_vx_ = 0.0f;
    float local_vy_ = 0.0f;
    float local_vz_ = 0.0f;
    bool local_position_valid_ = false;

    // ---- 圆周运动参数 ----
    float radius_ = 5.0f;
    float angular_speed_ = 0.0025f;
    float theta_ = 0.0f;

    // ---- 通用运动状态 ----
    float pos_x_ = 5.0f;
    float pos_y_ = 0.0f;
    float pos_z_ = -5.0f;
    float vel_x_ = 0.0f;
    float vel_y_ = 0.0f;
    float sim_time_ = 0.0f;
    static constexpr float dt_ = 0.1f;

    // ---- 活动范围约束 ----
    float max_range_ = 10.0f;       // 最大活动半径 (m), 可通过参数调节
    float boundary_center_x_ = 0.0f;
    float boundary_center_y_ = 0.0f;

    // ---- 正弦机动: a_T = A*sin(ωt) ----
    float sin_amplitude_ = 0.5f;   // A = 0.5 m/s²
    float sin_omega_ = 0.5f;       // ω = 0.5 rad/s
    float base_speed_ = 1.0f;      // 前向基础速度 m/s

    // ---- 随机游走: v_T(t+Δt) = v_T(t) + N(0, σ_v) ----
    float sigma_v_ = 0.2f;
    float max_speed_ = 3.0f;
    std::mt19937 rng_{std::random_device{}()};
    std::normal_distribution<float> normal_dist_{0.0f, 1.0f};

    std::atomic<uint64_t> timestamp_;
    uint64_t offboard_setpoint_counter_;

    void publish_offboard_control_mode();
    void publish_trajectory_setpoint(
        float x, float y, float z,
        float vx = 0.0f, float vy = 0.0f, float vz = 0.0f,
        float ax = 0.0f, float ay = 0.0f, float az = 0.0f);
    void execute_motion();
    void move_linear();
    void move_in_circle();
    void move_sinusoidal();
    void move_random_walk();
    void apply_boundary();
    bool update_staging_stability();

    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void global_position_callback(const SensorGps::SharedPtr msg);
    void local_position_callback(const VehicleLocalPosition::SharedPtr msg);
};

#endif //UAV_TARGET_SIM_H
