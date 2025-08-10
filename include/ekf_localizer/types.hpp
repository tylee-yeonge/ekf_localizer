#pragma once
#include <Eigen/Dense>

// Sensor data struct
struct SensorData
{
    double timestamp;
    Eigen::Vector3d imu_linear_acc;      // IMU linear acceleration
    Eigen::Vector3d imu_angular_vel;     // IMU angular velocity
    Eigen::Quaterniond imu_orientation;  // IMU attitude (quaternion)
    Eigen::Vector3d odom_position;       // Odometry position
    Eigen::Vector3d odom_linear_vel;     // Odometry linear velocity
    Eigen::Vector3d odom_angular_vel;    // Odometry angular velocity
    Eigen::Quaterniond odom_orientation; // Odometry attitude (quaternion)
};

// Struct for 3D state variables
struct State
{
    Eigen::Vector3d position;         // Position in world frame (px, py, pz)
    Eigen::Vector3d velocity;         // Velocity in world frame (vx, vy, vz)
    Eigen::Vector3d euler_angles;     // Attitude (roll, pitch, yaw)
    Eigen::Vector3d angular_velocity; // Angular velocity in body frame (wx, wy, wz)
};

// EKF constants definition
constexpr int STATE_SIZE = 12;      // [px, py, pz, vx, vy, vz, roll, pitch, yaw, wx, wy, wz]
constexpr int MEASUREMENT_SIZE = 6; // Only velocity and angular velocity are measured (position and attitude are calculated by integration)