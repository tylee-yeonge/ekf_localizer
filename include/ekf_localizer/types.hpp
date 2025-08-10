#pragma once
#include <Eigen/Dense>

// 센서 데이터 구조체
struct SensorData
{
    double timestamp;
    Eigen::Vector3d imu_linear_acc;      // IMU 선형 가속도
    Eigen::Vector3d imu_angular_vel;     // IMU 각속도
    Eigen::Quaterniond imu_orientation;  // IMU 자세 (쿼터니언)
    Eigen::Vector3d odom_position;       // Odometry 위치
    Eigen::Vector3d odom_linear_vel;     // Odometry 선형 속도
    Eigen::Vector3d odom_angular_vel;    // Odometry 각속도
    Eigen::Quaterniond odom_orientation; // Odometry 자세 (쿼터니언)
};

// 3D 상태 변수를 담는 구조체
struct State
{
    Eigen::Vector3d position;         // 월드 좌표계 기준 위치 (px, py, pz)
    Eigen::Vector3d velocity;         // 월드 좌표계 기준 속도 (vx, vy, vz)
    Eigen::Vector3d euler_angles;     // 자세 (roll, pitch, yaw)
    Eigen::Vector3d angular_velocity; // 본체 좌표계 기준 각속도 (wx, wy, wz)
};

// EKF 상수 정의
constexpr int STATE_SIZE = 12;      // [px, py, pz, vx, vy, vz, roll, pitch, yaw, wx, wy, wz]
constexpr int MEASUREMENT_SIZE = 6; // 속도 + 각속도만 측정 (위치와 자세는 적분으로 계산)