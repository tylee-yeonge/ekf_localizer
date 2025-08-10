#pragma once
#include "ekf_localizer/types.hpp"
#include <vector>
#include <string>

/**
 * @brief CSV 파일을 읽어 (타임스탬프, 값)의 벡터로 반환합니다.
 *
 * @param filename 읽어올 CSV 파일의 경로
 * @return std::vector<std::pair<double, double>> 타임스탬프와 값의 쌍으로 구성된 데이터 벡터
 * @details 파일의 각 줄은 "timestamp, unused_timestamp, topic, value" 형식을 따를 것으로 예상합니다.
 *          첫 줄이 헤더일 경우 자동으로 건너뜁니다.
 */
std::vector<std::pair<double, double>> readCSV(const std::string &filename);

/**
 * @brief 지정된 경로에서 모든 센서 데이터 CSV 파일을 로드하고, 타임스탬프를 기준으로 동기화합니다.
 *
 * @param data_path 센서 데이터 CSV 파일들이 있는 디렉토리 경로
 * @return std::vector<SensorData> 동기화된 센서 데이터의 벡터
 * @details
 * 1. 각 센서(IMU, Odometry)의 모든 CSV 파일을 읽습니다.
 * 2. IMU 가속도계(`imu_acc_x`)의 타임스탬프를 기준으로 모든 타임스탬프를 수집합니다.
 * 3. 수집된 각 타임스탬프에 대해, 다른 모든 센서 값을 선형 보간(linear interpolation)하여 값을 추정합니다.
 * 4. 이렇게 동기화된 데이터들을 `SensorData` 구조체에 담아 벡터로 반환합니다.
 *    이를 통해 서로 다른 주기로 수집된 센서 데이터들을 동일한 시간 축에서 처리할 수 있습니다.
 */
std::vector<SensorData> loadAllSensorData(const std::string &data_path);

/**
 * @brief 첫 번째 센서 데이터로부터 EKF의 초기 상태(State)를 설정합니다.
 *
 * @param initial_sensor_data 동기화된 센서 데이터 벡터의 첫 번째 요소
 * @return State EKF를 위한 초기 상태
 * @details
 * - 위치(position): 오도메트리 위치 값을 사용합니다. 오도메트리는 일반적으로 월드 좌표계 기준의 위치를 제공하므로 초기화에 적합합니다.
 * - 속도(velocity): 오도메트리 선형 속도 값을 사용합니다.
 * - 자세(euler_angles): IMU의 방향(quaternion)을 오일러 각으로 변환하여 사용합니다. IMU는 중력 방향을 기준으로 자세를 추정하므로 더 정확한 초기 자세를 제공할 수 있습니다.
 * - 각속도(angular_velocity): IMU의 각속도 값을 사용합니다.
 */
State initializeFromSensorData(const SensorData &initial_sensor_data);