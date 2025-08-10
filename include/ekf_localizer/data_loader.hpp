#pragma once
#include "ekf_localizer/types.hpp"
#include <vector>
#include <string>

/**
 * @brief Reads a CSV file and returns it as a vector of (timestamp, value) pairs.
 *
 * @param filename Path to the CSV file to be read.
 * @return std::vector<std::pair<double, double>> A vector of data pairs (timestamp, value).
 * @details Each line of the file is expected to follow the format "timestamp, unused_timestamp, topic, value".
 *          It automatically skips the header if the first line is a header.
 */
std::vector<std::pair<double, double>> readCSV(const std::string &filename);

/**
 * @brief Loads all sensor data CSV files from the specified path and synchronizes them by timestamp.
 *
 * @param data_path Directory path where the sensor data CSV files are located.
 * @return std.vector<SensorData> A vector of synchronized sensor data.
 * @details
 * 1. Reads all CSV files for each sensor (IMU, Odometry).
 * 2. Collects all timestamps, using the IMU accelerometer (`imu_acc_x`) timestamps as the reference.
 * 3. For each collected timestamp, it estimates the values of all other sensors using linear interpolation.
 * 4. The synchronized data is stored in `SensorData` structs and returned as a vector.
 *    This allows processing sensor data collected at different frequencies on the same time axis.
 */
std::vector<SensorData> loadAllSensorData(const std::string &data_path);

/**
 * @brief Sets the initial state of the EKF from the first sensor data point.
 *
 * @param initial_sensor_data The first element of the synchronized sensor data vector.
 * @return State The initial state for the EKF.
 * @details
 * - Position: Uses the odometry position values. Odometry typically provides position relative to the world frame, making it suitable for initialization.
 * - Velocity: Uses the odometry linear velocity values.
 * - Attitude (euler_angles): Converts the IMU's orientation (quaternion) to Euler angles. The IMU can provide a more accurate initial attitude as it measures orientation relative to gravity.
 * - Angular Velocity: Uses the IMU's angular velocity values.
 */
State initializeFromSensorData(const SensorData &initial_sensor_data);