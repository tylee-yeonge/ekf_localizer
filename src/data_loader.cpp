#include "ekf_localizer/data_loader.hpp"
#include "ekf_localizer/utils.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <set>
#include <algorithm>

/**
 * @brief Reads a CSV file and returns it as a vector of (timestamp, value) pairs.
 *
 * @param filename Path to the CSV file to be read.
 * @return std::vector<std::pair<double, double>> A vector of data pairs (timestamp, value).
 * @details Each line of the file is expected to follow the format "timestamp, unused_timestamp, topic, value".
 *          It automatically skips the header if the first line is a header.
 */
std::vector<std::pair<double, double>> readCSV(const std::string &filename)
{
    std::vector<std::pair<double, double>> data;
    std::ifstream file(filename);
    std::string line;

    if (!file.is_open())
    {
        std::cerr << "Error: Cannot open file " << filename << std::endl;
        return data;
    }

    // Logic to skip the header if it exists
    if (std::getline(file, line))
    {
        // Check if the first token of the first line is a number
        std::istringstream iss(line);
        std::string first_token;
        if (std::getline(iss, first_token, ','))
        {
            try
            {
                std::stod(first_token);
                // If conversion to number is successful, it's a data line, so reset file pointer to the beginning
                file.seekg(0);
            }
            catch (const std::exception &)
            {
                // If not a number, assume it's a header and do nothing (start reading from the next line)
            }
        }
    }

    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        std::string timestamp_str, unused_timestamp, unused_topic, value_str;

        // Parse CSV format: timestamp, unused_timestamp, topic, value
        if (std::getline(iss, timestamp_str, ',') &&
            std::getline(iss, unused_timestamp, ',') &&
            std::getline(iss, unused_topic, ',') &&
            std::getline(iss, value_str))
        {
            try
            {
                // Convert string to double and store
                double timestamp = std::stod(timestamp_str);
                double value = std::stod(value_str);
                data.emplace_back(timestamp, value);
            }
            catch (const std::exception &e)
            {
                std::cerr << "Error parsing line: " << line << " - " << e.what() << std::endl;
            }
        }
    }

    file.close();
    std::cout << "Loaded " << data.size() << " data points from " << filename << std::endl;
    return data;
}

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
std::vector<SensorData> loadAllSensorData(const std::string &data_path)
{
    // Read each sensor data file
    auto imu_acc_x = readCSV(data_path + "/imu_linear_acc_x.csv");
    auto imu_acc_y = readCSV(data_path + "/imu_linear_acc_y.csv");
    auto imu_acc_z = readCSV(data_path + "/imu_linear_acc_z.csv");

    auto imu_vel_x = readCSV(data_path + "/imu_angular_vel_x.csv");
    auto imu_vel_y = readCSV(data_path + "/imu_angular_vel_y.csv");
    auto imu_vel_z = readCSV(data_path + "/imu_angular_vel_z.csv");

    auto imu_ori_x = readCSV(data_path + "/imu_orientation_x.csv");
    auto imu_ori_y = readCSV(data_path + "/imu_orientation_y.csv");
    auto imu_ori_z = readCSV(data_path + "/imu_orientation_z.csv");
    auto imu_ori_w = readCSV(data_path + "/imu_orientation_w.csv");

    auto odom_pos_x = readCSV(data_path + "/odom_position_x.csv");
    auto odom_pos_y = readCSV(data_path + "/odom_position_y.csv");
    auto odom_pos_z = readCSV(data_path + "/odom_position_z.csv");

    auto odom_vel_x = readCSV(data_path + "/odom_linear_vel_x.csv");
    auto odom_vel_y = readCSV(data_path + "/odom_linear_vel_y.csv");
    auto odom_vel_z = readCSV(data_path + "/odom_linear_vel_z.csv");

    auto odom_avel_x = readCSV(data_path + "/odom_angular_vel_x.csv");
    auto odom_avel_y = readCSV(data_path + "/odom_angular_vel_y.csv");
    auto odom_avel_z = readCSV(data_path + "/odom_angular_vel_z.csv");

    auto odom_ori_x = readCSV(data_path + "/odom_orientation_x.csv");
    auto odom_ori_y = readCSV(data_path + "/odom_orientation_y.csv");
    auto odom_ori_z = readCSV(data_path + "/odom_orientation_z.csv");
    auto odom_ori_w = readCSV(data_path + "/odom_orientation_w.csv");

    // Collect and sort all timestamps
    std::set<double> all_timestamps;

    // Only use timestamps from imu_acc_x
    for (const auto &point : imu_acc_x)
    {
        all_timestamps.insert(point.first);
    }

    // Interpolation function (linear interpolation)
    auto interpolate = [](const std::vector<std::pair<double, double>> &data, double timestamp) -> double
    {
        if (data.empty())
            return 0.0;
        if (data.size() == 1)
            return data[0].second;

        // If the timestamp is outside the data range, return the boundary value to prevent extrapolation.
        if (timestamp <= data.front().first)
            return data.front().second;
        if (timestamp >= data.back().first)
            return data.back().second;

        // Efficiently find the timestamp interval using binary search with `std::lower_bound`.
        auto it = std::lower_bound(data.begin(), data.end(), std::make_pair(timestamp, 0.0));
        if (it == data.begin())
            return it->second;

        auto it_prev = it - 1;

        // Linearly interpolate the value between two timestamps.
        // v = v1 + (t - t1) * (v2 - v1) / (t2 - t1)
        double t1 = it_prev->first, v1 = it_prev->second;
        double t2 = it->first, v2 = it->second;
        // Calculate the ratio of how close the timestamp is to t1 and t2
        double ratio = (timestamp - t1) / (t2 - t1);

        return v1 + ratio * (v2 - v1);
    };

    // Create time-synchronized sensor data
    std::vector<SensorData> sensor_data;
    for (double timestamp : all_timestamps)
    {
        SensorData data;
        data.timestamp = timestamp;

        // Interpolate IMU data
        data.imu_linear_acc = Eigen::Vector3d(
            interpolate(imu_acc_x, timestamp),
            interpolate(imu_acc_y, timestamp),
            interpolate(imu_acc_z, timestamp));

        data.imu_angular_vel = Eigen::Vector3d(
            interpolate(imu_vel_x, timestamp),
            interpolate(imu_vel_y, timestamp),
            interpolate(imu_vel_z, timestamp));

        data.imu_orientation = Eigen::Quaterniond(
            interpolate(imu_ori_w, timestamp),
            interpolate(imu_ori_x, timestamp),
            interpolate(imu_ori_y, timestamp),
            interpolate(imu_ori_z, timestamp));

        // Interpolate Odometry data
        data.odom_position = Eigen::Vector3d(
            interpolate(odom_pos_x, timestamp),
            interpolate(odom_pos_y, timestamp),
            interpolate(odom_pos_z, timestamp));

        data.odom_linear_vel = Eigen::Vector3d(
            interpolate(odom_vel_x, timestamp),
            interpolate(odom_vel_y, timestamp),
            interpolate(odom_vel_z, timestamp));

        data.odom_angular_vel = Eigen::Vector3d(
            interpolate(odom_avel_x, timestamp),
            interpolate(odom_avel_y, timestamp),
            interpolate(odom_avel_z, timestamp));

        data.odom_orientation = Eigen::Quaterniond(
            interpolate(odom_ori_w, timestamp),
            interpolate(odom_ori_x, timestamp),
            interpolate(odom_ori_y, timestamp),
            interpolate(odom_ori_z, timestamp));

        sensor_data.push_back(data);
    }

    std::cout << "Created " << sensor_data.size() << " synchronized sensor data points" << std::endl;
    return sensor_data;
}

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
State initializeFromSensorData(const SensorData &initial_sensor_data)
{
    State initial_state;

    // Set initial position from odometry data
    initial_state.position = initial_sensor_data.odom_position;

    // Set initial velocity from odometry data
    initial_state.velocity = initial_sensor_data.odom_linear_vel;

    // Convert IMU quaternion to Euler angles to set initial attitude
    initial_state.euler_angles = quaternionToEuler(initial_sensor_data.imu_orientation);

    // Set initial angular velocity from IMU data
    initial_state.angular_velocity = initial_sensor_data.imu_angular_vel;

    return initial_state;
}