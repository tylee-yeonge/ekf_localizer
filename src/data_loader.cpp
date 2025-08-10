#include "ekf_localizer/data_loader.hpp"
#include "ekf_localizer/utils.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <set>
#include <algorithm>

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

    // 첫 번째 줄이 헤더인 경우 건너뛰기 위한 로직
    if (std::getline(file, line))
    {
        // 첫 번째 줄의 첫 토큰이 숫자인지 확인
        std::istringstream iss(line);
        std::string first_token;
        if (std::getline(iss, first_token, ','))
        {
            try
            {
                std::stod(first_token);
                // 성공적으로 숫자로 변환되면 데이터 줄이므로, 파일 포인터를 처음으로 되돌림
                file.seekg(0);
            }
            catch (const std::exception &)
            {
                // 숫자가 아니면 헤더로 간주하고 아무것도 하지 않음 (다음 줄부터 읽기 시작)
            }
        }
    }

    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        std::string timestamp_str, unused_timestamp, unused_topic, value_str;

        // CSV 형식 파싱: timestamp, unused_timestamp, topic, value
        if (std::getline(iss, timestamp_str, ',') &&
            std::getline(iss, unused_timestamp, ',') &&
            std::getline(iss, unused_topic, ',') &&
            std::getline(iss, value_str))
        {
            try
            {
                // 문자열을 double 타입으로 변환하여 저장
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

std::vector<SensorData> loadAllSensorData(const std::string &data_path)
{
    // 각 센서 데이터 파일 읽어오기
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

    // 모든 타임스탬프 수집 및 정렬
    std::set<double> all_timestamps;

    // Only use timestamps from imu_acc_x
    for (const auto &point : imu_acc_x)
    {
        all_timestamps.insert(point.first);
    }

    // 보간 함수 (선형 보간)
    auto interpolate = [](const std::vector<std::pair<double, double>> &data, double timestamp) -> double
    {
        if (data.empty())
            return 0.0;
        if (data.size() == 1)
            return data[0].second;

        // 타임스탬프가 데이터 범위를 벗어나면, 경계 값을 반환하여 외삽(extrapolation)을 방지합니다.
        if (timestamp <= data.front().first)
            return data.front().second;
        if (timestamp >= data.back().first)
            return data.back().second;

        // `std::lower_bound`를 사용한 이진 탐색으로 해당 타임스탬프 구간을 효율적으로 찾습니다.
        auto it = std::lower_bound(data.begin(), data.end(), std::make_pair(timestamp, 0.0));
        if (it == data.begin())
            return it->second;

        auto it_prev = it - 1;

        // 두 타임스탬프 사이의 값을 선형 보간합니다.
        // v = v1 + (t - t1) * (v2 - v1) / (t2 - t1)
        double t1 = it_prev->first, v1 = it_prev->second;
        double t2 = it->first, v2 = it->second;
        // timestamp가 t1과 t2 사이에 얼마나 가까운지를 비율로 계산
        double ratio = (timestamp - t1) / (t2 - t1);

        return v1 + ratio * (v2 - v1);
    };

    // 시간 동기화된 센서 데이터 생성
    std::vector<SensorData> sensor_data;
    for (double timestamp : all_timestamps)
    {
        SensorData data;
        data.timestamp = timestamp;

        // IMU 데이터 보간
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

        // Odometry 데이터 보간
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

State initializeFromSensorData(const SensorData &initial_sensor_data)
{
    State initial_state;

    // 오도메트리 데이터로부터 초기 위치 설정
    initial_state.position = initial_sensor_data.odom_position;

    // 오도메트리 데이터로부터 초기 속도 설정
    initial_state.velocity = initial_sensor_data.odom_linear_vel;

    // IMU 쿼터니언을 오일러 각으로 변환하여 초기 자세 설정
    initial_state.euler_angles = quaternionToEuler(initial_sensor_data.imu_orientation);

    // IMU 각속도로 초기 각속도 설정
    initial_state.angular_velocity = initial_sensor_data.imu_angular_vel;

    return initial_state;
}