#include <iostream>
#include <fstream>
#include <filesystem>
#include <iomanip>

#include "ekf_localizer/types.hpp"
#include "ekf_localizer/utils.hpp"
#include "ekf_localizer/data_loader.hpp"
#include "ekf_localizer/ekf.hpp"

int main()
{
    // 데이터 읽어오기
    std::string data_path = "/Users/yeonge/workspace/share/humble/ekf-ws/src/ekf_localizer/source";
    std::vector<SensorData> sensor_data = loadAllSensorData(data_path);

    if (sensor_data.empty())
    {
        std::cerr << "Error: No sensor data loaded!" << std::endl;
        return -1;
    }

    std::cout << "Successfully loaded " << sensor_data.size() << " sensor data points" << std::endl;
    std::cout << "Time range: " << sensor_data.front().timestamp << " to " << sensor_data.back().timestamp << " seconds" << std::endl;

    // 확인을 위해 첫 번째 데이터 포인트의 내용을 출력
    const auto &first_data = sensor_data[0];
    double current_timestamp = first_data.timestamp;
    std::cout << "\n--- First Data Point ---" << std::endl;
    std::cout << "Timestamp: " << first_data.timestamp << std::endl;
    std::cout << "IMU Acc: " << first_data.imu_linear_acc.transpose() << std::endl;
    std::cout << "IMU Gyro: " << first_data.imu_angular_vel.transpose() << std::endl;
    std::cout << "IMU Orientation (quat): " << first_data.imu_orientation.w() << ", "
              << first_data.imu_orientation.vec().transpose() << std::endl;
    std::cout << "Odom Pos: " << first_data.odom_position.transpose() << std::endl;
    std::cout << "Odom Vel: " << first_data.odom_linear_vel.transpose() << std::endl;

    // EKF의 초기 상태를 첫 번째 동기화된 센서 데이터로부터 설정
    State initial_state = initializeFromSensorData(first_data);
    sensor_data.erase(sensor_data.begin());

    // --- EKF 공분산 행렬들 설정 ---
    // 초기 상태 공분산 행렬 P: 초기 상태 추정치의 불확실성을 나타냅니다.
    // 큰 값은 해당 상태에 대한 초기 불확실성이 높다는 것을 의미합니다.
    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
    P.diagonal() << 100.0, 100.0, 100.0, // 위치(x, y, z)에 대한 높은 초기 불확실성
        50.0, 50.0, 50.0,                // 속도(vx, vy, vz)에 대한 중간 수준의 불확실성
        10.0, 10.0, 10.0,                // 자세(roll, pitch, yaw)에 대한 낮은 불확실성
        10.0, 10.0, 10.0;                // 각속도(wx, wy, wz)에 대한 낮은 불확실성

    // 프로세스 노이즈 공분산 행렬 Q: 예측 모델의 불확실성을 나타냅니다.
    // 예측 단계에서 상태 공분산에 더해지며, 모델이 현실을 얼마나 잘 반영하지 못하는지를 모델링합니다.
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
    Q.diagonal() << 10.0, 10.0, 1.0, // 위치 관련 프로세스 노이즈
        1.0, 1.0, 0.1,               // 속도 관련 프로세스 노이즈
        0.1, 0.1, 0.01,              // 자세 관련 프로세스 노이즈
        5.0, 5.0, 0.01;              // 각속도 관련 프로세스 노이즈

    // 측정 노이즈 공분산 행렬 R: 센서 측정값의 불확실성을 나타냅니다.
    // 작은 값은 센서 측정을 더 신뢰한다는 의미입니다.
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(MEASUREMENT_SIZE, MEASUREMENT_SIZE);
    R.diagonal() << 0.01, 0.01, 0.001, // 속도 측정(x, y, z)에 대한 노이즈 (오도메트리는 비교적 정확)
        1.0, 1.0, 1.0;                 // 각속도 측정(wx, wy, wz)에 대한 노이즈

    // EKF 객체 생성 및 위에서 설정한 행렬들로 초기화
    EKF ekf;
    ekf.initialize(P, Q, R);

    // --- 공분산 정보 출력 ---
    std::cout << "\n--- EKF Covariance Matrices ---" << std::endl;
    std::cout << "Initial State Covariance P diagonal:" << std::endl;
    std::cout << P.diagonal().transpose() << std::endl;
    std::cout << "\nProcess Noise Covariance Q diagonal:" << std::endl;
    std::cout << Q.diagonal().transpose() << std::endl;
    std::cout << "\nMeasurement Noise Covariance R diagonal:" << std::endl;
    std::cout << R.diagonal().transpose() << std::endl;

    // --- 초기 상태 출력 ---
    std::cout << "\n--- Initial State (from sensor data) ---" << std::endl;
    std::cout << "Position: " << initial_state.position.transpose() << std::endl;
    std::cout << "Velocity: " << initial_state.velocity.transpose() << std::endl;
    std::cout << "Euler Angles (r,p,y): " << initial_state.euler_angles.transpose() << std::endl;
    std::cout << "Angular Velocity: " << initial_state.angular_velocity.transpose() << std::endl;

    // --- CSV 파일 초기화 ---
    std::string output_dir = "/Users/yeonge/workspace/share/humble/ekf-ws/src/ekf_localizer/build";
    std::string output_file = output_dir + "/ekf_result.csv";

    // build 디렉토리가 없으면 생성
    std::filesystem::create_directories(output_dir);

    std::ofstream csv_file(output_file);
    if (!csv_file.is_open())
    {
        std::cerr << "Error: Cannot create output file " << output_file << std::endl;
        return -1;
    }

    // CSV 헤더 작성
    csv_file << "timestamp,"
             << "ekf_pos_x,ekf_pos_y,ekf_pos_z,ekf_vel_x,ekf_vel_y,ekf_vel_z,ekf_roll,ekf_pitch,ekf_yaw,ekf_ang_vel_x,ekf_ang_vel_y,ekf_ang_vel_z,"
             << "odom_pos_x,odom_pos_y,odom_pos_z,odom_vel_x,odom_vel_y,odom_vel_z,odom_roll,odom_pitch,odom_yaw,odom_ang_vel_x,odom_ang_vel_y,odom_ang_vel_z,"
             << "imu_acc_x,imu_acc_y,imu_acc_z,imu_ang_vel_x,imu_ang_vel_y,imu_ang_vel_z,imu_roll,imu_pitch,imu_yaw"
             << std::endl;

    // 초기 상태도 CSV에 저장
    Eigen::Vector3d initial_odom_euler = quaternionToEuler(first_data.odom_orientation);
    Eigen::Vector3d initial_imu_euler = quaternionToEuler(first_data.imu_orientation);
    csv_file << std::fixed << std::setprecision(6)
             << current_timestamp << ","
             << initial_state.position.x() << "," << initial_state.position.y() << "," << initial_state.position.z() << ","
             << initial_state.velocity.x() << "," << initial_state.velocity.y() << "," << initial_state.velocity.z() << ","
             << initial_state.euler_angles.x() << "," << initial_state.euler_angles.y() << "," << initial_state.euler_angles.z() << ","
             << initial_state.angular_velocity.x() << "," << initial_state.angular_velocity.y() << "," << initial_state.angular_velocity.z() << ","
             << first_data.odom_position.x() << "," << first_data.odom_position.y() << "," << first_data.odom_position.z() << ","
             << first_data.odom_linear_vel.x() << "," << first_data.odom_linear_vel.y() << "," << first_data.odom_linear_vel.z() << ","
             << initial_odom_euler.x() << "," << initial_odom_euler.y() << "," << initial_odom_euler.z() << ","
             << first_data.odom_angular_vel.x() << "," << first_data.odom_angular_vel.y() << "," << first_data.odom_angular_vel.z() << ","
             << first_data.imu_linear_acc.x() << "," << first_data.imu_linear_acc.y() << "," << first_data.imu_linear_acc.z() << ","
             << first_data.imu_angular_vel.x() << "," << first_data.imu_angular_vel.y() << "," << first_data.imu_angular_vel.z() << ","
             << initial_imu_euler.x() << "," << initial_imu_euler.y() << "," << initial_imu_euler.z()
             << std::endl;

    std::cout << "\nStarting EKF processing and saving results to: " << output_file << std::endl;

    // --- EKF 메인 루프 ---
    State current_state = initial_state;
    while (!sensor_data.empty())
    {
        const auto &current_data = sensor_data.front();
        double dt = current_data.timestamp - current_timestamp;

        // Predict 단계: IMU 데이터를 입력으로 사용하여 다음 상태를 예측합니다.
        auto [predicted_state, P_predicted] = ekf.predict(current_state, current_data.imu_linear_acc,
                                                          current_data.imu_angular_vel, dt);

        // Correction 단계: Odometry 측정값(선형/각속도)을 사용하여 예측된 상태를 보정합니다.
        State corrected_state = ekf.correct(predicted_state, current_data.odom_linear_vel,
                                            current_data.odom_angular_vel);

        // 결과를 CSV 파일에 저장
        Eigen::Vector3d corrected_euler = corrected_state.euler_angles;
        Eigen::Vector3d current_odom_euler = quaternionToEuler(current_data.odom_orientation);
        Eigen::Vector3d current_imu_euler = quaternionToEuler(current_data.imu_orientation);
        csv_file << std::fixed << std::setprecision(6)
                 << current_data.timestamp << ","
                 // BUG FIX: predicted_state.position.x() -> corrected_state.position.x()
                 // 예측값이 아닌 보정된 최종 상태값을 저장해야 일관성이 맞습니다.
                 << corrected_state.position.x() << "," << corrected_state.position.y() << "," << corrected_state.position.z() << ","
                 << corrected_state.velocity.x() << "," << corrected_state.velocity.y() << "," << corrected_state.velocity.z() << ","
                 << corrected_euler.x() << "," << corrected_euler.y() << "," << corrected_euler.z() << ","
                 << corrected_state.angular_velocity.x() << "," << corrected_state.angular_velocity.y() << "," << corrected_state.angular_velocity.z() << ","
                 << current_data.odom_position.x() << "," << current_data.odom_position.y() << "," << current_data.odom_position.z() << ","
                 << current_data.odom_linear_vel.x() << "," << current_data.odom_linear_vel.y() << "," << current_data.odom_linear_vel.z() << ","
                 << current_odom_euler.x() << "," << current_odom_euler.y() << "," << current_odom_euler.z() << ","
                 << current_data.odom_angular_vel.x() << "," << current_data.odom_angular_vel.y() << "," << current_data.odom_angular_vel.z() << ","
                 << current_data.imu_linear_acc.x() << "," << current_data.imu_linear_acc.y() << "," << current_data.imu_linear_acc.z() << ","
                 << current_data.imu_angular_vel.x() << "," << current_data.imu_angular_vel.y() << "," << current_data.imu_angular_vel.z() << ","
                 << current_imu_euler.x() << "," << current_imu_euler.y() << "," << current_imu_euler.z()
                 << std::endl;

        // 다음 iteration을 위해 상태 업데이트
        current_state = corrected_state;
        current_timestamp = current_data.timestamp;

        sensor_data.erase(sensor_data.begin());

        if (sensor_data.size() % 1000 == 0)
        {
            std::cout << "remaining data points: " << sensor_data.size() << std::endl;
        }
    }

    // CSV 파일 닫기
    csv_file.close();
    std::cout << "EKF processing completed. Results saved to: " << output_file << std::endl;

    return 0;
}