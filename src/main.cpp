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
    // Load data
    std::string data_path = "/Users/yeonge/workspace/share/humble/ekf-ws/src/ekf_localizer/source";
    std::vector<SensorData> sensor_data = loadAllSensorData(data_path);

    if (sensor_data.empty())
    {
        std::cerr << "Error: No sensor data loaded!" << std::endl;
        return -1;
    }

    std::cout << "Successfully loaded " << sensor_data.size() << " sensor data points" << std::endl;
    std::cout << "Time range: " << sensor_data.front().timestamp << " to " << sensor_data.back().timestamp << " seconds" << std::endl;

    // Print the content of the first data point for verification
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

    // Set the initial state of the EKF from the first synchronized sensor data
    State initial_state = initializeFromSensorData(first_data);
    sensor_data.erase(sensor_data.begin());

    // --- Set EKF Covariance Matrices ---
    // Initial state covariance matrix P: Represents the uncertainty of the initial state estimate.
    // A large value means high initial uncertainty for that state.
    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
    P.diagonal() << 100.0, 100.0, 100.0, // High initial uncertainty for position (x, y, z)
        50.0, 50.0, 50.0,                // Medium uncertainty for velocity (vx, vy, vz)
        10.0, 10.0, 10.0,                // Low uncertainty for attitude (roll, pitch, yaw)
        10.0, 10.0, 10.0;                // Low uncertainty for angular velocity (wx, wy, wz)

    // Process noise covariance matrix Q: Represents the uncertainty of the prediction model.
    // It's added to the state covariance in the prediction step and models how well the model reflects reality.
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
    Q.diagonal() << 10.0, 10.0, 1.0, // Process noise related to position
        1.0, 1.0, 0.1,               // Process noise related to velocity
        0.1, 0.1, 0.01,              // Process noise related to attitude
        5.0, 5.0, 0.01;              // Process noise related to angular velocity

    // Measurement noise covariance matrix R: Represents the uncertainty of sensor measurements.
    // A small value means we trust the sensor measurement more.
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(MEASUREMENT_SIZE, MEASUREMENT_SIZE);
    R.diagonal() << 0.01, 0.01, 0.001, // Noise for velocity measurement (x, y, z) (odometry is relatively accurate)
        1.0, 1.0, 1.0;                 // Noise for angular velocity measurement (wx, wy, wz)

    // Create EKF object and initialize with the matrices set above
    EKF ekf;
    ekf.initialize(P, Q, R);

    // --- Print Covariance Information ---
    std::cout << "\n--- EKF Covariance Matrices ---" << std::endl;
    std::cout << "Initial State Covariance P diagonal:" << std::endl;
    std::cout << P.diagonal().transpose() << std::endl;
    std::cout << "\nProcess Noise Covariance Q diagonal:" << std::endl;
    std::cout << Q.diagonal().transpose() << std::endl;
    std::cout << "\nMeasurement Noise Covariance R diagonal:" << std::endl;
    std::cout << R.diagonal().transpose() << std::endl;

    // --- Print Initial State ---
    std::cout << "\n--- Initial State (from sensor data) ---" << std::endl;
    std::cout << "Position: " << initial_state.position.transpose() << std::endl;
    std::cout << "Velocity: " << initial_state.velocity.transpose() << std::endl;
    std::cout << "Euler Angles (r,p,y): " << initial_state.euler_angles.transpose() << std::endl;
    std::cout << "Angular Velocity: " << initial_state.angular_velocity.transpose() << std::endl;

    // --- Initialize CSV file ---
    std::string output_dir = "/Users/yeonge/workspace/share/humble/ekf-ws/src/ekf_localizer/build";
    std::string output_file = output_dir + "/ekf_result.csv";

    // Create build directory if it doesn't exist
    std::filesystem::create_directories(output_dir);

    std::ofstream csv_file(output_file);
    if (!csv_file.is_open())
    {
        std::cerr << "Error: Cannot create output file " << output_file << std::endl;
        return -1;
    }

    // Write CSV header
    csv_file << "timestamp,"
             << "ekf_pos_x,ekf_pos_y,ekf_pos_z,ekf_vel_x,ekf_vel_y,ekf_vel_z,ekf_roll,ekf_pitch,ekf_yaw,ekf_ang_vel_x,ekf_ang_vel_y,ekf_ang_vel_z,"
             << "odom_pos_x,odom_pos_y,odom_pos_z,odom_vel_x,odom_vel_y,odom_vel_z,odom_roll,odom_pitch,odom_yaw,odom_ang_vel_x,odom_ang_vel_y,odom_ang_vel_z,"
             << "imu_acc_x,imu_acc_y,imu_acc_z,imu_ang_vel_x,imu_ang_vel_y,imu_ang_vel_z,imu_roll,imu_pitch,imu_yaw"
             << std::endl;

    // Also save the initial state to the CSV
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

    // --- EKF Main Loop ---
    State current_state = initial_state;
    while (!sensor_data.empty())
    {
        const auto &current_data = sensor_data.front();
        double dt = current_data.timestamp - current_timestamp;

        // Predict step: Predict the next state using IMU data as input.
        auto [predicted_state, P_predicted] = ekf.predict(current_state, current_data.imu_linear_acc,
                                                          current_data.imu_angular_vel, dt);

        // Correction step: Correct the predicted state using Odometry measurements (linear/angular velocity).
        State corrected_state = ekf.correct(predicted_state, current_data.odom_linear_vel,
                                            current_data.odom_angular_vel);

        // Save results to CSV file
        Eigen::Vector3d corrected_euler = corrected_state.euler_angles;
        Eigen::Vector3d current_odom_euler = quaternionToEuler(current_data.odom_orientation);
        Eigen::Vector3d current_imu_euler = quaternionToEuler(current_data.imu_orientation);
        csv_file << std::fixed << std::setprecision(6)
                 << current_data.timestamp << ","
                 // BUG FIX: predicted_state.position.x() -> corrected_state.position.x()
                 // To be consistent, the corrected final state value must be saved, not the predicted value.
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

        // Update state for the next iteration
        current_state = corrected_state;
        current_timestamp = current_data.timestamp;

        sensor_data.erase(sensor_data.begin());

        if (sensor_data.size() % 1000 == 0)
        {
            std::cout << "remaining data points: " << sensor_data.size() << std::endl;
        }
    }

    // Close CSV file
    csv_file.close();
    std::cout << "EKF processing completed. Results saved to: " << output_file << std::endl;

    return 0;
}