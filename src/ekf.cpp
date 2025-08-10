#include "ekf_localizer/ekf.hpp"
#include "ekf_localizer/utils.hpp"
#include <cmath>

EKF::EKF()
{
    // EKF constructor: Initializes state, process, and measurement covariance matrices to identity.
    // These values will be overwritten by the actual values in the `initialize` function.
    P_ = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
    Q_ = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
    R_ = Eigen::MatrixXd::Identity(MEASUREMENT_SIZE, MEASUREMENT_SIZE);
}

void EKF::initialize(const Eigen::MatrixXd &P_init,
                     const Eigen::MatrixXd &Q_process,
                     const Eigen::MatrixXd &R_measurement)
{
    // Initialize EKF: Set up the filter with user-defined covariance matrices.
    P_ = P_init;        // Initial state covariance
    Q_ = Q_process;     // Process noise covariance
    R_ = R_measurement; // Measurement noise covariance
}

double EKF::normalizeAngle(double angle)
{
    // Reliably normalize the angle to the range [-PI, PI] using atan2.
    return atan2(sin(angle), cos(angle));
}

double EKF::normalizeAngleDifference(double angle1, double angle2)
{
    // Calculate the difference between two angles and normalize it to the range [-PI, PI].
    double diff = angle1 - angle2;
    return atan2(sin(diff), cos(diff));
}

Eigen::Vector3d EKF::normalizeEulerAngles(const Eigen::Vector3d &euler_angles)
{
    // Normalize each component (roll, pitch, yaw) of the Euler angle vector.
    Eigen::Vector3d normalized_angles;
    for (int i = 0; i < 3; i++)
    {
        normalized_angles(i) = normalizeAngle(euler_angles(i));
    }
    return normalized_angles;
}

Eigen::Vector3d EKF::integrateEulerRatesRK4(const Eigen::Vector3d &euler_current,
                                            const Eigen::Vector3d &omega,
                                            double dt)
{
    // If angular velocity is very small, maintain the current angle.
    if (omega.norm() < 1e-6)
    {
        return normalizeEulerAngles(euler_current);
    }

    // By dividing a large time step (dt) into several smaller steps,
    // we enhance numerical stability and reduce errors.
    const int num_steps = std::max(1, static_cast<int>(dt / 0.01)); // Max 0.01s step
    const double small_dt = dt / num_steps;

    Eigen::Vector3d current_euler = normalizeEulerAngles(euler_current);

    for (int step = 0; step < num_steps; step++)
    {
        // From the current Euler angles, calculate the transformation matrix T that converts
        // body angular velocity (omega) to Euler rates (euler_rates).
        Eigen::Matrix3d T = eulerRateTransformMatrix(current_euler);

        // Calculate Euler rates: euler_rates = T * omega
        Eigen::Vector3d euler_rates = T * omega;

        // Calculate the Euler angles for the next step using simple Euler integration.
        current_euler += euler_rates * small_dt;

        // Normalize angles at each step to prevent cumulative errors.
        current_euler = normalizeEulerAngles(current_euler);
    }

    return current_euler;
}

std::pair<State, Eigen::MatrixXd> EKF::predict(const State &current_state,
                                               const Eigen::Vector3d &acceleration_body,
                                               const Eigen::Vector3d &angular_velocity_body,
                                               double dt)
{
    State predicted_state;

    // --- 1. State Prediction ---
    // Predict position: p_k = p_{k-1} + v_{k-1} * dt
    predicted_state.position = current_state.position + current_state.velocity * dt;

    // Transform acceleration to world frame and subtract gravity to get pure motion acceleration.
    Eigen::Matrix3d R = eulerToRotationMatrix(current_state.euler_angles);
    Eigen::Vector3d g(0.0, 0.0, -9.8); // Gravity vector
    Eigen::Vector3d world_acceleration = R * acceleration_body + g;
    // Predict velocity: v_k = v_{k-1} + a_{world} * dt
    predicted_state.velocity = current_state.velocity + world_acceleration * dt;

    // Predict Euler angles: use a stable integration method
    predicted_state.euler_angles = integrateEulerRatesRK4(current_state.euler_angles,
                                                          angular_velocity_body, dt);
    // Angular velocity is taken directly from IMU values.
    predicted_state.angular_velocity = angular_velocity_body;

    // --- 2. Calculate Jacobian Matrix F (State Transition Jacobian) ---
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);

    // Partial derivative of position change wrt velocity: ∂p/∂v = I * dt
    F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;

    // Partial derivative of velocity change wrt Euler angles: ∂v/∂euler
    Eigen::Matrix3d dR_droll, dR_dpitch, dR_dyaw;
    computeRotationMatrixDerivatives(current_state.euler_angles, dR_droll, dR_dpitch, dR_dyaw);
    F.block<3, 1>(3, 6) = dR_droll * acceleration_body * dt;  // ∂v/∂roll
    F.block<3, 1>(3, 7) = dR_dpitch * acceleration_body * dt; // ∂v/∂pitch
    F.block<3, 1>(3, 8) = dR_dyaw * acceleration_body * dt;   // ∂v/∂yaw

    // Partial derivative of Euler angle change wrt angular velocity: ∂euler/∂omega
    Eigen::Matrix3d T = eulerRateTransformMatrix(current_state.euler_angles);
    F.block<3, 3>(6, 9) = T * dt;

    // --- 3. Predict Covariance Matrix P (Covariance Prediction) ---
    // P_k|{k-1} = F * P_{k-1} * F^T + Q
    Eigen::MatrixXd P_predicted = F * P_ * F.transpose() + Q_;
    P_ = P_predicted;

    return std::make_pair(predicted_state, P_predicted);
}

State EKF::correct(const State &predicted_state,
                   const Eigen::Vector3d &measured_velocity_body,
                   const Eigen::Vector3d &measured_angular_velocity)
{
    // --- 1. Convert predicted state to a single vector ---
    Eigen::VectorXd x_pred(STATE_SIZE);
    x_pred.segment<3>(0) = predicted_state.position;
    x_pred.segment<3>(3) = predicted_state.velocity;
    x_pred.segment<3>(6) = predicted_state.euler_angles;
    x_pred.segment<3>(9) = predicted_state.angular_velocity;

    // --- 2. Transform measurement to world frame ---
    // The velocity measured by odometry is in the robot's body frame.
    // It must be transformed to the world frame to be compared with the state's world frame velocity.
    Eigen::Matrix3d R = eulerToRotationMatrix(predicted_state.euler_angles);
    Eigen::Vector3d measured_velocity_world = R * measured_velocity_body;

    // --- 3. Create measurement vector z ---
    Eigen::VectorXd z(MEASUREMENT_SIZE);
    z.segment<3>(0) = measured_velocity_world;   // Measured world frame velocity
    z.segment<3>(3) = measured_angular_velocity; // Measured body frame angular velocity

    // --- 4. Calculate predicted measurement h(x) ---
    // h(x) is the function that transforms the predicted state (x_pred) into the measurement space.
    Eigen::VectorXd h_x(MEASUREMENT_SIZE);
    h_x.segment<3>(0) = predicted_state.velocity;         // Predicted world frame velocity
    h_x.segment<3>(3) = predicted_state.angular_velocity; // Predicted body frame angular velocity

    // --- 5. Calculate Measurement Jacobian H ---
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(MEASUREMENT_SIZE, STATE_SIZE);

    // Jacobian for velocity measurement (h_v(x) = v_world)
    // The Jacobian is simple because the measurement function has a linear relationship with the state variables.
    H(0, 3) = 1.0; // ∂(measured vx)/∂(state vx) = 1
    H(1, 4) = 1.0; // ∂(measured vy)/∂(state vy) = 1
    H(2, 5) = 1.0; // ∂(measured vz)/∂(state vz) = 1
    // Note: In reality, since `measured_velocity_world` is calculated as `R * measured_velocity_body`,
    // derivative terms for Euler angles (R) could be added, but here we simplify by directly comparing with the state's velocity.

    // Jacobian for angular velocity measurement (h_w(x) = w_body)
    H(3, 9) = 1.0;  // ∂(measured wx)/∂(state wx) = 1
    H(4, 10) = 1.0; // ∂(measured wy)/∂(state wy) = 1
    H(5, 11) = 1.0; // ∂(measured wz)/∂(state wz) = 1

    // --- 6. Calculate Innovation (Measurement Residual) ---
    // y = z - h(x_pred) : Difference between actual measurement and predicted measurement
    Eigen::VectorXd y = z - h_x;

    // --- 7. Calculate Innovation Covariance ---
    // S = H * P_pred * H^T + R
    Eigen::MatrixXd S = H * P_ * H.transpose() + R_;

    // --- 8. Calculate Kalman Gain ---
    // K = P_pred * H^T * S^{-1}
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    // --- 9. State Update ---
    // x_corr = x_pred + K * y
    Eigen::VectorXd x_corrected = x_pred + K * y;

    // --- 10. Covariance Update (Joseph form) ---
    // P_corr = (I - K * H) * P_pred * (I - K * H)^T + K * R * K^T
    // The Joseph form is a more numerically stable way to update the covariance.
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
    P_ = (I - K * H) * P_ * (I - K * H).transpose() + K * R_ * K.transpose();

    // --- 11. Convert corrected state vector back to State struct ---
    State corrected_state;
    corrected_state.position = x_corrected.segment<3>(0);
    corrected_state.velocity = x_corrected.segment<3>(3);
    // Angles were not included in the measurement model, so we use the predicted value.
    // This is because the current measurement model (velocity, angular velocity) does not directly correct attitude.
    corrected_state.euler_angles = predicted_state.euler_angles;
    corrected_state.angular_velocity = x_corrected.segment<3>(9);

    return corrected_state;
}