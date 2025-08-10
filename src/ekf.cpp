#include "ekf_localizer/ekf.hpp"
#include "ekf_localizer/utils.hpp"
#include <cmath>

EKF::EKF()
{
    // EKF 생성자: 상태, 프로세스, 측정 공분산 행렬을 단위 행렬로 초기화합니다.
    // 이 값들은 이후 `initialize` 함수에서 실제 값으로 덮어써집니다.
    P_ = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
    Q_ = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
    R_ = Eigen::MatrixXd::Identity(MEASUREMENT_SIZE, MEASUREMENT_SIZE);
}

void EKF::initialize(const Eigen::MatrixXd &P_init,
                     const Eigen::MatrixXd &Q_process,
                     const Eigen::MatrixXd &R_measurement)
{
    // EKF 초기화: 사용자가 정의한 공분산 행렬들로 필터를 설정합니다.
    P_ = P_init;        // 초기 상태 공분산
    Q_ = Q_process;     // 프로세스 노이즈 공분산
    R_ = R_measurement; // 측정 노이즈 공분산
}

double EKF::normalizeAngle(double angle)
{
    // atan2 함수를 사용하여 각도를 [-PI, PI] 범위로 안정적으로 정규화합니다.
    return atan2(sin(angle), cos(angle));
}

double EKF::normalizeAngleDifference(double angle1, double angle2)
{
    // 두 각도의 차이를 계산하고 [-PI, PI] 범위로 정규화합니다.
    double diff = angle1 - angle2;
    return atan2(sin(diff), cos(diff));
}

Eigen::Vector3d EKF::normalizeEulerAngles(const Eigen::Vector3d &euler_angles)
{
    // 오일러 각 벡터의 각 요소(roll, pitch, yaw)를 정규화합니다.
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
    // 각속도가 매우 작으면 현재 각도 유지
    if (omega.norm() < 1e-6)
    {
        return normalizeEulerAngles(euler_current);
    }

    // 큰 시간 스텝(dt)을 여러 개의 작은 스텝으로 나누어 적분함으로써
    // 수치적 안정성을 높이고 오차를 줄입니다.
    const int num_steps = std::max(1, static_cast<int>(dt / 0.01)); // 최대 0.01초 스텝
    const double small_dt = dt / num_steps;

    Eigen::Vector3d current_euler = normalizeEulerAngles(euler_current);

    for (int step = 0; step < num_steps; step++)
    {
        // 현재 오일러 각으로부터 동체 각속도(omega)를 오일러 각속도(euler_rates)로 변환하는 행렬 T를 계산합니다.
        Eigen::Matrix3d T = eulerRateTransformMatrix(current_euler);

        // 오일러 각속도 계산: euler_rates = T * omega
        Eigen::Vector3d euler_rates = T * omega;

        // 단순 오일러 적분을 사용하여 다음 스텝의 오일러 각을 계산합니다.
        current_euler += euler_rates * small_dt;

        // 각 스텝마다 각도를 정규화하여 누적 오차를 방지합니다.
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

    // --- 1. 상태 예측 (State Prediction) ---
    // 위치 예측: p_k = p_{k-1} + v_{k-1} * dt
    predicted_state.position = current_state.position + current_state.velocity * dt;

    // 가속도를 월드 좌표계로 변환하고 중력을 빼서 순수 운동 가속도를 계산
    Eigen::Matrix3d R = eulerToRotationMatrix(current_state.euler_angles);
    Eigen::Vector3d g(0.0, 0.0, -9.8); // 중력 벡터
    Eigen::Vector3d world_acceleration = R * acceleration_body + g;
    // 속도 예측: v_k = v_{k-1} + a_{world} * dt
    predicted_state.velocity = current_state.velocity + world_acceleration * dt;

    // 오일러 각 예측: 안정적인 적분 방법을 사용
    predicted_state.euler_angles = integrateEulerRatesRK4(current_state.euler_angles,
                                                          angular_velocity_body, dt);
    // 각속도는 IMU 값을 그대로 사용
    predicted_state.angular_velocity = angular_velocity_body;

    // --- 2. 자코비안 행렬 F 계산 (State Transition Jacobian) ---
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);

    // 위치 변화율의 속도에 대한 편미분: ∂p/∂v = I * dt
    F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;

    // 속도 변화율의 오일러 각에 대한 편미분: ∂v/∂euler
    Eigen::Matrix3d dR_droll, dR_dpitch, dR_dyaw;
    computeRotationMatrixDerivatives(current_state.euler_angles, dR_droll, dR_dpitch, dR_dyaw);
    F.block<3, 1>(3, 6) = dR_droll * acceleration_body * dt;  // ∂v/∂roll
    F.block<3, 1>(3, 7) = dR_dpitch * acceleration_body * dt; // ∂v/∂pitch
    F.block<3, 1>(3, 8) = dR_dyaw * acceleration_body * dt;   // ∂v/∂yaw

    // 오일러 각 변화율의 각속도에 대한 편미분: ∂euler/∂omega
    Eigen::Matrix3d T = eulerRateTransformMatrix(current_state.euler_angles);
    F.block<3, 3>(6, 9) = T * dt;

    // --- 3. 공분산 행렬 P 예측 (Covariance Prediction) ---
    // P_k|{k-1} = F * P_{k-1} * F^T + Q
    Eigen::MatrixXd P_predicted = F * P_ * F.transpose() + Q_;
    P_ = P_predicted;

    return std::make_pair(predicted_state, P_predicted);
}

State EKF::correct(const State &predicted_state,
                   const Eigen::Vector3d &measured_velocity_body,
                   const Eigen::Vector3d &measured_angular_velocity)
{
    // --- 1. 예측된 상태를 단일 벡터로 변환 ---
    Eigen::VectorXd x_pred(STATE_SIZE);
    x_pred.segment<3>(0) = predicted_state.position;
    x_pred.segment<3>(3) = predicted_state.velocity;
    x_pred.segment<3>(6) = predicted_state.euler_angles;
    x_pred.segment<3>(9) = predicted_state.angular_velocity;

    // --- 2. 측정값을 월드 좌표계로 변환 ---
    // 오도메트리에서 측정된 속도는 로봇의 동체(body) 좌표계 기준이므로,
    // 상태의 월드 좌표계 속도와 비교하기 위해 월드 좌표계로 변환합니다.
    Eigen::Matrix3d R = eulerToRotationMatrix(predicted_state.euler_angles);
    Eigen::Vector3d measured_velocity_world = R * measured_velocity_body;

    // --- 3. 측정 벡터 z 생성 ---
    Eigen::VectorXd z(MEASUREMENT_SIZE);
    z.segment<3>(0) = measured_velocity_world;   // 측정된 월드 좌표계 속도
    z.segment<3>(3) = measured_angular_velocity; // 측정된 동체 좌표계 각속도

    // --- 4. 예측된 측정값 h(x) 계산 ---
    // h(x)는 예측된 상태(x_pred)를 측정 공간으로 변환하는 함수입니다.
    Eigen::VectorXd h_x(MEASUREMENT_SIZE);
    h_x.segment<3>(0) = predicted_state.velocity;         // 예측된 월드 좌표계 속도
    h_x.segment<3>(3) = predicted_state.angular_velocity; // 예측된 동체 좌표계 각속도

    // --- 5. 측정 자코비안 H 계산 ---
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(MEASUREMENT_SIZE, STATE_SIZE);

    // 속도 측정에 대한 자코비안 (h_v(x) = v_world)
    // 측정 함수가 상태 변수와 선형 관계이므로 자코비안이 간단해집니다.
    H(0, 3) = 1.0; // ∂(측정 vx)/∂(상태 vx) = 1
    H(1, 4) = 1.0; // ∂(측정 vy)/∂(상태 vy) = 1
    H(2, 5) = 1.0; // ∂(측정 vz)/∂(상태 vz) = 1
    // 참고: 실제로는 `measured_velocity_world`가 `R * measured_velocity_body`로 계산되므로,
    // 오일러 각(R)에 대한 미분항이 추가될 수 있으나, 여기서는 상태의 속도와 직접 비교하는 모델을 사용하여 단순화했습니다.

    // 각속도 측정에 대한 자코비안 (h_w(x) = w_body)
    H(3, 9) = 1.0;  // ∂(측정 wx)/∂(상태 wx) = 1
    H(4, 10) = 1.0; // ∂(측정 wy)/∂(상태 wy) = 1
    H(5, 11) = 1.0; // ∂(측정 wz)/∂(상태 wz) = 1

    // --- 6. Innovation(측정 잔차) 계산 ---
    // y = z - h(x_pred) : 실제 측정값과 예측된 측정값의 차이
    Eigen::VectorXd y = z - h_x;

    // --- 7. Innovation 공분산 계산 ---
    // S = H * P_pred * H^T + R
    Eigen::MatrixXd S = H * P_ * H.transpose() + R_;

    // --- 8. 칼만 이득(Kalman Gain) 계산 ---
    // K = P_pred * H^T * S^{-1}
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    // --- 9. 상태 보정 (State Update) ---
    // x_corr = x_pred + K * y
    Eigen::VectorXd x_corrected = x_pred + K * y;

    // --- 10. 공분산 보정 (Covariance Update) ---
    // P_corr = (I - K * H) * P_pred * (I - K * H)^T + K * R * K^T (Joseph form)
    // Joseph form은 수치적으로 더 안정적인 공분산 업데이트 방식입니다.
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
    P_ = (I - K * H) * P_ * (I - K * H).transpose() + K * R_ * K.transpose();

    // --- 11. 보정된 상태를 State 구조체로 변환 ---
    State corrected_state;
    corrected_state.position = x_corrected.segment<3>(0);
    corrected_state.velocity = x_corrected.segment<3>(3);
    // 각도는 측정 모델에 포함되지 않았으므로 예측값을 그대로 사용합니다.
    // 이는 현재 측정 모델(속도, 각속도)이 자세를 직접적으로 보정하지 않기 때문입니다.
    corrected_state.euler_angles = predicted_state.euler_angles;
    corrected_state.angular_velocity = x_corrected.segment<3>(9);

    return corrected_state;
}