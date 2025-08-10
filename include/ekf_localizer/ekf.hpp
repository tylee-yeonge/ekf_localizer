#include <Eigen/Dense>
#include "ekf_localizer/types.hpp"
#include <utility>

/**
 * @class EKF
 * @brief 확장 칼만 필터(EKF)를 구현한 클래스.
 * 로봇의 상태(위치, 속도, 자세)를 추정합니다.
 */
class EKF
{
private:
    // --- 칼만 필터의 핵심 행렬들 ---

    /**
     * @brief 상태 공분산 행렬 (P)
     * 상태 추정치에 대한 불확실성을 나타냅니다.
     * 대각 원소는 각 상태 변수의 분산을, 비대각 원소는 상태 변수 간의 공분산을 의미합니다.
     */
    Eigen::MatrixXd P_;

    /**
     * @brief 프로세스 노이즈 공분산 행렬 (Q)
     * 예측 모델의 불확실성을 나타냅니다.
     * 시스템 모델이 얼마나 정확한지에 대한 정보를 담고 있으며, 가속도나 각속도의 노이즈 등을 모델링합니다.
     */
    Eigen::MatrixXd Q_;

    /**
     * @brief 측정 노이즈 공분산 행렬 (R)
     * 측정값의 불확실성을 나타냅니다.
     * 센서(예: 휠 오도메트리) 측정에 포함된 노이즈의 크기를 모델링합니다.
     */
    Eigen::MatrixXd R_;

    // --- 헬퍼 함수 ---

    /**
     * @brief 각도를 [-PI, PI] 범위로 정규화합니다.
     * @param angle 정규화할 각도 (라디안)
     * @return 정규화된 각도 (라디안)
     */
    double normalizeAngle(double angle);

    /**
     * @brief 두 각도의 차이를 계산하고 정규화합니다.
     * @param angle1 첫 번째 각도 (라디안)
     * @param angle2 두 번째 각도 (라디안)
     * @return 정규화된 각도 차이 (라디안)
     */
    double normalizeAngleDifference(double angle1, double angle2);

    /**
     * @brief 오일러 각 벡터의 각 요소를 정규화합니다.
     * @param euler_angles 정규화할 오일러 각 벡터 (롤, 피치, 요)
     * @return 정규화된 오일러 각 벡터
     */
    Eigen::Vector3d normalizeEulerAngles(const Eigen::Vector3d &euler_angles);

    /**
     * @brief 4차 룽게-쿠타(RK4) 방법을 사용하여 오일러 각속도를 적분하여 다음 시점의 오일러 각을 계산합니다.
     * @param euler_current 현재 오일러 각
     * @param omega 현재 각속도
     * @param dt 시간 간격
     * @return 다음 시점의 오일러 각
     */
    Eigen::Vector3d integrateEulerRatesRK4(const Eigen::Vector3d &euler_current,
                                           const Eigen::Vector3d &omega,
                                           double dt);

public:
    /**
     * @brief EKF 클래스 생성자
     */
    EKF();

    /**
     * @brief EKF 필터를 초기화합니다.
     * @param P_init 초기 상태 공분산 행렬
     * @param Q_process 프로세스 노이즈 공분산 행렬
     * @param R_measurement 측정 노이즈 공분산 행렬
     */
    void initialize(const Eigen::MatrixXd &P_init,
                    const Eigen::MatrixXd &Q_process,
                    const Eigen::MatrixXd &R_measurement);

    /**
     * @brief EKF의 예측 단계를 수행합니다.
     * 현재 상태와 입력(가속도, 각속도)을 기반으로 다음 상태와 공분산을 예측합니다.
     * @param current_state 현재 상태
     * @param acceleration_body 동체 좌표계에서의 가속도
     * @param angular_velocity_body 동체 좌표계에서의 각속도
     * @param dt 시간 간격
     * @return 예측된 상태와 예측된 공분산 행렬의 쌍
     */
    std::pair<State, Eigen::MatrixXd> predict(const State &current_state,
                                              const Eigen::Vector3d &acceleration_body,
                                              const Eigen::Vector3d &angular_velocity_body,
                                              double dt);

    /**
     * @brief EKF의 수정(업데이트) 단계를 수행합니다.
     * 예측된 상태를 측정값(속도, 각속도)을 사용하여 보정합니다.
     * @param predicted_state 예측된 상태
     * @param measured_velocity 측정된 속도
     * @param measured_angular_velocity 측정된 각속도
     * @return 수정된 상태
     */
    State correct(const State &predicted_state,
                  const Eigen::Vector3d &measured_velocity,
                  const Eigen::Vector3d &measured_angular_velocity);
};