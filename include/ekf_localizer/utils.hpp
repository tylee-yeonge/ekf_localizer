#pragma once
#include <Eigen/Dense>

/**
 * @brief Z-Y-X 순서(yaw, pitch, roll)로 회전을 적용하여 회전 행렬을 생성합니다.
 *
 * @param euler 회전 각도 (roll, pitch, yaw)
 * @return Eigen::Matrix3d 3x3 회전 행렬
 */
Eigen::Matrix3d eulerToRotationMatrix(const Eigen::Vector3d &euler);

/**
 * @brief 이 행렬 T는 동체 좌표계의 각속도(omega)를 월드 좌표계의 오일러 각속도(euler_dot)로 변환하는 데 사용됩니다.
 *        (euler_dot = T * omega). 짐벌락(gimbal lock) 상황 (pitch가 +/- 90도)에서는 계산이 불안정해질 수 있습니다.
 *
 * @param euler 동체 좌표계의 각속도 (roll, pitch, yaw)
 * @return Eigen::Matrix3d 3x3 변환 행렬
 */
Eigen::Matrix3d eulerRateTransformMatrix(const Eigen::Vector3d &euler);

/**
 * @brief 쿼터니언으로부터 ZYX 오일러 각을 계산합니다.
 *
 * @param q 변환할 쿼터니언
 * @return Eigen::Vector3d 오일러 각 (roll, pitch, yaw)
 */
Eigen::Vector3d quaternionToEuler(const Eigen::Quaterniond &q);

/**
 * @brief Z-Y-X 오일러 각을 쿼터니언으로 변환합니다.
 *
 * @param euler 변환할 오일러 각 (roll, pitch, yaw)
 * @return Eigen::Quaterniond 쿼터니언
 */
Eigen::Quaterniond eulerToQuaternion(const Eigen::Vector3d &euler);

/**
 * @brief 이 함수는 현재 EKF 구현에서 직접 사용되지는 않지만,
 *        오일러 각을 직접 측정값으로 사용하는 경우에 필요할 수 있습니다.
 *        중앙 차분을 이용한 수치 미분으로 자코비안을 근사합니다.
 *
 * @param euler 자코비안을 계산할 기준 오일러 각
 * @return Eigen::Matrix3d 3x3 자코비안 행렬
 */
Eigen::Matrix3d computeEulerMeasurementJacobian(const Eigen::Vector3d &euler);

/**
 * @brief EKF의 예측 단계에서 상태 전이 행렬(F)을 계산하는 데 사용됩니다.
 *        속도 예측식이 회전 행렬 R에 의존하기 때문에, R의 각 오일러 각에 대한 미분이 필요합니다.
 *
 * @param euler 현재 오일러 각 (roll, pitch, yaw)
 * @param[out] dR_droll 회전 행렬의 roll에 대한 편미분 행렬
 * @param[out] dR_dpitch 회전 행렬의 pitch에 대한 편미분 행렬
 * @param[out] dR_dyaw 회전 행렬의 yaw에 대한 편미분 행렬
 */
void computeRotationMatrixDerivatives(const Eigen::Vector3d &euler,
                                      Eigen::Matrix3d &dR_droll,
                                      Eigen::Matrix3d &dR_dpitch,
                                      Eigen::Matrix3d &dR_dyaw);

/**
 * @brief 이 함수는 현재 EKF 구현에서 직접 사용되지는 않지만,
 *        더 복잡한 상태 전이 모델(예: 각속도도 함께 예측하는 모델)에서 자코비안을 계산할 때 필요할 수 있습니다.
 *
 * @param euler 현재 오일러 각 (roll, pitch, yaw)
 * @param[out] dT_droll 변환 행렬의 roll에 대한 편미분 행렬
 * @param[out] dT_dpitch 변환 행렬의 pitch에 대한 편미분 행렬
 * @param[out] dT_dyaw 변환 행렬의 yaw에 대한 편미분 행렬
 */
void computeTransformMatrixDerivatives(const Eigen::Vector3d &euler,
                                       Eigen::Matrix3d &dT_droll,
                                       Eigen::Matrix3d &dT_dpitch,
                                       Eigen::Matrix3d &dT_dyaw);