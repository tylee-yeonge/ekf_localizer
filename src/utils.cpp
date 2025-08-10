#include "ekf_localizer/utils.hpp"
#include <cmath>
#include <iostream>

Eigen::Matrix3d eulerToRotationMatrix(const Eigen::Vector3d &euler)
{
    double roll = euler.x();
    double pitch = euler.y();
    double yaw = euler.z();

    // Eigen의 AngleAxis를 사용하여 각 축에 대한 회전 생성
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

    // 쿼터니언으로 결합 후 회전 행렬로 변환
    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    return q.matrix();
}

Eigen::Matrix3d eulerRateTransformMatrix(const Eigen::Vector3d &euler)
{
    double roll = euler.x();
    double pitch = euler.y();

    double sin_r = std::sin(roll);
    double cos_r = std::cos(roll);
    double tan_p = std::tan(pitch);
    double cos_p = std::cos(pitch);

    Eigen::Matrix3d transform_matrix;
    // 짐벌락(Gimbal Lock) 방지: pitch가 90도에 가까워지면 cos(pitch)가 0이 됨
    if (std::abs(cos_p) < 1e-6)
    {
        std::cerr << "Warning: Gimbal lock detected! Transformation matrix may be inaccurate." << std::endl;
        // 불안정한 계산을 피하기 위해 근사치 또는 영행렬 반환 (실제 적용 시 정책 필요)
        return Eigen::Matrix3d::Identity();
    }

    transform_matrix << 1, sin_r * tan_p, cos_r * tan_p,
        0, cos_r, -sin_r,
        0, sin_r / cos_p, cos_r / cos_p;

    return transform_matrix;
}

Eigen::Vector3d quaternionToEuler(const Eigen::Quaterniond &q)
{
    Eigen::Vector3d euler;

    // Roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
    euler.x() = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
    if (std::abs(sinp) >= 1)
        euler.y() = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        euler.y() = std::asin(sinp);

    // Yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    euler.z() = std::atan2(siny_cosp, cosy_cosp);

    return euler;
}

Eigen::Quaterniond eulerToQuaternion(const Eigen::Vector3d &euler)
{
    double roll = euler.x();
    double pitch = euler.y();
    double yaw = euler.z();

    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

    return yawAngle * pitchAngle * rollAngle;
}

Eigen::Matrix3d computeEulerMeasurementJacobian(const Eigen::Vector3d &euler)
{
    // 수치적 미분으로 자코비안 계산
    // h(euler) = quaternion_to_euler(euler_to_quaternion(euler))
    const double eps = 1e-6;
    Eigen::Matrix3d jacobian;

    for (int i = 0; i < 3; i++)
    {
        Eigen::Vector3d euler_plus = euler;
        euler_plus(i) += eps;
        Eigen::Vector3d h_plus = quaternionToEuler(eulerToQuaternion(euler_plus));

        Eigen::Vector3d euler_minus = euler;
        euler_minus(i) -= eps;
        Eigen::Vector3d h_minus = quaternionToEuler(eulerToQuaternion(euler_minus));

        Eigen::Vector3d diff = h_plus - h_minus;

        // 각도 차이 정규화
        for (int j = 0; j < 3; j++)
        {
            while (diff(j) > M_PI)
                diff(j) -= 2 * M_PI;
            while (diff(j) < -M_PI)
                diff(j) += 2 * M_PI;
        }

        jacobian.col(i) = diff / (2 * eps);
    }

    return jacobian;
}

void computeRotationMatrixDerivatives(const Eigen::Vector3d &euler,
                                      Eigen::Matrix3d &dR_droll,
                                      Eigen::Matrix3d &dR_dpitch,
                                      Eigen::Matrix3d &dR_dyaw)
{
    double roll = euler.x();
    double pitch = euler.y();
    double yaw = euler.z();

    double sr = std::sin(roll);
    double cr = std::cos(roll);
    double sp = std::sin(pitch);
    double cp = std::cos(pitch);
    double sy = std::sin(yaw);
    double cy = std::cos(yaw);

    // dR/droll (회전 행렬의 roll에 대한 미분)
    dR_droll << 0, 0, 0,
        sy * sp * cr + cy * sr, cy * cr - sy * sp * sr, -cp * sr,
        sy * sp * sr - cy * cr, cy * sr + sy * sp * cr, -cp * cr;

    // dR/dpitch (회전 행렬의 pitch에 대한 미분)
    dR_dpitch << -sy * sp, 0, -sy * cp,
        cy * sp, 0, cy * cp,
        -cp, 0, sp;

    // dR/dyaw (회전 행렬의 yaw에 대한 미분)
    dR_dyaw << -sy * cp, -cy, sy * sp,
        cy * cp, -sy, -cy * sp,
        0, 0, 0;
}

void computeTransformMatrixDerivatives(const Eigen::Vector3d &euler,
                                       Eigen::Matrix3d &dT_droll,
                                       Eigen::Matrix3d &dT_dpitch,
                                       Eigen::Matrix3d &dT_dyaw)
{
    double roll = euler.x();
    double pitch = euler.y();

    double sr = std::sin(roll);
    double cr = std::cos(roll);
    double sp = std::sin(pitch);
    double cp = std::cos(pitch);
    double tp = std::tan(pitch);

    // 짐벌락 방지
    if (std::abs(cp) < 1e-6)
    {
        std::cerr << "Warning: Gimbal lock in transform matrix derivatives!" << std::endl;
        dT_droll = Eigen::Matrix3d::Zero();
        dT_dpitch = Eigen::Matrix3d::Zero();
        dT_dyaw = Eigen::Matrix3d::Zero();
        return;
    }

    // dT/droll (변환 행렬의 roll에 대한 미분)
    dT_droll << 0, cr * tp, -sr * tp,
        0, -sr, -cr,
        0, cr / cp, -sr / cp;

    // dT/dpitch (변환 행렬의 pitch에 대한 미분)
    double sec_p = 1.0 / cp;       // sec(pitch) = 1/cos(pitch)
    double sec2_p = sec_p * sec_p; // sec^2(pitch)

    dT_dpitch << 0, sr * sec2_p, cr * sec2_p,
        0, 0, 0,
        0, sr * sp * sec2_p, cr * sp * sec2_p;

    // dT/dyaw (변환 행렬의 yaw에 대한 미분)
    // T 행렬은 yaw에 의존하지 않으므로 0
    dT_dyaw = Eigen::Matrix3d::Zero();
}