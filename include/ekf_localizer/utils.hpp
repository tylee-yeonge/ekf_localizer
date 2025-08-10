#pragma once
#include <Eigen/Dense>

/**
 * @brief Creates a rotation matrix by applying rotations in Z-Y-X order (yaw, pitch, roll).
 *
 * @param euler Rotation angles (roll, pitch, yaw).
 * @return Eigen::Matrix3d 3x3 rotation matrix.
 */
Eigen::Matrix3d eulerToRotationMatrix(const Eigen::Vector3d &euler);

/**
 * @brief This matrix T is used to transform body frame angular velocity (omega) to world frame Euler rates (euler_dot).
 *        (euler_dot = T * omega). The calculation can become unstable in gimbal lock situations (pitch is +/- 90 degrees).
 *
 * @param euler Angular velocity in the body frame (roll, pitch, yaw).
 * @return Eigen::Matrix3d 3x3 transformation matrix.
 */
Eigen::Matrix3d eulerRateTransformMatrix(const Eigen::Vector3d &euler);

/**
 * @brief Calculates ZYX Euler angles from a quaternion.
 *
 * @param q The quaternion to convert.
 * @return Eigen::Vector3d Euler angles (roll, pitch, yaw).
 */
Eigen::Vector3d quaternionToEuler(const Eigen::Quaterniond &q);

/**
 * @brief Converts Z-Y-X Euler angles to a quaternion.
 *
 * @param euler The Euler angles to convert (roll, pitch, yaw).
 * @return Eigen::Quaterniond Quaternion.
 */
Eigen::Quaterniond eulerToQuaternion(const Eigen::Vector3d &euler);

/**
 * @brief This function is not directly used in the current EKF implementation,
 *        but might be needed if Euler angles are used as direct measurements.
 *        It approximates the Jacobian using numerical differentiation (central difference).
 *
 * @param euler The reference Euler angles for which to calculate the Jacobian.
 * @return Eigen::Matrix3d 3x3 Jacobian matrix.
 */
Eigen::Matrix3d computeEulerMeasurementJacobian(const Eigen::Vector3d &euler);

/**
 * @brief Used in the EKF prediction step to calculate the state transition matrix (F).
 *        Since the velocity prediction equation depends on the rotation matrix R,
 *        the derivative of R with respect to each Euler angle is required.
 *
 * @param euler The current Euler angles (roll, pitch, yaw).
 * @param[out] dR_droll Partial derivative matrix of the rotation matrix with respect to roll.
 * @param[out] dR_dpitch Partial derivative matrix of the rotation matrix with respect to pitch.
 * @param[out] dR_dyaw Partial derivative matrix of the rotation matrix with respect to yaw.
 */
void computeRotationMatrixDerivatives(const Eigen::Vector3d &euler,
                                      Eigen::Matrix3d &dR_droll,
                                      Eigen::Matrix3d &dR_dpitch,
                                      Eigen::Matrix3d &dR_dyaw);

/**
 * @brief This function is not directly used in the current EKF implementation,
 *        but may be needed to calculate the Jacobian in a more complex state transition model
 *        (e.g., a model that also predicts angular velocity).
 *
 * @param euler The current Euler angles (roll, pitch, yaw).
 * @param[out] dT_droll Partial derivative matrix of the transformation matrix with respect to roll.
 * @param[out] dT_dpitch Partial derivative matrix of the transformation matrix with respect to pitch.
 * @param[out] dT_dyaw Partial derivative matrix of the transformation matrix with respect to yaw.
 */
void computeTransformMatrixDerivatives(const Eigen::Vector3d &euler,
                                       Eigen::Matrix3d &dT_droll,
                                       Eigen::Matrix3d &dT_dpitch,
                                       Eigen::Matrix3d &dT_dyaw);