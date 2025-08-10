#include <Eigen/Dense>
#include "ekf_localizer/types.hpp"
#include <utility>

/**
 * @class EKF
 * @brief Class implementing the Extended Kalman Filter (EKF).
 * Estimates the state of the robot (position, velocity, attitude).
 */
class EKF
{
private:
    // --- Core matrices of the Kalman Filter ---

    /**
     * @brief State covariance matrix (P)
     * Represents the uncertainty of the state estimate.
     * Diagonal elements are the variances of each state variable, and off-diagonal elements are covariances between them.
     */
    Eigen::MatrixXd P_;

    /**
     * @brief Process noise covariance matrix (Q)
     * Represents the uncertainty of the prediction model.
     * It contains information about how accurate the system model is, modeling noise from sources like acceleration or angular velocity.
     */
    Eigen::MatrixXd Q_;

    /**
     * @brief Measurement noise covariance matrix (R)
     * Represents the uncertainty of the measurement.
     * It models the magnitude of noise in sensor measurements (e.g., wheel odometry).
     */
    Eigen::MatrixXd R_;

    // --- Helper functions ---

    /**
     * @brief Normalizes an angle to the range [-PI, PI].
     * @param angle The angle to normalize (in radians).
     * @return The normalized angle (in radians).
     */
    double normalizeAngle(double angle);

    /**
     * @brief Calculates the difference between two angles and normalizes it.
     * @param angle1 The first angle (in radians).
     * @param angle2 The second angle (in radians).
     * @return The normalized angle difference (in radians).
     */
    double normalizeAngleDifference(double angle1, double angle2);

    /**
     * @brief Normalizes each element of a an Euler angle vector.
     * @param euler_angles The Euler angle vector (roll, pitch, yaw) to normalize.
     * @return The normalized Euler angle vector.
     */
    Eigen::Vector3d normalizeEulerAngles(const Eigen::Vector3d &euler_angles);

    /**
     * @brief Integrates Euler rates using the 4th-order Runge-Kutta (RK4) method to calculate the next Euler angles.
     * @param euler_current The current Euler angles.
     * @param omega The current angular velocity.
     * @param dt The time interval.
     * @return The Euler angles at the next time step.
     */
    Eigen::Vector3d integrateEulerRatesRK4(const Eigen::Vector3d &euler_current,
                                           const Eigen::Vector3d &omega,
                                           double dt);

public:
    /**
     * @brief EKF class constructor.
     */
    EKF();

    /**
     * @brief Initializes the EKF filter.
     * @param P_init Initial state covariance matrix.
     * @param Q_process Process noise covariance matrix.
     * @param R_measurement Measurement noise covariance matrix.
     */
    void initialize(const Eigen::MatrixXd &P_init,
                    const Eigen::MatrixXd &Q_process,
                    const Eigen::MatrixXd &R_measurement);

    /**
     * @brief Performs the prediction step of the EKF.
     * Predicts the next state and covariance based on the current state and inputs (acceleration, angular velocity).
     * @param current_state The current state.
     * @param acceleration_body Acceleration in the body frame.
     * @param angular_velocity_body Angular velocity in the body frame.
     * @param dt The time interval.
     * @return A pair of the predicted state and the predicted covariance matrix.
     */
    std::pair<State, Eigen::MatrixXd> predict(const State &current_state,
                                              const Eigen::Vector3d &acceleration_body,
                                              const Eigen::Vector3d &angular_velocity_body,
                                              double dt);

    /**
     * @brief Performs the correction (update) step of the EKF.
     * Corrects the predicted state using measurements (velocity, angular velocity).
     * @param predicted_state The predicted state.
     * @param measured_velocity The measured velocity.
     * @param measured_angular_velocity The measured angular velocity.
     * @return The corrected state.
     */
    State correct(const State &predicted_state,
                  const Eigen::Vector3d &measured_velocity,
                  const Eigen::Vector3d &measured_angular_velocity);
};