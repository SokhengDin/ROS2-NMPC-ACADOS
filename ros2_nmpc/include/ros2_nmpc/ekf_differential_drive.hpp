#pragma once

#include "ros2_nmpc/extended_kalman_filter.hpp"
#include <Eigen/Dense>
#include <memory>

namespace ros2_estimation {

class DifferentialDriveEKF {
public:
    static constexpr int STATE_SIZE = 6;  // x, y, yaw, vx, vy, vyaw
    static constexpr int MEASUREMENT_SIZE = 4;  // x_encoder, y_encoder, yaw_encoder, yaw_imu

    using EKF = ExtendedKalmanFilter<STATE_SIZE, MEASUREMENT_SIZE>;
    using StateVector = EKF::StateVector;
    using MeasurementVector = EKF::MeasurementVector;
    using StateMatrix = EKF::StateMatrix;
    using MeasurementMatrix = EKF::MeasurementMatrix;

    struct EKFParams {
        StateVector initial_state;
        StateMatrix initial_covariance;
        StateMatrix process_noise_covariance;
        Eigen::Matrix<double, MEASUREMENT_SIZE, MEASUREMENT_SIZE> measurement_noise_covariance;
        double wheel_base;
        double wheel_radius;
    };

    DifferentialDriveEKF(const EKFParams& params);

    void predict(double dt);
    void updateWithEncoderData(const MeasurementVector& measurement);
    void updateWithIMUData(double yaw);

    StateVector getState() const;
    StateMatrix getCovariance() const;

    void setLinearVelocity(double linear_velocity);
    void setAngularVelocity(double angular_velocity);

private:
    std::unique_ptr<EKF> ekf_;
    double last_linear_velocity_;
    double last_angular_velocity_;
    double wheel_base_;
    double wheel_radius_;

    static StateVector processModel(const StateVector& state, double dt);
    static MeasurementVector measurementModel(const StateVector& state);
    static StateMatrix processJacobian(const StateVector& state, double dt);
    static MeasurementMatrix measurementJacobian(const StateVector& state);
};

} // namespace ros2_estimation