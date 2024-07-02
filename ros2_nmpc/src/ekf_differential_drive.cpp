#include "ros2_nmpc/ekf_differential_drive.hpp"
#include <cmath>

namespace ros2_estimation {

    DifferentialDriveEKF::DifferentialDriveEKF(const EKFParams& params)
        : last_linear_velocity_(0.0),
        last_angular_velocity_(0.0),
        wheel_base_(params.wheel_base),
        wheel_radius_(params.wheel_radius)
    {
        ekf_ = EKFFactory::createEKF<STATE_SIZE, MEASUREMENT_SIZE>(
            params.initial_state,
            params.initial_covariance,
            params.process_noise_covariance,
            params.measurement_noise_covariance,
            std::bind(&DifferentialDriveEKF::processModel, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&DifferentialDriveEKF::measurementModel, this, std::placeholders::_1),
            std::bind(&DifferentialDriveEKF::processJacobian, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&DifferentialDriveEKF::measurementJacobian, this, std::placeholders::_1)
        );

    }

    void DifferentialDriveEKF::predict(double dt)
    {
        ekf_->predict(dt);
    }

    void DifferentialDriveEKF::updateWithEncoderData(const MeasurementVector& measurement)
    {
        ekf_->update(measurement);
    }

    void DifferentialDriveEKF::updateWithIMUData(double yaw)
    {
        MeasurementVector measurement = ekf_->getState().head<MEASUREMENT_SIZE>();
        measurement(3) = yaw;
        ekf_->update(measurement);
    }

    DifferentialDriveEKF::StateVector DifferentialDriveEKF::getState() const
    {
        return ekf_->getState();
    }

    DifferentialDriveEKF::StateMatrix DifferentialDriveEKF::getCovariance() const
    {
        return ekf_->getCovariance();
    }

    void DifferentialDriveEKF::setLinearVelocity(double linear_velocity)
    {
        last_linear_velocity_ = linear_velocity;
    }

    void DifferentialDriveEKF::setAngularVelocity(double angular_velocity)
    {
        last_angular_velocity_ = angular_velocity;
    }

    DifferentialDriveEKF::StateVector DifferentialDriveEKF::processModel(const StateVector& state, double dt)
    {
        StateVector new_state = state;
        double theta = state(2);
        double v = last_linear_velocity_;
        double omega = last_angular_velocity_;

        new_state(0) += v * std::cos(theta) * dt;
        new_state(1) += v * std::sin(theta) * dt;
        new_state(2) += omega * dt;
        new_state(3) = v;
        new_state(4) = 0;  
        new_state(5) = omega;

        return new_state;
    }

    DifferentialDriveEKF::MeasurementVector DifferentialDriveEKF::measurementModel(const StateVector& state)
    {
        MeasurementVector measurement;
        measurement << state(0), state(1), state(2), state(2);
        return measurement;
    }

    DifferentialDriveEKF::StateMatrix DifferentialDriveEKF::processJacobian(const StateVector& state, double dt)
    {
        StateMatrix F = StateMatrix::Identity();
        double theta = state(2);
        double v = last_linear_velocity_;

        F(0, 2) = -v * std::sin(theta) * dt;
        F(0, 3) = std::cos(theta) * dt;
        F(1, 2) = v * std::cos(theta) * dt;
        F(1, 3) = std::sin(theta) * dt;
        F(2, 5) = dt;

        return F;
    }

    DifferentialDriveEKF::MeasurementMatrix DifferentialDriveEKF::measurementJacobian(const StateVector& state)
    {
        MeasurementMatrix H = MeasurementMatrix::Zero();
        H(0, 0) = 1;  // x
        H(1, 1) = 1;  // y
        H(2, 2) = 1;  // yaw from encoder
        H(3, 2) = 1;  // yaw from IMU
        return H;
    }

} // namespace ros2_estimation