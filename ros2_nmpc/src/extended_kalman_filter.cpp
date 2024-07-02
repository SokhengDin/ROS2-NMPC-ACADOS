#include "ros2_nmpc/extended_kalman_filter.hpp"

namespace ros2_estimation {

    template<int StateSize, int MeasurementSize>
    ExtendedKalmanFilter<StateSize, MeasurementSize>::ExtendedKalmanFilter(
        const StateVector& initial_state,
        const StateMatrix& initial_covariance,
        const StateMatrix& process_noise_covariance,
        const Eigen::Matrix<double, MeasurementSize, MeasurementSize>& measurement_noise_covariance,
        ProcessModel process_model,
        MeasurementModel measurement_model,
        ProcessJacobian process_jacobian,
        MeasurementJacobian measurement_jacobian
    )
        : state_(initial_state),
        covariance_(initial_covariance),
        process_noise_covariance_(process_noise_covariance),
        measurement_noise_covariance_(measurement_noise_covariance),
        process_model_(process_model),
        measurement_model_(measurement_model),
        process_jacobian_(process_jacobian),
        measurement_jacobian_(measurement_jacobian)
    {}

    template<int StateSize, int MeasurementSize>
    void ExtendedKalmanFilter<StateSize, MeasurementSize>::predict(double dt) {
        // Predict state
        state_ = process_model_(state_, dt);

        // Compute Jacobian
        StateMatrix F = process_jacobian_(state_, dt);

        // Predict covariance
        covariance_ = F * covariance_ * F.transpose() + process_noise_covariance_;
    }

    template<int StateSize, int MeasurementSize>
    void ExtendedKalmanFilter<StateSize, MeasurementSize>::update(const MeasurementVector& measurement) {
        // Compute Jacobian
        MeasurementMatrix H = measurement_jacobian_(state_);

        // Compute Kalman gain
        auto S = H * covariance_ * H.transpose() + measurement_noise_covariance_;
        auto K = covariance_ * H.transpose() * S.inverse();

        // Update state
        auto y = measurement - measurement_model_(state_);
        state_ += K * y;

        // Update covariance
        auto I = StateMatrix::Identity();
        covariance_ = (I - K * H) * covariance_;
    }

    template<int StateSize, int MeasurementSize>
    typename ExtendedKalmanFilter<StateSize, MeasurementSize>::StateVector 
    ExtendedKalmanFilter<StateSize, MeasurementSize>::getState() const {
        return state_;
    }

    template<int StateSize, int MeasurementSize>
    typename ExtendedKalmanFilter<StateSize, MeasurementSize>::StateMatrix 
    ExtendedKalmanFilter<StateSize, MeasurementSize>::getCovariance() const {
        return covariance_;
    }

    template<int StateSize, int MeasurementSize>
    std::unique_ptr<ExtendedKalmanFilter<StateSize, MeasurementSize>> EKFFactory::createEKF(
        const typename ExtendedKalmanFilter<StateSize, MeasurementSize>::StateVector& initial_state,
        const typename ExtendedKalmanFilter<StateSize, MeasurementSize>::StateMatrix& initial_covariance,
        const typename ExtendedKalmanFilter<StateSize, MeasurementSize>::StateMatrix& process_noise_covariance,
        const Eigen::Matrix<double, MeasurementSize, MeasurementSize>& measurement_noise_covariance,
        typename ExtendedKalmanFilter<StateSize, MeasurementSize>::ProcessModel process_model,
        typename ExtendedKalmanFilter<StateSize, MeasurementSize>::MeasurementModel measurement_model,
        typename ExtendedKalmanFilter<StateSize, MeasurementSize>::ProcessJacobian process_jacobian,
        typename ExtendedKalmanFilter<StateSize, MeasurementSize>::MeasurementJacobian measurement_jacobian
    ) {
        return std::make_unique<ExtendedKalmanFilter<StateSize, MeasurementSize>>(
            initial_state, initial_covariance, process_noise_covariance, measurement_noise_covariance,
            process_model, measurement_model, process_jacobian, measurement_jacobian
        );
    }

} // namespace ros2_estimation