#pragma once

#include <Eigen/Dense>
#include <functional>
#include <memory>

namespace ros2_estimation {
    
    template<int StateSize, int MeasurementSize>

    class ExtendedKalmanFilter {
        public:
            using StateVector = Eigen::Matrix<double, StateSize, 1>;
            using MeasurementVector = Eigen::Matrix<double, MeasurementSize, 1>;
            using StateMatrix = Eigen::Matrix<double, StateSize, StateSize>;
            using MeasurementMatrix = Eigen::Matrix<double, MeasurementSize, StateSize>;

            using ProcessModel = std::function<StateVector(const StateVector&, double)>;
            using MeasurementModel = std::function<MeasurementVector(const StateVector&)>;
            using ProcessJacobian = std::function<StateMatrix(const StateVector&, double)>;
            using MeasurementJacobian = std::function<MeasurementMatrix(const StateVector&)>;

            ExtendedKalmanFilter(
                const StateVector& initial_state,
                const StateMatrix& initial_convariance,
                const StateMatrix& process_noise_covariance,
                const Eigen::Matrix<double, MeasurementSize, MeasurementSize>& measurement_noise_covariance,
                ProcessModel process_model,
                MeasurementModel measurement_model,
                ProcessJacobian process_jacobian,
                MeasurementJacobian measurement_jacobian
            );

            void predict(double dt);
            void update(const MeasurementVector& measurement);

            StateVector getState() const;
            StateMatrix getCovariance() const;

        private:
            StateVector state_;
            StateMatrix covariance_;
            StateMatrix process_noise_covariance_;
            Eigen::Matrix<double, MeasurementSize, MeasurementSize> measurement_noise_covariance_;

            ProcessModel process_model_;
            MeasurementModel measurement_model_;
            ProcessJacobian process_jacobian_;
            MeasurementJacobian measurement_jacobian_;

    };

    class EKFFactory {
        public:
            template<int StateSize, int MeasurementSize>
            static std::unique_ptr<ExtendedKalmanFilter<StateSize, MeasurementSize>> createEKF(
                const typename ExtendedKalmanFilter<StateSize, MeasurementSize>::StateVector& initial_state,
                const typename ExtendedKalmanFilter<StateSize, MeasurementSize>::StateMatrix& initial_covariance,
                const typename ExtendedKalmanFilter<StateSize, MeasurementSize>::StateMatrix& process_noise_covariance,
                const Eigen::Matrix<double, MeasurementSize, MeasurementSize>& measurement_noise_covariance,
                typename ExtendedKalmanFilter<StateSize, MeasurementSize>::ProcessModel process_model,
                typename ExtendedKalmanFilter<StateSize, MeasurementSize>::MeasurementModel measurement_model,
                typename ExtendedKalmanFilter<StateSize, MeasurementSize>::ProcessJacobian process_jacobian,
                typename ExtendedKalmanFilter<StateSize, MeasurementSize>::MeasurementJacobian measurement_jacobian
            );
    };
} // namespace ros2_estimation