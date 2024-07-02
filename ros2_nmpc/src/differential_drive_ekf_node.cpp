#include "rclcpp/rclcpp.hpp"
#include "ros2_nmpc/extended_kalman_filter.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <Eigen/Dense>
#include <cmath>

class DifferentialDriveEKFFusionNode : public rclcpp::Node {
    public:
        DifferentialDriveEKFFusionNode() : Node("differential_drive_ekf_fusion_node") {
            // Initialize EKF
            initializeEKF();
            
            encoder_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "encoder_odom", 10, std::bind(&DifferentialDriveEKFFusionNode::encoderCallback, this, std::placeholders::_1));
            gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
                "gps", 10, std::bind(&DifferentialDriveEKFFusionNode::gpsCallback, this, std::placeholders::_1));
            imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
                "imu", 10, std::bind(&DifferentialDriveEKFFusionNode::imuCallback, this, std::placeholders::_1));

            // Create publisher
            state_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("estimated_state", 10);

            // Create timer for state publishing
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&DifferentialDriveEKFFusionNode::publishState, this));

            RCLCPP_INFO(this->get_logger(), "Differential Drive EKF Fusion Node has been initialized.");
        }

    private:
        static constexpr int StateSize = 3;  // [x, y, yaw]
        static constexpr int MeasurementSize = 5;  // [x_encoder, y_encoder, x_gps, y_gps, yaw_imu]
        using EKF = ros2_estimation::ExtendedKalmanFilter<StateSize, MeasurementSize>;

        std::unique_ptr<EKF> ekf_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr encoder_sub_;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr state_pub_;
        rclcpp::TimerBase::SharedPtr timer_;

        double last_time_ = 0.0;
        EKF::MeasurementVector last_measurement_ = EKF::MeasurementVector::Zero();
        bool gps_updated_ = false;
        bool imu_updated_ = false;

        void initializeEKF() {
            // Initial state [x, y, yaw]
            EKF::StateVector initial_state = EKF::StateVector::Zero();

            // Initial covariance
            EKF::StateMatrix initial_covariance = EKF::StateMatrix::Identity() * 1.0;

            // Process noise covariance
            EKF::StateMatrix process_noise_covariance = EKF::StateMatrix::Identity() * 0.1;

            // Measurement noise covariance
            Eigen::Matrix<double, MeasurementSize, MeasurementSize> measurement_noise_covariance =
                Eigen::Matrix<double, MeasurementSize, MeasurementSize>::Identity();
            measurement_noise_covariance.diagonal() << 0.1, 0.1, 0.5, 0.5, 0.1;

            // Create EKF
            ekf_ = ros2_estimation::EKFFactory::createEKF<StateSize, MeasurementSize>(
                initial_state, initial_covariance, process_noise_covariance, measurement_noise_covariance,
                std::bind(&DifferentialDriveEKFFusionNode::processModel, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&DifferentialDriveEKFFusionNode::measurementModel, this, std::placeholders::_1),
                std::bind(&DifferentialDriveEKFFusionNode::processJacobian, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&DifferentialDriveEKFFusionNode::measurementJacobian, this, std::placeholders::_1)
            );
        }

        void encoderCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
            double current_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
            double dt = current_time - last_time_;
            if (dt > 0) {
                // Predict step
                ekf_->predict(dt);
                last_time_ = current_time;
            }

            // Update encoder measurements
            last_measurement_[0] = msg->pose.pose.position.x;
            last_measurement_[1] = msg->pose.pose.position.y;

            updateEKF();
        }

        void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
            // Update GPS measurements
            last_measurement_[2] = msg->latitude;
            last_measurement_[3] = msg->longitude;
            gps_updated_ = true;

            updateEKF();
        }

        void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
            // Extract yaw from quaternion
            tf2::Quaternion q(
                msg->orientation.x,
                msg->orientation.y,
                msg->orientation.z,
                msg->orientation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            // Update IMU measurement
            last_measurement_[4] = yaw;
            imu_updated_ = true;

            updateEKF();
        }

        void updateEKF() {
            if (gps_updated_ && imu_updated_) {
                ekf_->update(last_measurement_);
                gps_updated_ = false;
                imu_updated_ = false;
            }
        }

        void publishState() {
            auto state = ekf_->getState();
            auto covariance = ekf_->getCovariance();

            auto msg = std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();
            msg->header.stamp = this->now();
            msg->header.frame_id = "map";
            msg->pose.pose.position.x = state(0);
            msg->pose.pose.position.y = state(1);
            msg->pose.pose.orientation.z = std::sin(state(2) / 2.0);
            msg->pose.pose.orientation.w = std::cos(state(2) / 2.0);

            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    msg->pose.covariance[i * 6 + j] = covariance(i, j);
                }
            }

            state_pub_->publish(std::move(msg));
        }

        // Process model
        EKF::StateVector processModel(const EKF::StateVector& state, double dt) {
            // Use actual encoder model
            EKF::StateVector new_state = state;

            return new_state;
        }

        // Measurement model
        EKF::MeasurementVector measurementModel(const EKF::StateVector& state) {
            EKF::MeasurementVector measurement;
            measurement << state(0), state(1), state(0), state(1), state(2);
            return measurement;
        }

        // Process model Jacobian
        EKF::StateMatrix processJacobian(const EKF::StateVector& state, double dt) {

            return EKF::StateMatrix::Identity();
        }

        // Measurement model Jacobian
        EKF::MeasurementMatrix measurementJacobian(const EKF::StateVector& state) {
            EKF::MeasurementMatrix H = EKF::MeasurementMatrix::Zero();
            H << 1, 0, 0,
                0, 1, 0,
                1, 0, 0,
                0, 1, 0,
                0, 0, 1;
            return H;
        }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DifferentialDriveEKFFusionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}