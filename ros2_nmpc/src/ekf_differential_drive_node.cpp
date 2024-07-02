#include "rclcpp/rclcpp.hpp"
#include "ros2_nmpc/ekf_differential_drive.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <memory>
#include <mutex>

namespace ros2_estimation {

    class DifferentialDriveEKFNode : public rclcpp::Node {  
    public:
        DifferentialDriveEKFNode() : Node("differential_drive_ekf_node")
        {
            this->declare_parameter("initial_state", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
            this->declare_parameter("initial_covariance", std::vector<double>(36, 0.0));
            this->declare_parameter("process_noise_covariance", std::vector<double>(36, 0.0));
            this->declare_parameter("measurement_noise_covariance", std::vector<double>(16, 0.0));
            this->declare_parameter("wheel_base", 0.5);
            this->declare_parameter("wheel_radius", 0.1);
            this->declare_parameter("update_rate", 50.0);

            initializeEKF();
            initializeROSInterfaces();
        }

    private:
        void initializeEKF()
        {
            DifferentialDriveEKF::EKFParams params;
            
            auto initial_state = this->get_parameter("initial_state").as_double_array();
            params.initial_state = Eigen::Map<DifferentialDriveEKF::StateVector>(initial_state.data());

            auto initial_covariance = this->get_parameter("initial_covariance").as_double_array();
            params.initial_covariance = Eigen::Map<DifferentialDriveEKF::StateMatrix>(initial_covariance.data());

            auto process_noise = this->get_parameter("process_noise_covariance").as_double_array();
            params.process_noise_covariance = Eigen::Map<DifferentialDriveEKF::StateMatrix>(process_noise.data());

            auto measurement_noise = this->get_parameter("measurement_noise_covariance").as_double_array();
            params.measurement_noise_covariance = Eigen::Map<Eigen::Matrix<double, 4, 4>>(measurement_noise.data());

            params.wheel_base = this->get_parameter("wheel_base").as_double();
            params.wheel_radius = this->get_parameter("wheel_radius").as_double();

            ekf_ = std::make_unique<DifferentialDriveEKF>(params);
        }

        void initializeROSInterfaces()
        {
            encoder_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "encoder_odom", 10, std::bind(&DifferentialDriveEKFNode::encoderOdomCallback, this, std::placeholders::_1));
            
            imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
                "imu", 10, std::bind(&DifferentialDriveEKFNode::imuCallback, this, std::placeholders::_1));

            cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
                "cmd_vel", 10, std::bind(&DifferentialDriveEKFNode::cmdVelCallback, this, std::placeholders::_1));

            estimated_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("estimated_odom", 10);

            double update_rate = this->get_parameter("update_rate").as_double();
            timer_ = this->create_wall_timer(
                std::chrono::duration<double>(1.0 / update_rate),
                std::bind(&DifferentialDriveEKFNode::timerCallback, this));

            last_update_time_ = this->now();
        }

        void encoderOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lock(ekf_mutex_);
            DifferentialDriveEKF::MeasurementVector measurement;
            measurement << msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0, 0.0;

            tf2::Quaternion q(
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            measurement(2) = yaw;

            ekf_->updateWithEncoderData(measurement);
        }

        void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lock(ekf_mutex_);
            tf2::Quaternion q(
                msg->orientation.x,
                msg->orientation.y,
                msg->orientation.z,
                msg->orientation.w);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

            ekf_->updateWithIMUData(yaw);
        }

        void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lock(ekf_mutex_);
            ekf_->setLinearVelocity(msg->linear.x);
            ekf_->setAngularVelocity(msg->angular.z);
        }

        void timerCallback()
        {
            rclcpp::Time current_time = this->now();
            double dt = (current_time - last_update_time_).seconds();
            last_update_time_ = current_time;

            {
                std::lock_guard<std::mutex> lock(ekf_mutex_);
                ekf_->predict(dt);
            }

            publishEstimatedOdometry(current_time);
        }

        void publishEstimatedOdometry(const rclcpp::Time& current_time)
        {
            std::lock_guard<std::mutex> lock(ekf_mutex_);
            auto state = ekf_->getState();
            auto covariance = ekf_->getCovariance();

            nav_msgs::msg::Odometry estimated_odom;
            estimated_odom.header.stamp = current_time;
            estimated_odom.header.frame_id = "odom";
            estimated_odom.child_frame_id = "base_link";

            estimated_odom.pose.pose.position.x = state(0);
            estimated_odom.pose.pose.position.y = state(1);
            estimated_odom.pose.pose.position.z = 0.0;

            tf2::Quaternion q;
            q.setRPY(0, 0, state(2));
            estimated_odom.pose.pose.orientation = tf2::toMsg(q);

            for (int i = 0; i < 36; ++i) {
                estimated_odom.pose.covariance[i] = 0;
            }
            for (int i = 0; i < DifferentialDriveEKF::STATE_SIZE; ++i) {
                for (int j = 0; j < DifferentialDriveEKF::STATE_SIZE; ++j) {
                    estimated_odom.pose.covariance[i*6 + j] = covariance(i, j);
                }
            }

            estimated_odom.twist.twist.linear.x = state(3);
            estimated_odom.twist.twist.linear.y = state(4);
            estimated_odom.twist.twist.angular.z = state(5);

            estimated_odom_pub_->publish(estimated_odom);
        }

        std::unique_ptr<DifferentialDriveEKF> ekf_;
        std::mutex ekf_mutex_;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr encoder_odom_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr estimated_odom_pub_;
        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Time last_update_time_;
    };

} // namespace ros2_estimation

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ros2_estimation::DifferentialDriveEKFNode>());
    rclcpp::shutdown();
    return 0;
}