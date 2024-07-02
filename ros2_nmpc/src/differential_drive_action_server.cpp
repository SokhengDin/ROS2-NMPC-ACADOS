#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "trajectory_generator.hpp"
#include "nmpc_controller.hpp"
#include "extended_kalman_filter.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <Eigen/Dense>
#include <vector>
#include <memory>

// Define a custom action
#include "example_interfaces/action/fibonacci.hpp"  // Replace with your custom action

using namespace std::placeholders;

class DifferentialDriveActionServer : public rclcpp::Node {
public:
    using PickAndPlace = example_interfaces::action::Fibonacci;  // Replace with your custom action
    using GoalHandlePickAndPlace = rclcpp_action::ServerGoalHandle<PickAndPlace>;

    DifferentialDriveActionServer() : Node("differential_drive_action_server") {
        using namespace std::placeholders;

        this->action_server_ = rclcpp_action::create_server<PickAndPlace>(
            this,
            "pick_and_place",
            std::bind(&DifferentialDriveActionServer::handle_goal, this, _1, _2),
            std::bind(&DifferentialDriveActionServer::handle_cancel, this, _1),
            std::bind(&DifferentialDriveActionServer::handle_accepted, this, _1));

        // Initialize components
        initializeTrajectoryGenerator();
        initializeNMPCController();
        initializeEKF();

        // Create subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&DifferentialDriveActionServer::odomCallback, this, _1));
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "gps", 10, std::bind(&DifferentialDriveActionServer::gpsCallback, this, _1));
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu", 10, std::bind(&DifferentialDriveActionServer::imuCallback, this, _1));

        // Create publishers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        estimated_state_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("estimated_state", 10);

        // Create timer for control loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&DifferentialDriveActionServer::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Differential Drive Action Server has been initialized.");
    }

private:
    rclcpp_action::Server<PickAndPlace>::SharedPtr action_server_;

    std::unique_ptr<ros2_nmpc::TrajectoryGenerator> trajectory_generator_;
    std::unique_ptr<ros2_nmpc::NMPCController> nmpc_controller_;
    std::unique_ptr<ros2_estimation::ExtendedKalmanFilter<3, 5>> ekf_;  // 3 states, 5 measurements

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr estimated_state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<ros2_nmpc::TrajectoryPoint> current_trajectory_;
    size_t trajectory_index_ = 0;
    Eigen::Vector3d current_state_;
    Eigen::Vector<double, 5> last_measurement_;

    // Action server methods
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const PickAndPlace::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request");
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandlePickAndPlace> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandlePickAndPlace> goal_handle)
    {
        using namespace std::placeholders;
        std::thread{std::bind(&DifferentialDriveActionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandlePickAndPlace> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<PickAndPlace::Feedback>();
        auto result = std::make_shared<PickAndPlace::Result>();
        
        // Generate trajectory for pick and place task
        generatePickAndPlaceTrajectory(goal);

        for (size_t i = 0; i < current_trajectory_.size(); ++i) {
            if (goal_handle->is_canceling()) {
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }

            // Update trajectory index
            trajectory_index_ = i;

            // Provide feedback
            feedback->sequence = {static_cast<int>(i), static_cast<int>(current_trajectory_.size())};
            goal_handle->publish_feedback(feedback);

            // Sleep to simulate work
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if (rclcpp::ok()) {
            result->sequence = feedback->sequence;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
    }

    void generatePickAndPlaceTrajectory(const std::shared_ptr<const PickAndPlace::Goal> goal) {
        // Generate trajectory based on goal
        // This is a placeholder - replace with actual trajectory generation logic
        std::vector<double> start = {0.0, 0.0, 0.0};
        std::vector<double> end = {5.0, 5.0, M_PI};
        current_trajectory_ = trajectory_generator_->generateCubic(start, end, 60.0);
    }

    void initializeTrajectoryGenerator() {
        trajectory_generator_ = ros2_nmpc::TrajectoryFactory::createTrajectoryGenerator(10.0, 3, 2);
    }

    void initializeNMPCController() {
        ros2_nmpc::NMPCParams params;
        params.dt = 0.1;
        params.prediction_horizon = 10;
        params.Q_diag = {10.0, 10.0, 1.0};
        params.R_diag = {1.0, 0.1};
        params.state_constraints = {{-10.0, -10.0, -M_PI}, {10.0, 10.0, M_PI}};
        params.control_constraints = {{-1.0, -1.0}, {1.0, 1.0}};

        nmpc_controller_ = std::make_unique<ros2_nmpc::NMPCController>(params);
    }

    void initializeEKF() {
        Eigen::Vector3d initial_state = Eigen::Vector3d::Zero();
        Eigen::Matrix3d initial_covariance = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d process_noise_covariance = Eigen::Matrix3d::Identity() * 0.1;
        Eigen::Matrix<double, 5, 5> measurement_noise_covariance = Eigen::Matrix<double, 5, 5>::Identity() * 0.1;

        ekf_ = ros2_estimation::EKFFactory::createEKF<3, 5>(
            initial_state, initial_covariance, process_noise_covariance, measurement_noise_covariance,
            std::bind(&DifferentialDriveActionServer::processModel, this, _1, _2),
            std::bind(&DifferentialDriveActionServer::measurementModel, this, _1),
            std::bind(&DifferentialDriveActionServer::processJacobian, this, _1, _2),
            std::bind(&DifferentialDriveActionServer::measurementJacobian, this, _1)
        );
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        last_measurement_[0] = msg->pose.pose.position.x;
        last_measurement_[1] = msg->pose.pose.position.y;
        updateEKF();
    }

    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        last_measurement_[2] = msg->latitude;
        last_measurement_[3] = msg->longitude;
        updateEKF();
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        last_measurement_[4] = yaw;
        updateEKF();
    }

    void updateEKF() {
        ekf_->predict(0.1);  // Assuming 10 Hz update rate
        ekf_->update(last_measurement_);
        current_state_ = ekf_->getState();
        publishEstimatedState();
    }

    void controlLoop() {
        if (trajectory_index_ >= current_trajectory_.size()) {
            return;
        }

        const auto& reference = current_trajectory_[trajectory_index_];
        nmpc_controller_->setReference(reference.state, reference.control);
        auto control = nmpc_controller_->solve(current_state_);

        auto cmd_vel_msg = geometry_msgs::msg::Twist();
        cmd_vel_msg.linear.x = control[0];
        cmd_vel_msg.angular.z = control[1];
        cmd_vel_pub_->publish(cmd_vel_msg);
    }

    void publishEstimatedState() {
        auto pose_msg = geometry_msgs::msg::PoseStamped();
        pose_msg.header.frame_id = "map";
        pose_msg.header.stamp = this->now();
        pose_msg.pose.position.x = current_state_[0];
        pose_msg.pose.position.y = current_state_[1];
        pose_msg.pose.orientation.z = std::sin(current_state_[2] / 2.0);
        pose_msg.pose.orientation.w = std::cos(current_state_[2] / 2.0);

        estimated_state_pub_->publish(pose_msg);
    }

    // EKF model functions (implement these based on your robot's dynamics)
    Eigen::Vector3d processModel(const Eigen::Vector3d& state, double dt) {
        // Implement process model
        return state;
    }

    Eigen::Vector<double, 5> measurementModel(const Eigen::Vector3d& state) {
        // Implement measurement model
        Eigen::Vector<double, 5> measurement;
        measurement << state[0], state[1], state[0], state[1], state[2];
        return measurement;
    }

    Eigen::Matrix3d processJacobian(const Eigen::Vector3d& state, double dt) {
        // Implement process Jacobian
        return Eigen::Matrix3d::Identity();
    }

    Eigen::Matrix<double, 5, 3> measurementJacobian(const Eigen::Vector3d& state) {
        // Implement measurement Jacobian
        Eigen::Matrix<double, 5, 3> H;
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
    auto node = std::make_shared<DifferentialDriveActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}