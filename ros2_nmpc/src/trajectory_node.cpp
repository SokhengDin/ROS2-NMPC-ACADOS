#include "rclcpp/rclcpp.hpp"
#include "ros2_nmpc/trajectory_generator.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <vector>
#include <string>
#include <memory>

class TrajectoryNode: public rclcpp::Node
{
    public:
        TrajectoryNode(): Node("trajectory_node")
        {
            // Declare parameters
            this->declare_parameter("trajectory_type", "linear");
            this->declare_parameter("num_points", 100);
            this->declare_parameter("start_point", std::vector<double>{0.0, 0.0, 0.0});
            this->declare_parameter("end_point", std::vector<double>{1.0, 1.0, 0.0});
            this->declare_parameter("control_points", std::vector<std::vector<double>>{{0.0, 0.0, 0.0}, {1.0, 1.0, 0.0}});
            this->declare_parameter("duration", 10.0);
            this->declare_parameter("publish_frequency", 10.0);
            this->declare_parameter("trajectory_topic", "trajectory");

            // Get parameters
            std::string trajectory_type = this->get_parameter("trajectory_type").as_string();
            int num_points = this->get_parameter("num_points").as_int();
            std::vector<double> start_point = this->get_parameter("start_point").as_double_array();
            std::vector<double> end_point = this->get_parameter("end_point").as_double_array();
            std::vector<std::vector<double>> control_points = this->get_parameter("control_points").as_double_array_array();
            double duration = this->get_parameter("duration").as_double();
            double publish_frequency = this->get_parameter("publish_frequency").as_double();
            std::string trajectory_topic = this->get_parameter("trajectory_topic").as_string();

            // Create trajectory generator
            trajectory_generator_ = ros2_nmpc::TrajectoryFactory::createTrajectoryGenerator(
                publish_frequency, 3, 2, ros2_nmpc::control_calculators::differentialDrive
            );

            // Generator trajectory 
            if (trajectory_type == "linear") {
                trajectory_ = trajectory_generator_->generateLinear(start_point, end_point, duration);
            } else if (trajectory_type == "cubic") {
                trajectory_ = trajectory_generator_->generateCubic(start_point, end_point, duration);
            } else if (trajectory_type == "cubic_spline") {
                trajectory_ = trajectory_generator_->generateCubicSpline(control_points, duration);
            } else if (trajectory_type == "bezier") {
                trajectory_ = trajectory_generator_->generateBezierPolynomial(control_points, duration);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Unsupported trajectory type: %s", trajectory_type.c_str());
                return;
            }

            // Create publisher
            trajectory_publisher_ = this->create_publisher<nav_msg::msg::Path>(trajectory_topic, 10);

            // Create timer for publishing
            timer_ = this->create_wall_timer(
                std::chrono::duration<double>(1.0 / publish_frequency),
                std::bind(&TrajectoryNode::publish_trajectory, this)
            );

            RCLCPP_INFO(this->get_logger(), "Trajectory node initialized");
        }

    private:
        void publish_trajectory()
        {
            auto message = nav_msg::msg::Path();
            message.header.stamp = this->now();
            message.header.frame_id = "map";

            for (const auto& point : trajectory_) {
                geometry_msgs::msg::PoseStamped pose;
                pose.pose.position.x = point.state[0];
                pose.pose.position.y = point.state[1];
                pose.pose.position.z = 0.0;
                
                // Convert yaw to quaternion
                tf2::Quaternion q;
                q.setRPY(0, 0, point.state[2]);
                pose.pose.orientation.x = q.x();
                pose.pose.orientation.y = q.y();
                pose.pose.orientation.z = q.z();
                pose.pose.orientation.w = q.w();

                message.poses.push_back(pose);
            }

            trajectory_publisher_ -> publish(message);
        }

        std::unique_ptr<ros2_nmpc::TrajectoryGenerator> trajectory_generator_;
        std::vector<ros2_nmpc::TrajectoryPoint> trajectory_;
        
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryNode>());
    rclcpp::shutdown();

    return 0;
}