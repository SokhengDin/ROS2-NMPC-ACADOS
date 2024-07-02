#include "rclcpp/rclcpp.hpp"
#include "ros2_nmpc/nmpc_differential_drive.hpp"
#include <vector>

class NMPCDifferentialDriveNode : public rclcpp::Node
{
    public:
        NMPCDifferentialDriveNode() : Node("nmpc_differential_drive_node")
        {
            // Declare and get parameters
            this->declare_parameter("lbx", std::vector<double>{-10.0, -10.0, -M_PI});
            this->declare_parameter("ubx", std::vector<double>{10.0, 10.0, M_PI});
            this->declare_parameter("lbu", std::vector<double>{-1.5, -1.5});
            this->declare_parameter("ubu", std::vector<double>{1.5, 1.5});
            this->declare_parameter("Q_diag", std::vector<double>{10.0, 10.0, 1.0});
            this->declare_parameter("R_diag", std::vector<double>{0.5, 0.05});
            this->declare_parameter("R_rate_diag", std::vector<double>{0.1, 0.1});
            this->declare_parameter("dt", 0.1);
            this->declare_parameter("num_trajectory_points", 20);
            this->declare_parameter("distance_threshold", 0.1);

            ros2_nmpc::NMPCDifferentialDrive::NMPCParams params;
            params.lbx = this->get_parameter("lbx").as_double_array();
            params.ubx = this->get_parameter("ubx").as_double_array();
            params.lbu = this->get_parameter("lbu").as_double_array();
            params.ubu = this->get_parameter("ubu").as_double_array();
            params.Q_diag = this->get_parameter("Q_diag").as_double_array();
            params.R_diag = this->get_parameter("R_diag").as_double_array();
            params.R_rate_diag = this->get_parameter("R_rate_diag").as_double_array();
            params.dt = this->get_parameter("dt").as_double();
            params.num_trajectory_points = this->get_parameter("num_trajectory_points").as_int();
            params.distance_threshold = this->get_parameter("distance_threshold").as_double();

            // Create NMPC controller
            controller_ = std::make_unique<ros2_nmpc::NMPCDifferentialDrive>("nmpc_controller", params);

            // Create a service to set the target
            set_target_service_ = this->create_service<example_interfaces::srv::SetTarget>(
                "set_target",
                std::bind(&NMPCDifferentialDriveNode::set_target_callback, this, std::placeholders::_1, std::placeholders::_2));

            RCLCPP_INFO(this->get_logger(), "NMPC Differential Drive Node is ready.");
        }

    private:
        void set_target_callback(
            const std::shared_ptr<example_interfaces::srv::SetTarget::Request> request,
            std::shared_ptr<example_interfaces::srv::SetTarget::Response> response)
        {
            if (request->target.size() != 3) {
                RCLCPP_ERROR(this->get_logger(), "Invalid target size. Expected 3 (x, y, theta), got %zu", request->target.size());
                response->success = false;
                return;
            }

            try {
                controller_->setTarget(request->target);
                RCLCPP_INFO(this->get_logger(), "New target set: x=%f, y=%f, theta=%f",
                            request->target[0], request->target[1], request->target[2]);
                response->success = true;
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to set target: %s", e.what());
                response->success = false;
            }
        }

        std::unique_ptr<ros2_nmpc::NMPCDifferentialDrive> controller_;
        rclcpp::Service<example_interfaces::srv::SetTarget>::SharedPtr set_target_service_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NMPCDifferentialDriveNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}