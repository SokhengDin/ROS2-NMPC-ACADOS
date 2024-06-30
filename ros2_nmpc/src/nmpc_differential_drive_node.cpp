#include "rclcpp/rclcpp.hpp"
#include "ros2_nmpc/nmpc_differential_drive.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    ros2_nmpc::NMPCDifferentialDrive::NMPCParams params;
    params.lbx = {-10.0, -10.0, -M_PI};
    params.ubx = {10.0, 10.0, M_PI};
    params.lbu = {-1.5, -1.5};
    params.ubu = {1.5, 1.5};
    params.Q_diag = {25.0, 10.0, 1.0};
    params.R_diag = {0.1, 0.01};
    params.R_rate_diag = {1.0, 2.0};
    params.dt = 0.05;
    params.num_trajectory_points = 100;
    params.distance_threshold = 0.03;

    auto node = std::make_shared<ros2_nmpc::NMPCDifferentialDrive>("nmpc_diff_drive_node", params);

    // Set the target state
    node->setTarget({3.0, 1.5, M_PI/2});

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}