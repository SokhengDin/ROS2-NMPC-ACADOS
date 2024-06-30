#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ros2_nmpc/acados_cpp_wrapper.hpp"
#include <vector>
#include <memory>

namespace ros2_nmpc {

class NMPCDifferentialDrive : public rclcpp::Node {
    public:
        struct NMPCParams {
            std::vector<double> lbx;
            std::vector<double> ubx;
            std::vector<double> lbu;
            std::vector<double> ubu;
            std::vector<double> Q_diag;
            std::vector<double> R_diag;
            std::vector<double> R_rate_diag;
            double dt;
            int num_trajectory_points;
            double distance_threshold;
        };

        NMPCDifferentialDrive(const std::string& node_name, const NMPCParams& params);

        void setTarget(const std::vector<double>& target);

    private:
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void control_loop();
        std::vector<std::vector<double>> generate_cubic_trajectory(
            const std::vector<double>& start,
            const std::vector<double>& end,
            int num_points
        );
        std::vector<std::vector<double>> calculate_reference_control(
            const std::vector<std::vector<double>>& trajectory,
            double total_time
        );
        void update_reference_trajectory();

        std::unique_ptr<AcadosSolver> solver_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::TimerBase::SharedPtr timer_;

        std::vector<double> x_curr_;
        std::vector<double> x_target_;
        std::vector<std::vector<double>> x_ref_traj_;
        std::vector<std::vector<double>> u_ref_traj_;
        NMPCParams params_;
        int iteration_;
        bool is_initialized_;
    };

} // namespace ros2_nmpc