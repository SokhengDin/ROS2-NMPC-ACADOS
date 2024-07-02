#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "acados_solver_differential_drive.h"
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
            ~NMPCDifferentialDrive();

            void setTarget(const std::vector<double>& target);

        private:
            // ROS 2 related methods
            void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
            void control_loop();

            // NMPC related methods
            void update_reference_trajectory();
            std::vector<std::vector<double>> generate_cubic_trajectory(
                const std::vector<double>& start,
                const std::vector<double>& end,
                int num_points);
            std::vector<std::vector<double>> calculate_reference_control(
                const std::vector<std::vector<double>>& trajectory,
                double total_time);

            // Acados related methods
            void setupSolver();
            int solve();
            void setInitialState(const std::vector<double>& x0);
            void setReference(int stage, const std::vector<double>& yref);
            std::vector<double> getState(int stage);
            std::vector<double> getControl(int stage);
            double getSolveTime();
            int getSQPIterations();
            void printSolverInfo();

            // ROS 2 members
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
            rclcpp::TimerBase::SharedPtr timer_;

            // NMPC members
            std::vector<double> x_curr_;
            std::vector<double> x_target_;
            std::vector<std::vector<double>> x_ref_traj_;
            std::vector<std::vector<double>> u_ref_traj_;
            NMPCParams params_;
            int iteration_;
            bool is_initialized_;

            // Acados members
            differential_drive_solver_capsule* capsule;
            ocp_nlp_config* nlp_config;
            ocp_nlp_dims* nlp_dims;
            ocp_nlp_in* nlp_in;
            ocp_nlp_out* nlp_out;
            ocp_nlp_solver* nlp_solver;
            void* nlp_opts;
    };

} // namespace ros2_nmpc