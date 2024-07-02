#include "ros2_nmpc/nmpc_differential_drive.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>
#include <stdexcept>
#include <iostream>

namespace ros2_nmpc {

    NMPCDifferentialDrive::NMPCDifferentialDrive(const std::string& node_name, const NMPCParams& params)
        : Node(node_name), params_(params), iteration_(0), is_initialized_(false)
    {
        setupSolver();

        x_curr_.resize(DIFFERENTIAL_DRIVE_NX, 0.0);
        x_target_.resize(DIFFERENTIAL_DRIVE_NX, 0.0);

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&NMPCDifferentialDrive::odom_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(params_.dt), std::bind(&NMPCDifferentialDrive::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "NMPC Differential Drive Node initialized");
    }

    NMPCDifferentialDrive::~NMPCDifferentialDrive()
    {
        if (capsule) {
            differential_drive_acados_free(capsule);
            differential_drive_acados_free_capsule(capsule);
        }
    }

    void NMPCDifferentialDrive::setTarget(const std::vector<double>& target)
    {
        if (target.size() != DIFFERENTIAL_DRIVE_NX) {
            throw std::invalid_argument("Target state size mismatch");
        }
        x_target_ = target;
        update_reference_trajectory();
    }

    void NMPCDifferentialDrive::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        x_curr_[0] = msg->pose.pose.position.x;
        x_curr_[1] = msg->pose.pose.position.y;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        x_curr_[2] = yaw;

        is_initialized_ = true;
    }

    void NMPCDifferentialDrive::control_loop()
    {
        if (!is_initialized_) {
            RCLCPP_WARN(this->get_logger(), "Waiting for initial state...");
            return;
        }

        try {
            setInitialState(x_curr_);

            for (int j = 0; j <= DIFFERENTIAL_DRIVE_N; ++j) {
                int ref_index = std::min(iteration_ + j, static_cast<int>(x_ref_traj_.size()) - 1);
                std::vector<double> yref(DIFFERENTIAL_DRIVE_NY, 0.0);
                std::copy(x_ref_traj_[ref_index].begin(), x_ref_traj_[ref_index].end(), yref.begin());
                std::copy(u_ref_traj_[ref_index].begin(), u_ref_traj_[ref_index].end(), yref.begin() + DIFFERENTIAL_DRIVE_NX);
                
                if (j > 0) {
                    int prev_ref_index = std::min(iteration_ + j - 1, static_cast<int>(x_ref_traj_.size()) - 1);
                    for (int k = 0; k < DIFFERENTIAL_DRIVE_NU; ++k) {
                        yref[DIFFERENTIAL_DRIVE_NX + DIFFERENTIAL_DRIVE_NU + k] = u_ref_traj_[ref_index][k] - u_ref_traj_[prev_ref_index][k];
                    }
                }
                
                setReference(j, yref);
            }

            int status = solve();
            if (status != 0) {
                throw std::runtime_error("Solver failed with status " + std::to_string(status));
            }

            std::vector<double> u = getControl(0);

            auto cmd_vel_msg = geometry_msgs::msg::Twist();
            cmd_vel_msg.linear.x = u[0];
            cmd_vel_msg.angular.z = u[1];
            cmd_vel_pub_->publish(cmd_vel_msg);

            RCLCPP_INFO(this->get_logger(), "Published velocity command: linear = %f, angular = %f", u[0], u[1]);

            double distance = std::sqrt(std::pow(x_curr_[0] - x_target_[0], 2) + 
                                        std::pow(x_curr_[1] - x_target_[1], 2) +
                                        std::pow(x_curr_[2] - x_target_[2], 2));
            if (distance < params_.distance_threshold) {
                RCLCPP_INFO(this->get_logger(), "Target reached!");
            }

            iteration_++;
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in control loop: %s", e.what());
        }
    }

    void NMPCDifferentialDrive::update_reference_trajectory()
    {
        x_ref_traj_ = generate_cubic_trajectory(x_curr_, x_target_, params_.num_trajectory_points);
        double total_time = (params_.num_trajectory_points - 1) * params_.dt;
        u_ref_traj_ = calculate_reference_control(x_ref_traj_, total_time);
        iteration_ = 0;
    }

    std::vector<std::vector<double>> NMPCDifferentialDrive::generate_cubic_trajectory(
        const std::vector<double>& start,
        const std::vector<double>& end,
        int num_points)
    {
        std::vector<std::vector<double>> trajectory(num_points, std::vector<double>(start.size()));
        for (int i = 0; i < num_points; ++i) {
            double t = static_cast<double>(i) / (num_points - 1);
            for (size_t j = 0; j < start.size(); ++j) {
                trajectory[i][j] = start[j] + (3 * t * t - 2 * t * t * t) * (end[j] - start[j]);
            }
        }
        return trajectory;
    }

    std::vector<std::vector<double>> NMPCDifferentialDrive::calculate_reference_control(
        const std::vector<std::vector<double>>& trajectory,
        double total_time)
    {
        int num_points = trajectory.size();
        double dt = total_time / (num_points - 1);
        std::vector<std::vector<double>> u_ref(num_points, std::vector<double>(2));

        for (int i = 0; i < num_points - 1; ++i) {
            double dx = trajectory[i+1][0] - trajectory[i][0];
            double dy = trajectory[i+1][1] - trajectory[i][1];
            u_ref[i][0] = std::sqrt(dx*dx + dy*dy) / dt; 
            u_ref[i][1] = (trajectory[i+1][2] - trajectory[i][2]) / dt;  
        }
        u_ref[num_points-1] = u_ref[num_points-2]; 
        return u_ref;
    }

    void NMPCDifferentialDrive::setupSolver()
    {
        capsule = differential_drive_acados_create_capsule();
        if (capsule == nullptr) {
            throw std::runtime_error("Failed to create acados capsule");
        }

        int status = differential_drive_acados_create(capsule);
        if (status != 0) {
            throw std::runtime_error("Failed to create acados solver");
        }

        nlp_config = differential_drive_acados_get_nlp_config(capsule);
        nlp_dims = differential_drive_acados_get_nlp_dims(capsule);
        nlp_in = differential_drive_acados_get_nlp_in(capsule);
        nlp_out = differential_drive_acados_get_nlp_out(capsule);
        nlp_solver = differential_drive_acados_get_nlp_solver(capsule);
        nlp_opts = differential_drive_acados_get_nlp_opts(capsule);

        // Set constraints and cost matrices
        for (int i = 0; i <= DIFFERENTIAL_DRIVE_N; ++i) {
            ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lbx", params_.lbx.data());
            ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ubx", params_.ubx.data());
        }

        for (int i = 0; i < DIFFERENTIAL_DRIVE_N; ++i) {
            ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lbu", params_.lbu.data());
            ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ubu", params_.ubu.data());
        }

        std::vector<double> W(DIFFERENTIAL_DRIVE_NY * DIFFERENTIAL_DRIVE_NY, 0.0);
        for (int i = 0; i < DIFFERENTIAL_DRIVE_NX; ++i) {
            W[i * DIFFERENTIAL_DRIVE_NY + i] = params_.Q_diag[i];
        }
        for (int i = 0; i < DIFFERENTIAL_DRIVE_NU; ++i) {
            W[(DIFFERENTIAL_DRIVE_NX + i) * DIFFERENTIAL_DRIVE_NY + DIFFERENTIAL_DRIVE_NX + i] = params_.R_diag[i];
        }
        for (int i = 0; i < DIFFERENTIAL_DRIVE_NU; ++i) {
            W[(DIFFERENTIAL_DRIVE_NX + DIFFERENTIAL_DRIVE_NU + i) * DIFFERENTIAL_DRIVE_NY + DIFFERENTIAL_DRIVE_NX + DIFFERENTIAL_DRIVE_NU + i] = params_.R_rate_diag[i];
        }

        for (int i = 0; i < DIFFERENTIAL_DRIVE_N; ++i) {
            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "W", W.data());
        }

        std::vector<double> W_e(DIFFERENTIAL_DRIVE_NX * DIFFERENTIAL_DRIVE_NX, 0.0);
        for (int i = 0; i < DIFFERENTIAL_DRIVE_NX; ++i) {
            W_e[i * DIFFERENTIAL_DRIVE_NX + i] = params_.Q_diag[i];
        }
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, DIFFERENTIAL_DRIVE_N, "W", W_e.data());
    }

    int NMPCDifferentialDrive::solve()
    {
        return differential_drive_acados_solve(capsule);
    }

    void NMPCDifferentialDrive::setInitialState(const std::vector<double>& x0)
    {
        differential_drive_acados_set_initialx(capsule, x0.data());
    }

    void NMPCDifferentialDrive::setReference(int stage, const std::vector<double>& yref)
    {
        differential_drive_acados_set_reference(capsule, stage, yref.data());
    }

    std::vector<double> NMPCDifferentialDrive::getState(int stage)
    {
        std::vector<double> state(DIFFERENTIAL_DRIVE_NX);
        differential_drive_acados_get_state(capsule, stage, state.data());
        return state;
    }

    std::vector<double> NMPCDifferentialDrive::getControl(int stage)
    {
        std::vector<double> control(DIFFERENTIAL_DRIVE_NU);
        differential_drive_acados_get_control(capsule, stage, control.data());
        return control;
    }

    double NMPCDifferentialDrive::getSolveTime()
    {
        return differential_drive_acados_get_solve_time(capsule);
    }

    int NMPCDifferentialDrive::getSQPIterations()
    {
        return differential_drive_acados_get_sqp_iterations(capsule);
    }

    void NMPCDifferentialDrive::printSolverInfo()
    {
        std::cout << "Solver info:" << std::endl;
        std::cout << "  SQP iterations: " << getSQPIterations() << std::endl;
        std::cout << "  Solve time: " << getSolveTime() * 1000 << " ms" << std::endl;
    }

} // namespace ros2_nmpc