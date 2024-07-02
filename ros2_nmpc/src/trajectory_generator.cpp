#include "ros2_nmpc/trajectory_generator.hpp"
#include <stdexcept>
#include <cmath>

namespace ros2_nmpc {

    TrajectoryGenerator::TrajectoryGenerator(double frequency, int state_dim, int control_dim, ControlCalculator control_calculator)
        : frequency_(frequency), state_dim_(state_dim), control_dim_(control_dim), control_calculator_(control_calculator) {}

    std::vector<TrajectoryPoint> TrajectoryGenerator::generateLinear(const std::vector<double>& start, const std::vector<double>& end, double duration) {
        int num_points = static_cast<int>(duration * frequency_);
        std::vector<TrajectoryPoint> trajectory(num_points);

        for (int i = 0; i < num_points; ++i) {
            double t = static_cast<double>(i) / (num_points - 1);
            TrajectoryPoint point;
            point.state.resize(state_dim_);
            for (int j = 0; j < state_dim_; ++j) {
                point.state[j] = start[j] + (end[j] - start[j]) * t;
            }
            point.time = t * duration;
            trajectory[i] = point;
        }

        calculateControlReferences(trajectory);
        return trajectory;
    }

    std::vector<TrajectoryPoint> TrajectoryGenerator::generateCircle(const std::vector<double>& center, double radius, double angular_velocity, double duration) {
        int num_points = static_cast<int>(duration * frequency_);
        std::vector<TrajectoryPoint> trajectory(num_points);

        for (int i = 0; i < num_points; ++i) {
            double t = static_cast<double>(i) / frequency_;
            TrajectoryPoint point;
            point.state.resize(state_dim_);
            point.state[0] = center[0] + radius * std::cos(angular_velocity * t);
            point.state[1] = center[1] + radius * std::sin(angular_velocity * t);
            point.state[2] = angular_velocity * t;
            point.time = t;
            trajectory[i] = point;
        }

        calculateControlReferences(trajectory);
        return trajectory;
    }

    std::vector<TrajectoryPoint> TrajectoryGenerator::generateLemniscate(const std::vector<double>& center, double scale, double duration) {
        int num_points = static_cast<int>(duration * frequency_);
        std::vector<TrajectoryPoint> trajectory(num_points);

        for (int i = 0; i < num_points; ++i) {
            double t = 2 * M_PI * static_cast<double>(i) / (num_points - 1);
            TrajectoryPoint point;
            point.state.resize(state_dim_);
            point.state[0] = center[0] + scale * std::sin(t) / (1 + std::pow(std::cos(t), 2));
            point.state[1] = center[1] + scale * std::sin(t) * std::cos(t) / (1 + std::pow(std::cos(t), 2));
            point.state[2] = std::atan2(std::cos(3 * t), -std::sin(3 * t));
            point.time = t * duration / (2 * M_PI);
            trajectory[i] = point;
        }

        calculateControlReferences(trajectory);
        return trajectory;
    }

    std::vector<TrajectoryPoint> TrajectoryGenerator::generateCubic(const std::vector<double>& start, const std::vector<double>& end, double duration) {
        int num_points = static_cast<int>(duration * frequency_);
        std::vector<TrajectoryPoint> trajectory(num_points);

        for (int i = 0; i < num_points; ++i) {
            double t = static_cast<double>(i) / (num_points - 1);
            TrajectoryPoint point;
            point.state.resize(state_dim_);
            for (int j = 0; j < state_dim_; ++j) {
                point.state[j] = start[j] + (end[j] - start[j]) * (3 * std::pow(t, 2) - 2 * std::pow(t, 3));
            }
            point.time = t * duration;
            trajectory[i] = point;
        }

        calculateControlReferences(trajectory);
        return trajectory;
    }

    std::vector<TrajectoryPoint> TrajectoryGenerator::generatePolynomial(
        const std::vector<double>& start, const std::vector<double>& end,
        const std::vector<double>& start_vel, const std::vector<double>& end_vel,
        double duration) {
        
        int num_points = static_cast<int>(duration * frequency_);
        std::vector<TrajectoryPoint> trajectory(num_points);

        for (int i = 0; i < num_points; ++i) {
            double t = static_cast<double>(i) / (num_points - 1);
            TrajectoryPoint point;
            point.state.resize(state_dim_);
            for (int j = 0; j < state_dim_; ++j) {
                double a0 = start[j];
                double a1 = start_vel[j];
                double a2 = 3 * (end[j] - start[j]) / std::pow(duration, 2) - (end_vel[j] + 2 * start_vel[j]) / duration;
                double a3 = -2 * (end[j] - start[j]) / std::pow(duration, 3) + (end_vel[j] + start_vel[j]) / std::pow(duration, 2);
                point.state[j] = a0 + a1 * t + a2 * std::pow(t, 2) + a3 * std::pow(t, 3);
            }
            point.time = t * duration;
            trajectory[i] = point;
        }

        calculateControlReferences(trajectory);
        return trajectory;
    }

    std::vector<TrajectoryPoint> TrajectoryGenerator::generateCustom(
        std::function<std::vector<double>(double)> state_function,
        double duration) {
        
        int num_points = static_cast<int>(duration * frequency_);
        std::vector<TrajectoryPoint> trajectory(num_points);

        for (int i = 0; i < num_points; ++i) {
            double t = static_cast<double>(i) / frequency_;
            TrajectoryPoint point;
            point.state = state_function(t);
            point.time = t;
            trajectory[i] = point;
        }

        calculateControlReferences(trajectory);
        return trajectory;
    }

    void TrajectoryGenerator::calculateControlReferences(std::vector<TrajectoryPoint>& trajectory) {
        for (size_t i = 0; i < trajectory.size() - 1; ++i) {
            double dt = trajectory[i + 1].time - trajectory[i].time;
            trajectory[i].control = calculateControl(trajectory[i].state, trajectory[i + 1].state, dt);
        }
        // For the last point, use the same control as the previous point
        if (!trajectory.empty()) {
            trajectory.back().control = trajectory[trajectory.size() - 2].control;
        }
    }

    std::vector<double> TrajectoryGenerator::calculateControl(const std::vector<double>& current_state, const std::vector<double>& next_state, double dt) {
        std::vector<double> control(control_dim_);
        
        // For differential drive robot
        if (state_dim_ == 3 && control_dim_ == 2) {
            double dx = next_state[0] - current_state[0];
            double dy = next_state[1] - current_state[1];
            double dtheta = next_state[2] - current_state[2];

            // Linear velocity
            control[0] = std::sqrt(dx * dx + dy * dy) / dt;

            // Angular velocity
            control[1] = dtheta / dt;
        } else {
            throw std::runtime_error("Unsupported state or control dimensions for control calculation");
        }

        return control;
    }

    std::unique_ptr<TrajectoryGenerator> TrajectoryFactory::createTrajectoryGenerator(
        double frequency, 
        int state_dim, 
        int control_dim, 
        ControlCalculator control_calculator) 
    {
        return std::make_unique<TrajectoryGenerator>(frequency, state_dim, control_dim, control_calculator);
    }

    std::vector<double> TrajectoryGenerator::calculateControl(const std::vector<double>& current_state, const std::vector<double>& next_state, double dt) {
        return control_calculator_(current_state, next_state, dt);
    }

    namespace control_calculators {

        std::vector<double> differentialDrive(const std::vector<double>& current_state, const std::vector<double>& next_state, double dt)
        {
            std::vector<double> control(2);
            double dx = next_state[0] - current_state[0];
            double dy = next_state[1] - current_state[1];
            double dtheta = next_state[2] - current_state[2];

            // Linear velocity
            control[0] = std::sqrt(dx * dx + dy * dy) / dt;

            // Angular velocity
            control[1] = dtheta / dt;

            return control;
        }

        std::vector<double> ackermann(const std::vector<double>& current_state, const std::vector<double>& next_state, double dt) {
            std::vector<double> control(2);
            double dx = next_state[0] - current_state[0];
            double dy = next_state[1] - current_state[1];
            double dtheta = next_state[2] - current_state[2];

            // Linear velocity
            control[0] = std::sqrt(dx * dx + dy * dy) / dt;
            // Steering angle
            control[1] = std::atan2(2 * std::sin(dtheta) * control[0] * dt, std::sqrt(dx * dx + dy * dy));

            return control;
        }

        std::vector<double> OmniWheel(const std::vector<double>& current_state, const std::vector<double>& next_state, double dt)
        {
            // Pass
        }
    }

} // namespace ros2_nmpc