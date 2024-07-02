#pragma once

#include <vector>
#include <functional>
#include <memory>

namespace ros2_nmpc {

    struct TrajectoryPoint {
        std::vector<double> state;
        std::vector<double> control;
        double time;
    };

    // Define a type for the control calculation function
    using ControlCalculator = std::function<std::vector<double>(const std::vector<double>&, const std::vector<double>&, double)>;

    class TrajectoryGenerator {
        public:
            TrajectoryGenerator(double frequency, int state_dim, int control_dim, ControlCalculator control_calculator);

            // Trajectory generation methods
            std::vector<TrajectoryPoint> generateLinear(const std::vector<double>& start, const std::vector<double>& end, double duration);
            std::vector<TrajectoryPoint> generateCircle(const std::vector<double>& center, double radius, double angular_velocity, double duration);
            std::vector<TrajectoryPoint> generateLemniscate(const std::vector<double>& center, double scale, double duration);
            std::vector<TrajectoryPoint> generateCubic(const std::vector<double>& start, const std::vector<double>& end, double duration);
            std::vector<TrajectoryPoint> generatePolynomial(const std::vector<double>& start, const std::vector<double>& end, 
                                                            const std::vector<double>& start_vel, const std::vector<double>& end_vel, 
                                                            double duration);

            // Custom trajectory generation
            std::vector<TrajectoryPoint> generateCustom(
                std::function<std::vector<double>(double)> state_function,
                double duration);

            // Control reference calculation
            void calculateControlReferences(std::vector<TrajectoryPoint>& trajectory);

        private:
            double frequency_;
            int state_dim_;
            int control_dim_;
            ControlCalculator control_calculator_;
            
            // Updated calculateControl method
            std::vector<double> calculateControl(const std::vector<double>& current_state, const std::vector<double>& next_state, double dt);
    };

    // Factory for creating different types of trajectories
    class TrajectoryFactory {
        public:
            static std::unique_ptr<TrajectoryGenerator> createTrajectoryGenerator(
                double frequency, 
                int state_dim, 
                int control_dim, 
                ControlCalculator control_calculator);
    };

    // Predefined control calculators for different robot models
    namespace control_calculators {
        std::vector<double> differentialDrive(const std::vector<double>& current_state, const std::vector<double>& next_state, double dt);
        std::vector<double> omniWheel(const std::vector<double>& current_state, const std::vector<double>& next_state, double dt);
        std::vector<double> ackermann(const std::vector<double>& current_state, const std::vector<double>& next_state, double dt);
    } // namespace control_calculators

} // namespace ros2_nmpc