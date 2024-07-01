#include "ros2_nmpc/acados_base.hpp"
#include "stdexcept"
#include "cmath"

AcadosBase::AcadosBase(const std::vector<double>& lbx, const std::vector<double>& ubx,
                       const std::vector<double>& lbu, const std::vector<double>& ubu,
                       const std::vector<double>& Q_diag,
                       const std::vector<double>& R_diag,
                       const std::vector<double>& R_rate_diag)
    : lbx_(lbx), ubx_(ubx), lbu_(lbu), ubu_(ubu),
      Q_diag_(Q_diag), R_diag_(R_diag), R_rate_diag_(R_rate_diag) {
    setupSolver();
}

std::vector<std::vector<double>> AcadosBase::generate_cubic_trajectory(
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

std::vector<std::vector<double>> AcadosBase::calculate_reference_control(
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

std::unique_ptr<AcadosBase> AcadosSolverFactory::createSolver(
    const std::string& model_type,
    const std::vector<double>& lbx,
    const std::vector<double>& ubx,
    const std::vector<double>& lbu,
    const std::vector<double>& ubu,
    const std::vector<double>& Q_diag,
    const std::vector<double>& R_diag,
    const std::vector<double>& R_rate_diag)
{
    // This function will be implemented when we add specific robot models
    throw std::runtime_error("Not implemented yet");
}