#pragma once

#include <vector>
#include <memory>
#include <string>

class AcadosBase {
    public:
        AcadosBase(const std::vector<double>& lbx, const std::vector<double>& ubx,
                const std::vector<double>& lbu, const std::vector<double>& ubu,
                const std::vector<double>& Q_diag,
                const std::vector<double>& R_diag,
                const std::vector<double>& R_rate_diag);
        virtual ~AcadosBase() = default;

        virtual int solve() = 0;
        virtual void setInitialState(const std::vector<double>& x0) = 0;
        virtual void setReference(int stage, const std::vector<double>& yref) = 0;
        virtual std::vector<double> getState(int stage) = 0;
        virtual std::vector<double> getControl(int stage) = 0;
        virtual double getSolveTime() = 0;
        virtual int getSQPIterations() = 0;
        virtual void printSolverInfo() = 0;

    protected:
        std::vector<double> lbx_, ubx_, lbu_, ubu_, Q_diag_, R_diag_, R_rate_diag_;
        virtual void setupSolver() = 0;

        // Common utility functions
        std::vector<std::vector<double>> generate_cubic_trajectory(
            const std::vector<double>& start,
            const std::vector<double>& end,
            int num_points);
        
        std::vector<std::vector<double>> calculate_reference_control(
            const std::vector<std::vector<double>>& trajectory,
            double total_time);
};

class AcadosSolverFactory {
    public:
        static std::unique_ptr<AcadosBase> createSolver(const std::string& model_type,
                                                        const std::vector<double>& lbx,
                                                        const std::vector<double>& ubx,
                                                        const std::vector<double>& lbu,
                                                        const std::vector<double>& ubu,
                                                        const std::vector<double>& Q_diag,
                                                        const std::vector<double>& R_diag,
                                                        const std::vector<double>& R_rate_diag);
};