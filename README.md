# ros2_nmpc

A ROS 2 package implementing Nonlinear Model Predictive Control (NMPC) for differential drive robots using the Acados solver.

## Description

This package provides a ROS 2 node that implements NMPC for trajectory tracking of differential drive robots. It uses the Acados solver for efficient solution of the optimal control problem.

## Prerequisites

- ROS 2 (tested on Humble, but should work on other distributions)
- Acados (with Python interface)
- Python 3
- CMake (>= 3.8)

## Installation

1. Install ROS 2 and create a workspace:

   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```

2. Clone this repository into your ROS 2 workspace:

   ```bash
   git clone https://github.com/SokhengDin/ROS2-NMPC-ACADOS.git
   ```

3. Install Acados following the [official installation guide](https://docs.acados.org/installation/).

4. Update the `ACADOS_INSTALL_DIR` in `CMakeLists.txt` to point to your Acados installation directory.

5. Build the package:

   ```bash
   cd ~/ros2_ws
   colcon build --packages-select ros2_nmpc
   ```

6. Source the workspace:

   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

## Usage

1. Configure the NMPC parameters in `config/nmpc_diff_params.yaml`.

2. Launch the NMPC node:

   ```bash
   ros2 launch ros2_nmpc nmpc_differential_drive_node.launch.py
   ```

3. The node will subscribe to `/odom` for the current robot state and publish velocity commands to `/cmd_vel`.

## Configuration

You can modify the NMPC parameters in `config/nmpc_diff_params.yaml`. The main parameters are:

- `lbx`, `ubx`: State constraints (lower and upper bounds)
- `lbu`, `ubu`: Control constraints (lower and upper bounds)
- `Q_diag`: State cost diagonal
- `R_diag`: Control cost diagonal
- `R_rate_diag`: Control rate cost diagonal
- `dt`: Time step
- `num_trajectory_points`: Number of points in the reference trajectory
- `distance_threshold`: Threshold for considering target reached
- `target_x`, `target_y`, `target_theta`: Target pose

## Customization

To use this NMPC controller with a different robot model:

1. Modify the `differential_drive_model` in `acados_generated/` to match your robot's dynamics.
2. Update the `NMPCDifferentialDrive` class in `src/nmpc_differential_drive.cpp` to use the new model.
3. Adjust the parameters in `config/nmpc_diff_params.yaml` to suit your robot and control requirements.

## Troubleshooting

- If you encounter "file not found" errors during compilation, ensure that all Acados generated files are in the correct location and that the `ACADOS_INSTALL_DIR` in `CMakeLists.txt` is set correctly.
- If the node fails to start, check that the Acados solver library `libacados_ocp_solver_differential_drive.so` is present in the `acados_generated/differential_drive/` directory.
