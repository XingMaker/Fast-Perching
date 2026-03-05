# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Fast-Perching is a ROS-based trajectory planning system for real-time aerial perching on micro aerial robots. It uses MINCO (Minimum Control) trajectory representation with L-BFGS optimization to generate trajectories that adaptively adjust terminal states and duration.

## Build System

This is a ROS catkin workspace using C++14.

```bash
# Build the project
catkin_make

# Source the environment (required before running)
source devel/setup.zsh  # or setup.bash
```

## Running the Simulation

```bash
# Terminal 1: Launch RViz and visualization
roslaunch planning perching.launch

# Terminal 2: Trigger the planner
./sh_utils/pub_triger.sh
```

## Package Structure

- **traj_opt**: Core trajectory optimization library. Contains MINCO implementation and L-BFGS solver. Exposes `traj_opt::TrajOpt` class.
- **planning**: ROS nodelet (`planning/Nodelet`) that handles ROS I/O and calls traj_opt. Main launch file: `src/planning/launch/perching.launch`.
- **vis_utils**: Header-only visualization utilities for RViz markers and trajectories.
- **odom_vis**: Odometry visualization components (pose_utils and odom_visualization).

## Key Configuration

Perching parameters are configured in `src/planning/launch/perching.launch`:

**Target state:**
- `perching_px/py/pz`: Target position (meters)
- `perching_vx/vy/vz`: Target velocity (m/s)
- `perching_axis_x/y/z`: Rotation axis for landing orientation
- `perching_theta`: Rotation angle (radians)

**Robot dynamics:**
- `vmax`/`amax`: Velocity/acceleration limits
- `thrust_max`/`thrust_min`: Thrust limits (m/s²)
- `omega_max`: Maximum body angular velocity (rad/s)
- `robot_l`: Robot length, `robot_r`: Robot radius

**Algorithm parameters:**
- `N` (in code): Number of trajectory pieces (default 16)
- `K`: Integration samples per piece for constraints
- `rhoT`: Time regularization weight
- `rhoP`/`rhoV`/`rhoA`: Penalty weights for position/velocity/acceleration constraints

**Features:**
- `replan`: Enable replanning mode
- `pause_debug`: Enable step-by-step optimization visualization

## Key Classes and Files

- `traj_opt::TrajOpt` (`src/traj_opt/include/traj_opt/traj_opt.h`): Main optimization interface. Key method: `generate_traj()`.
- `minco::MINCO_S4_Uniform` (`src/traj_opt/include/traj_opt/minco.hpp`): Trajectory representation with uniform time allocation.
- `vis_utils::VisUtils` (`src/vis_utils/include/vis_utils/vis_utils.hpp`): Visualization helper for RViz.
- `planning::Nodelet` (`src/planning/src/planning_nodelet.cpp`): ROS nodelet implementation.

## Algorithm Overview

The planner solves an optimization problem where:
- **Decision variables**: Total duration, intermediate waypoints, terminal thrust, terminal velocity (in tangent plane)
- **Objective**: Minimize snap (4th derivative) + time regularization
- **Constraints**: Velocity, thrust limits, angular velocity, collision avoidance with landing platform
- **Optimization**: L-BFGS with smoothed L1 penalties for constraints

The trajectory is represented as piecewise 7th-degree polynomials (Snap-optimal, MINCO formulation).

## Development Notes

- All packages use `Eigen3` for linear algebra.
- The L-BFGS solver is embedded in `src/traj_opt/include/traj_opt/lbfgs_raw.hpp`.
- Trajectory polynomials are defined in `src/traj_opt/include/traj_opt/poly_traj_utils.hpp`.
- Visualization topics use the `world` frame.
