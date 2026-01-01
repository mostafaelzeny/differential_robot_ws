# Gemini Code Assistant Context

This document provides a comprehensive overview of the "Autonomous Differential Drive Robot" project, designed to serve as a persistent context for the Gemini Code Assistant.

## Project Overview

This is a ROS 2 project simulating an autonomous differential drive robot. The project implements a full navigation stack, including:

*   **SLAM (Simultaneous Localization and Mapping):** Using the `slam_toolbox` package for creating maps of simulated environments.
*   **Localization:** Using `amcl` (Adaptive Monte Carlo Localization) to determine the robot's position within a known map.
*   **Path Planning:** Includes implementations for A*, Dijkstra, and PD motion planning algorithms.
*   **Simulation:** Utilizes Gazebo for simulating the robot in various environments, including a small house and a small warehouse, with assets from AWS RoboMaker.

The project is structured as a ROS 2 workspace with multiple packages, each responsible for a specific functionality.

## Advanced Features

This project demonstrates a complete autonomous navigation stack with several advanced features:

-   **SLAM Implementation**: Real-time mapping using the `slam_toolbox` for robust map generation.
-   **Localization**: Utilizes the AMCL (Adaptive Monte Carlo Localization) algorithm for accurate and real-time global localization.
-   **Path Planning Algorithms**: Implements multiple path planning algorithms, providing flexibility for different scenarios:
    -   **A* Planner**: A heuristic-based planner for optimal pathfinding.
    -   **Dijkstra's Algorithm**: Guarantees the shortest path in an unweighted graph.
    -   **PD Motion Planner**: A Proportional-Derivative controller for smooth and reactive motion.
-   **High-Fidelity Simulation**: The robot is simulated in realistic Gazebo environments using detailed models from AWS RoboMaker, including a residential house and a warehouse.
-   **Full ROS 2 Integration**: The project is fully integrated with the ROS 2 ecosystem, using launch files for orchestration, YAML files for configuration, and standard message types for communication.

## Project Structure

The project is organized into several ROS 2 packages within the `src` directory:

```
differential_robot/
├── robot_controller/          # Robot control, hardware interfaces, and sensor processing
├── robot_description/         # URDF models, simulation worlds, and launch files
│   ├── models/               # AWS RoboMaker residential & warehouse models
│   ├── urdf/                 # Robot URDF and Xacro files for robot structure
│   └── worlds/               # Gazebo simulation environments
├── robot_localization/        # AMCL configuration and launch files for localization
├── robot_mapping/             # Costmap configuration and saved maps for navigation
├── robot_path_planning/       # C++ implementations of path planning algorithms
└── robot_slam/               # SLAM Toolbox integration and launch files
```

## Building and Running

### Building the Workspace

To build the project, use the standard ROS 2 build tool, `colcon`:

```bash
colcon build
```

After building, source the workspace to make the packages available in your environment:

```bash
source install/setup.bash
```

### Running the Simulation and Key Tasks

The project uses ROS 2 launch files to start the simulation and various nodes.

*   **To run SLAM and build a map:**
    ```bash
    ros2 launch robot_slam slam.launch.py
    ```
    You can then use `teleop_twist_keyboard` to manually drive the robot and create a map.

*   **To save the map:**
    ```bash
    ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
    ```

*   **To run localization with a pre-existing map:**
    ```bash
    ros2 launch robot_localization global_localization.launch.py
    ```

*   **To run autonomous navigation:**
    ```bash
    ros2 launch nav2_bringup navigation_launch.py
    ```

## Development Conventions

*   **ROS 2 Packages:** The codebase is organized into distinct ROS 2 packages, each with its own `package.xml` and `CMakeLists.txt`.
*   **Launch Files:** Functionality is launched via Python-based launch files located in the `launch` directory of the relevant packages.
*   **Configuration:** Parameters for nodes (e.g., for AMCL, SLAM) are stored in YAML files within the `config` directories of the packages.
*   **Robot Model:** The robot's physical properties are defined in URDF and XACRO files in the `robot_description` package.
