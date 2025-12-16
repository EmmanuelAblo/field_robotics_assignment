# 🤖 Field Robotics Assignment: Autonomous TurtleBot3 Waffle

This repository contains the implementation for a field robotics assignment, demonstrating core robotic capabilities—low-level control, mapping (SLAM), navigation, and autonomous exploration—using the **TurtleBot3 Waffle** in a **ROS 2 Humble** and **Gazebo** simulation environment.

The project structure is modular and unified, designed to host all tasks of the assignment (Tasks 1–3).

## 🌟 Project Overview

The assignment is divided into four main tasks:

* **Task 1: Control System Implementation**
    * Designed and implemented a custom controller for the differential-drive robot.
    * Functionality includes **pose-to-pose navigation** and **circular trajectory tracking**.
    * The controller is based on a classic unicycle pose stabilization law.
* **Task 2 & 3: Autonomous Exploration Framework**
    * Integrated the state-of-the-art ROS 2 navigation stack: **SLAM Toolbox** for mapping and localization and **Nav2 (Navigation2)** for autonomous navigation and obstacle avoidance.
    * A custom `frontier_explorer` node implements a **frontier-based exploration strategy** to autonomously cover unknown environments.

## 🔧 Package Structure

The project components are contained within the single ROS 2 package, `field_robotics_assignment`.
```bash
field_robotics_assignment/ 
├─ field_robotics_assignment/ 
│  ├─ controller_node.py # Task 1: Pose & circular trajectory controller 
│  └─ frontier_explorer.py # Task 3: Frontier-based exploration logic 
├─ launch/ 
│  ├─ task1.launch.py # Launch file for Task 1 (Control System) 
│  └─ exploration.launch.py # Launch file for Task 2 & 3 (SLAM + Nav2 + Exploration) 
├─ resource/ 
│  ├─ Config_assignment.rviz # RViz configuration for visualization 
│  ├─ nav2_params.yaml # Configuration parameters for Nav2 
│  └─ mapper_params_online_async.yaml # Configuration for SLAM Toolbox 
└─ ... (setup files: package.xml, setup.py, setup.cfg, README.md)
```
## 💻 Setup and Execution

### Prerequisites

* **ROS 2 Distribution:** Humble Hawksbill
* **Simulation Environment:** Gazebo (Classic) with the official TurtleBot3 simulation packages
* **Dependencies:** `ros-humble-nav2-bringup` and `ros-humble-slam-toolbox`.

### Build the Package

1.  Clone the repository into your ROS 2 workspace (`~/ros2_ws/src/`):
    ```bash
    cd ~/ros2_ws/src/
    git clone [https://github.com/EmmanuelAblo/field_robotics_assignment.git](https://github.com/EmmanuelAblo/field_robotics_assignment.git)
    ```
2.  Build the workspace:
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select field_robotics_assignment
    source install/setup.bash
    ```
### Execution Commands

#### 1. Task 1: Custom Controller Demonstration

Launches the TurtleBot3 Waffle in an empty Gazebo world and starts the custom control node (`pose_circle_controller`).

```bash
ros2 launch field_robotics_assignment task1.launch.py
```
* **Test Commands (in a separate terminal):**
    * Pose-to-Pose Navigation: Move to position x=2.0 (m),y=1.0 with heading θ=0 (rad).
  ```bash
  ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'odom'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
  ```
    * Circular Trajectory Tracking: Track a circle with radius 2.0 m, center at (0,0), and angular speed 1.0 rad/s (counter-clockwise motion).
  ```bash
  ros2 topic pub /circle_cmd std_msgs/Float32MultiArray "data: [0.0, 0.0, 2.0, 1.0]"  ```
#### 2. Task 2 & 3: Autonomous Exploration
Launches the simulation, SLAM Toolbox, Nav2, and the custom frontier_explorer node for full autonomous operation.
  ```bash
  ros2 launch field_robotics_assignment exploration.launch.py
```
  * The system autonomously builds a map online while navigating safely.
  * The frontier_explorer node analyzes the map, detects unexplored frontiers, and automatically sends navigation goals to Nav2 using a greedy nearest-frontier strategy.

### 🔑 Key Control Law

The Pose-to-Pose control law computes linear (v) and angular (ω) velocities based on:
  * ρ: distance to the target
  * α: heading error toward the goal
  * β: final orientation error
```bash
v=kρ​⋅ρwhere kρ​=0.5 (linear gain)
w​=kα​⋅α+kβ​⋅β where kα​=1.0,kβ​=−0.3 (angular gains)​
```
### 📹 Video Demonstrations
* Task 1: Control System (Pose and Circular Tracking)
.[Watch Task1 Demonstration Video](https://www.youtube.com/watch?v=VpNzVZ7LGwg)

* Task 3: Autonomous Frontier-Based Exploration
.[Watch Task3 Demonstration Video](https://www.youtube.com/watch?v=lAMUuS6fOls)

### Future Work
* Improve frontier selection with clustering and information gain metrics
* Integrate exploration objectives into Nav2 local planning
* Explore adaptive SLAM–exploration strategies and learning-based approaches

## 📜 License
This project is for academic and research purposes.
Feel free to use and adapt it with proper citation.
