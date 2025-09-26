# ROS2 Line Follower

## Overview
This project is a simple ROS2 implementation of a Gazebo-simulated robot that follows a line using computer vision techniques in Python. It's designed for those interested in robotics, simulation, and autonomous navigation systems.

## Features
- **ROS2 Integration:** Leverages the power of ROS2 for messaging and system management.
- **Gazebo Simulation:** Utilizes the Gazebo simulator for a realistic robotics simulation environment.
- **Computer Vision:** Employs Python-based computer vision algorithms to detect and follow lines.

## Prerequisites
- Ubuntu 22.04 LTS
- ROS2 Humble
- Gazebo simulator
- Python 3.6 or newer
- OpenCV for Python

## Installation
1. **Install ROS2:** Follow the official ROS2 documentation to install ROS2 on your system.
2. **Install Gazebo:** Ensure Gazebo is installed and properly configured with ROS2.
3. **Clone the Repository:** Clone this repository into your system.
5. **Build the Workspace:** Use `colcon build` to build your ROS2 workspace.

## Running the Simulation
1. **Launch Gazebo:** Start the Gazebo simulator with the provided world file.
2. **Run the Line Follower Node:** Execute the line follower node to start the robot.
3. **Visualize in RViz:** Optionally, use RViz to visualize the robot's sensors and path.

Run through "ros2 launch line_follower_bringup gazebo.launch.py"
