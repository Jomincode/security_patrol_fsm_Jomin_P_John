# Autonomous Security Patrol Robot (ROS 2 Humble / Gazebo)

A deterministic Finite State Machine (FSM) implementation for an autonomous security patrol robot using a TurtleBot3 Waffle in Gazebo.

## Repository Contents
* `security_patrol_fsm.py`: ROS 2 Humble node implementing the multi-state reactive controller (`PATROLLING`, `OBSTACLE_AVOIDANCE`, `ALERT`).

## Subscribed Topics
* `/scan` (`sensor_msgs/msg/LaserScan`): 360° planar LiDAR point ranges.
* `/imu` (`sensor_msgs/msg/Imu`): 3-axis linear acceleration monitoring.

## Published Topics
* `/cmd_vel` (`geometry_msgs/msg/Twist`): Differential drive wheel velocity commands.
* `/robot_alert` (`std_msgs/msg/String`): Critical incident notifications on kinetic threshold violation (|a| > 12.0 m/s²).

## Execution Instructions
In a sourced ROS 2 Humble workspace with TurtleBot3 Gazebo packages installed:
```bash
python3 security_patrol_fsm.py
