# manipulation_planner
This repository provides a ROS 2 package for right arm manipulation planning in Justina.

## Configuration
The right arm IK solver uses a simplified 2-link planar model with the following parameters:
* *L1*: 0.216 m 
* *L2*: 0.37 m 
* *Shoulder height: 1.07 m

The trajectory planner generates fixed intermediate waypoints and appends the requested target pose as the final waypoint.

## Description
The package contains two nodes:

### right_arm_ik_solver
Receives a target pose and computes the joint positions using an analytical inverse kinematics solution. If the target is outside the reachable workspace, it returns failure.

### trajectory_planner
Receives a target pose, generates a sequence of waypoints, and for each waypoint:
1. Calls the IK solver.
2. Publishes the resulting joint positions to the hardware topic.
3. Waits before proceeding to the next waypoint.
If any IK call fails, the trajectory execution stops.

## Interfaces

### right_arm_ik_solver
Service:
* `/solve_right_arm_ik` [`manipulation_planner_interfaces::srv::SolveRightArmIK`]

Request:
* `target_pose` [`geometry_msgs::msg::PoseStamped`]

Response:
* `joint_positions` [`float64[]`]
* `success` [`bool`]
* `message` [`string`]

### trajectory_planner

Service:
* `/plan_arm_trajectory` [`manipulation_planner_interfaces::srv::PlanTrajectory`]

Request:
* `target_pose` [`geometry_msgs::msg::PoseStamped`]

Response:
* `success` [`bool`]

Publishes:
* `/hardware/arms/goal_pose` [`std_msgs::msg::Float64MultiArray`]
