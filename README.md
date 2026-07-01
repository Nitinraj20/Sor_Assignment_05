# ROS 2 Navigation and Arm Control

This repository contains two ROS 2 programs:

- `patrol_node` – Moves the mobile robot through a set of predefined waypoints.
- `arm_ik_mover` – Calculates inverse kinematics for a 4-DOF robotic arm and moves the arm to a target position.

## Requirements

- ROS 2 Jazzy
- Nav2
- ros2_control
- Joint Trajectory Controller

---

## Patrol Node

The patrol node sends a list of waypoints to the Nav2 waypoint follower. The robot visits each waypoint one by one until the patrol is complete.

### Run

```bash
ros2 run <package_name> patrol_node
```

To change the patrol path, edit the `WAYPOINTS` list in `patrol_node.py`.

---

## Arm IK Mover

The arm IK mover calculates the joint angles required to reach a given `(x, y, z)` position and sends the command to the arm controller.

### Run

```bash
ros2 run <package_name> arm_ik_mover <x> <y> <z> [duration_sec]
```

### Example

```bash
ros2 run <package_name> arm_ik_mover 0.30 0.00 0.20 3.0
```

### Parameters

| Parameter | Description |
|-----------|-------------|
| `x` | Target x position (m) |
| `y` | Target y position (m) |
| `z` | Target z position (m) |
| `duration_sec` | Time to reach the target (default: `3.0` s) |

---

## Author

Nitin Raj
