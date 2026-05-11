# Unitree G1 Pose Estimation via ROS2 and Forward Kinematics

## Overview

This project implements a real-time pose estimation system for the Unitree G1 humanoid robot using ROS2, IMU data, and joint states obtained from the `/lowstate` topic.

The system estimates and visualizes:

- Sagittal plane posture (side view)
- Coronal plane posture (front view)
- Approximate Center of Mass (COM)
- Global postural stability state
- IMU orientation (Roll, Pitch, Yaw)

The implementation is entirely developed in Python using:

- ROS2 Jazzy
- Unitree ROS2 SDK
- NumPy
- Matplotlib

The project performs a simplified forward kinematics reconstruction of the humanoid skeleton based on the joint angles published by the robot.

---

# Features

## Real-Time Skeleton Estimation

The software reconstructs the robot posture in:

### Sagittal Plane
Displays:

- Torso inclination
- Hip flexion
- Knee flexion
- Arm motion
- Forward/backward leaning

### Coronal Plane
Displays:

- Lateral body inclination
- Hip roll
- Shoulder alignment
- Side balance

---

# Center of Mass (COM)

The system estimates an approximate Center of Mass (COM) from the reconstructed skeleton joints.

The COM is displayed:

- Graphically on the sagittal view
- Numerically in the CLI

The COM helps evaluate:

- Static balance
- Body leaning
- Instability conditions
- Weight transfer during posture changes

---

# Postural Stability Estimation

The software classifies the robot posture into:

| State | Description |
|---|---|
| STABLE | Small roll/pitch angles |
| SLIGHT IMBALANCE | Moderate inclination |
| CRITICAL IMBALANCE | High inclination / fall risk |

The estimation is based on IMU orientation:

- Roll
- Pitch

---

# CLI Telemetry

The terminal continuously displays:

- Message counter
- Roll angle
- Pitch angle
- Yaw angle
- COM coordinates
- Estimated posture state

Example:

```text
================================================
Messages Received : 89989
Postural State    : STABLE
Roll              : 1.25 deg
Pitch             : -0.74 deg
Yaw               : -62.75 deg
COM X             : 0.009 m
COM Y             : 0.670 m
================================================
````

---

# GUI Visualization

The GUI displays:

## Sagittal Plane

* Side-view skeleton
* COM position
* Ground reference
* Stability classification

## Coronal Plane

* Front-view skeleton
* Shoulder and hip alignment
* Lateral balance

---

# Requirements

## Operating System

* Ubuntu 24.04 recommended

## ROS2

* ROS2 Jazzy

## Python

* Python 3.12+

---

# Dependencies

Install required Python packages:

```bash
pip install numpy matplotlib
```

---

# ROS2 Workspace

The following Unitree ROS2 packages are required:

* `unitree_hg`
* `unitree_go`
* `unitree_api`

---

# Environment Setup

Source your ROS2 workspace:

```bash
cd ~/unitree_ros2
source install/setup.bash
```

---

# Running the Program

Execute:

```bash
python3 g1_pose_estimator.py
```

---

# Expected Behavior

After execution:

* The robot telemetry starts updating
* Two graphical windows appear:

  * Sagittal plane
  * Coronal plane
* The COM marker is shown
* Stability state is continuously updated

---

# Experimental Suggestions

The following experiments can be performed:

## Static Stability Tests

* Robot standing still
* Small external pushes
* Controlled leaning

## Single-Leg Support

Lift one leg and observe:

* COM displacement
* Roll increase
* Stability classification changes

## Dynamic Motion Analysis

* Arm movement
* Squat motion
* Walking initialization

---

# Scientific Contributions

This project can support research in:

* Humanoid robotics
* Real-time pose estimation
* ROS2-based robot perception
* Low-cost biomechanical analysis
* Forward kinematics estimation
* Humanoid balance monitoring

---

# Possible Extensions

Future improvements may include:

* Full 3D visualization
* Kalman filtering
* Inverse kinematics
* ZMP (Zero Moment Point)
* Gait analysis
* Foot contact estimation
* Dynamic COM projection
* ROS2 RViz integration
* Deep learning posture classification

---

# Example Research Topics

Possible paper titles:

* "Real-Time Humanoid Pose Estimation Using ROS2 and Forward Kinematics on the Unitree G1"
* "Low-Cost Postural Stability Analysis for Humanoid Robots Using IMU and Joint States"
* "Forward Kinematics-Based Skeleton Reconstruction for the Unitree G1 EDU Platform"
* "ROS2-Based Real-Time Humanoid Pose Visualization and Stability Estimation"

---

# License

This project is intended for academic and research purposes.

---

# Author

Vitor Amadeu Souza

Research interests:

* Humanoid Robotics
* ROS2
* Reinforcement Learning
* Computer Vision
* Human-Robot Interaction
