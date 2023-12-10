# Bellows Arm Control Package

## Overview

This repository contains the code for controlling a pneumatic bellows arm designed and manufactured in the BYU RaD Lab. Manual control is facilitated through an Xbox controller, with the left and right joysticks governing the xy and yz planes. The Xbox controller commands update the goal position, which is then converted to desired joint angles through inverse kinematics. A PID controller, developed in the RaD Lab, transforms joint angles into pressures for each bellows in the robot.

## Dependencies

This package requires the following packages from the RaD Lab (not publicly available):
- bellows_arm
- rad_models
- rad_msgs

## Usage

The actual joint angles are recorded and published by the Vive motion capture system. Pressure control is executed on a BeagleBone, which serves as the ROS_MASTER.

## Project Background

This project was the final submission for ME EN 537: Robotics at BYU. The objective was to control a new pneumatic bellows arm developed in the BYU RaD Lab using inverse kinematics. The Xbox controller sent commands to update the goal position for the end effector.

## Package Components

- Created a CMake package, available in the GitHub repository
- Interfaced with various other packages, including:
  - Vive motion control for capturing raw position data and translating it into joint angles
  - Low-level PD Controller for managing pressures sent to each bellow
  - A fork of a package for forward simulating a different bellows arm, edited to represent the dynamics of the worked-on arm

## Validation and Simulation

The project began with a basic simulation in RVIS to visualize the current and target positions for the end effector. Key validations included ensuring proper interpretation of Xbox controller commands, correct goal position updating through inverse kinematics, and accurate visualization of the current position using forward kinematics. The machine successfully communicated with the ROS_MASTER.

## Hardware Implementation

After validating major components in simulation, the project transitioned to hardware for real-time control of the robot. Tuning the low-level PD controller on a new robot was challenging due to the inherent instability of soft robots. Subsequently, the inverse kinematics package was run on the arm, providing real-time control. While control speed is limited to prevent instability, achieving real-time control marked a significant milestone.

## Video and More Details

View the video demonstration and access additional project details in [my portfolio](https://sites.google.com/view/danielcheney/portfolio/ik-on-bellows-arm?authuser=0).
