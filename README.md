# Serial Robot Toolkit: Kinematics, Dynamics & ROS

This repository provides a structured and educational explanation of serial robot modeling, control, and simulation — covering everything from the mathematical foundations to practical ROS applications. It’s designed as both a learning reference and a hands-on toolkit for anyone studying or developing robotic systems.

## 🧠 Overview

This project walks through the essential components of serial manipulator analysis and control, including:

Forward and Inverse Kinematics

Differential Kinematics and the Jacobian

Cartesian Control Principles

Robot Dynamics Formulation

ROS-based Simulations

Each topic is explained clearly with supporting equations, Python/Matlab snippets, and visualization examples (when applicable).

## ⚙️ 1. Serial Robot Kinematics
Forward Kinematics

Derivation of end-effector position and orientation using Denavit–Hartenberg (DH) parameters.

Homogeneous transformation matrices and the relationship between frames.

Examples for 2-DOF, 3-DOF, and 6-DOF arms.

Inverse Kinematics

Analytical and numerical methods for solving joint variables given a desired pose.

Handling redundancy and singularities.

## 🔁 2. Differential Kinematics

Derivation of the Jacobian matrix for mapping joint velocities to end-effector linear and angular velocities.

Discussion on Jacobian singularities, velocity propagation, and manipulability.

## 🎯 3. Cartesian Control

Implementation of control strategies in Cartesian space rather than joint space.

Position and velocity control in end-effector coordinates.

Example: controlling the end-effector trajectory along a desired path.

## ⚡ 4. Robot Dynamics

Derivation of dynamic equations using the Lagrangian method.

Computation of kinetic and potential energy for a serial manipulator.

Torque control and inverse dynamics for motion control and simulation.

## 🤖 5. ROS Simulations

Integration of the theoretical models into ROS and Gazebo environments.

Visualization in RViz and simulation of kinematic and dynamic control.

Example launch files, URDF models, and controller implementations.
