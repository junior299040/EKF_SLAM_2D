# EKF-SLAM

## Overview
Implementation of a 2D Extended Kalman Filter SLAM (EKF-SLAM)
algorithm using MATLAB.

## Objectives
- Simulate a mobile robot navigating in a 2D environment.
- Estimate the robot pose using an Extended Kalman Filter.
- Detect and initialize new landmarks.
- Build and update a map of the environment.
- Evaluate the localization and mapping performance.

## Project Structure

```text
EKF-SLAM/
├── EKFSLAM2D.m
├── move.m
├── from_F_to_W.m
├── from_W_to_F.m
├── from_polar_to_cart.m
├── from_cart_to_polar.m
├── measurement_to_map.m
├── back_project_from_map.m
├── cov_to_ellipse.m
├── W.mat
└── README.md
