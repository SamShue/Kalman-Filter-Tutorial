# Kalman-Filter-Tutorial

This repository contains a MATLAB implementation of a 2D Kalman filter for tracking the position and velocity of a robot in a 2D space using accelerometer and GPS data.

## Introduction

The Kalman filter is a widely used mathematical algorithm for estimating the state of a linear dynamic system from a series of noisy measurements. In this project, we apply the Kalman filter to track the position and velocity of a robot moving in a 2D space. The filter combines data from an accelerometer and GPS to provide a more accurate estimate of the robot's state.

## File Structure

- `SIMULATION_PositionTracking2d_Functionalized.m`: This is the main MATLAB script that contains the Kalman filter implementation. It processes the accelerometer and GPS data, estimates the state of the robot, and visualizes the tracking results.

- `GaussianDistExamples.m`: This script will plot an example of DC signals perturbed by 0-mean Gaussian noise and the PDF used to generate the noise. This is script is simply to help visualize what Gaussian noise looks like with an array of variances.

- `SensorFusionWithKalmanFilters.pdf`: This PDF contains slides which describe the Kalman filter in a simple way using the model in the simulation script.

## Dependencies

This code requires MATLAB to run. Make sure you have MATLAB installed on your system.

## Usage

1. Clone the repository to your local machine:

   ```bash
   git clone https://github.com/SamShue/2D-Position-Tracking-Kalman-Filter.git

2. Open MATLAB and navigate to the project directory.

3. Open the SIMULATION_PositionTracking2d_Functionalized.m script in MATLAB.

4. Execute the script by running it in MATLAB.

The script will process the provided data and display a plot showing the estimated position and velocity of the robot.