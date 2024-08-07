# TurtleBot3 Reinforcement Learning Project

This project demonstrates the implementation of a Q-learning algorithm enhancement to control a TurtleBot3 robot. The robot navigates different environments simulated in Gazebo, learns to reach target positions from various initial conditions, and avoids obstacles using reinforcement learning.

## Table of Contents
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Setup](#setup)
- [Usage](#usage)
  - [Empty World](#empty-world)
  - [House](#house)
  - [World](#world)
- [Data Availability](#data-availability)
- [Conclusion](#conclusion)

## Prerequisites
Ensure you have the following installed:
- ROS (Robot Operating System)
- Gazebo
- TurtleBot3 packages

## Installation

1. **Install TurtleBot3 Gazebo package:**
    ```bash
    sudo apt-get install ros-$ROS_DISTRO-turtlebot3-gazebo
    ```

2. **Set TurtleBot3 Model:**
    ```bash
    export TURTLEBOT3_MODEL=waffle
    ```

## Setup

1. **Clone the repository:**
    ```bash
    cd ~/catkin_ws/src
    git clone https://github.com/AliiRezaei/turtlebot3_rl.git
    ```

2. **Build the workspace:**
    ```bash
    cd ~/catkin_ws
    catkin_make
    ```

## Usage

1. **Start ROS master:**
    ```bash
    roscore
    ```

2. **Launch the simulation environments and run the learning nodes.**

### Empty World

1. **Launch Empty World in Gazebo:**
    ```bash
    roslaunch turtlebot3_rl turtlebot3_env_empty_world.launch
    ```

2. **Run the Empty World learning node:**
    ```bash
    rosrun turtlebot3_rl learning_empty_world
    ```

3. **Wait for the robot to complete learning and observe the results.**

### House

1. **Launch House in Gazebo:**
    ```bash
    roslaunch turtlebot3_rl turtlebot3_env_house.launch
    ```

2. **Run the House learning node:**
    ```bash
    rosrun turtlebot3_rl learning_house
    ```

3. **Wait for the robot to complete learning and observe the results.**

### World

1. **Launch World in Gazebo:**
    ```bash
    roslaunch turtlebot3_rl turtlebot3_env_world.launch
    ```

2. **Run the World learning node:**
    ```bash
    rosrun turtlebot3_rl learning_world
    ```

3. **Wait for the robot to complete learning and observe the results.**

## Data Availability
Learning data is logged in the `LogData` directory:
- **Empty World:** 50,000 episodes
- **House:** 30,000 episodes
- **World:** 25,000 episodes

## Conclusion
This project showcases the efficiency of a Q-learning algorithm implemented in C++ for controlling the TurtleBot3 robot. The robot demonstrates significant learning performance improvements, achieving training in approximately 30 seconds compared to 13 hours with MATLAB.
For campare the MATLAB and C++ performance, run the MATLAB related script from `MATLAB Scripts` directory.