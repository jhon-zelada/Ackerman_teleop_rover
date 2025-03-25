# Ackerman_teleop_rover
Keyboard and Joystick control of a 6 wheeled rover with Ackermann configuration
Ackerman Teleop Rover 🚗🎮

This ROS 2 package provides teleoperation capabilities for a 6-wheeled Ackerman-style rover using both a keyboard and a joystick. It includes position and velocity controllers for various joints, including the boogie system, angular control, wheel velocity control, and a LIDAR velocity controller.

## 📌 Features

Keyboard and Joystick Control: Allows seamless teleoperation using either a keyboard or a game controller.

Ackerman Steering: Implements position and velocity controllers for precise movement.

ROS 2 Controllers:

    Joint position controllers for boogie and angular joints.

    Velocity controllers for wheel movement and LIDAR rotation.

Customizable Gains: PID tuning for optimized performance.

## 🚀 Installation

Clone the repository and build the package within a ROS 2 workspace:

    mkdir -p ~/ros2_ws/src 
    cd ~/ros2_ws/src  
    git clone https://github.com/jhon-zelada/Ackerman_teleop_rover.git  
    cd ~/ros2_ws  
    colcon build  
    source install/setup.bash  

## 🎮 Usage
Keyboard Control

    ros2 run rover_control teleop_keyboard_ackermann_executable
Joystick Control

    ros2 run rover_control teleop_joystick_ackermann_executable

## 📖 Configuration

The controllers are defined in config/teleop_controllers.yaml, where you can modify joint names, control modes, and PID gains to match your hardware setup.

## 🛠 Dependencies

Ensure you have the following ROS 2 packages installed:

    ros2_controllers

    joint_state_broadcaster

    teleop_twist_keyboard

    joy (for joystick support)

## 🤖 Future Improvements

    Implement autonomous navigation features.

    Optimize Ackerman kinematics for improved handling.

    Add support for additional joystick configurations.