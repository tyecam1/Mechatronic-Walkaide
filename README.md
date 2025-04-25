Mechatronic Walking Aid for Visually Impaired Individuals
Project Overview
This project developed a wearable navigation aid to assist visually impaired individuals in navigating indoor environments autonomously. The system integrates environmental perception using an Intel RealSense RGB-D camera, ROS-based SLAM algorithms, and two distinct haptic feedback systems (compass-style rotary mechanism and shoulder strap pulling system) to guide users safely around obstacles.

Key Features
Environmental Perception: Uses Intel RealSense D435i camera for real-time 3D mapping and obstacle detection

Navigation System: Implements RTAB-Map for SLAM and Artificial Potential Field (APF) algorithm for path planning

Haptic Feedback:

Compass system with 6-way directional commands

Shoulder-pulling system for intuitive guidance

Modular Design: Wearable vest with adjustable straps for comfort

ROS Integration: Full system built on ROS Noetic for modularity and extensibility

System Architecture
System Block Diagram

Hardware Components
Processing Units:

LattePanda 3 Delta (perception system)

Raspberry Pi 4B (navigation system)

Sensors:

Intel RealSense D435i RGB-D camera

Arduino Nano 33 BLE (IMU)

Actuation:

FS90 micro servo motors for haptic feedback

Power: Dual-output 3000mAh power bank

Software Stack
Core Framework: ROS Noetic

Key Packages:

RTAB-Map for SLAM

Point Cloud Library (PCL) for obstacle processing

Hector SLAM (initial robot testing phase)

Custom Nodes:

Obstacle detection and clustering

APF navigation algorithm

System diagnostics monitoring

Performance Metrics
Tested across two environments with four participants:

Success Rate: Up to 83.25% with compass feedback

Collisions: 0.33-1.17 per test depending on feedback type

Reaction Time: 3.43-4.93 seconds

Localization Stability: Significant environmental impact observed

Getting Started
Prerequisites
Ubuntu 20.04

ROS Noetic

Python 3

Intel RealSense SDK

Installation
bash
# Clone repository
git clone https://github.com/yourusername/mechatronic-walking-aid.git
cd mechatronic-walking-aid

# Build ROS workspace
catkin_make
source devel/setup.bash
Running the System
bash
# Launch main system
roslaunch walkaide walkaide2.launch
Future Work
Integration of higher-torque actuators (DYNAMIXEL XC430-W150-T)

Addition of gimbal stabilization for camera

Implementation of secondary camera (RealSense T265)

Expanded testing with visually impaired participants

Team
Trystan Barnett (Electrical Systems)

Tye Cameron-Robson (Control Algorithms)

Samuel Griffin (Medical Applications)

Oscar Meads (System Integration)

Dylan Williams (Mechanical Design)

Supervised by Dr. Shou-Han Zhou

License
This project is licensed under the MIT License - see the LICENSE file for details.

Acknowledgments
Special thanks to Dr. Shou-Han Zhou for his guidance and support throughout this project.