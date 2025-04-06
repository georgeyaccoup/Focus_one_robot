# 6-DOF Industrial Robotic Arm for Oil Canister Palletizing

This repository contains the complete implementation of an industrial 6 Degrees of Freedom (6-DOF) robotic arm designed to automate the process of transferring and sorting oil canisters (up to 500 kg total payload) from a production line to a packaging zone.

## Project Overview

The robotic arm is designed using a modular mechatronics approach integrating:

- Mechanical Modeling using SolidWorks  
- Stress and Load Analysis with ANSYS  
- Embedded Control using Raspberry Pi 5 and ESP32 microcontrollers  
- Motion Planning and Simulation using ROS 2, MoveIt, RViz, and Gazebo  
- Kinematic Modeling using MATLAB and Pinocchio  
- Electrical System Simulation with Proteus  

## Design Features

- Arm Height: 2 meters  
- Joints: 5 revolute + 1 prismatic  
- Payload Capacity: Up to 500 kg  
- End-Effector: Vacuum gripper and pneumatic actuator (35 cm stroke, 3 bar pressure)

## Components and Hardware

- Main Controller: Raspberry Pi 5  
- Joint Controllers: 6 × ESP32 microcontrollers (1 per joint)  
- Motor Types:
  - Joints 1–3: HDS-1600 Servo Motors  
  - Joints 4–5: NEMA 34 Closed-Loop Stepper Motors (20 Nm torque)  
  - Drivers: DM860T Stepper Drivers  
- Prismatic Joint: Pneumatic cylinder (SMC CQ2B series, 3 bar)  

## Software Stack

- ROS 2 (Humble Hawksbill) for robot control and node architecture  
- MoveIt for motion planning and inverse kinematics  
- RViz for visualization  
- Gazebo for real-time simulation  
- MATLAB for analytical kinematics and control  
- Pinocchio for rigid-body dynamics and torque estimation  
- Model Predictive Control (MPC) for accurate and smooth motion  
- Proteus for circuit simulation and testing  

## Simulation and Visualization

- ROS 2 node graph managed using rqt_graph  
- Motion planning visualized in RViz  
- Realistic behavior tested in Gazebo simulation environment  
- Joint communication with ESP32 via UART from Raspberry Pi 5  

## Project Files

- SolidWorks 3D design files  
- ANSYS mechanical analysis reports  
- Electrical wiring diagrams and Proteus simulations  
- ROS 2 workspace and ESP firmware  
- Complete project documentation (PDF)

Design Drive Folder:  
[Google Drive – Project Files](https://drive.google.com/drive/folders/1k2CcFMRKb08UxOz_LgReaP4hRir851lc?usp=drive_link)

## Torque Summary Table

| Joint       | Max Torque (Nm) | Motor Model            | Driver Model |
|-------------|------------------|-------------------------|--------------|
| Joint 1     | 1600             | HDS-1600 Servo Motor    | Integrated   |
| Joint 2     | 1100             | HDS-1600 Servo Motor    | Integrated   |
| Joint 3     | 850              | HDS-1600 Servo Motor    | Integrated   |
| Joint 4     | 300              | NEMA 34 Stepper Motor   | DM860T       |
| Joint 5     | 200              | NEMA 34 Stepper Motor   | DM860T       |
| Prismatic   | Pneumatic System | SMC CQ2B Cylinder       | Air Control  |

## Skills Demonstrated

- 3D Mechanical CAD Design  
- Embedded Systems Programming (ESP32, Raspberry Pi 5)  
- ROS 2 Real-Time Control and Architecture  
- Kinematics and Inverse Kinematics (MATLAB + Pinocchio)  
- MPC (Model Predictive Control)  
- Pneumatic Actuation and System Sizing  
- Electrical Testing and Simulation  
- Multi-environment Integration and Simulation  

## Author

**George Read**  
- Email: georgeyaccoup124@gmail.com  
- Phone: +20 103 016 7198  

## License

This project is licensed under the MIT License.
