# Design, Kinematic Analysis, and Prototyping of a Delta Robot 
## Project overview
2103322 (2026/1) MECHANICS OF MACHINERY
- **Project name:** Delta Robot
- **Purpose:** To investigate the kinematics of a 3D Delta parallel robot driven by motors on a fixed base, focusing on the link-and-joint relationships that govern the motion of the lower moving platform.
- **Team:** 

  Thanyarat Roongsitmeteekul 6730233721 (Physical prototype)

  Waratchanon Jaroenwong 6730466621 (Multibody simulation)

  Waritsara Jongsomboonpoka 6730472321 (Symbolic mathematics)
- **Status:** Planning
- **Last updated:** [2026-11-DD]

## Delta Robot

## Mechanism Design, Kinematic Analysis, and Prototyping
This repository contains the complete design and analysis workflow for a Delta Robot, a 3-DOF parallel manipulator driven by three revolute actuators through parallelogram linkages. The project integrates analytical kinematics, symbolic computation, multibody simulation, and hardware design.

## Part I: Kinematic Diagram and DoF analysis
The Delta robot has 3 degrees of freedom (3 DoF) and employs a parallel-kinematic manipulator mechanism.

[Click to open folder] [docs](docs/)

## Part II: Symbolic Math
Analysis of Position, Angle, Velocity, Acceleration, and Force using MATLAB.

Workspace volume (usable, theta in [15,170] deg) = 332.0 cm^3

<img width="625" height="272" alt="image" src="https://github.com/user-attachments/assets/e6227780-7c6d-4f35-9f7e-55ee09cd5e41" />

Total width approx = 229.5 mm 

<img width="625" height="376" alt="image" src="https://github.com/user-attachments/assets/23b1d2ed-b51f-4edb-b01d-192803e11118" />


Velocity check  max|J*thdot - dp/dt| = 3484.253 mm/s

Max torque = 38.4 N.mm (0.39 kg.cm), 3.6 % of stall

Max cond(J) = 1.46 (The closer infinity, the closer to a singularity)

<img width="625" height="486" alt="image" src="https://github.com/user-attachments/assets/cbfede25-a4af-4f28-b8bb-808b6e78283d" />

<img width="1600" height="900" alt="image" src="https://github.com/user-attachments/assets/8daf070b-0038-4514-a19e-c91c1f173d7e" />

Animation

<video src="./delta_robot_motion.mp4" controls width="100%"></video>

[Click to open folder] [software](software/)

[Click to open file] [Symbolic Math](software/Symbolic%20Math)

## Part III: Simscape Multibody 
Delta Robot Motion Simulation.

[Click to open folder] [simulation](simulation/)

## Part IV: Prototype
Design and Fabrication of a Delta Robot Prototype Using 3D Printing, with MG996R Servo Motor Installation.

[Click to open folder] [hardware](hardware/)

