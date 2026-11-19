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
<p align="center">
<img width="600" height="500" alt="S__29188100_0" src="https://github.com/user-attachments/assets/e888013e-174d-4910-b518-cbc404ddf0c8" />
</p>

## Mechanism Design, Kinematic Analysis, and Prototyping
This repository contains the complete design and analysis workflow for a Delta Robot, a 3-DOF parallel manipulator driven by three revolute actuators through parallelogram linkages. The project integrates analytical kinematics, symbolic computation, multibody simulation, and hardware design.

## Part I: Kinematic Diagram and DoF analysis
The Delta robot has 3 degrees of freedom (3 DoF) and employs a parallel-kinematic manipulator mechanism.

[Click to open folder] [docs](docs/)

## Part II: Symbolic Math
Analysis of Position, Angle, Velocity, Acceleration, and Force using MATLAB.

Workspace volume (usable, theta in [15,170] deg) = 332.0 cm^3
<p align="center">
<img width="625" height="272" alt="image" src="https://github.com/user-attachments/assets/e6227780-7c6d-4f35-9f7e-55ee09cd5e41" />
</p>
Total width approx = 229.5 mm 

<p align="center">
<img width="625" height="376" alt="image" src="https://github.com/user-attachments/assets/23b1d2ed-b51f-4edb-b01d-192803e11118" />
</p>

Velocity check  max|J*thdot - dp/dt| = 3484.253 mm/s

Max torque = 38.4 N.mm (0.39 kg.cm), 3.6 % of stall

Max cond(J) = 1.46 (The closer infinity, the closer to a singularity)

<p align="center">
<img width="625" height="693" alt="image" src="https://github.com/user-attachments/assets/edff152e-b785-4e8c-92b9-fa34219badd9" />

<img width="1600" height="900" alt="image" src="https://github.com/user-attachments/assets/8daf070b-0038-4514-a19e-c91c1f173d7e" />
</p>

Animation

https://github.com/user-attachments/assets/49e93c41-96c3-493c-aa9b-d0f54fe8aff5

[Click to open folder] [software](software/)

[Click to open file] [Symbolic Math](software/Symbolic%20Math)

## Part III: Simscape Multibody
Delta Robot motion simulation in Simscape Multibody: reachable workspace, a circular trajectory driven by inverse kinematics, and the angle, angular velocity, angular acceleration and torque that each servo joint must deliver.

**Actuator:** `Clock → circle_IK (MATLAB Function) → 3 servo revolute joints`. The block computes the effector position on the path at time *t*, then solves the inverse kinematics (A cos θ + B sin θ = K) to get the three servo angles. The motion parameters are set in one file, `params_delta.m`.

| Geometry (measured from the model) | Value |
|---|---|
| Servo arm length L1 | 40.0 mm |
| Rod length L2 | 82.0 mm |
| Base radius f / Effector radius e | 40.72 mm / 29.64 mm |

### Workspace
Usable full circle r = 43 mm for z = −59 … −105 mm below the servo shafts (best height z = −81 mm), vertical stroke 72 mm.

<p align="center">
<img width="625" alt="Multibody workspace 3D" src="simulation/multibody/results/workspace/workspace_3d.png" />
<img width="450" alt="Top view at best height" src="simulation/multibody/results/workspace/top_view_best_height.png" />
</p>

### Circular motion
Home → straight line → circle R = 30 mm at z = −81 mm (2 laps in 4 s) → home, 7.3 s total, minimum-jerk profile s = 10x³ − 15x⁴ + 6x⁵ (velocity and acceleration start and end at zero).

| Servo joint | Servo 1 | Servo 2 | Servo 3 |
|---|---|---|---|
| Angle θ (below horizontal) | 2.5 … 80.6° | 2.5 … 80.6° | 2.5 … 80.6° |
| Max angular velocity | 157.4 °/s | 151.8 °/s | 150.0 °/s |
| Max angular acceleration | 892 °/s² | 937 °/s² | 787 °/s² |
| Max torque (no payload) | 22.5 mN·m | 23.0 mN·m | 22.0 mN·m |

Effector max speed 176.7 mm/s, max acceleration 1041 mm/s², path deviation ≤ 0.71 mm.
Simscape joint results agree with the analytical velocity/acceleration analysis (θ̇ = sᵀẊ / sᵀC′) within 0.63°, 3.8 °/s and 31 °/s².

<p align="center">
<img width="625" alt="Servo angle" src="simulation/multibody/results/circle/2_servo_angle.png" />
<img width="625" alt="Servo angular velocity" src="simulation/multibody/results/circle/3_servo_velocity.png" />
<img width="625" alt="Servo angular acceleration" src="simulation/multibody/results/circle/4_servo_acceleration.png" />
<img width="625" alt="Servo torque" src="simulation/multibody/results/circle/5_servo_torque.png" />
</p>

Animation

https://github.com/user-attachments/assets/072f497b-4779-4905-a7a6-2cdd2447a263


[Click to open folder] [simulation](simulation/)

[Click to open file] [Multibody README](simulation/README.md)

## Part IV: Prototype
Design and Fabrication of a Delta Robot Prototype Using 3D Printing, with MG996R Servo Motor Installation.

Prototype
<p align="center">
<img width="750" height="750" alt="S__29188105_0" src="https://github.com/user-attachments/assets/00d800ea-82e6-425f-8161-ee607eed0dff" />

<img width="750" height="750" alt="S__29188104_0" src="https://github.com/user-attachments/assets/a291d17b-46d2-44cd-8956-7c7e5269bbc7" />
</p>
Video prototype

https://github.com/user-attachments/assets/066ec897-29dd-44cb-ad2d-f1037f18571c

[Click to open folder] [hardware](hardware/)
