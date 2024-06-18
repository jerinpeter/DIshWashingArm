# Plate Cleaning Robot using ABB IRB 120

This project implements a plate cleaning robot utilizing the ABB IRB 120 robotic arm. The robot mimics human dishwashing motions through a sophisticated spiral trajectory, ensuring thorough cleaning of plates placed randomly within a predefined workspace. The system leverages numerical inverse kinematics (IK) and Proportional-Derivative (PD) control strategies for precise end-effector positioning and trajectory following.

## Table of Contents
1. [Introduction](#introduction)
2. [System Overview](#system-overview)
3. [Control and Motion Planning](#control-and-motion-planning)
4. [Trajectory Generation](#trajectory-generation)
5. [Experimental Setup and Results](#experimental-setup-and-results)
6. [Conclusion](#conclusion)
7. [Acknowledgment](#acknowledgment)
8. [References](#references)
9. [Appendix](#appendix)

## Introduction
The automation of routine tasks, such as dishwashing, has seen significant advancements with the integration of robotic systems. This project employs the ABB IRB 120 robotic arm to perform dishwashing tasks by executing a spiral motion to clean plates, emulating human-like washing techniques.

## System Overview
The robotic system comprises the following components:
- **ABB IRB 120 robotic arm**: The primary hardware used for the project.
- **Inverse Kinematics (IK)**: Numerical IK methods are used to determine the required joint angles for specific end-effector positions and orientations.
- **Task Space Definition**: The workspace boundaries are defined to ensure safe operation.
- **Plate Position Generation**: Random plate positions are generated within the task space, ensuring no overlap and a minimum distance from the robot’s base.

## Control and Motion Planning
The robot’s motion planning involves two primary phases:
1. **Spiral Trajectory for Cleaning**: A spiral trajectory is generated above each plate to simulate the cleaning motion.
2. **Transition Trajectory**: Smooth trajectories are generated for transitioning between plates, avoiding collisions with the robot’s base and other obstacles.

## Trajectory Generation
### Cleaning Trajectory
The spiral trajectory for cleaning is generated using a parametric approach, where the radius increases linearly with the angle to form a spiral.

### Transition Trajectory
Transitioning between plates involves generating smooth trajectories using a cubic polynomial trajectory planning method to ensure smooth acceleration and deceleration phases.

## Experimental Setup and Results
### Initial Setup
The robot starts at the origin with a zero joint configuration. Plates are placed randomly within the task space, ensuring compliance with spacing constraints.

### Simulation Execution
The simulation involves the robot identifying the nearest unvisited plate, executing the spiral cleaning motion, and transitioning to the next plate. This loop continues until all plates are cleaned.

### Visualization
The simulation provides real-time visualization of the robot’s movements, including the spiral trajectories and transitions.

## Conclusion
The robotic system successfully demonstrates the capability to perform dishwashing tasks through a controlled spiral motion. Future work may include optimizing the control parameters and exploring advanced trajectory planning algorithms to enhance the system’s performance.

## Acknowledgment
I would like to express my gratitude to Professor Jonathan Realmutofor his exceptional teaching in the Kinematics and Dynamics (ME221) course. His insightful instruction provided the foundational understanding and intuition necessary for the concepts utilized in this project.

## References
1. MathWorks, “Inverse Kinematics - MATLAB Simulink,” [Link](https://www.mathworks.com/help/robotics/ref/inversekinematics.html).
2. MathWorks, “Analytical Solutions of the Inverse Kinematics,” [Link](https://www.mathworks.com/help/robotics/ug/solve-closed-form-inverse-kinematics.html).
3. MathWorks, “Trajectory Control Modeling with Inverse Kinematics,” [Link](https://www.mathworks.com/help/robotics/ref/cubicpolytraj.html).
4. MathWorks, “Implement PID Control,” [Link](https://www.mathworks.com/help/control/examples/implement-pid-control.html).
5. MathWorks, “Constraint Objects,” [Link](https://www.mathworks.com/help/robotics/ug/constraint-objects.html).
6. MathWorks, “Model and Control a Manipulator Arm with Robotics and Simscape,” [Link](https://www.mathworks.com/help/robotics/ug/model-and-control-a-manipulator-arm.html).
7. MathWorks, “Multi-Loop PI Control of a Robotic Arm,” [Link](https://www.mathworks.com/help/control/ug/multi-loop-pi-control-of-a-robotic-arm.html).
8. MathWorks, “Connect to Kinova Gen3 Robot and Manipulate the Arm Using MATLAB,” [Link](https://www.mathworks.com/help/robotics/ug/connect-to-kinova-gen3-robot-and-manipulate-the-arm-using-matlab.html).
9. MathWorks, “Modeling Inverse Kinematics in a Robotic Arm,” [Link](https://www.mathworks.com/help/robotics/ug/modeling-inverse-kinematics-in-a-robotic-arm.html).
10. MathWorks, “Sliding Mode Control Design for a Robotic Manipulator,” [Link](https://www.mathworks.com/help/robotics/ug/sliding-mode-control-design-for-a-robotic-manipulator.html).
11. MathWorks, “Perform Safe Trajectory Tracking Control Using Robotics Manipulator Blocks,” [Link](https://www.mathworks.com/help/robotics/ug/safe-trajectory-tracking-control.html).
