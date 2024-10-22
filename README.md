# DC Motor Control with Raspberry Pi Pico and Micro-ROS

This repository presents a **C++ program** designed to control a **DC motor** using the **Raspberry Pi Pico** and **Micro-ROS**. It integrates motor control with encoder feedback, PWM speed control, and ROS2 communication for robotics and embedded systems.


## Prerequisites

Before working with the code, it is recommended to review the following **reStructuredText (reST)** documentation files:

- **[1_MotorClass_Doc.rst](1_MotorClass_Doc.rst)**: Detailed documentation of the `Motor` class, covering the code logic and functionality.
- **[2_MotorObject_uROS_Doc.rst](2_MotorObject_uROS_Doc.rst)**: Explanation of the `Motor` object and how it integrates with Micro-ROS.

## Getting Started

This program enables:
- Control of a DC motor’s speed and direction using **GPIO** and **PWM**.
- Real-time feedback on motor position and RPM through **encoder interrupts**.
- Integration with **ROS2** to receive speed commands and publish motor status.

### Features
- **Micro-ROS support**: Communicates with a ROS2 network using a Micro-ROS agent.
- **PWM-based motor control**: Allows precise speed adjustments.
- **ROS2 Topics**:
  - `cmd`: Receives speed setpoints.
  - `debug_pub`: Publishes motor position, RPM, and command status.

## How to Use

1. Review the reST documentation mentioned above to understand the program’s structure and flow.
2. Set up the **Raspberry Pi Pico** with the necessary GPIO connections for motor control and encoder feedback.
3. Ensure that the Micro-ROS agent is configured and running to communicate with the Pico.
4. It is necessary that after you clone the repository, you add the **Micro-ROS** libraries to the folder.

---

For more details, explore the reST documentation and the code provided in this repository.

## Presentation

**EIA University**, Mechatronical Eng. - Industrial Robotics Laboratory 
*Professor*: David Rozo Osorio M.Sc. email: david.rozo31@eia.edu.co
