<h1 align="center">CERBRUS</h1>
<p align="center">
  <b>An Open-Source, Bio-Inspired Quadruped Robot</b><br>
  <i>Equipped with a Front-Mounted Gripper Inspired by Ant Morphology</i>
</p>

<p align="center">
  <img src="Cerbrus Logo.png" alt="CERBRUS Logo" width="80%">
</p>

## 🧠 Overview

**CERBRUS** is a fully open-source quadruped robot platform designed with inspiration from the morphology of ants. It features:
- A **head-mounted 2-DOF gripper** (mimicking ant mandibles),
- Stable **3-DOF legs**,
- A **custom ROS2 control stack**,
- Real-time **PID-based balance correction**, and
- A **touchscreen joystick controller** with wireless telemetry.

This robot is designed for navigating and interacting with cluttered environments, with potential applications in **search and rescue**, **space robotics**, and **agriculture**.

---

## 🔩 Features

- 🦿 Bio-inspired mechanical design with 3-DOF legs and a 2-DOF gripper
- 🤖 Modular ROS2-based control architecture
- ⚖️ PID-based posture stabilization (pitch/roll)
- 📷 Onboard vision + pose detection (MoveNet)
- 🎮 Wireless touchscreen controller (custom interface)
- 📡 HC-12-based telemetry & real-time command transmission
- 🔧 Developer Mode for SSH and live ROS debugging


---

### 🖥️ Requirements

- ROS2 Jazzy on Ubuntu 24.04 (Robot)
- Raspbian OS (Controller)
- Raspberry Pi 5 (x2), Arduino Mega/Nano
- 12 Servos (legs), 2 SG90 (gripper)
- BNO055 IMU, HC-12 Modules, 3S LiPo Battery

### 🧰 Robot Setup

1. Clone the repo.
2. Create a new ROS ws folder and copy the src folder to the workspace.
3. Build the workspace.
4. Read the report linked below to understand various aspects as well as the user guidelines.

[Cerbrus Report](https://drive.google.com/file/d/167SGrlnD2wC8xEmvnSjF_YPM4pek4O9T/view?usp=sharing)

[CAD Files](https://drive.google.com/drive/folders/1i38UzL1JV2BpZAZQdjU2TJO_qoiLwZAB?usp=sharing)

