Markdown

# 🚁 Warehouse Autonomous Drone System (ROS 2 + PX4)

[![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![PX4 Autopilot](https://img.shields.io/badge/PX4-v1.14-green.svg)](https://px4.io/)
[![Gazebo Garden](https://img.shields.io/badge/Gazebo-Garden-orange.svg)](https://gazebosim.org/)
[![Status](https://img.shields.io/badge/Status-Development-yellow.svg)]()

> **Project Magang R&D - PT Global Digital Niaga Tbk (Blibli)**
> *Pengembangan sistem navigasi drone otonom untuk pemetaan dan pengawasan gudang dalam ruangan tanpa GPS.*

---

## 📖 Daftar Isi
- [Tentang Proyek](#-tentang-proyek)
- [Arsitektur Sistem](#-arsitektur-sistem)
- [Fitur Utama](#-fitur-utama)
- [Struktur Folder](#-struktur-folder)
- [Prasyarat Instalasi](#-prasyarat-instalasi)
- [Cara Instalasi & Build](#-cara-instalasi--build)
- [Cara Menjalankan Simulasi](#-cara-menjalankan-simulasi)
- [Troubleshooting](#-troubleshooting)
- [Kontributor](#-kontributor)

---

## 🧐 Tentang Proyek

Repositori ini berisi *source code* lengkap untuk sistem kendali drone otonom yang dirancang khusus untuk lingkungan **Indoor Warehouse (Gudang)**. Sistem ini mengatasi tantangan lingkungan tanpa GPS (*GPS-denied*) dengan menggunakan sensor Lidar dan IMU untuk lokalisasi dan pemetaan (SLAM).

Proyek ini mengintegrasikan **ROS 2 Humble** (sebagai *High-Level Planner*) dengan **PX4 Autopilot** (sebagai *Low-Level Flight Controller*) melalui jembatan komunikasi **Micro XRCE-DDS**.

---

## 🏗 Arsitektur Sistem

Sistem ini terdiri dari tiga lapisan utama:
1.  **Simulation Layer (Gazebo):** Menjalankan *Digital Twin* gudang (`warehouse.sdf`) lengkap dengan rak dan rintangan fisik.
2.  **Flight Control Layer (PX4):** Menangani stabilitas penerbangan, pencampuran motor, dan estimasi status kendaraan.
3.  **Application Layer (ROS 2):** Menangani navigasi cerdas (Nav2), pemetaan (SLAM Toolbox), dan logika misi (`offboard_control`).

---

## ✨ Fitur Utama

* ✅ **Custom Warehouse Environment:** Simulasi dunia `warehouse.sdf` yang realistis dengan model rak gudang kustom.
* ✅ **GPS-Denied Navigation:** Navigasi otonom menggunakan Lidar 2D dan Odometry, tanpa ketergantungan pada satelit.
* ✅ **ENU-NED Coordinate Transform:** Node jembatan kustom (`cmd_sender.py`) yang menerjemahkan perintah ROS 2 ke standar penerbangan PX4 secara *real-time*.
* ✅ **Offboard Control Logic:** *State machine* otomatis untuk *arming*, *takeoff*, dan *failsafe* prevention.
* ✅ **Occupancy Grid Mapping:** Pemetaan gudang 2D secara *real-time* untuk mendeteksi lorong dan rintangan.

---

## 📂 Struktur Folder

Berikut adalah struktur direktori utama proyek setelah pembaruan terkini:

```text
warehouse-drone-ros2-px4/
├── src/
│   ├── px4_clean_nav/          # [CORE] Paket logika navigasi & bridge
│   │   ├── config/             # Parameter Nav2, SLAM, & Explore
│   │   ├── launch/             # File peluncuran simulasi & node
│   │   └── px4_clean_nav/      # Script Python (cmd_sender, offboard, dll)
│   ├── simple_explorer/        # Paket tambahan untuk eksplorasi
│   └── ...
│
├── PX4-Autopilot/              # [SUBMODULE] Firmware penerbangan PX4
│
├── Micro-XRCE-DDS-Agent/       # [MIDDLEWARE] Jembatan komunikasi DDS
│
├── worlds/
│   └── warehouse.sdf           # [BARU] File dunia simulasi gudang
│
├── models/
│   └── warehouse_rack/         # [BARU] Aset 3D untuk rak gudang
│
└── README.md                   # Dokumentasi proyek
# 🚁 Autonomous Indoor Warehouse Drone System

![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-blue?style=for-the-badge&logo=ros)
![PX4 Autopilot](https://img.shields.io/badge/PX4-v1.14-green?style=for-the-badge&logo=px4)
![Gazebo Garden](https://img.shields.io/badge/Gazebo-Garden-orange?style=for-the-badge&logo=gazebo)
![Python](https://img.shields.io/badge/Python-3.10-yellow?style=for-the-badge&logo=python)
![License](https://img.shields.io/badge/License-MIT-lightgrey?style=for-the-badge)

> **Final Internship Project - R&D Division**
> **PT Global Digital Niaga Tbk (Blibli)**
> *August 2025 – December 2025*

---

## 📋 Overview (Tinjauan Proyek)

This repository contains the complete source code and simulation environment for an **Autonomous UAV System** designed to operate in **GPS-Denied Indoor Environments** (specifically large-scale logistics warehouses).

The primary goal of this project was to solve the problem of manual stock counting in high-rack storage areas. By utilizing a custom-built software stack bridging **ROS 2 Humble** and **PX4 Autopilot**, this drone is capable of:
1.  **Self-Localization:** Using Lidar and IMU fusion (Odometry) to navigate without GPS.
2.  **Mapping:** Generating 2D Occupancy Grid Maps of the warehouse aisles in real-time.
3.  **Autonomous Navigation:** Planning paths around static (racks) and dynamic obstacles using the Nav2 stack.

This system has been validated in a High-Fidelity "Digital Twin" simulation of the Blibli Warehouse using Gazebo Garden.

---

## 🏗 System Architecture (Arsitektur Teknis)

The system operates on a distributed architecture to separate high-level logic from low-level flight control:

| Layer | Technology | Function |
| :--- | :--- | :--- |
| **Application Layer** | **ROS 2 Humble** | Handles Path Planning (Nav2), Mapping (SLAM Toolbox), and Mission Logic (`px4_clean_nav`). |
| **Middleware Layer** | **Micro XRCE-DDS** | Acts as a high-speed bridge transferring `uORB` topics from the flight controller to ROS 2 DDS topics. |
| **Flight Control Layer** | **PX4 Autopilot** | Handles motor mixing, attitude stabilization, and state estimation (EKF2). |
| **Simulation Layer** | **Gazebo Garden** | Simulates physics, gravity, collision (`warehouse_rack`), and sensors (2D Lidar). |

### 🔄 Coordinate Transformation Logic
One of the key innovations in this repo is the `cmd_sender.py` node, which solves the **ENU (ROS) to NED (PX4)** coordinate mismatch in real-time:
```python
# Real-time Vector Rotation Logic
vel_north = cmd_x * cos(yaw) - cmd_y * sin(yaw)
vel_east  = cmd_x * sin(yaw) + cmd_y * cos(yaw)

📂 Repository Structure
Plaintext

warehouse-drone-ros2-px4/
├── src/
│   ├── px4_clean_nav/          # [CORE] Custom package for Navigation Logic
│   │   ├── config/             # Configuration files (Nav2 params, SLAM, Explore)
│   │   ├── launch/             # Launch files (final_demo.launch.py, etc.)
│   │   └── px4_clean_nav/      # Python Nodes (cmd_sender, offboard_control)
│   ├── simple_explorer/        # [EXTRA] Autonomous frontier exploration logic
│   └── ...
│
├── PX4-Autopilot/              # [SUBMODULE] The Flight Control Firmware
├── Micro-XRCE-DDS-Agent/       # [SUBMODULE] Communication Bridge
│
├── worlds/
│   └── warehouse.sdf           # [ASSET] Custom Blibli Warehouse World
├── models/
│   └── warehouse_rack/         # [ASSET] 3D Models for Warehouse Racks
│
└── README.md                   # This documentation

🛠 Installation Guide (Panduan Instalasi)
Prerequisites

    OS: Ubuntu 22.04 LTS (Jammy Jellyfish)

    Framework: ROS 2 Humble

    Simulator: Gazebo Garden

Step 1: Clone the Repository

Use the --recursive flag to ensure PX4 and DDS Agent submodules are downloaded.
Bash

git clone --recursive [https://github.com/plovaxsz/warehouse-drone-ros2-px4.git](https://github.com/plovaxsz/warehouse-drone-ros2-px4.git)
cd warehouse-drone-ros2-px4

Step 2: Build Middleware (DDS Agent)

The agent is required to translate ROS 2 messages to PX4 uORB messages.
Bash

cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/

Step 3: Setup PX4 Autopilot Simulation

Build the SITL (Software In The Loop) firmware.
Bash

cd ../../PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
make px4_sitl

(Note: If the build finishes, press Ctrl+C to exit).
Step 4: Build ROS 2 Packages

Compile the custom navigation packages.
Bash

cd ..
colcon build --symlink-install
source install/setup.bash

🚀 How to Run the Simulation

You will need 3 separate terminals to run the full stack.
TERMINAL 1: The Communication Bridge

This starts the DDS Agent to listen on UDP port 8888.
Bash

MicroXRCEAgent udp4 -p 8888

TERMINAL 2: The Simulation Physics (Gazebo + PX4)

This launches the drone in the custom warehouse environment.
Bash

cd PX4-Autopilot
# Ensure warehouse.sdf is copied to PX4 worlds directory or set GZ_SIM_RESOURCE_PATH
make px4_sitl gz_x500_lidar

TERMINAL 3: The "Brain" (ROS 2 Navigation)

This launches Nav2, SLAM, Rviz, and the Offboard Control logic.
Bash

cd warehouse-drone-ros2-px4
source install/setup.bash
ros2 launch px4_clean_nav final_demo.launch.py

🔧 Troubleshooting & Tips

    Issue: "Zero poses in plan" error in Nav2.

        Fix: Ensure allow_unknown: true is set in nav2_params.yaml. The planner initially doesn't know where it can fly, so it needs permission to explore unknown space.

    Issue: Drone drifts when hovering.

        Fix: The offboard_control.py node includes a "Position Hold" logic. Ensure the node is running and receiving odom data.

👨‍💻 Author & Maintainer

Josh Abraham Efendi (plovaxsz)

    Role: AI & Robotics Intern at Blibli R&D

    Institution: President University (Informatics)

    Email: josh.efendi@student.president.ac.id

    GitHub: github.com/plovaxsz

Built with ❤️ using ROS 2 and PX4.

Dibuat untuk keperluan laporan magang President University x Blibli.
