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

🛠 Prasyarat Instalasi

Pastikan sistem Anda menggunakan Ubuntu 22.04 LTS dengan spesifikasi berikut terinstal:

    ROS 2 Humble Hawksbill

    Python 3.10

    Gazebo Garden

    Git & Build Essentials

⚙️ Cara Instalasi & Build

Ikuti langkah-langkah ini secara berurutan untuk menghindari error dependency.
1. Clone Repository (Recursive)

Kita perlu mengambil repositori ini beserta submodule PX4 dan DDS Agent di dalamnya.
Bash

git clone --recursive [https://github.com/USERNAME_ANDA/warehouse-drone-ros2-px4.git](https://github.com/USERNAME_ANDA/warehouse-drone-ros2-px4.git)
cd warehouse-drone-ros2-px4

2. Build Micro XRCE-DDS Agent

Agent ini wajib di-compile agar ROS 2 bisa "berbicara" dengan PX4.
Bash

cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/

3. Setup PX4 Autopilot

Pastikan dependensi PX4 terpenuhi.
Bash

cd ../../PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
make px4_sitl

(Tekan Ctrl+C jika simulasi PX4 sudah berhasil muncul pertama kali)
4. Build Workspace ROS 2

Kembali ke root folder dan build paket ROS 2.
Bash

cd ..
colcon build --symlink-install
source install/setup.bash

🚀 Cara Menjalankan Simulasi

Untuk menjalankan sistem penuh, Anda memerlukan 3 Terminal berbeda.
Terminal 1: Jalankan Micro XRCE-DDS Agent

Ini adalah "jembatan" data. Jalankan ini dulu.
Bash

MicroXRCEAgent udp4 -p 8888

Terminal 2: Jalankan Simulasi PX4 (Headless/GUI)

Menjalankan firmware PX4 dengan model x500 di dunia gudang kustom.
Bash

cd PX4-Autopilot
# Pastikan path world mengarah ke file warehouse.sdf yang baru ditambahkan
make px4_sitl gz_x500_lidar -j4

Catatan: Anda mungkin perlu menyalin warehouse.sdf ke folder Tools/simulation/gz/worlds di dalam direktori PX4 jika tidak terdeteksi otomatis.
Terminal 3: Jalankan Logika ROS 2 (Navigasi & Kontrol)

Menjalankan Nav2, SLAM, Rviz, dan Script Kontrol Offboard.
Bash

source install/setup.bash
ros2 launch px4_clean_nav final_demo.launch.py

🔧 Troubleshooting

Q: Drone tidak mau bergerak (Error: "Zero poses in plan")?

    A: Cek parameter nav2_params.yaml. Pastikan allow_unknown: true agar drone berani melewati area yang belum terpetakan.

Q: Rak gudang tidak muncul di Gazebo?

    A: Pastikan folder models/warehouse_rack sudah ditambahkan ke GZ_SIM_RESOURCE_PATH.
    Bash

    export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)/models

Q: Error MicroXRCEAgent: command not found?

    A: Anda belum melakukan sudo make install pada langkah build Agent. Ulangi langkah Instalasi nomor 2.

👨‍💻 Kontributor

Josh Abraham Efendi

    Role: Robotics Software Engineer Intern

    Email: [Masukkan Email Anda]

    LinkedIn: [Link LinkedIn Anda]

Dibuat untuk keperluan laporan magang President University x Blibli.
