# 🚁 Sistem Drone Otonom Gudang Dalam Ruangan (Autonomous Indoor Warehouse Drone)

![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-blue?style=for-the-badge&logo=ros)
![PX4 Autopilot](https://img.shields.io/badge/PX4-v1.14-green?style=for-the-badge&logo=px4)
![Gazebo Garden](https://img.shields.io/badge/Gazebo-Garden-orange?style=for-the-badge&logo=gazebo)
![Python](https://img.shields.io/badge/Python-3.10-yellow?style=for-the-badge&logo=python)
![License](https://img.shields.io/badge/License-MIT-lightgrey?style=for-the-badge)

> **Proyek Akhir Magang - Divisi Research & Development (R&D)**
> **PT Global Digital Niaga Tbk (Blibli)**
> *Agustus 2025 – Desember 2025*

---

## 📋 Tinjauan Proyek (Project Overview)

Repositori ini berisi kode sumber lengkap (*full source code*) dan lingkungan simulasi untuk **Sistem UAV Otonom** yang dirancang khusus untuk beroperasi di **Lingkungan Dalam Ruangan Tanpa GPS (GPS-Denied)**, secara spesifik untuk kasus penggunaan di gudang logistik berskala besar.

Tujuan utama dari proyek ini adalah mengatasi tantangan penghitungan stok manual di area penyimpanan rak tinggi (*high-rack storage*). Dengan memanfaatkan tumpukan perangkat lunak kustom yang menjembatani **ROS 2 Humble** dan **PX4 Autopilot**, drone ini memiliki kemampuan:

1.  **Lokalisasi Mandiri (Self-Localization):** Menggunakan fusi sensor Lidar 2D dan IMU (Odometry) untuk navigasi presisi tanpa bantuan sinyal satelit.
2.  **Pemetaan Real-time (Mapping):** Menghasilkan Peta Okupansi Grid 2D (*Occupancy Grid Map*) dari lorong-lorong gudang secara langsung saat terbang.
3.  **Navigasi Otonom:** Merencanakan jalur penerbangan yang aman menghindari rintangan statis (rak) dan dinamis menggunakan stack Nav2.

Sistem ini telah divalidasi sepenuhnya dalam simulasi "Digital Twin" berfideliatas tinggi dari Gudang Blibli menggunakan simulator Gazebo Garden.

---

## 🏗 Arsitektur Sistem (System Architecture)

Sistem ini beroperasi pada arsitektur terdistribusi untuk memisahkan logika tingkat tinggi (*high-level logic*) dari kontrol penerbangan tingkat rendah (*low-level control*):

| Lapisan (Layer) | Teknologi | Fungsi Utama |
| :--- | :--- | :--- |
| **Application Layer** | **ROS 2 Humble** | Menangani Perencanaan Jalur (Nav2), Pemetaan (SLAM Toolbox), dan Logika Misi (`px4_clean_nav`). |
| **Middleware Layer** | **Micro XRCE-DDS** | Bertindak sebagai jembatan berkecepatan tinggi yang mentransfer topik `uORB` dari flight controller ke jaringan DDS ROS 2. |
| **Flight Control Layer** | **PX4 Autopilot** | Menangani pencampuran motor (*mixer*), stabilisasi sikap (*attitude*), dan estimasi status kendaraan (EKF2). |
| **Simulation Layer** | **Gazebo Garden** | Mensimulasikan fisika, gravitasi, tabrakan (`warehouse_rack`), dan data sensor Lidar. |

### 🔄 Logika Transformasi Koordinat (Coordinate Transformation)
Salah satu inovasi kunci dalam repositori ini adalah node `cmd_sender.py`, yang menyelesaikan ketidakcocokan sistem koordinat antara **ENU (ROS)** dan **NED (PX4)** secara *real-time*:

```python
# Logika Rotasi Vektor Kecepatan Real-time
vel_north = cmd_x * cos(yaw) - cmd_y * sin(yaw)
vel_east  = cmd_x * sin(yaw) + cmd_y * cos(yaw)
# Hal ini memastikan perintah "Maju" dari ROS diterjemahkan dengan benar
# sesuai arah hadap (heading) drone di dunia PX4.
```
## 📂 Struktur Repositori



```text
warehouse-drone-ros2-px4/
├── src/
│   ├── px4_clean_nav/          # [CORE] Paket Kustom Logika Navigasi
│   │   ├── config/             # File Konfigurasi (Nav2 params, SLAM, Explore)
│   │   ├── launch/             # File Peluncuran (final_demo.launch.py, dll.)
│   │   ├── px4_clean_nav/      # Node Python (cmd_sender, offboard_control)
│   │   └── package.xml         # Definisi dependensi paket
│   │
│   ├── simple_explorer/        # [EXTRA] Logika eksplorasi frontier otonom
│   └── px4_msgs/               # [SUBMODULE] Definisi pesan uORB PX4
│
├── PX4-Autopilot/              # [SUBMODULE] Firmware Kontrol Penerbangan
│   ├── Tools/
│   ├── build/
│   └── ...
│
├── Micro-XRCE-DDS-Agent/       # [SUBMODULE] Jembatan Komunikasi DDS
│
├── worlds/
│   └── warehouse.sdf           # [ASSET] Dunia Simulasi Gudang Blibli Kustom
│
├── models/
│   └── warehouse_rack/         # [ASSET] Model 3D Rak Gudang
│       ├── model.sdf
│       └── model.config
│
├── .gitignore                  # Mengabaikan folder build/install/log
└── README.md                   # Dokumentasi ini
```


🛠️ Panduan Instalasi Komprehensif

Menyiapkan lingkungan pengembangan drone otonom memerlukan konfigurasi yang presisi. Harap ikuti langkah-langkah ini secara berurutan untuk memastikan middleware ROS 2 dapat berinteraksi dengan flight stack PX4 dengan benar.
1. Prasyarat Sistem

Sebelum melakukan cloning, pastikan mesin host Anda menjalankan Ubuntu 22.04 LTS (Jammy Jellyfish). Versi OS ini wajib karena ROS 2 Humble adalah Tier 1 supported hanya pada Jammy.

    Versi Python: 3.10+

    Sistem Build: Colcon (Collective Construction)

    Simulator: Gazebo Garden (gz-sim-7)

2. Mengambil Repositori (Cloning)

Kita menggunakan flag --recursive untuk mengambil tidak hanya kode kita, tetapi juga submodule masif untuk Flight Controller dan DDS Middleware yang terhubung dalam repo ini.
Bash

git clone --recursive [https://github.com/plovaxsz/warehouse-drone-ros2-px4.git](https://github.com/plovaxsz/warehouse-drone-ros2-px4.git)
cd warehouse-drone-ros2-px4

3. Kompilasi Middleware: Micro XRCE-DDS Agent

Micro XRCE-DDS Agent adalah komponen "penerjemah" yang kritis. Ia duduk di antara drone (yang berbicara protokol uORB) dan komputer (yang berbicara protokol DDS ROS 2). Kita harus mengompilasinya dari sumber (source) untuk memastikan kompatibilitas tipe pesan.
Bash

cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/

    Catatan Teknis: Perintah sudo ldconfig sangat krusial. Ini memperbarui cache pustaka bersama sistem sehingga saat Anda menjalankan MicroXRCEAgent nanti, Ubuntu tahu persis di mana menemukan pustaka .so yang baru dibuat.

4. Setup Firmware Penerbangan: PX4 Autopilot

Kita perlu mengompilasi firmware PX4 secara khusus untuk simulasi Software-In-The-Loop (SITL). Ini menciptakan binary executable yang mampu mensimulasikan fisika tanpa perangkat keras nyata.
Bash

cd ../../PX4-Autopilot
# Install dependensi standar PX4 (ini mungkin memakan waktu)
bash ./Tools/setup/ubuntu.sh

# Kompilasi binary SITL
make px4_sitl

(Setelah shell "px4>" muncul atau build selesai dengan sukses, Anda bisa menekan Ctrl+C untuk keluar).
5. Kompilasi Workspace: Paket ROS 2

Terakhir, kita build logika navigasi kustom kita (px4_clean_nav). Kita menggunakan --symlink-install agar setiap perubahan yang Anda buat pada skrip Python (cmd_sender.py, dll.) langsung terrefleksi tanpa perlu rebuild.
Bash

cd ..
# Source environment ROS 2 Humble terlebih dahulu
source /opt/ros/humble/setup.bash

# Install dependensi yang didefinisikan di package.xml
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
colcon build --symlink-install

🌏 Konfigurasi Aset Gudang (Environment Setup)

Karena kita menggunakan lingkungan gudang kustom (warehouse.sdf) dan aset 3D kustom (warehouse_rack), kita harus memberitahu Gazebo di mana menemukan file-file ini.

Tambahkan baris berikut ke file ~/.bashrc Anda (atau jalankan di terminal sebelum meluncurkan simulasi):
Bash

# Arahkan ke folder models kustom kita agar Gazebo bisa merender rak
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)/models

# (Opsional) Jika PX4 mengeluh world tidak ditemukan:
# Salin file world kita ke direktori default PX4
cp worlds/warehouse.sdf PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf

🚀 Protokol Eksekusi (Cara Menjalankan Simulasi)

Untuk mengorkestrasi sistem kompleks ini, kita memerlukan tiga (3) sesi terminal yang berjalan bersamaan. Setiap terminal menangani lapisan arsitektur tertentu.
TERMINAL 1: Jembatan Komunikasi (The Bridge)

Ini menginisiasi DDS Agent. Ia mendengarkan pada UDP Port 8888 untuk telemetri masuk dari drone dan meneruskannya ke topik ROS 2.
Bash

MicroXRCEAgent udp4 -p 8888

    Indikator Sukses: Anda akan melihat output "Agent created", menunggu klien.

TERMINAL 2: Simulasi Fisika (Gazebo + PX4)

Ini meluncurkan instance PX4 headless dan GUI Gazebo. Ini memuat dunia warehouse.sdf kita dan memunculkan drone x500 yang dilengkapi dengan Lidar 2D.
Bash

cd PX4-Autopilot
export PX4_SIM_MODEL=gz_x500_lidar
./build/px4_sitl_default/bin/px4

    Indikator Sukses: Jendela Gazebo terbuka, menampilkan rak-rak gudang dan drone di lantai.

TERMINAL 3: Stack Otonom (ROS 2 "The Brain")

Di sinilah kode kustom kita bekerja. Ini meluncurkan Navigation Stack (Nav2), SLAM Toolbox untuk pemetaan, RViz untuk visualisasi, dan logika Kontrol Offboard.
Bash

cd warehouse-drone-ros2-px4
source install/setup.bash

# Luncurkan demo otonom penuh
ros2 launch px4_clean_nav final_demo.launch.py

🔧 Pemecahan Masalah Mendalam (Troubleshooting)
🔴 Critical Error: "Zero poses in plan"

Gejala: Drone lepas landas tetapi menolak bergerak ke tujuan. Log Nav2 menunjukkan error ini. Akar Masalah: Perencana Nav2 (Algoritma A*) dikonfigurasi terlalu konservatif. Ia menganggap "Unknown Space" (area abu-abu di peta) sebagai rintangan mematikan. Solusi:

    Buka src/px4_clean_nav/config/nav2_params.yaml.

    Cari konfigurasi perencana GridBased.

    Pastikan allow_unknown: true diset. Ini memaksa perencana untuk mengasumsikan ruang yang belum dipetakan aman untuk dilalui.

🔴 Critical Error: Drone Drifting / Unstable Hover

Gejala: Drone bergeser (sliding) saat seharusnya diam. Akar Masalah: Kurangnya kunci GPS di mode indoor memaksa drone mengandalkan "Dead Reckoning". Jika cmd_sender.py tidak beralih ke mode "Position Hold", noise simulasi angin akan mendorong drone. Solusi: Node offboard_control.py kami mengimplementasikan Hybrid Control Loop. Ia secara otomatis mengunci posisi odometry (x, y) saat ini sebagai setpoint setiap kali perintah kecepatan bernilai nol selama lebih dari 0.5 detik.
🔴 Critical Error: "MicroXRCEAgent command not found"

Akar Masalah: Langkah instalasi sudo make install terlewat atau gagal. Solusi: Jalankan kembali perintah instalasi di bagian Kompilasi Middleware dan pastikan sudo ldconfig dieksekusi.
📜 Kredit & Ucapan Terima Kasih

Proyek ini dikembangkan sebagai syarat kelulusan Program Magang Akhir di President University bekerja sama dengan PT Global Digital Niaga Tbk (Blibli).

Pengembang Utama:

    Josh Abraham Efendi (plovaxsz)

        Peran: AI & Robotics Intern (Divisi R&D)

        Fokus: Implementasi SLAM, Arsitektur ROS 2, dan Rekayasa Simulasi.

        Institusi: President University (Informatika)

        Kontak: josh.efendi@student.president.ac.id

        GitHub: github.com/plovaxsz

Terima Kasih Khusus kepada:

    Tim R&D Blibli atas bimbingan mengenai kendala logistik gudang nyata.

    Komunitas Open Source PX4 & ROS 2 atas dokumentasi yang kuat.

© 2025 Josh Abraham Efendi. All Rights Reserved.
