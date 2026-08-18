# Robotic
# KUKA KR210 — Inverse Kinematics Project (Pick & Place)

Proyek ini adalah implementasi **Forward & Inverse Kinematics** untuk lengan robot 6-DOF **KUKA KR210**, berbasis project resmi Udacity *RoboND-Kinematics-Project*, dijalankan di atas **ROS Noetic (Ubuntu 20.04)**.

Repo dasar: `https://github.com/udacity/RoboND-Kinematics-Project`

---

## 1. Struktur File yang Ditambahkan/Dimodifikasi

| File | Status | Keterangan |
|---|---|---|
| `kuka_arm/scripts/kr210_kinematics.py` | **Baru** | Inti matematika FK & IK (numpy murni, tanpa ROS) |
| `kuka_arm/scripts/IK_server.py` | **Baru** (menimpa template kosong) | Wrapper ROS service yang memanggil `kr210_kinematics.py` |
| `test_kinematics.py` | **Baru** | Verifikasi standalone (round-trip FK→IK→FK), tidak butuh ROS |
| `kuka_arm/scripts/target_spawn.py` | **Diedit** | Baris `print"..."` (Python 2) diubah jadi `print("...")` (Python 3) |
| `kuka_arm/worlds/kr210_light.world` | **Baru** | Dunia Gazebo super ringan (cuma lantai + cahaya) pengganti `cafe.world` yang berat |
| `kuka_arm/launch/cafe.launch` | **Diedit** | `world_name` diarahkan ke `kr210_light.world`; `gui` diset `false` (matikan jendela 3D Gazebo, hemat RAM) |
| `kuka_arm/scripts/safe_spawner.sh` | **Diedit** | Waktu `sleep` antar-tahap diperbesar (untuk VM dengan spek terbatas) |

> Catatan: perubahan `kr210_light.world` dan `gui=false` bersifat opsional — kalau komputer target lebih kuat (RAM 8GB+, 4+ core), boleh dikembalikan ke `cafe.world` asli dan `gui=true` untuk tampilan visual yang lebih lengkap (meja kafe & rak).

---

## 2. Prasyarat Sistem

- Ubuntu 20.04 LTS
- ROS Noetic sudah ter-install (`rosversion -d` harus menampilkan `noetic`)
- Koneksi internet (untuk instalasi paket)

---

## 3. Instalasi Dependency

Jalankan semua perintah berikut secara berurutan:

```bash
# Dasar
sudo apt update
sudo apt install python-is-python3
sudo apt install python3-rosdep2
sudo rosdep init      # boleh skip kalau muncul "already initialized"
rosdep update

# Package inti simulasi & kinematics
sudo apt install ros-noetic-gazebo-ros ros-noetic-xacro
sudo apt install ros-noetic-moveit-visual-tools
sudo apt install ros-noetic-tf-conversions
sudo apt install ros-noetic-moveit ros-noetic-moveit-ros-planning-interface \
                  ros-noetic-eigen-conversions ros-noetic-object-recognition-msgs \
                  ros-noetic-industrial-core

# Controller manager & plugin Gazebo (WAJIB, sering ke-skip rosdep)
sudo apt install ros-noetic-ros-control ros-noetic-ros-controllers \
                  ros-noetic-gazebo-ros-control

# Tipe-tipe controller spesifik (WAJIB, sumber error paling umum)
sudo apt install ros-noetic-joint-state-controller \
                  ros-noetic-position-controllers \
                  ros-noetic-effort-controllers \
                  ros-noetic-joint-trajectory-controller \
                  ros-noetic-gripper-action-controller \
                  ros-noetic-velocity-controllers \
                  ros-noetic-forward-command-controller
```

**Verifikasi plugin controller ada:**
```bash
find / -name 'libgazebo_ros_control.so' 2>/dev/null
# harus menampilkan: /opt/ros/noetic/lib/libgazebo_ros_control.so
```

---

## 4. Setup Project

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/udacity/RoboND-Kinematics-Project.git

cd ~/catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro=noetic -y
catkin_make
source devel/setup.bash
```

Lalu **copy 3 file custom** (`kr210_kinematics.py`, `IK_server.py`, `test_kinematics.py`) ke:
```
~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts/
```

Beri izin eksekusi:
```bash
cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
chmod +x IK_server.py
```

Terapkan juga perubahan di `target_spawn.py`, `cafe.launch`, `safe_spawner.sh`, dan tambahkan `kr210_light.world` seperti tabel di atas (lihat detail di bagian 6 kalau mau reproduksi manual).

---

## 5. Cara Menjalankan (butuh 2 terminal)

**Terminal 1 — jalankan simulasi:**
```bash
cd ~/catkin_ws
source devel/setup.bash
cd src/RoboND-Kinematics-Project/kuka_arm/scripts
./safe_spawner.sh
```
Tunggu sampai muncul `Ready to take commands for planning group arm_group.`

**Terminal 2 — jalankan IK server (WAJIB, jalankan SEBELUM atau SELAMA Terminal 1 masih aktif):**
```bash
cd ~/catkin_ws
source devel/setup.bash
cd src/RoboND-Kinematics-Project/kuka_arm/scripts
python3 IK_server.py
```
Tunggu sampai muncul `Ready to receive an IK request`.

Robot akan mulai bergerak secara otomatis begitu kedua proses ini aktif bersamaan.

---

## 6. Verifikasi Matematika (opsional, tanpa perlu ROS/Gazebo)

```bash
cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
python3 test_kinematics.py
```
Skrip ini menjalankan 200 uji round-trip (FK → IK → FK) dengan sudut sendi acak, dan harus lolos dengan error di bawah `1e-6`.

---

## 7. Troubleshooting Umum

| Gejala | Penyebab | Solusi |
|---|---|---|
| `/usr/bin/env: 'python': No such file` | Ubuntu 20.04 tidak punya alias `python` | `sudo apt install python-is-python3` |
| `SyntaxError` pada `print"..."` | Sisa kode Python 2 | Ubah ke `print("...")` |
| `Spawn service timed out` | VM terlalu berat/lambat | Pakai `kr210_light.world`, matikan GUI Gazebo (`gui=false`) |
| `Failed to fetch current robot state` (crash `-11`) | Timing race / controller belum siap | Perbesar `sleep` di `safe_spawner.sh`; pastikan semua package controller ter-install |
| `Controller Spawner couldn't find controller_manager` | `libgazebo_ros_control.so` tidak ada | Install `ros-noetic-gazebo-ros-control` |
| `Could not load controller ... type does not exist` | Package controller spesifik belum ada | Install `joint-state-controller`, `position-controllers`, `effort-controllers`, `joint-trajectory-controller` |
| Robot diam, tidak bergerak sama sekali | `IK_server.py` belum dijalankan | Jalankan di terminal terpisah sebelum/selama simulasi aktif |
| `run_id on parameter server does not match` | Ada proses ROS lama nyangkut | `pkill -9 -f gzserver; pkill -9 -f gzclient; killall -9 roscore rosmaster rosout roslaunch` |

---

## 8. Ringkasan Teknis

- **Forward Kinematics**: transformasi homogen modified-DH per sendi (7 link termasuk offset gripper).
- **Inverse Kinematics**: solusi geometris tertutup (closed-form) — bukan numerik/iteratif:
  - θ1 dari proyeksi wrist center ke bidang XY
  - θ2, θ3 dari hukum cosinus segitiga (shoulder–elbow–wrist)
  - θ4, θ5, θ6 dari dekomposisi matriks rotasi `R3_6 = R0_3^T · R_EE`
- **Verifikasi**: 200 uji round-trip acak, error posisi ~1e-15 m, error rotasi ~1e-16.
