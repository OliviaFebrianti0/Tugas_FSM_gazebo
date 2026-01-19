# Finite State Machine (FSM) Mobile Robot Using ROS 2, LiDAR, and Gazebo
### Anggota Kelompok
1. Nedia Waty	4222201010
2. Olivia Febrianti	4222201022

## Ringkasan Proyek
Repository ini berisi implementasi Finite State Machine (FSM) pada mobile robot differential drive menggunakan ROS 2 dan Gazebo Simulator. Robot dilengkapi dengan sensor LiDAR untuk mendeteksi rintangan di sekitarnya dan mengambil keputusan navigasi secara otomatis, real-time, dan terstruktur. FSM digunakan sebagai metode pengambilan keputusan tingkat tinggi (high-level control) untuk mengatur perilaku robot berdasarkan kondisi lingkungan, khususnya jarak rintangan di depan robot.

## Latar Belakang
Dalam sistem robotika, robot tidak hanya dituntut untuk bergerak, tetapi juga memiliki logika pengambilan keputusan. Salah satu pendekatan paling dasar dan mudah dipahami untuk mengatur perilaku robot adalah Finite State Machine (FSM).
FSM memungkinkan robot:
- Memiliki state (kondisi) yang jelas
- Melakukan transisi state berdasarkan aturan tertentu
- Mudah dikembangkan dan dianalisis
- Cocok untuk sistem real-time dan embedded

## Konsep Finite State Machine (FSM)
Finite State Machine adalah model komputasi yang terdiri dari:
- State: kondisi sistem saat ini
- Transition: perpindahan antar state
- Condition/Event: pemicu perpindahan state
- Action: aksi yang dijalankan pada setiap state
FSM pada robot ini terdiri dari dua state utama:
1. FORWARD: Robot bergerak maju jika tidak ada rintangan di depan
2. TURN: Robot berbelok untuk menghindari rintangan

## Diagram Alur FSM
<img width="236" height="372" alt="Untitled Diagram drawio" src="https://github.com/user-attachments/assets/863cc5d3-49d4-4b9e-8426-468cda52dee5" />

## Teknologi & Tools yang Digunakan
- Middleware	ROS 2 (Humble / Jazzy)
- Simulator	Gazebo
- Bahasa Pemrograman	Python (rclpy)
- Sensor	LiDAR (LaserScan)
- Kontrol Gerak	Differential Drive Controller
- Sistem Operasi	Ubuntu Linux

## Spesifikasi Robot (Simulasi)
- Tipe robot: Mobile robot differential drive
- Jumlah roda: 2 roda utama
- Sensor: LiDAR 360° (topic /scan)
- Kontrol kecepatan: Linear velocity (linear.x) dan Angular velocity (angular.z)
- Topic kontrol: `/diff_cont/cmd_vel_unstamped`

## Langkah Instalasi dan Build
1. Clone Repository
`git clone https://github.com/USERNAME_KAMU/simple_fsm_robot.git`
`cd simple_fsm_robot`

2. Build Workspace ROS 2
`cd ~/fsm_ws`
`colcon build`
`source install/setup.bash`

## Cara Menjalankan Sistem
1. Jalankan Gazebo dan Robot
- Cek topic LiDAR:
`ros2 topic list | grep scan`
- Cek data LiDAR:
`ros2 topic echo /scan`

2. Jalankan FSM Node
`ros2 run simple_fsm_robot fsm_node`
Jika berhasil, akan muncul log:
FSM + LiDAR STARTED
STATE -> TURN
STATE -> FORWARD

## Cara Kerja Sistem 
1. Node FSM dijalankan
2. Node membaca data LiDAR dari topic /scan
3. Data LiDAR bagian depan dianalisis
4. FSM menentukan state:
- Jarak aman → FORWARD
- Jarak dekat → TURN
4. Robot dikontrol melalui topic /cmd_vel
5. FSM berjalan terus secara real-time
