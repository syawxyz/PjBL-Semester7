# PjBL Semester 7 — Week 1

**Pengembangan Forklift Elektrik (Pallet Stacker) Otonom Berbasis ROS 2**
Mitra industri: PT Integrasi Bisnis Eksekutif

## Daftar Isi

- [Hasil Diskusi Proyek](#hasil-diskusi-proyek-dengan-pt-integrasi-bisnis-eksekutif)
- [Logbook Minggu 1](#logbook-minggu-1)
- [SLAM](#slam)
- [Nav2](#nav2)
- [RViz](#rviz)
- [Uji Coba Simulasi Nav2](#uji-coba-simulasi-nav2)
- [Algoritma A*](#algoritma-a)
- [Algoritma Theta*](#algoritma-theta)
- [Modul Sensor Ketinggian Fork](#modul-sensor-ketinggian-fork)
- [Rencana Selanjutnya](#rencana-selanjutnya)
- [Referensi](#referensi)

---

## Hasil Diskusi Proyek dengan PT Integrasi Bisnis Eksekutif

Terdapat forklift elektrik (pallet stacker) yang telah dikerjakan pada semester sebelumnya. Forklift ini dapat dikendalikan menggunakan remote nirkabel, tetapi belum mampu beroperasi secara otonom.

### Kondisi Saat Ini

- Kontroler = STM32
- Komunikasi = CAN bus
- Sensor = LiDAR 2D, IMU, encoder motor
- Mode kendali = Manual melalui remote nirkabel
- Fork hidrolik = Naik/turun manual, belum ada sensor ketinggian fork

### Permasalahan

- Forklift belum dapat bernavigasi secara otonom antar area kerja.
- Tinggi fork tidak terukur, sehingga tidak dapat dikendalikan ke posisi tertentu.

Kondisi tersebut membatasi penerapan forklift untuk proses *material handling* yang membutuhkan perpindahan otomatis antar area dan pengaturan tinggi fork sesuai tingkat rak.

### Lingkup Pengembangan

1. Sistem navigasi otonom berbasis ROS 2 (SLAM + Nav2).
2. Instrumentasi dan kendali ketinggian fork.

### Dokumentasi

<table>
  <tr>
    <td align="center">
      <img src="images/IMG_4042.png" alt="Pallet stacker forklift" height="360"><br>
      <sub>Pallet stacker forklift</sub>
    </td>
    <td align="center">
      <img src="images/IMG_3208.jpeg" alt="Hardware forklift" height="360"><br>
      <sub>Hardware forklift: panel elektronik dan unit hidrolik</sub>
    </td>
  </tr>
</table>

---

## Logbook Minggu 1

Periode 28/09/2026 s.d. 02/10/2026, sesuai logbook individu DTEO ITS.

- Sen, 28/09 = Diskusi masalah dengan mitra di industri (2 jam); menyusun Project Charter (6 jam)
- Sel, 29/09 = Mempelajari Nav2 (4 jam); mempelajari RViz (3 jam)
- Rab, 30/09 = Membuat arsitektur sistem hardware pembaca ketinggian fork (4 jam); membuat skematik PCB (3 jam)
- Kam, 01/10 = Routing PCB (4 jam); mempersiapkan firmware pembaca ketinggian fork (3 jam)
- Jum, 02/10 = Mempelajari SLAM (4 jam); mempelajari algoritma A* dan Theta* (3 jam)
- Total = 36 jam

Kegiatan Minggu 2 (Min, 04/10): simulasi Nav2, pembuatan peta dengan slam_toolbox di simulasi, dan latihan A* serta Theta*.

---

## SLAM

**SLAM (*Simultaneous Localization and Mapping*)** adalah proses membangun peta lingkungan sekaligus memperkirakan posisi robot di dalam peta tersebut. Pada proyek ini, SLAM digunakan untuk membuat peta area kerja yang nantinya dipakai Nav2 untuk navigasi.

Package yang digunakan adalah **slam_toolbox**, yaitu SLAM 2D berbasis LiDAR yang direkomendasikan oleh Nav2. Alternatifnya adalah Cartographer.

### Input dan Output

**Input**

- `/scan` (`sensor_msgs/msg/LaserScan`) = Driver LiDAR 2D
- TF `odom → base_link` = Odometri encoder + IMU (EKF)
- TF `base_link → laser` = URDF / static transform

**Output** (dari slam_toolbox)

- `/map` (`nav_msgs/msg/OccupancyGrid`) = Peta hasil SLAM
- TF `map → odom` = Koreksi posisi robot terhadap peta

### Hasil Belajar

**Apa yang dihasilkan SLAM, dan apa bedanya dengan AMCL?**

- SLAM = Membuat peta dari masukan LiDAR dan odometri, sekaligus memperkirakan posisi robot di peta yang sedang dibuat
- AMCL = Menggunakan peta yang dihasilkan SLAM dan masukan sensor untuk mengetahui posisi robot

Jadi AMCL tidak bisa bekerja tanpa peta, sedangkan SLAM yang membuat peta tersebut.

**Kenapa Localization *inactive* di mode SLAM, tapi robot tetap bisa bernavigasi?**
Di mode SLAM, `amcl` dan `map_server` tidak jalan, jadi yang menggantikan tugasnya adalah slam_toolbox. slam_toolbox menerbitkan peta (`/map`) dan TF `map → odom`, sehingga Nav2 tetap bisa bernavigasi.

**Isi file peta (`tb3_sim.yaml`)**

- `resolution` = Meter per piksel; `0.05` berarti 1 piksel = 5 cm
- `origin` = Posisi pojok kiri bawah peta dalam meter (x, y, sudut); pada peta ini `[-2.97, -2.58, 0]`
- Ukuran peta = Jumlah piksel × resolution; gambar 112 × 103 piksel × 0,05 = 5,6 × 5,15 meter

### Langkah Pembuatan Peta

1. Jalankan driver LiDAR, node bridge CAN, dan EKF (`robot_localization`).
2. Jalankan slam_toolbox dalam mode *mapping*:
   ```bash
   ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false
   ```
3. Gerakkan forklift perlahan dengan remote mengelilingi seluruh area kerja.
4. Simpan peta (menghasilkan `area_kerja.pgm` dan `area_kerja.yaml`):
   ```bash
   ros2 run nav2_map_server map_saver_cli -f ~/maps/area_kerja
   ```

### Catatan

- Gerakkan forklift dengan kecepatan rendah dan kembali ke titik awal agar terjadi *loop closure*.
- Kualitas peta sangat bergantung pada akurasi odometri, sehingga kalibrasi encoder dan IMU perlu dilakukan terlebih dahulu.
- Periksa apakah pandangan LiDAR terhalang mast atau fork; jika ya, batasi sudut scan atau gunakan filter laser.

---

## Nav2

**Nav2** adalah framework navigasi pada ROS 2 untuk membawa robot dari posisi awal ke posisi tujuan secara aman. Nav2 menangani lokalisasi, perencanaan jalur, penghindaran halangan, dan *recovery* ketika robot terjebak.

### Komponen Utama

- `map_server` = Memuat peta (`.yaml` + `.pgm`)
- `amcl` = Lokalisasi robot pada peta menggunakan LiDAR (*particle filter*)
- `planner_server` = Membuat jalur global dari posisi robot ke tujuan
- `controller_server` = Mengikuti jalur dan menghasilkan perintah kecepatan `/cmd_vel`
- Costmap 2D (global & local) = Peta biaya berisi area bebas dan halangan
- `behavior_server` = Perilaku *recovery*: spin, backup, wait
- `bt_navigator` = Mengatur alur navigasi menggunakan *Behavior Tree*
- `velocity_smoother` = Menghaluskan perintah kecepatan sebelum dikirim ke motor
- `lifecycle_manager` = Mengelola start/stop seluruh node Nav2

### Hasil Belajar

**Kenapa Global Status di RViz Error sebelum initial pose diberikan?**
Karena robot tidak tahu posisi aktualnya di peta. Odometri hanya tahu perpindahan robot dari titik awalnya, bukan posisinya di peta, untuk itu `amcl` membutuhkan pose awal (initial pose). Setelah pose awal diberikan, `amcl` menerbitkan TF `map → odom` sehingga Global Status menjadi Ok.

**Apa beda tugas planner dan controller?**

- Planner = Bertugas menghitung lintasan yang aman berdasarkan peta costmap global
- Controller = Menghasilkan perintah kecepatan (`/cmd_vel`) yang akan dieksekusi oleh hardware, dengan tujuan dan jalur yang sudah ditentukan oleh planner

**Apa beda costmap global dan costmap lokal?**

- Costmap global = Mempunyai peta keseluruhan, dibangun dari peta hasil SLAM dan data sensor, digunakan oleh planner
- Costmap lokal = Bertugas membaca lingkungan sekitar robot secara aktual dari data sensor saat itu, digunakan oleh controller untuk antisipasi

**Contoh: ada orang lewat di depan robot**

- Yang pertama mendeteksi = Costmap lokal
- Yang bereaksi = Controller lebih dulu, lalu planner membuat jalur baru
- Kenapa controller lebih dulu = Karena frekuensi kerja controller lebih cepat
- Kenapa planner juga bereaksi = Karena costmap global juga membaca data sensor, jadi orang itu ikut tercatat di costmap global

### Menjalankan Nav2

```bash
ros2 launch nav2_bringup bringup_launch.py \
  map:=$HOME/maps/area_kerja.yaml \
  params_file:=<path>/nav2_params.yaml \
  use_sim_time:=false
```

---

## RViz

**RViz2** adalah tool visualisasi 3D pada ROS 2 untuk menampilkan data sensor, TF tree, peta, dan jalur navigasi robot. RViz hanya menampilkan data, bukan simulator. Tool ini dipakai untuk memantau proses SLAM dan Nav2 serta melakukan debugging.

### Display yang Digunakan

- Map (`/map`) = Menampilkan peta hasil SLAM
- LaserScan (`/scan`) = Memeriksa data LiDAR
- TF = Memeriksa pohon frame (`map`, `odom`, `base_link`, `laser`)
- RobotModel (`/robot_description`) = Menampilkan model forklift dari URDF
- Odometry (`/odom`) = Memeriksa hasil odometri
- Path (`/plan`) = Menampilkan jalur global dari Nav2
- Map costmap (`/global_costmap/costmap`, `/local_costmap/costmap`) = Menampilkan costmap Nav2

### Tool Interaktif

- **2D Pose Estimate**: memberikan posisi awal robot ke AMCL (topic `/initialpose`).
- **Nav2 Goal**: mengirim posisi tujuan navigasi ke Nav2.

### Menjalankan RViz

```bash
# RViz kosong
ros2 run rviz2 rviz2

# RViz dengan konfigurasi bawaan Nav2
ros2 launch nav2_bringup rviz_launch.py

# RViz dengan konfigurasi sendiri (File → Save Config As)
rviz2 -d config/forklift.rviz
```

---

## Uji Coba Simulasi Nav2

Sebelum diterapkan pada forklift, Nav2 diuji terlebih dahulu menggunakan simulasi robot TurtleBot3 Waffle bawaan Nav2 di Gazebo. Uji coba ini bertujuan memahami alur kerja Nav2: lokalisasi dengan AMCL, perencanaan jalur, dan pengiriman goal melalui RViz.

**Lingkungan:** Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11.

> **Catatan:** Quickstart Nav2 versi Rolling memakai paket `nav2_minimal_tb*` dan Gazebo baru (Harmonic). Pada ROS 2 Humble, simulasi memakai paket `turtlebot3_gazebo` dengan Gazebo Classic, sehingga langkahnya sedikit berbeda.

### 1. Instalasi

`$ROS_DISTRO` hanya terisi setelah ROS 2 di-*source*, jadi lakukan `source` terlebih dahulu.

```bash
source /opt/ros/humble/setup.bash      # pengguna zsh: setup.zsh

sudo apt update
sudo apt install \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-turtlebot3-gazebo
```

### 2. Menjalankan Simulasi

```bash
source /usr/share/gazebo/setup.sh
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models

ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
```

Perintah ini membuka Gazebo (dunia simulasi) dan RViz. Pada tahap ini **Global Status di RViz masih Error**, dan terminal menampilkan pesan `Timed out waiting for transform from base_link to map`. Hal ini normal karena AMCL belum menerima posisi awal robot, sehingga transform `map → odom` belum tersedia.

> Jangan menutup jendela RViz. Pada launch ini, menutup RViz akan menghentikan seluruh proses.

![Gazebo dan RViz sebelum initial pose](images/nav2-sim-sebelum-initial-pose.png)
*Gazebo (kiri) menampilkan TurtleBot3 dengan sinar LiDAR berwarna biru. RViz (kanan) sudah menampilkan peta, tetapi Global Status masih Error karena initial pose belum diberikan.*

### 3. Memberikan Initial Pose

Klik **2D Pose Estimate** pada toolbar RViz, klik posisi robot pada peta, lalu tarik ke arah hadap robot. Cara lain melalui terminal:

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: -2.0, y: -0.5, z: 0.0}, orientation: {w: 1.0}}}}"
```

Setelah initial pose diterima, Global Status berubah menjadi **Ok**, dan panel Navigation 2 menunjukkan **Navigation: active** dan **Localization: active**.

![RViz setelah initial pose](images/nav2-sim-setelah-initial-pose.png)
*RViz setelah initial pose diberikan. Costmap global dan lokal, data LaserScan (titik merah), dan partikel AMCL (panah hijau di sekitar robot) mulai tampil.*

### 4. Mengirim Goal Navigasi

Klik **Nav2 Goal** pada toolbar RViz, klik titik tujuan pada peta, lalu tarik untuk menentukan arah akhir robot. Cara lain melalui terminal:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.5, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}}"
```

Jika berhasil, terminal menampilkan `Goal finished with status: SUCCEEDED`.

![RViz saat robot bernavigasi](images/nav2-sim-navigasi.png)
*Robot bergerak menuju goal. Panel Navigation 2 menampilkan feedback secara real-time: ETA 7 s, sisa jarak 1,65 m, waktu tempuh 6 s, dan 0 recovery.*

### Hasil Pengamatan

- Sebelum initial pose = Global Status: Error; frame `map` belum tersedia sehingga costmap global menunggu transform
- Setelah initial pose = Global Status: Ok; Navigation dan Localization *active*; costmap, LaserScan, dan partikel AMCL tampil
- Saat navigasi = Robot mengikuti jalur hasil planner sambil menghindari halangan; feedback (ETA, sisa jarak, recovery) tampil di panel Navigation 2

---

## Algoritma A*

A* adalah algoritma pencarian jalur terpendek pada grid. Berbeda dengan Dijkstra yang menjelajah ke segala arah secara merata, A* mendahulukan arah yang kelihatannya menuju goal.

### Rumus

```
f = g + h
```

- `g` = Biaya yang sudah ditempuh dari start ke kotak ini
- `h` = Perkiraan biaya dari kotak ini ke goal (heuristik)
- `f` = Perkiraan total biaya jika lewat kotak ini

Untuk grid 4 arah (atas, bawah, kiri, kanan), `h` dihitung dengan jarak Manhattan ke goal: `h = |selisih x| + |selisih y|`.

### Open List dan Closed List

- Open list = Kotak yang sudah ditemukan tapi belum dikembangkan; isinya menumpuk
- Closed list = Kotak yang sudah dikembangkan; tidak diperiksa lagi

Setiap langkah, A* mengambil kotak dengan `f` terkecil dari open list, memindahkannya ke closed list, lalu memasukkan tetangga barunya ke open list. Pencarian berhenti saat G diambil dari open list.

### Latihan 1: Menghitung g, h, dan f

```
        x=0  x=1  x=2  x=3  x=4
y=0      .    .    .    .    .
y=1      S    .    #    .    G
y=2      .    .    #    .    .
```

Tetangga S (0,1):

- (1,1) = g 1, h 3, f 4
- (0,0) = g 1, h 5, f 6
- (0,2) = g 1, h 5, f 6

A* mengembangkan (1,1) lebih dulu karena nilai `f`-nya paling kecil.

Jalur akhir: S → (1,1) → (1,0) → (2,0) → (3,0) → (3,1) atau (4,0) → G = 6 langkah.

Kalau `h` selalu bernilai 0, maka `f = g`, sehingga `g` terkecil yang didahulukan. A* tidak lagi terarah ke goal dan berubah menjadi algoritma Dijkstra.

### Latihan 2: Open List dan Closed List

```
        x=0  x=1  x=2  x=3
y=0      S    .    .    .
y=1      .    #    #    .
y=2      .    .    .    G
```

Format: kotak[g, h, f]

```
Langkah 1: ambil S
  Open   : (1,0)[1,4,5]  (0,1)[1,4,5]
  Closed : S

Langkah 2: ambil (1,0)
  Open   : (0,1)[1,4,5]  (2,0)[2,3,5]
  Closed : S, (1,0)

Langkah 3: ambil (2,0)
  Open   : (0,1)[1,4,5]  (3,0)[3,2,5]
  Closed : S, (1,0), (2,0)

Langkah 4: ambil (3,0)
  Open   : (0,1)[1,4,5]  (3,1)[4,1,5]
  Closed : S, (1,0), (2,0), (3,0)

Langkah 5: ambil (3,1)
  Open   : (0,1)[1,4,5]  G[5,0,5]
  Closed : S, (1,0), (2,0), (3,0), (3,1)

Langkah 6: ambil G → selesai
```

Jalur akhir: S → (1,0) → (2,0) → (3,0) → (3,1) → G = 5 langkah.

(0,1) tidak pernah masuk closed list. Di langkah 2, (0,1) kalah seri karena (1,0) lebih dulu masuk open list. Di langkah berikutnya, (0,1) selalu kalah karena `h`-nya lebih besar. Saat G diambil, pencarian langsung berhenti.

---

## Algoritma Theta*

Theta* adalah pengembangan A* supaya jalurnya tidak harus mengikuti arah grid (*any-angle*).

### Masalah A* di Grid

Pada grid kosong dari (0,0) ke (3,2), A* 4 arah menghasilkan jalur berbentuk tangga sepanjang 5 m. Padahal garis lurusnya hanya √(3² + 2²) ≈ 3,61 m.

### Line-of-Sight

Setiap menemukan tetangga, Theta* mengecek apakah parent dari kotak saat ini bisa melihat langsung tetangga itu, yaitu apakah garis lurus dari tengah kotak ke tengah kotak tidak melewati tembok.

- Terlihat (Path 2) = Tetangga langsung dihubungkan ke parent, kotak di tengah dilewati
- Tidak terlihat (Path 1) = Sama seperti A*, tetangga dihubungkan ke kotak saat ini

Karena jalurnya bisa miring, biaya dihitung dengan jarak lurus (Euclidean), bukan jumlah langkah.

### Latihan 3: Line-of-Sight

Grid sama dengan Latihan 1 A* (jalur A* = 6 langkah).

- S (0,1) → G (4,1) = Tidak, karena terhalang tembok di (2,1)
- S (0,1) → (2,0) = Ya, karena garis lurusnya tidak melewati tembok
- (2,0) → G (4,1) = Ya, karena garis lurusnya tidak melewati tembok; tembok (2,1) ada di bawah garis

Jalur Theta*: S → (2,0) → G

Panjang jalur: √5 + √5 = 2,24 + 2,24 = 4,48 m. Dibanding A* (6 m), Theta* lebih pendek 1,52 m.

### Kelebihan dan Kekurangan

- Kelebihan = Jalur lebih pendek dan belokannya lebih sedikit
- Kekurangan = Perlu hitungan tambahan untuk cek line-of-sight, dan belum memperhitungkan radius belok kendaraan (penting untuk forklift)

---

## Modul Sensor Ketinggian Fork

Modul ini membaca ketinggian fork menggunakan encoder dan 2 limit switch, dengan mikrokontroler STM32F401CCU6.

### Arsitektur Sistem

![Arsitektur sistem modul sensor ketinggian fork](images/fork-arsitektur.png)

- Encoder incremental = Menghasilkan pulsa kanal A dan B saat fork bergerak, untuk menghitung posisi dan arah gerak fork
- Limit switch (2 buah) = Penanda batas atas dan batas bawah fork, juga sebagai titik acuan saat homing
- STM32F401 Black Pill = Menghitung posisi encoder, mengubahnya menjadi ketinggian (cm), dan membaca kondisi limit switch
- UART = Jalur komunikasi dua arah ke sistem utama: data ketinggian dikirim ke sistem utama, perintah dari sistem utama diterima STM32, dengan checksum
- Sistem utama = Menerima data ketinggian fork dan mengirim perintah ke modul sensor fork

### Skematik

- Catu daya = Input 24V (konektor XT30), diturunkan ke 5V dengan modul DC-DC (U2)
- Mikrokontroler = STM32F401CCU6 Black Pill (U1)
- Encoder = ENC_A (PA8) dan ENC_B (PA9) melalui konektor J5, dengan catu 5V
- Limit switch = LS_DOWN (PB14) dan LS_UP (PB15), masing-masing dengan pull-up 10k, resistor 1k, dioda 1N4148, dan kapasitor 100nF
- Komunikasi = UART TX (PA2) dan RX (PA3) melalui konektor J4
- Reset = Tombol SW1 ke pin NRST STM32; jalur RST_ALL (PA1) keluar melalui konektor J6
- Indikator = LED D1 pada jalur 3V3

![Skematik modul sensor fork](images/fork-skematik.png)

### PCB

- Software = KiCad 9.0.5
- Layer = 2 layer (F.Cu dan B.Cu)
- Status = Masih ada 2 koneksi yang belum di-routing

![Layout PCB modul sensor fork](images/fork-pcb-layout.png)

![Render 3D PCB modul sensor fork](images/fork-pcb-3d.png)

### Firmware

- IDE = STM32CubeIDE, project `encoder_forklift`
- File utama = `Core/Src/handler.c`
- Konversi = `EncoderToHeight()`: ketinggian = encoder × `HEIGHT_MAX_CM` / `ENCODER_MAX`, nilai encoder dibatasi 0 sampai `ENCODER_MAX`
- Konstanta di kode = `ENCODER_MAX` = 53808, `HEIGHT_MAX_CM` = 250 cm
- Komunikasi = UART (`huart_encoder_fork`) dengan checksum (`calc_checksum`)
- Fungsi lain = `Encoder_Update`, `encoderForkInit`, `encoderForkReceive`, `encoderForkRoutine`

![Firmware handler.c di STM32CubeIDE](images/fork-firmware-handler.png)

---

## Rencana Selanjutnya

- [ ] Pembuatan PCB fork dan tes firmware
- [ ] Integrasi firmware MCU fork ke sistem utama dengan komunikasi UART
- [ ] Membuat arsitektur sistem
- [x] Uji coba Nav2 pada simulasi TurtleBot3 (Gazebo)
- [ ] Identifikasi format pesan CAN dari STM32 (encoder, perintah motor, hidrolik)
- [ ] Membuat node bridge CAN ↔ ROS 2 (SocketCAN)
- [ ] Membuat URDF forklift beserta TF `base_link → laser` dan `base_link → imu_link`
- [ ] Fusi odometri encoder dan IMU menggunakan `robot_localization`
- [ ] Mapping area kerja menggunakan slam_toolbox
- [ ] Konfigurasi dan uji Nav2 pada forklift
- [ ] Pemilihan sensor ketinggian fork dan perancangan kendali posisi fork

---

## Referensi

- [Dokumentasi Nav2](https://docs.nav2.org/)
- [Nav2 Quickstart (versi Rolling)](https://docs.nav2.org/rolling/getting_started/quickstart/quickstart/#quickstart)
- [Nav2 First-Time Robot Setup Guide](https://docs.nav2.org/setup_guides/index.html)
- [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [robot_localization](https://github.com/cra-ros-pkg/robot_localization)
- [RViz2](https://github.com/ros2/rviz)
- [Dokumentasi ROS 2 Humble](https://docs.ros.org/en/humble/)
