# 🐭 Micromouse v0.1 (STM32F411CEU6)

[![PlatformIO](https://img.shields.io/badge/PlatformIO-Core%20v6.1+-orange.svg)](https://platformio.org/)
[![Framework](https://img.shields.io/badge/Framework-STM32%20Arduino-blue.svg)](https://github.com/stm32duino/Arduino_Core_STM32)
[![MCU](https://img.shields.io/badge/MCU-STM32F411CEU6%20100MHz-green.svg)](https://www.st.com/en/microcontrollers-microprocessors/stm32f411ce.html)
[![Hardware](https://img.shields.io/badge/EDA-EasyEDA%20Pro-critical.svg)](https://pro.easyeda.com/)
[![Build Status](https://img.shields.io/badge/Build-Passing%20(0%20Errors)-brightgreen.svg)]()

Firmware robot **Micromouse Otonom 16×16 Labirin** berperforma tinggi berbasis mikrokontroler **STM32F411CEU6 (WeAct Black Pill)**. Dirancang dan disesuaikan secara presisi berdasarkan skematik dan layout PCB EasyEDA Pro dengan sistem kemudi diferensial belakang (*rear-wheel drive*) dan susunan 5 sensor optik IR di depan.

---

## 📑 Daftar Isi
1. [Fitur Utama](#-fitur-utama)
2. [Spesifikasi Perangkat Keras & Kinematika](#-spesifikasi-perangkat-keras--kinematika)
3. [Daftar Pinout & Netlist Lengkap](#-daftar-pinout--netlist-lengkap)
4. [Arsitektur Perangkat Lunak](#-arsitektur-perangkat-lunak)
5. [Strategi Navigasi: Scan vs Fast Run](#-strategi-navigasi-scan-vs-fast-run)
6. [Sensor Fusion: Peran MPU6050 vs Encoder](#-sensor-fusion-peran-mpu6050-vs-encoder)
7. [Antarmuka Pengguna (Tombol, OLED, Buzzer, LED)](#-antarmuka-pengguna-tombol-oled-buzzer-led)
8. [Penyimpanan Memori Flash Non-Volatile](#-penyimpanan-memori-flash-non-volatile)
9. [Perintah Serial CLI](#-perintah-serial-cli)
10. [Panduan Kompilasi & Pengunggahan](#-panduan-kompilasi--pengunggahan)
11. [Prosedur Kalibrasi Lapangan](#-prosedur-kalibrasi-lapangan)
12. [7 Fitur Unggulan Tingkat Kompetisi](#-7-fitur-unggulan-tingkat-kompetisi)

---

## 🚀 Fitur Utama

- **Dual-Mode Navigation Engine**:
  - **Mode Scan (Eksplorasi Gesit & Stabil)**: Pemetaan labirin diskrit sel demi sel dengan laju lincah dan responsif (`185 PWM` maju, `155 PWM` pivot turn), koreksi garis tengah lorong (*wall centering* PD), dan belokan pivot berakurasi $\pm 2.5^\circ$.
  - **Mode Fast Run (Balapan Jalur Tercepat)**: Eksekusi otomatis lintasan optimal dari memori dengan kompresi sprint multi-sel lurus dan **gerakan mengular (*slalom curve turns* 90° radius 90 mm)** tanpa henti untuk waktu tempuh minimal.
- **Penyimpanan Permanen di Flash STM32**: Peta labirin 16×16 (256 sel) otomatis disimpan ke Flash Sector 7 (`0x08060000`). Data tidak hilang saat baterai dicabut.
- **Sensor Fusion Cerdas**: Sinergi MPU6050 Gyro Z (kecepatan sudut & heading) dan Wheel Encoders kuadratur (jarak translasi mm) membebaskan robot dari galat akibat selip roda (*wheel slip*).
- **OLED Dashboard Modular & Bebas Dilepas (SSD1306 128×64)**: Program dirancang **100% aman dan lancar dijalankan tanpa layar OLED**. Layar bersifat opsional dan dapat dicolok sewaktu-waktu (auto-detect hot-plug). Saat layar tidak dipasang, pemilihan mode dipandu secara akustik oleh **nada Buzzer (bip 1× s.d. 5×)**.
- **Sistem Pengaman Bodi Panjang (Auto Clearance & Anti-Stall Gesit)**: Logika mundur cepat ~20 mm (-110 PWM selama 90 ms) sebelum mengeksekusi belokan pivot 90°/180° jika ada dinding depan, menjamin moncong dan sudut bumper depan memiliki ruang putar bebas tanpa risiko menyenggol dinding labirin. Dilengkapi toleransi putar cerdas $\pm 2.5^\circ$ dan batas minimum 85 PWM untuk mengatasi *stiction* motor N20 secara lincah.
- **Monitor Daya Baterai 1S**: Deteksi voltase baterai terintegrasi dengan proteksi *Low Battery Alert*.

---

## 📐 Spesifikasi Perangkat Keras & Kinematika (Hasil Scan PCB)

| Parameter | Dimensi / Keterangan |
| :--- | :--- |
| **Dimensi Papan PCB** | Panjang Outline PCB: **119.38 mm**, Lebar Outline PCB: **114.10 mm** |
| **Dimensi Fisik Robot Total** | Panjang Total: **~122.0 mm** (Ujung TCRT5000 ke pantat belakang), Lebar Total: **~104.0 mm** (Termasuk ban roda kiri & kanan) |
| **Jarak Poros Roda ke Sensor** | **78.60 mm** (Dihitung presisi dari koordinat PCB: as motor $y=-2274$ ke sensor depan $y=+821$) |
| **Wheelbase (Jarak Antar Roda)**| **~82.0 mm** (Jarak antar titik tengah tapak ban roda kiri dan kanan) |
| **Diameter Roda** | **32.0 mm** (Roda standar micromouse ban silikon/karet) |
| **Mikrokontroler (U1)** | **STM32F411CEU6** (ARM Cortex-M4F @ 100 MHz, 512 KB Flash, 128 KB RAM) |
| **Driver Motor (U3)** | **TI DRV8833** Dual MOSFET H-Bridge (PWM 100% duty cycle capable) |
| **Motor Penggerak** | 2× Micro Metal Gearmotor N20 dengan Hall Magnetic Quadrature Encoders (360 CPR) |
| **Sensor Dinding (5x)** | 5× **TCRT5000** Optical Reflective IR Sensors (ADC1 12-bit) |
| **IMU / Giroskop (U4)** | **InvenSense MPU6050** 6-DOF IMU (I2C 400 kHz, Gyro Z full-scale $\pm 500^\circ/\text{s}$) |
| **Layar OLED (Opsional)** | **SSD1306** 0.96 inci I2C (128×64 pixel, alamat `0x3C`/`0x3D`, Plug-and-Play) |
| **Catu Daya** | 1S LiPo/Li-Ion (3.7V - 4.2V), pembagi tegangan 10k/10k di pin `PA4` |
| **Dimensi Labirin** | Standar IEEE Micromouse: 16×16 sel, ukuran sel $180\text{ mm} \times 180\text{ mm}$, tebal dinding $12\text{ mm}$ |

---

## 🔌 Daftar Pinout & Netlist Lengkap

Pinout di bawah ini diekstraksi langsung dari skematik EasyEDA Pro:

```
                          ┌───────────────────────────┐
                          │      STM32F411CEU6        │
                          │        BLACK PILL         │
                          ├───────────────────────────┤
  ENC Right A   ◄─────────┤ PA0                   PB0 ├─────────► IR Front-Left (45°)
  ENC Right B   ◄─────────┤ PA1                   PB1 ├─────────► IR Left (90°)
  IR Right (90°)◄─────────┤ PA2                   PB3 ├─────────► Status LED 2
  IR Front-R(45)◄─────────┤ PA3                   PB4 ├─────────► ENC Left A
  Battery Sense ◄─────────┤ PA4                   PB5 ├─────────► ENC Left B
  IR Front-C (0)◄─────────┤ PA7                   PB6 ├─────────► I2C1 SCL (IMU & OLED)
  Motor L IN1   ──────────┤ PA9                   PB7 ├─────────► I2C1 SDA (IMU & OLED)
  Motor L IN2   ──────────┤ PA10                  PB8 ├─────────► Motor R IN1
  Status LED 1  ──────────┤ PA11                  PB9 ├─────────► Motor R IN2
  Motor nSLEEP  ──────────┤ PA15                 PB10 ├─────────► Piezo Buzzer
  Push SW4      ◄─────────┤ PC14                 PB12 ├─────────► Push SW2
  Push SW3      ◄─────────┤ PC15                 PB13 ├─────────► Push SW1
  Onboard LED   ──────────┤ PC13                      │
                          └───────────────────────────┘
```

### Tabel Rincian Pin:

| Pin STM32 | Nama Net Skematik | Tipe I/O | Komponen Terhubung & Fungsi |
| :--- | :--- | :---: | :--- |
| `PA15` | `M_SLEEP` | Output | DRV8833 `nSLEEP` (HIGH = Aktif, LOW = Standby/Hemat Daya) |
| `PA9` | `M_L_IN1` | Output (PWM) | Motor Kiri Input 1 (Maju) |
| `PA10` | `M_L_IN2` | Output (PWM) | Motor Kiri Input 2 (Mundur) |
| `PB8` | `M_R_IN1` | Output (PWM) | Motor Kanan Input 1 (Maju) |
| `PB9` | `M_R_IN2` | Output (PWM) | Motor Kanan Input 2 (Mundur) |
| `PB4` | `ENC_L_A` | Input (ExtI) | Encoder Roda Kiri Channel A (Interrupt Kuadratur) |
| `PB5` | `ENC_L_B` | Input (ExtI) | Encoder Roda Kiri Channel B |
| `PA0` | `ENC_R_A` | Input (ExtI) | Encoder Roda Kanan Channel A (Interrupt Kuadratur) |
| `PA1` | `ENC_R_B` | Input (ExtI) | Encoder Roda Kanan Channel B |
| `PB1` | `IR_L` | Input (ADC1) | Sensor IR Samping Kiri (90°) |
| `PB0` | `IR_FL` | Input (ADC1) | Sensor IR Serong Kiri (45°) |
| `PA7` | `IR_FC` | Input (ADC1) | Sensor IR Tengah Depan (0°) |
| `PA3` | `IR_FR` | Input (ADC1) | Sensor IR Serong Kanan (45°) |
| `PA2` | `IR_R` | Input (ADC1) | Sensor IR Samping Kanan (90°) |
| `PA4` | `BAT_SENSE` | Input (ADC1) | Pembagi Tegangan Baterai 10k + 10k ($V_{bat} = 2 \times V_{pin}$) |
| `PB6` | `I2C1_SCL` | I2C Clock | Bus I2C Bersama: MPU6050 (`0x68`) & SSD1306 OLED (`0x3C`) |
| `PB7` | `I2C1_SDA` | I2C Data | Bus I2C Bersama: MPU6050 (`0x68`) & SSD1306 OLED (`0x3C`) |
| `PB10` | `BUZZER` | Output | Transistor Driver Buzzer Piezo (Active HIGH) |
| `PA11` | `LED1` | Output | Indikator LED 1 (Active HIGH) |
| `PB3` | `LED2` | Output | Indikator LED 2 (Active HIGH) |
| `PC13` | `LED_BOARD` | Output | LED Bawaan WeAct Black Pill (Active LOW) |
| `PB13` | `SW1` | Input (Pullup) | Tombol 1: Ganti Pilihan Mode Menu (1-5) |
| `PB12` | `SW2` | Input (Pullup) | Tombol 2: Pintasan Speed Tier (Mode 4) / Kalibrasi Cepat |
| `PC15` | `SW3` | Input (Pullup) | Tombol 3: Emergency Brake Darurat Instan |
| `PC14` | `SW4` | Input (Pullup) | Tombol 4: START / STOP Mode Aktif |

---

## 🏛 Arsitektur Perangkat Lunak

Program dirancang secara modular dan berorientasi objek dalam C++:

```
📁 micromouse_v0.1/
├── 📁 include/
│   ├── config.h       # Konfigurasi pin, dimensi fisik robot, konstanta kecepatan, Flash
│   ├── motors.h       # Driver H-Bridge DRV8833 (PWM, Brake, Coast, Sleep)
│   ├── encoders.h     # Interrupt kuadratur & konversi jarak milimeter
│   ├── sensors.h      # Filter EMA 5-channel ADC, deteksi dinding, error centering PD
│   ├── imu.h          # Driver I2C MPU6050 cepat, kalibrasi bias Z, integrasi yaw
│   ├── battery.h      # Pembacaan voltase 1S LiPo & kalkulasi persentase daya
│   ├── ui.h           # Debounce tombol SW1-SW4 & kendali LED indikator
│   ├── buzzer.h       # Notifikasi nada suara & melodi kemenangan
│   ├── display.h      # Driver OLED SSD1306 dashboard & grafik telemetri
│   ├── maze.h         # Solver Floodfill 16x16, persistensi Flash, kompresi jalur tercepat
│   └── navigator.h    # Primitif gerak: moveForwardCell, turn90, sprintDistance, smoothCurveTurn
└── 📁 src/
    ├── main.cpp       # Finite State Machine, menu kontrol, dispatching mode operasional
    └── *.cpp          # Implementasi masing-masing modul perangkat keras
```

---

## 🏁 Strategi Navigasi: Scan vs Fast Run

Sesuai kebutuhan robot dengan bodi panjang (119 mm) dan roda di belakang:

```
[ START DI SUDUT (0,0) ]
           │
           ▼
┌──────────────────────────────────────┐
│  MODE 3: SCAN MAZE (EKSPLORASI)      │
│  - Kecepatan sedang (75 PWM)         │
│  - Diskrit maju per 1 sel (180 mm)   │
│  - Dinding dibaca saat robot diam    │
│  - Belok 90° pivot dengan Gyro MPU   │
│  - TIDAK MENGULAR (Prioritas Akurasi)│
└──────────────────────────────────────┘
           │
           ▼ (Goal 7,7 - 8,8 tercapai)
┌──────────────────────────────────────┐
│  GOAL REACHED! SIMPAN FLASH MEMORY   │
│  - Peta 256 sel disimpan permanen    │
│  - Target Floodfill beralih ke (0,0) │
└──────────────────────────────────────┘
           │
           ▼ (Kembali ke Start)
┌──────────────────────────────────────┐
│  AUTO RETURN TO START (0,0)          │
│  - Kecepatan sedang aman (80 PWM)    │
│  - Memperkaya pemetaan dinding       │
│  - Tiba di (0,0) -> Putar ke UTARA!  │
└──────────────────────────────────────┘
           │
           ▼ (Robot siap di kotak Start, pilih Mode 4)
┌──────────────────────────────────────┐
│  MODE 4: FAST RUN (BALAPAN)          │
│  - Hitung Jalur Tercepat (A*)        │
│  - Kompresi sel lurus -> SPRINT 180  │
│  - Belokan 90° -> SLALOM CURVE 90°   │
│  - Zig-Zag Kompresi -> DIAGONAL RUN  │
│    * Kurva Masuk 45° Halus (Entry)   │
│    * Sprint Diagonal (254.56 mm/step)│
│    * Post Balancing Sensor IR Serong │
│    * Kurva Keluar 45° Halus (Exit)   │
│  - Tiba di Finish -> Auto Return     │
│    kembali ke (0,0) dengan tenang    │
└──────────────────────────────────────┘
```

### Perbandingan Karakteristik Operasi:

| Parameter | Mode 3: Scan Maze (Eksplorasi) | Mode 4: Fast Run (Balapan) |
| :--- | :--- | :--- |
| **Tujuan Utama** | Pemetaan dinding 100% akurat tanpa crash | Waktu tempuh tercepat mencapai pusat labirin |
| **Kecepatan Maju** | `75.0` PWM (Tenang, tanpa slip roda) | `185.0` PWM (Sprint akselerasi trapesium) |
| **Tipe Belokan** | **Pivot di tempat 90° / 180°** | **Slalom Curve 90° & Kurva Landai 45°** |
| **Kecepatan Belok** | `65.0` PWM (Toleransi gyro ketat $\pm 1.5^\circ$) | `125.0` - `130.0` PWM (Kurva dinamis berkesinambungan) |
| **Lintasan Diagonal** | Tidak (Ortogonal demi pemetaan dinding) | **Ya (Sprint 45° hipotenusa $\mathbf{254.56\text{ mm}}$/step)** |
| **Radius Belok** | $R = 0\text{ mm}$ (Pivot poros roda) | $R = 90\text{ mm}$ (90°) / $R = 120\text{ mm}$ (45°) |
| **Rasio Roda Belok** | Roda kiri/kanan berlawanan arah (+/-) | Kurva 90°: Roda dalam $41\%$, Kurva 45°: Roda dalam $58\%$ |
| **Pemberhentian** | Berhenti 50 ms tiap sel untuk sampling IR | **Tanpa berhenti** dari start hingga finish |
| **Auto Clearance** | Ya (Mundur 20 mm jika dekat dinding depan) | Jalur bebas hambatan dihitung di awal |

---

## 🐍 Kinematika Navigasi Diagonal & Gerakan Mengular Mulus

Penerapan lintasan diagonal pada Micromouse memberikan efisiensi gerak dan fluiditas berkendara yang drastis:

```
[ LINTASAN ZIG-ZAG ORTOGONAL ]             [ LINTASAN DIAGONAL MENGULAR MULUS ]
       (Kasar, Patah-patah)                       (Fluid, Cepat & Efisien)

       ┌──────┐                                           ▲
       │      │                                          ╱  Kurva Keluar 45°
       │      │  (360 mm)                               ╱
┌──────┘      │                                        ╱   Sprint Diagonal
│             │                                       ╱    (254.56 mm, -29.3%)
│             ▼                                      ╱
                                            Kurva   ╱
                                            Masuk  ╱
                                             45°  ▲
```

### 1. Keuntungan Utama Gerakan Diagonal:
- **Pengurangan Jarak Tempuh Hingga 29.3%**:
  Pada labirin standar $180\text{ mm}$, melintasi pola zig-zag ortogonal (maju 1 sel + belok + maju 1 sel) menempuh jarak $180 + 180 = \mathbf{360\text{ mm}}$. Dengan lintasan diagonal, robot langsung melintasi garis miring (hipotenusa) sel:
  $$\text{Jarak Diagonal} = \sqrt{180^2 + 180^2} = \mathbf{254.56\text{ mm}}$$
- **Eliminasi Gaya Sentrifugal Berulang**:
  Gerakan mengular zig-zag 90° yang tajam menciptakan hentakan bolak-balik ke kiri dan ke kanan yang dapat menyebabkan roda selip dan bodi robot tidak stabil. Dengan kurva masuk 45° yang landai, momentum robot tetap terjaga mulus (*fluid*) ke arah diagonal.
- **Transisi 45° Berkelanjutan**:
  Kurva masuk dan keluar 45° menggunakan rasio diferensial roda $58\%$ pada kecepatan `125 PWM` dan dikontrol oleh integrasi yaw MPU6050, sehingga robot tidak perlu menghentikan lajunya saat berganti dari orientasi ortogonal ke diagonal.

### 2. Optical Post Balancing (Penyeimbangan Tiang Labirin):
Saat robot melaju diagonal di antara sel-sel labirin, sensor serong 45° (`IR_FRONT_L` di `PB0` dan `IR_FRONT_R` di `PA3`) berada pada sudut pandang yang **persis berhadapan dengan pasak/tiang pojok labirin**:
- Jika robot bergeser terlalu dekat ke tiang kiri, sensor `IR_FRONT_L` akan mendeteksi kenaikan intensitas pantulan ADC ($>1400$).
- Algoritma kemudi secara instan mengoreksi PWM roda untuk mengarahkan robot kembali ke sumbu tengah diagonal:
  $$\Delta \text{Steering} = (\text{ADC}_{FL} - \text{ADC}_{FR}) \times K_{post} + \text{YawError} \times K_{gyro}$$
- Hasilnya, robot melesat lurus di antara celah-celah tiang labirin dengan stabilitas tinggi tanpa risiko menabrak sudut tiang.

---

## 📐 Pelurusan Otomatis Tanpa IMU (Trigonometric Wall Auto-Alignment)

Robot dilengkapi algoritma **Self-Alignment** cerdas yang mampu **meluruskan posisinya secara mandiri tanpa bergantung pada IMU** saat diletakkan miring di dalam sel labirin:

```
[ ROBOT DILETAKKAN MIRING ]                  [ AUTO-ALIGN SELESAI ]
 (d_FL mendekat, d_FR menjauh)                (Parallel 100% Sejajar Dinding)

      Dinding Kiri                                Dinding Kiri
    │              │                            │              │
    │    ╲        │                            │     │        │
    │     ╲ FL    │  Trigonometri Optik        │  FL │ FR     │
    │   L  ╲  FR  │ ──────────────────────►    │ L   │   R    │
    │       ╲     │   (Putar Mikro Otomatis)   │     │        │
    │        R    │                            │              │
    │              │                            │              │
      Dinding Kanan                               Dinding Kanan
                                                KUNCI IMU YAW = 0.00°!
```

### 1. Pembuktian Matematis Trigonometri Sensor:
Sensor serong depan 45° (`IR_FRONT_L`, `IR_FRONT_R`) berada di depan poros roda ($L_{front} \approx +20\text{ mm}$), sedangkan sensor samping 90° (`IR_LEFT`, `IR_RIGHT`) berada di dekat as roda belakang ($L_{side} \approx -58\text{ mm}$).

Ketika robot ditaruh miring dengan sudut $\theta$ dan bergeser secara lateral sejauh $x$:
- Selisih jarak sensor depan: $(d_{FL} - d_{FR}) = 2x + 2 L_{front} \cdot \sin\theta$
- Selisih jarak sensor samping: $(d_{L} - d_{R}) = 2x + 2 L_{side} \cdot \sin\theta$

Jika kedua persamaan dikurangkan:
$$\text{Optical Angle Error } e_\theta = (d_{FL} - d_{FR}) - (d_{L} - d_{R}) = \mathbf{2 (L_{front} - L_{side}) \cdot \sin\theta}$$

> 💡 **Kesimpulan Matematis**: Pergeseran posisi robot ($x$) **tereliminasi 100%**! Nilai $e_\theta$ murni mengukur sudut kemiringan $\theta$ terhadap dinding, terbebas dari apakah robot ditaruh lebih dekat ke dinding kiri atau kanan.

### 2. Prosedur Pelurusan Otomatis & Penguncian IMU:
1. **Mechanical Tail Squaring**: Robot mundur perlahan 140 ms (-42 PWM) menyentuh dinding belakang sel start agar pantat robot rata dengan dinding.
2. **Optical Micro-Alignment**: Robot membaca $e_\theta$ dan memutar kedua motornya secara halus (32-48 PWM) dengan closed-loop PD hingga $e_\theta \approx 0$ (paralel sempurna dengan lorong).
3. **IMU Heading Zero Lock**: Begitu badan robot terbukti sejajar 100% dengan dinding labirin secara optik, robot memanggil [`IMU::calibrate()`](file:///c:/Users/Pramudya%20Wibowo/OneDrive/Dokumen/PlatformIO/Projects/micromouse_v0.1/src/imu.cpp#L48) dan [`IMU::resetYaw()`](file:///c:/Users/Pramudya%20Wibowo/OneDrive/Dokumen/PlatformIO/Projects/micromouse_v0.1/src/imu.cpp#L103) untuk **mengunci sumbu heading Yaw $= 0.00^\circ$ persis sejajar dengan grid arena**!

---

## ⚖ Sensor Fusion: Peran MPU6050 vs Encoder

Sistem navigasi menggunakan arsitektur **Cascaded Dual-Loop PID Controller** dengan **S-Curve (Jerk-Limited) Velocity Profile**:

```
                 ┌────────────────────────────────────────────────────────┐
                 │          CASCADED DUAL-LOOP PID CONTROLLER            │
                 ├────────────────────────────────────────────────────────┤
                 │                                                        │
Target Jarak ───►│ [LOOP 1: TRANSLASI S-CURVE] ────► PWM Cruise (Base)   │
(Tiap Sel/Multi) │  • Smoothstep Hermite Ramping                          │
                 │  • Bebas selip mikro roda 44mm                         │
                 │                                                        │
Target Sudut ───►│ [LOOP 2: ROTASI & ANTI-WOBBLE] ──► PWM Diff (Steering) │
(Dinding / Gyro) │  • P-Term: Sensor Dinding ADC / Gyro Yaw Z             │
                 │  • D-Term: Laju perubahan error dinding (de/dt)        │
                 │  • Active Gyro Z Damping: Meredam osilasi zig-zag      │
                 └───────────────────────────┬────────────────────────────┘
                                             ▼
                         Motor Kiri  = PWM Base + PWM Diff
                         Motor Kanan = PWM Base - PWM Diff
```

1. **Mengapa Belokan Menggunakan MPU6050?**
   Roda karet micromouse di atas triplek labirin mudah mengalami selip mikro saat berputar cepat. Jika belokan 90° hanya mengandalkan perhitungan selisih tick encoder, akumulasi kesalahan sudut akan membuat robot menabrak tiang setelah beberapa tikungan. Gyro Z membaca rotasi inersial mutlak tanpa terpengaruh kondisi permukaan lantai.

2. **Mengapa Jarak Menggunakan Encoder?**
   Integrasi ganda akselerometer IMU untuk menghitung posisi memiliki efek *drift* yang besar dalam hitungan detik. Encoder magnetik memberikan akurasi diskrit mutlak per milimeter perjalanan lurus.

3. **Bagaimana Jika Menggunakan Motor N20 Tanpa Encoder (*Encoderless Mode*)?**
   Firmware memiliki sistem **Dual-Mode Auto-Detect** (`ENCODER_HARDWARE_CONFIG`):
   - **Auto-Detection**: Saat motor pertama kali berputar maju ~100 ms, sistem mengecek apakah ada pulsa interrupt encoder. Jika tidak ada (kabel tidak dicolok / motor N20 polos), sistem langsung beralih ke mode *Encoderless* secara otomatis tanpa berhenti.
   - **Estimasi Jarak Waktu Terkompensasi Voltase**: Kecepatan motor N20 berubah seiring melemahnya baterai (4.2V ke 3.7V). Sistem membaca voltase baterai aktual melalui pin ADC `PA4` dan mengompensasi durasi tempuh per sel (`TIME_PER_CELL_NOMINAL_MS = 560 ms`) secara *real-time*.
   - **Koreksi Dinding Depan (*Front Wall Lock*)**: Ketika robot mendekati dinding depan sel berikutnya, sensor IR depan (`frontCenter > 2200`) mengambil alih untuk memposisikan robot tepat di tengah sel, mengeliminasi galat akumulasi jarak waktu.
   - **Belokan 90° & Mengular Tetap Akurat 100%**: Karena belokan pivot 90° dan slalom kurva mengular dikontrol secara murni oleh **MPU6050 Gyro Z**, ketiadaan encoder tidak mempengaruhi akurasi manuver belok sama sekali.

---

## 🎮 Antarmuka Pengguna (Tombol, OLED, Buzzer, LED)

### 1. Alur Operasi Tombol Fisik (One-Click Start di Arena)

Sistem dirancang untuk kenyamanan maksimal saat lomba: **Pilih mode sebelum robot ditaruh $\rightarrow$ Taruh robot di sel Start $\rightarrow$ Tinggal pencet START (`SW4`)!**

1. **SEBELUM ROBOT DITARUH (Di Tangan / Meja Persiapan)**:
   - Nyalakan saklar baterai.
   - **Mode Cerdas Otomatis**: Robot otomatis memilih mode yang tepat!
     - Jika labirin belum pernah discan $\rightarrow$ Otomatis memilih **Mode 3 (Scan Maze)**.
     - Jika peta labirin sudah tersimpan di Flash $\rightarrow$ Otomatis memilih **Mode 4 (Fast Run)**.
   - Jika ingin mengganti mode (1-5), cukup tekan tombol **`SW1` (PB13)**. Buzzer akan membunyikan bip $N\times$ sesuai mode yang dipilih.
2. **TARUH ROBOT DI ARENA**:
   - Letakkan robot di sel sudut Start `(0,0)` merapat ke dinding belakang, menghadap ke **UTARA**.
3. **TINGGAL PENCET START (SW4)**:
   - Tekan tombol **`SW4` (PC14)** satu kali!
   - Robot membunyikan **Hitung Mundur 2 Detik** (*Bip... Bip... GO!*):
     - *Detik 1*: Memberikan waktu bagi Anda untuk menarik tangan sepenuhnya menjauhi bodi dan sensor robot (Ambient Calibration).
     - *Detik 2*: Robot diam di lantai sel untuk mengunci kalibrasi **nol-bias Gyro MPU6050** (`Yaw = 0.00°`) secara presisi di atas lantai arena.
     - *GO*: Nada tinggi dan robot langsung melesat menjalankan misi tanpa gangguan tangan sama sekali!
4. **Fitur Tombol Lainnya**:
   - **`SW3` (PC15)**: **Emergency Brake** instan memotong PWM kedua motor seketika jika terjadi kendala.
   - **`SW2` (PB12)**: **Pintasan Speed Tier (Mode 4)** / **Kalibrasi Cepat** (IR & Gyro) tanpa perlu masuk menu.
   - **`SW4` (PC14)**: Berfungsi ganda sebagai toggle **START / STOP** (menghentikan laju robot jika ditekan kembali saat robot sedang berjalan).

### 2. Tampilan Menu OLED (SSD1306)
- **Menu Utama**: Status mode yang dipilih, voltase baterai, persentase baterai, dan kesiapan peta (*READY FOR FAST RUN*).
- **Diagnostics Screen**: Menampilkan nilai numerik ADC 5 sensor IR, grafik batang langsung, status deteksi dinding `[L F R]`, sudut Yaw saat ini, dan jarak odometer encoder.
- **Maze Screen**: Koordinat sel robot $(X,Y)$, orientasi mata angin (N, E, S, W), aksi Floodfill berikutnya, serta animasi sel.
- **Fast Run Screen**: Nomor langkah yang sedang dijalankan, total langkah, perintah (*SPRINT*, *MENGULAR <*, *MENGULAR >*), dan jarak lintasan.

### 3. Konfigurasi Saklar Status LED (`ENABLE_LEDS`) & Indikator Buzzer
Pada [`include/config.h`](file:///c:/Users/Pramudya%20Wibowo/OneDrive/Dokumen/PlatformIO/Projects/micromouse_v0.1/include/config.h#L55), Anda dapat menyetel saklar:
```cpp
#define ENABLE_LEDS true  // true = Aktifkan LED kanan-kiri | false = Matikan LED total (Gunakan Buzzer)
```

- **Jika `ENABLE_LEDS == true`**:
  - **LED1 (PA11 / Kiri)** dan **LED2 (PB3 / Kanan)** aktif sebagai indikator visual mode:
    - **Mode 1 (Diagnostics)**: LED1 ON, LED2 OFF *(juga menampilkan deteksi dinding kiri & kanan saat running)*
    - **Mode 2 (Wall Follower)**: LED1 OFF, LED2 ON
    - **Mode 3 (Scan Maze)**: LED1 ON, LED2 ON (Keduanya Nyala)
    - **Mode 4 (Fast Run)**: LED1 OFF, LED2 OFF (Keduanya Mati)
    - **Mode 5 (Calibrate)**: Berkedip bersamaan 3×
- **Jika `ENABLE_LEDS == false`**:
  - LED1 dan LED2 dipadamkan total (*Always OFF*).
  - Indikator mode dialihkan sepenuhnya ke **Buzzer** saat tombol `SW1` ditekan:
    - **Mode 1**: 1 bip
    - **Mode 2**: 2 bip
    - **Mode 3**: 3 bip
    - **Mode 4**: 4 bip
    - **Mode 5**: 5 bip
- **Umpan Balik Akustik Buzzer**: Setiap kali tombol `SW1` ditekan, Buzzer **SELALU berbunyi bip $N\times$** (Mode 1 = 1 bip, Mode 2 = 2 bip, Mode 3 = 3 bip, Mode 4 = 4 bip, Mode 5 = 5 bip) sebagai konfirmasi audio yang jelas dan tegas baik saat LED menyala maupun saat LED dimatikan.
- **Onboard LED (PC13)**: Berkedip perlahan (*heartbeat*) sebagai tanda mikrokontroler STM32 berjalan normal.
- **Catatan Pin Hardware PA11 (LED1)**: LED1 berbagi jalur dengan USB Data (`USB_DM`). Pastikan `#define ENABLE_SERIAL_DEBUG false` di [`config.h`](file:///c:/Users/Pramudya%20Wibowo/OneDrive/Dokumen/PlatformIO/Projects/micromouse_v0.1/include/config.h#L50) saat robot dijalankan dengan baterai agar pin PA11 berfungsi sebagai GPIO output untuk menyalakan LED1.
- **Startup**: Nada arpeggio 4 nada saat daya dinyalakan.
- **Click**: Nada klik pendek saat tombol menu ditekan.
- **Confirm**: Nada ganda saat robot mulai berjalan atau kalibrasi sukses.
- **Alert**: Bunyi bip peringatan (3 bip bernada rendah) jika rem darurat ditekan atau terjadi tabrakan fatal.
- **Victory Melodi**: Melodi kemenangan bernada ceria saat robot berhasil mencapai kotak Finish labirin.

---

## 💾 Penyimpanan Memori Flash Non-Volatile

Untuk mempertahankan peta labirin yang telah dipelajari setelah robot dimatikan:
- **Lokasi Memori**: STM32F411 Flash **Sector 7** beralamat di `0x08060000` (Ukuran 128 KB).
- **Keamanan Firmware**: Program firmware hanya menggunakan ~67 KB (menempati Sector 0 hingga Sector 2). Sector 7 berada di ujung kapasitas Flash 512 KB, sehingga aman dari tertimpa kode program.
- **Struktur Data**:
  ```cpp
  struct FlashMazeStorage {
      uint32_t magicNumber; // 0xA55A2026 (Validasi integritas data)
      uint8_t  walls[256];  // Bitmask dinding untuk seluruh 16x16 sel
      uint16_t checksum;    // Verifikasi error
  };
  ```
- **Prosedur**:
  - Saat robot mencapai target pada Mode 3, fungsi [`Maze::saveToFlash()`](file:///c:/Users/Pramudya%20Wibowo/OneDrive/Dokumen/PlatformIO/Projects/micromouse_v0.1/src/maze.cpp#L162-L195) menghapus Sector 7 via HAL Flash Erase dan memprogram 256 byte data dinding.
  - Saat booting, robot memverifikasi `magicNumber`. Jika valid, menu menampilkan `[READY FOR FAST RUN]`.

---

## 💻 Perintah Serial CLI

Robot dapat dipantau dan dikendalikan melalui kabel USB Type-C virtual COM port (115200 baud):

| Perintah | Deskripsi |
| :--- | :--- |
| `menu` / `help` | Menampilkan bantuan ringkas dan status memori peta |
| `diag` / `1` | Mengaktifkan Mode 1 (Streaming data sensor live) |
| `wall` / `2` | Mengaktifkan Mode 2 (Wall Follower sederhana) |
| `scan` / `3` | Mengaktifkan Mode 3 (Scan Floodfill & Simpan Flash) |
| `fast` / `4` | Mengaktifkan Mode 4 (Fast Run Jalur Tercepat & Mengular) |
| `cal`  / `5` | Menjalankan kalibrasi sensor di tempat |
| `map` | Mencetak peta labirin 16×16 ke terminal serial format ASCII |
| `stop` / `s` | Menghentikan laju robot seketika (*Brake*) |
| `tier` / `speed` | Mengganti Fast Run Speed Tier (Tier 1 -> Tier 2 -> Tier 3) |
| `tier1` / `safe` | Menyetel Fast Run ke Tier 1 (Safe, 180 PWM sprint) |
| `tier2` / `aggr` | Menyetel Fast Run ke Tier 2 (Aggressive, 220 PWM sprint) |
| `tier3` / `insane` | Menyetel Fast Run ke Tier 3 (Insane Attack, 250 PWM sprint) |

---

## 🛠 Panduan Kompilasi & Pengunggahan

### Prasyarat
1. Terpasang [Visual Studio Code](https://code.visualstudio.com/) + Ekstensi [PlatformIO IDE](https://platformio.org/).
2. Mikrokontroler WeAct Black Pill terhubung via kabel USB Type-C (USB DFU Bootloader) atau ST-Link V2.
3. **Khusus Pengguna Linux**:
   Agar user Linux dapat mengakses USB STM32 DFU dan serial monitor tanpa masalah izin (*permission denied*), jalankan skrip konfigurasi udev sekali:
   ```bash
   sudo ./scripts/setup_udev.sh
   ```

### Kompilasi Proyek
Buka terminal di direktori proyek dan jalankan:
```bash
pio run
```
*Hasil kompilasi tipikal:*
- RAM: `~6.096 Bytes` (~4.7% dari 128 KB pada F411CE / ~9.3% pada F401CC)
- Flash: `~79.080 Bytes` (~15.1% dari 512 KB pada F411CE / ~29.1% pada F401CC)

### Mengunggah Firmware (Upload)

#### Metode 1: USB DFU Bootloader (Default - Tanpa ST-Link, Cukup Kabel Type-C)
1. Hubungkan Black Pill ke komputer via kabel USB Type-C.
2. Masuk ke **Mode DFU**:
   - **Tahan tombol `BOOT0`**
   - **Tekan dan lepas tombol `NRST` (Reset)**
   - **Lepaskan tombol `BOOT0`**
   *(Di Linux, dapat dipastikan dengan perintah `lsusb` yang menampilkan `0483:df11 STMicroelectronics STM Device in DFU Mode`)*
3. Jalankan perintah upload (otomatis mendeteksi Linux/Windows dan alat upload):
   ```bash
   pio run -t upload
   ```
   *Skrip otomatis menggunakan `STM32CubeProgrammer CLI` atau fallback ke `dfu-util` serta langsung menjalankan program (`-s 0x08000000`).*

#### Metode 2: ST-Link V2 (SWD Programmer)
Hubungkan programmer ST-Link ke pin `SWDIO`, `SWCLK`, `GND`, `3V3` pada Black Pill, lalu jalankan:
```bash
pio run -e blackpill_f411ce_stlink -t upload
```

---

## 🎯 Prosedur Kalibrasi Lapangan

Sebelum memulai perlombaan, lakukan kalibrasi di dalam sel start:
1. Tempatkan robot persis di tengah sel sudut awal (0,0).
2. Tekan tombol **`SW2`** (atau pilih menu **Mode 5** lalu tekan **`SW4`**).
3. **Jangan menyentuh atau menggerakkan robot selama 2 detik**:
   - Robot mengambil 200 sampel gyro Z untuk menghitung nilai offset nol gravitasi bumi (*zero-rate bias*).
   - Robot mengukur intensitas cahaya sekitar (*ambient light*) dan pantulan dinding standar untuk mengunci threshold IR samping dan depan.
4. Buzzer akan membunyikan nada konfirmasi dan layar OLED menampilkan `CALIBRATION OK`.
5. Robot kini siap mengeksplorasi labirin!

---

## 🏆 7 Fitur Unggulan Tingkat Kompetisi (Competition Ready)

1. **Wall Edge & Post Peaking Longitudinal Odometry Reset**:
   - Deteksi *falling-edge* pantulan IR samping ($\Delta\text{ADC} < -600$) saat robot melewati ujung tiang/pilar kisi labirin ($180\text{ mm}$).
   - Mereset akumulator jarak translasi encoder ke kelipatan pas $N \times 180\text{ mm}$, menghilangkan galat kumulatif jarak akibat selip ban silikon saat sprint kencang.

2. **Deteksi Tabrakan & Anti-Stall Failsafe**:
   - Memantau kondisi motor saat menerima sinyal daya tinggi (PWM $> 70$) namun roda tidak bergerak ($< 2\text{ mm}$ dalam $220\text{ ms}$).
   - Otomatis memutus arus motor (*emergency cut-off*), menyalakan buzzer alarm, dan menampilkan peringatan di layar OLED untuk mencegah *gearbox stripping* dan lonjakan arus berlebih.

3. **Proteksi Baterai LiPo Cut-off (Critical Under-Voltage)**:
   - Mengunci start motor jika tegangan baterai terdeteksi di bawah $3.30\text{V}$ ($< 3.30\text{V}$ per sel LiPo 1S).
   - Melindungi sel baterai dari kerusakan permanen (*swelling / over-discharge*) akibat *voltage sag* saat akselerasi tinggi.

4. **Auto Ambient-Light Calibration & Extended Front-Wall Detection**:
   - Pengambilan 60 sampel cahaya lingkungan sekitar secara otomatis selama jeda hitung mundur 2 detik sebelum start.
   - Ambang batas (*threshold*) sensor IR depan menyesuaikan secara adaptif (`ambientFC + 90 ADC`, clamped antara 170 hingga 200 ADC) sehingga robot mampu mendeteksi dinding depan dari jarak jauh (~100–120 mm).
   - Dilengkapi sistem konfirmasi ganda (*dual-condition*): jika sensor tengah mendeteksi kenaikan pantulan awal ($\ge 140\text{ ADC}$) dan kedua sensor serong 45° (`FL` & `FR`) menangkap pantulan dinding depan, dinding langsung terdeteksi valid sebelum robot tiba di dekat dinding.
   - Sistem deselerasi halus (*smooth deceleration*) otomatis mengurangi kecepatan maju dari cruise speed ke min speed saat mendekati dinding depan dan mengerem presisi pada $\ge 700\text{ ADC}$.

5. **Multi-Tier Fast Run Speed Presets**:
   - Pengguna dapat memilih preset kecepatan Mode 4 secara instan menggunakan tombol **`SW3`**:
     - **Tier 1 (Safe)**: 180 PWM sprint (~160 mm/s), 140 PWM kurva (Aman & stabil tanpa risiko tergelincir).
     - **Tier 2 (Aggressive - Default)**: 220 PWM sprint (~200 mm/s), 170 PWM kurva (Optimal untuk kompetisi).
     - **Tier 3 (Insane Attack)**: 250 PWM sprint (~228 mm/s), 195 PWM kurva (Kecepatan puncak mutlak untuk memecahkan rekor waktu tercepat).

6. **Turn-Penalty Floodfill (Prioritas Koridor Lurus Panjang)**:
   - Solver Floodfill kini memberikan penalti jarak virtual (`TURN_PENALTY_COST = 15`) untuk setiap belokan 90° dan 30 untuk belokan 180°.
   - Menghasilkan jalur tercepat yang memprioritaskan koridor lurus panjang daripada jalur zig-zag, sehingga robot dapat melaju pada kecepatan penuh lebih lama.

7. **Stopwatch Lap Timer & OLED 16×16 Mini-Map**:
   - Pengukuran waktu tempuh presisi milidetik sejak tombol start dilepaskan hingga robot menyentuh garis finish.
   - Layar OLED SSD1306 menampilkan peta grafis labirin $16\times 16$ (64×64 piksel canvas) dengan posisi real-time robot dan kotak finish secara visual, serta hasil waktu tempuh lap (contoh: `07.842s`).
