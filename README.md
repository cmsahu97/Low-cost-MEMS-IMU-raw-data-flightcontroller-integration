# An Integration of Low-Cost MEMS Inertial Sensor on Flightcontroller

This repository provides a complete, production-ready embedded C++ engineering pipeline that integrates a low-cost MEMS inertial sensor (**LSM9DS1**) into ArduPilot flight controllers (**Cube Orange+** / **Pixhawk 6C**). By emulating the industrial **SBG Systems ECom Binary Protocol** over hardware UART, this system functions as a high-frequency, reliable **External AHRS (Attitude and Heading Reference System)** node.

### 📊 Raw Sensor Utilization Profile
The underlying firmware ingests raw, independent data streams from the external MEMS inertial sensor and transforms them into standard aerospace vectors for native Extended Kalman Filter (EKF3) processing:
* **Raw Gyroscope ($g_x, g_y, g_z$):** Sampled continuously at 250Hz. After processing a rigid power-on zero-bias calibration, these high-frequency angular rates are integrated into delta angles ($\Delta\theta$) to handle rapid rotational velocity tracking and immediate attitude stabilization.
* **Raw Accelerometer ($a_x, a_y, a_z$):** Captured at 250Hz and mapped to the standard Forward-Right-Down (FRD) airframe configuration. These linear acceleration metrics are mathematically processed into delta velocities ($\Delta V$) to track translational motion changes, vibrations, and physical tilt constraints.
* **Raw Magnetometer ($m_x, m_y, m_z$):** Interleaved into the streaming pipeline at a stable 50Hz cadence. This tri-axial magnetic flux density data acts as an absolute earth-field reference vector, continuously mitigating long-term heading gyroscopic drift and locking in real-world true-North heading alignment.
---

## 📊 System Performance & Demonstrations

### Hardware Circuit Configuration
Below is the verified electrical routing topology linking the STM32 development platform to the MEMS sensor array over Fast-Mode I2C and out to the flight controller transceiver.

![Circuit Diagram Matrix](Circuit_diagram.svg)

### System Operation Overview
Below is a demonstration of the system booting up, completing its initial gyro calibration matrix, and streaming raw high-speed data directly into the flight controller's EKF3 state estimation engine without dropping frames.

<p align="center">
  <img src="video_demonstration.gif" width="48%" alt="System Demonstration" />
</p>

---

## 🔬 Core System Architecture
* **IMU Sampling Frequency:** 250 Hz (Hard-locked hardware loop executing every exactly 4,000 µs).
* **Magnetometer Frequency:** 50 Hz (Interleaved seamlessly every 5th frame to prevent IMU data pacing disruptions).
* **Communication Interface:** Packed Binary SBG Systems ECom (`EAHRS_TYPE = 8`).
* **Baud Rate Configuration:** 230,400 bps (Downshifted specifically to prevent Direct Memory Access [DMA] ring-buffer packet-chopping on STM32H7 processors).
* **Kinematic Alignment:** Native physical sensor coordinate mapping to the aerospace standard **FRD (Forward-Right-Down)** airframe frame.

---

## 🛠️ Hardware Requirements & Pinout Connections

### 1. System Components
* **Microcontroller:** STM32F411CE or STM32F401CC "Black Pill" development board.
* **MEMS Inertial Unit:** Adafruit LSM9DS1 Breakout (9-DoF Accelerometer, Gyroscope, Magnetometer).
* **Target Flight Controller:** Cube Orange+ (or equivalent hardware running ArduCopter V4.7+ Beta firmware).

### 2. Wiring Matrix

| Black Pill Pin | Target Device Pin | Signal Type | Description |
| :--- | :--- | :--- | :--- |
| **5V / VCC** | LSM9DS1 VIN & Cube GPS1 5V | Power | Common 5V System Power Rail |
| **GND** | LSM9DS1 GND & Cube GPS1 GND | Ground | Common Ground Reference |
| **PB6** | LSM9DS1 SCL | I2C Clock | 400kHz Fast-Mode I2C Serial Clock Bus |
| **PB7** | LSM9DS1 SDA | I2C Data | 400kHz Fast-Mode I2C Serial Data Bus |
| **PA2 (TX1)** | Cube GPS1 Port **RX Pin** | UART TX | High-Speed Packed Binary Transmission Line |
| **PA3 (RX1)** | Cube GPS1 Port **TX Pin** | UART RX | Receive (Used for physical loopback testing) |


---

## 📂 Repository File Structure

* **`README.md`** - This comprehensive integration and configuration manual.
* **`external_IMU_main.cpp`** - The core high-speed embedded execution firmware containing the packed binary structs, dynamic dt loop calculus filters, and I2C registers routing.
* **`Circuit_diagram.svg`** - Complete schematics and physical wiring layout between the MCU, IMU, and Autopilot.
* **`video_demonstration.mp4`** - Physical hardware wiring validation video clip.



---

## ⚙️ ArduPilot Parameter Configuration Matrix
Connect to your flight controller via Mission Planner, navigate to **Config > Full Parameter List**, and apply the following parameters to bind the internal estimation architecture to your external hardware node:

### 1. Serial Port Route Mapping (Physical GPS1 Port)
* `SERIAL3_PROTOCOL` = `36` *(Assigns External AHRS Driver Interface to the port)*
* `SERIAL3_BAUD` = `230` *(Sets hardware communication clock bounds to 230,400 bps)*

### 2. External AHRS Activation Engine
* `AHRS_EKF_TYPE` = `3` *(Enforces Kalman Filter 3 execution)*
* `EAHRS_TYPE` = `8` *(Activates the native SBG Systems hardware binary parser)*

### 3. Flight Core Sensor Optimization (The Anti-Rejection Suite)
To keep the Cube's high-speed internal isolated IMUs from out-voting or cutting off your custom stream via the lane configuration manager, apply these values:
* `INS_USE2` = `0` *(Blinds the flight core to internal sensor board 2)*
* `INS_USE3` = `0` *(Blinds the flight core to internal sensor board 3)*
* `EK3_IMU_MASK` = `1` *(Forces EKF3 calculations exclusively onto Lane 1/External IMU0)*
* `EK3_GYR_NOISE` = `0.05` *(Relaxes math validation windows to fit custom breadboard noise envelopes)*
* `EK3_ACC_NOISE` = `0.50` *(Relaxes acceleration noise acceptance thresholds)*

### 4. Boot-Race Coordination
* `BRD_BOOT_DELAY` = `4000` *(Forces ArduPilot to pause 4 seconds at boot, letting the Black Pill complete internal gyro sweeps)*
* `INS_GYR_CAL` = `0` *(Bypasses second-stage gyro initialization since the Black Pill calibrates natively at boot)*

---

## 🛠️ Step-by-Step Deployment Checklist

1. Clone this repository and flash the code inside `external_IMU_main.cpp` onto your Black Pill using PlatformIO or Arduino IDE.
2. Rigidly attach the **Black Pill**, the **LSM9DS1**, and your **Flight Controller** to a single, un-bendable testing frame or plate. *Warning: If these components move independently, the EKF will trigger a safety fallback and drop the sensor stream.*
3. Wire the communication lines directly into the flight controller's physical **GPS1** port (`PA2` to `RX`).
4. Apply system power completely hands-off. Do not touch or vibrate the desk for the first 5 seconds to allow the system to record its zero-bias gyro offsets.
5. In Mission Planner, go to **Setup > Mandatory Hardware > Accel Calibration** and run **Calibrate Level** to wipe out mechanical alignment mounting tilts.
6. Configure your aircraft geometry under `FRAME_CLASS` and `FRAME_TYPE` to clear structural failsafes.

---

## 📋 Troubleshooting & Common Diagnostics

* **Symptom: Horizon stays alive for a few seconds, then completely freezes.**
  * *Cause:* Kinematic Contradiction. The flight controller is sitting flat on your table while you are holding and shaking the custom breadboard node with your hand. Ensure all units are physically locked to the same base plate.
* **Symptom: The console throws a persistent "Bad Gyro Health" message.**
  * *Cause:* Microscopic loop latency. Ensure any inline debug `Serial.print` blocks are completely commented out inside your main execution frames.
* **Symptom: Horizon is locked out right from boot-up, and MAVLink registers "No Data" on IMU0.**
  * *Cause:* Signal wiring issue. Double-check that your Black Pill's `PA2 (TX)` line is connected to the physical **RX pin** of the GPS1 connector block.

---
## 📄 License
Feel free to use, adapt, and deploy it on your commercial uncrewed vehicles! but give credit to this page. 
