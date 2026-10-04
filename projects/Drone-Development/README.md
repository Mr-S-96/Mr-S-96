# 🚁 ESP32 Quadcopter Flight Controller

> **Embedded Systems / Robotics / Flight Control** · **Firmware v2.0** · **Status: Working implementation / flight-testing stage**

A custom quadcopter flight-controller firmware project built around an **ESP32**, integrating an **MPU6050 IMU**, **BMP280 barometer**, **HMC5883L magnetometer**, **FlySky FS-iA6B iBUS receiver**, **SSD1306 OLED display**, and four ESC-driven brushless motors.

The current firmware implements receiver decoding, flight-mode selection, arming/disarming, sensor processing, complementary-filter attitude estimation, PID stabilization, X-configuration motor mixing, altitude-hold logic, telemetry/debug output, and signal-loss safety handling.

![Project Preview](preview.svg)

---

## 🛩️ Project Overview

This project is a custom embedded flight-control system rather than a simple motor-control demo.

The ESP32 acts as the flight controller and continuously:

1. Reads control commands from the **FlySky FS-iA6B receiver over iBUS**
2. Reads attitude data from the **MPU6050**
3. Reads altitude from the **BMP280**
4. Reads magnetometer data from the **HMC5883L**
5. Processes flight-mode and arming inputs
6. Runs the flight-control logic at a configured **200 Hz**
7. Calculates stabilization corrections using PID controllers
8. Mixes throttle, roll, pitch, yaw and altitude corrections for an X-frame
9. Generates **50 Hz ESC PWM signals**
10. Displays flight information on the **128×64 SSD1306 OLED**
11. Performs signal-loss and emergency-disarm handling

---

## ⚙️ Current Hardware Architecture

| Component | Role |
|---|---|
| **ESP32 WROOM-32** | Main flight controller |
| **FlySky FS-i6** | Radio transmitter |
| **FlySky FS-iA6B** | Receiver / iBUS input |
| **MPU6050** | Gyroscope + accelerometer |
| **BMP280** | Barometric altitude sensing |
| **HMC5883L** | Magnetometer / compass |
| **SSD1306 128×64 OLED** | Flight-status display |
| **4× Brushless Motors** | Propulsion |
| **4× ESCs** | Brushless motor control |
| **Li-Po battery / power system** | Flight power |

The firmware source explicitly defines the four motor outputs as GPIO **25, 26, 27 and 14**, with iBUS receiver input on GPIO **16** and I²C on GPIO **21/22**.

---

## 🔌 Pin Mapping

### Motor / ESC outputs

| Motor | Position | Rotation | ESP32 GPIO |
|---|---|---|---:|
| M1 | Front Left | CCW | GPIO 25 |
| M2 | Front Right | CW | GPIO 26 |
| M3 | Rear Right | CCW | GPIO 27 |
| M4 | Rear Left | CW | GPIO 14 |

### Receiver

| Signal | ESP32 |
|---|---|
| iBUS data | GPIO 16 / Serial2 RX |
| Receiver GND | ESP32 GND |
| Receiver +5V | 5V supply / ESC BEC |

### I²C sensors / display

| Signal | ESP32 |
|---|---|
| SDA | GPIO 21 |
| SCL | GPIO 22 |

The project firmware also expects the ESCs and ESP32 to share a **common ground**.

---

## 🎮 Radio Channel Mapping

| Channel | Function | Input |
|---|---|---|
| CH1 | Roll | Right stick left/right |
| CH2 | Pitch | Right stick forward/back |
| CH3 | Throttle | Left stick up/down |
| CH4 | Yaw | Left stick left/right |
| CH5 | Flight mode | 3-position switch |
| CH6 | Arm / Disarm | 2-position switch |

### Flight modes

- **ACRO** — direct manual rate control
- **ANGLE** — self-leveling using attitude feedback
- **ALT_HOLD** — self-leveling + barometer-based altitude hold
- **EMERGENCY** — immediate motor cutoff

---

## 🧠 Flight-Control Logic

### ACRO mode

Control inputs are mapped directly into roll, pitch and yaw control outputs.

### ANGLE mode

The controller calculates target roll and pitch angles, limited to approximately ±30°, and uses PID controllers to move the measured attitude toward those targets.

### ALT_HOLD mode

Altitude control builds on ANGLE mode and uses the BMP280 barometer.

The throttle stick modifies the altitude target, while the altitude PID produces an additional motor-mixing correction.

---

## 📐 Sensor Fusion

The MPU6050 accelerometer and gyroscope are combined using a **complementary filter**.

The current firmware uses approximately:

- **98% gyroscope contribution**
- **2% accelerometer contribution**

This provides a lightweight attitude-estimation approach suitable for the ESP32 firmware architecture.

Yaw is integrated from gyroscope Z-axis data in the current implementation.

---

## 🎛️ PID Controllers

The firmware contains independent PID controllers for:

- Roll
- Pitch
- Yaw
- Altitude

The current source initializes these with configurable gains, allowing the controller to be tuned for the actual airframe and propulsion system.

> ⚠️ PID values in the source should be treated as starting/tuning values, not universal flight-safe settings.

---

## 🔄 X-Configuration Motor Mixing

The four motors are mixed according to an X-frame configuration.

Conceptually:

```text
                 FRONT

          M1 ↻             ↺ M2
             \             /
              \     X     /
               \         /
               /         \
              /     X     \
             /             \
          M4 ↺             ↻ M3

                  REAR
```

The mixer combines:

- Throttle
- Roll correction
- Pitch correction
- Yaw correction
- Altitude correction

Each result is constrained to the configured **1000–2000 µs ESC pulse range**.

---

## 🛡️ Safety Features

The firmware contains several safety mechanisms:

- ✅ Throttle must be low before arming
- ✅ Dedicated arm/disarm switch
- ✅ Immediate motor idle on disarm
- ✅ Signal-loss detection
- ✅ Automatic disarm after approximately **1 second** without a valid iBUS frame
- ✅ Emergency-stop mode
- ✅ Motor idle below the minimum throttle threshold
- ✅ PID reset during arming
- ✅ Continuous receiver validation
- ✅ OLED status indication
- ✅ Serial debug telemetry

### ⚠️ Important

**Remove all propellers during firmware, wiring and motor testing.**

The source itself recommends bench testing, ESC calibration, PID tuning, a secure test stand, and a readily accessible kill switch before flight.

---

## 📟 OLED Telemetry

The SSD1306 display is used to show live flight-controller information including:

- Arm/disarm state
- Current flight mode
- Throttle
- Roll
- Pitch
- Yaw
- IMU roll/pitch
- Barometric altitude
- Four motor outputs
- Receiver signal status

Serial debug output is configured for **115200 baud**.

---

## 💻 Software / Libraries

The firmware is written for the **Arduino ESP32 environment** and uses:

- Arduino framework
- `Wire.h`
- Adafruit BMP280
- Adafruit Sensor
- Adafruit HMC5883 Unified
- I2Cdev
- MPU6050
- Adafruit GFX
- Adafruit SSD1306

---

## 🚀 Setup & Upload

### 1. Prepare the ESP32 environment

Install the ESP32 board support package in Arduino IDE and select the appropriate ESP32 board.

### 2. Install required libraries

Install the libraries used by the firmware:

```text
Adafruit BMP280 Library
Adafruit Unified Sensor
Adafruit HMC5883 Unified
I2Cdev / MPU6050
Adafruit GFX Library
Adafruit SSD1306
```

### 3. Upload the firmware

Open:

```text
ESP32_Drone_Complete_v2.ino
```

Select the ESP32 board and its serial port, compile, then upload.

### 4. Open Serial Monitor

Use:

```text
115200 baud
```

The controller reports initialization results for the receiver, sensors, motors and OLED.

### 5. Bench-test before flight

With **propellers removed**:

1. Power the controller.
2. Confirm the receiver is detected.
3. Confirm MPU6050 initialization.
4. Confirm the OLED starts.
5. Confirm ESC outputs remain at idle.
6. Verify arm/disarm operation.
7. Move the sticks and observe the telemetry.
8. Confirm motor numbering and rotation direction.
9. Only proceed to controlled testing after the complete system is verified.

---

## 📁 Project Structure

```text
Drone-Development/
├── README.md
├── preview.svg
└── ESP32_Drone_Complete_v2.ino
```

---

## 🔎 Engineering Notes

The uploaded component-planning document describes a different earlier architecture involving **two ESP32 boards, hand-motion MPU6050 control, ESP-NOW and eight coreless DC motors**. fileciteturn43file0L5-L20

The current **v2.0 firmware**, however, implements a different flight-controller architecture: **one ESP32, FlySky iBUS receiver, four brushless motors through ESCs, MPU6050, BMP280, HMC5883L and SSD1306**.

This README therefore documents the **actual v2.0 firmware architecture** rather than mixing the two designs together.

---

## 📊 Current Project Status

**Firmware implementation documented — hardware/flight validation remains an ongoing stage.**

### Implemented in the uploaded firmware

- [x] ESP32 flight-controller structure
- [x] FlySky iBUS frame parsing
- [x] Receiver channel mapping
- [x] Arm/disarm logic
- [x] Four-motor PWM generation
- [x] MPU6050 attitude estimation
- [x] Complementary filtering
- [x] ACRO mode
- [x] ANGLE mode
- [x] ALT_HOLD logic
- [x] Roll/Pitch/Yaw PID controllers
- [x] Altitude PID controller
- [x] X-frame motor mixing
- [x] BMP280 altitude reading
- [x] HMC5883L initialization
- [x] SSD1306 telemetry
- [x] Signal-loss failsafe
- [x] Emergency stop
- [x] Serial diagnostics

### Still requires real-world validation

- [ ] ESC/motor direction verification on the physical frame
- [ ] Sensor calibration validation
- [ ] PID tuning for the actual airframe
- [ ] Controlled tethered testing
- [ ] Stable free-flight validation
- [ ] Flight-time measurement
- [ ] Hardware revision documentation
- [ ] Flight logs / telemetry recording

---

## 🗺️ Future Improvements

- [ ] Improve sensor calibration and filtering
- [ ] Add persistent PID configuration
- [ ] Add better yaw/heading estimation
- [ ] Add battery-voltage monitoring
- [ ] Add flight-data logging
- [ ] Add GPS support
- [ ] Improve altitude-hold tuning
- [ ] Add configurable flight parameters
- [ ] Create a complete wiring diagram
- [ ] Document final airframe and propulsion specifications

---

## ⚠️ Safety

This is an experimental DIY flight-controller project.

Never test motors with propellers installed while debugging firmware or wiring. Perform initial tests on a secure stand, keep people away from the test area, and use an accessible emergency-disarm mechanism.

Do not treat the repository's default PID values as guaranteed-safe flight settings.

---

## 📄 Source

The main firmware in this portfolio page is based on the uploaded **ESP32_Drone_Complete_v2.ino** project source.

The repository documentation is intended to describe the implementation accurately rather than claim flight performance that has not been verified.

---

**Built and documented by [Mr-S-96](https://github.com/Mr-S-96).**
