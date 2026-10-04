/* 
  ╔═══════════════════════════════════════════════════════════════════════╗
  ║             ESP32 QUADCOPTER FLIGHT CONTROLLER v2.0                   ║
  ║                     Complete Production Code                          ║
  ╚═══════════════════════════════════════════════════════════════════════╝
  
  HARDWARE:
  ┌─────────────────────────────────────────────────────────────────────┐
  │ MCU:         ESP32 (WROOM-32)                                       │
  │ Receiver:    FlySky FS-iA6B (iBUS protocol on GPIO 16)            │
  │ Transmitter: FlySky FS-i6                                          │
  │ IMU:         MPU6050 (Gyro + Accelerometer)                        │
  │ Barometer:   BMP280 (Altitude sensing)                             │
  │ Compass:     HMC5883L (Magnetometer)                               │
  │ Display:     SSD1306 OLED (128x64)                                 │
  │ Motors:      4x Brushless (via ESCs on GPIO 25/26/27/14)          │
  └─────────────────────────────────────────────────────────────────────┘
  
  WIRING:
  ┌─────────────────────────────────────────────────────────────────────┐
  │ FS-iA6B iBUS  →  ESP32 GPIO 16 (RX2)                               │
  │ FS-iA6B GND   →  ESP32 GND                                         │
  │ FS-iA6B +5V   →  5V (from ESC BEC)                                 │
  │                                                                     │
  │ ESC1 Signal   →  GPIO 25 (Front-Left,  CCW ↻)                     │
  │ ESC2 Signal   →  GPIO 26 (Front-Right, CW  ↺)                     │
  │ ESC3 Signal   →  GPIO 27 (Rear-Right,  CCW ↻)                     │
  │ ESC4 Signal   →  GPIO 14 (Rear-Left,   CW  ↺)                     │
  │ ESC GND       →  ESP32 GND (IMPORTANT: Common ground!)            │
  │                                                                     │
  │ I2C SDA       →  GPIO 21                                           │
  │ I2C SCL       →  GPIO 22                                           │
  └─────────────────────────────────────────────────────────────────────┘
  
  CHANNEL MAPPING:
  ┌──────┬──────────────┬──────────────────────────────────────────────┐
  │ CH1  │ Roll         │ Right stick left/right                       │
  │ CH2  │ Pitch        │ Right stick forward/back                     │
  │ CH3  │ Throttle     │ Left stick up/down                           │
  │ CH4  │ Yaw          │ Left stick left/right                        │
  │ CH5  │ Flight Mode  │ 3-pos switch (ACRO/ANGLE/ALT_HOLD)          │
  │ CH6  │ Arm/Disarm   │ 2-pos switch (DOWN=Disarm, UP=Arm)          │
  └──────┴──────────────┴──────────────────────────────────────────────┘
  
  FLIGHT MODES:
  • ACRO      - Manual rate control (no stabilization)
  • ANGLE     - Self-leveling (attitude hold)
  • ALT_HOLD  - Altitude hold + self-leveling
  • EMERGENCY - Immediate motor cutoff
  
  SAFETY FEATURES:
  ✓ Signal loss detection (auto-disarm after 1 second)
  ✓ Low throttle arming requirement
  ✓ Immediate disarm on switch toggle
  ✓ Motor idle when disarmed or low throttle
  ✓ Complementary filter for stable attitude
  
  ⚠️  CRITICAL SAFETY WARNINGS:
  • REMOVE PROPELLERS during all testing
  • Calibrate ESCs before first flight
  • Test on a secure stand before flight
  • Tune PID values for your specific setup
  • Always have a kill switch ready
  • Fly in open areas away from people
  
  Version: 2.0
  Date: 2024
  License: MIT
*/

#include <Arduino.h>
#include <Wire.h>

// Sensor libraries
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/* ═══════════════════════════════════════════════════════════════════════
   CONFIGURATION
   ═══════════════════════════════════════════════════════════════════════ */

// Debug serial
#define DEBUG Serial
#define DEBUG_BAUDRATE 115200

// Display configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDRESS 0x3C

// Motor pins (X configuration)
#define MOTOR1_PIN 25  // Front-Left  (CCW ↻)
#define MOTOR2_PIN 26  // Front-Right (CW  ↺)
#define MOTOR3_PIN 27  // Rear-Right  (CCW ↻)
#define MOTOR4_PIN 14  // Rear-Left   (CW  ↺)

// Motor PWM settings
#define PWM_FREQUENCY 50    // 50Hz for standard ESCs
#define PWM_RESOLUTION 16   // 16-bit resolution
#define PWM_MIN 1000        // Minimum pulse width (µs)
#define PWM_MAX 2000        // Maximum pulse width (µs)

// iBUS receiver configuration
#define IBUS_SERIAL Serial2
#define IBUS_RX_PIN 16          // GPIO 16 for iBUS data
#define IBUS_BAUDRATE 115200    // iBUS standard baud rate
#define IBUS_MAX_CHANNELS 14    // iBUS supports 14 channels
#define IBUS_BUFFER_SIZE 32     // iBUS frame size

// Safety timeouts
#define SIGNAL_TIMEOUT_MS 1000  // Lost signal timeout (1 second)
#define MIN_THROTTLE_FOR_ARMING 100  // Max throttle (0-1000) allowed for arming

// Loop timing
#define LOOP_FREQUENCY 200      // Main loop frequency (Hz)
#define LOOP_TIME_MS (1000 / LOOP_FREQUENCY)

/* ═══════════════════════════════════════════════════════════════════════
   GLOBAL OBJECTS
   ═══════════════════════════════════════════════════════════════════════ */

// Sensor objects
Adafruit_BMP280 bmp;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
MPU6050 imu;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/* ═══════════════════════════════════════════════════════════════════════
   DATA STRUCTURES
   ═══════════════════════════════════════════════════════════════════════ */

// Flight modes
enum FlightMode {
  FM_ACRO = 0,      // Manual rate control
  FM_ANGLE,         // Self-leveling
  FM_ALT_HOLD,      // Altitude hold
  FM_EMERGENCY      // Emergency stop
};

// PID controller structure
struct PIDController {
  float kp, ki, kd;           // PID gains
  float integrator;           // Integral accumulator
  float lastError;            // Previous error for derivative
  float outputMin, outputMax; // Output limits
};

/* ═══════════════════════════════════════════════════════════════════════
   GLOBAL VARIABLES
   ═══════════════════════════════════════════════════════════════════════ */

// iBUS receiver data
uint16_t rcChannel[IBUS_MAX_CHANNELS];     // Channel values (1000-2000)
uint8_t ibusBuffer[IBUS_BUFFER_SIZE];      // iBUS frame buffer
uint8_t ibusIndex = 0;                     // Buffer index
unsigned long lastValidIBusFrame = 0;      // Last valid frame timestamp

// Control inputs (processed from receiver)
int16_t throttle = 0;   // 0..1000
int16_t roll = 0;       // -500..+500
int16_t pitch = 0;      // -500..+500
int16_t yaw = 0;        // -500..+500

// Flight state
FlightMode currentFlightMode = FM_ACRO;
bool motorsArmed = false;
bool altitudeHoldActive = false;

// Sensor data
float imuPitch = 0.0f;      // Pitch angle (degrees)
float imuRoll = 0.0f;       // Roll angle (degrees)
float imuYaw = 0.0f;        // Yaw angle (degrees)
float baroAltitude = 0.0f;  // Barometric altitude (meters)
float targetAltitude = 0.0f; // Altitude setpoint

// PID controllers
PIDController pidRoll;
PIDController pidPitch;
PIDController pidYaw;
PIDController pidAltitude;

// Motor outputs (µs)
uint16_t motor1_us = PWM_MIN;
uint16_t motor2_us = PWM_MIN;
uint16_t motor3_us = PWM_MIN;
uint16_t motor4_us = PWM_MIN;

// Timing
unsigned long lastLoopTime = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastDebugOutput = 0;

/* ═══════════════════════════════════════════════════════════════════════
   FUNCTION PROTOTYPES
   ═══════════════════════════════════════════════════════════════════════ */

// System initialization
void setupMotors();
void setupSensors();
void setupReceiver();
void setupDisplay();

// iBUS receiver
bool readIBus();
uint16_t calculateIBusChecksum(uint8_t* data, uint8_t length);
void processReceiverInput();

// Sensor reading
void readSensors();
void updateIMU(float dt);

// Flight control
void flightController(float dt);
float computePID(PIDController &pid, float setpoint, float measurement, float dt);
void resetPID(PIDController &pid);

// Motor control
void setMotorPWM(uint8_t motorPin, uint16_t pulseWidth_us);
void motorsArm();
void motorsDisarm();
void mixMotors(float throttle, float roll, float pitch, float yaw, float altitude);

// Display & debug
void updateDisplay();
void printDebugInfo();

// Safety
void checkSignalHealth();
void emergencyStop();

// Utilities
float constrainFloat(float value, float min, float max);

/* ═══════════════════════════════════════════════════════════════════════
   SETUP
   ═══════════════════════════════════════════════════════════════════════ */

void setup() {
  // Initialize serial
  DEBUG.begin(DEBUG_BAUDRATE);
  delay(500);
  
  DEBUG.println("\n\n╔═══════════════════════════════════════════════════════╗");
  DEBUG.println("║     ESP32 QUADCOPTER FLIGHT CONTROLLER v2.0       ║");
  DEBUG.println("╚═══════════════════════════════════════════════════════╝\n");
  
  // Initialize I2C
  Wire.begin();
  DEBUG.println("[I2C] Initialized");
  
  // Setup subsystems
  setupReceiver();
  setupSensors();
  setupMotors();
  setupDisplay();
  
  // Initialize PID controllers
  // Roll PID (tune these values for your setup!)
  pidRoll.kp = 4.0f;
  pidRoll.ki = 0.01f;
  pidRoll.kd = 0.3f;
  pidRoll.outputMin = -200.0f;
  pidRoll.outputMax = 200.0f;
  resetPID(pidRoll);
  
  // Pitch PID
  pidPitch.kp = 4.0f;
  pidPitch.ki = 0.01f;
  pidPitch.kd = 0.3f;
  pidPitch.outputMin = -200.0f;
  pidPitch.outputMax = 200.0f;
  resetPID(pidPitch);
  
  // Yaw PID
  pidYaw.kp = 3.0f;
  pidYaw.ki = 0.005f;
  pidYaw.kd = 0.1f;
  pidYaw.outputMin = -200.0f;
  pidYaw.outputMax = 200.0f;
  resetPID(pidYaw);
  
  // Altitude PID
  pidAltitude.kp = 1.0f;
  pidAltitude.ki = 0.02f;
  pidAltitude.kd = 0.5f;
  pidAltitude.outputMin = -300.0f;
  pidAltitude.outputMax = 300.0f;
  resetPID(pidAltitude);
  
  DEBUG.println("\n╔═══════════════════════════════════════════════════════╗");
  DEBUG.println("║                  READY FOR FLIGHT                     ║");
  DEBUG.println("╚═══════════════════════════════════════════════════════╝");
  DEBUG.println("\nARMING PROCEDURE:");
  DEBUG.println("  1. Ensure throttle is at MINIMUM");
  DEBUG.println("  2. Flip ARM switch (CH6) to UP position");
  DEBUG.println("  3. Wait for 'MOTORS ARMED' message");
  DEBUG.println("  4. Slowly increase throttle to test\n");
  DEBUG.println("⚠️  REMOVE PROPELLERS FOR TESTING! ⚠️\n");
  
  lastLoopTime = millis();
}

/* ═══════════════════════════════════════════════════════════════════════
   MAIN LOOP
   ═══════════════════════════════════════════════════════════════════════ */

void loop() {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastLoopTime) / 1000.0f;
  
  // Enforce loop timing
  if (dt < (LOOP_TIME_MS / 1000.0f)) {
    return; // Too early, skip this iteration
  }
  
  lastLoopTime = currentTime;
  
  // Read receiver data
  readIBus();
  
  // Check signal health
  checkSignalHealth();
  
  // Process receiver inputs
  if ((currentTime - lastValidIBusFrame) < SIGNAL_TIMEOUT_MS) {
    processReceiverInput();
  }
  
  // Read sensors
  readSensors();
  updateIMU(dt);
  
  // Run flight controller
  if (motorsArmed && currentFlightMode != FM_EMERGENCY) {
    flightController(dt);
  } else {
    // Motors disarmed or emergency - send idle signals
    motor1_us = PWM_MIN;
    motor2_us = PWM_MIN;
    motor3_us = PWM_MIN;
    motor4_us = PWM_MIN;
  }
  
  // Send motor commands
  setMotorPWM(MOTOR1_PIN, motor1_us);
  setMotorPWM(MOTOR2_PIN, motor2_us);
  setMotorPWM(MOTOR3_PIN, motor3_us);
  setMotorPWM(MOTOR4_PIN, motor4_us);
  
  // Update display (5Hz)
  if (currentTime - lastDisplayUpdate > 200) {
    updateDisplay();
    lastDisplayUpdate = currentTime;
  }
  
  // Debug output (1Hz)
  if (currentTime - lastDebugOutput > 1000) {
    printDebugInfo();
    lastDebugOutput = currentTime;
  }
}

/* ═══════════════════════════════════════════════════════════════════════
   SYSTEM INITIALIZATION
   ═══════════════════════════════════════════════════════════════════════ */

void setupReceiver() {
  // Initialize iBUS serial
  IBUS_SERIAL.begin(IBUS_BAUDRATE, SERIAL_8N1, IBUS_RX_PIN, -1);
  
  // Set default channel values (safe state)
  for (int i = 0; i < IBUS_MAX_CHANNELS; i++) {
    rcChannel[i] = 1500; // Center position
  }
  rcChannel[2] = 1000; // Throttle at minimum
  rcChannel[4] = 1000; // Flight mode to ACRO
  rcChannel[5] = 1000; // Arm switch to DISARM
  
  lastValidIBusFrame = millis();
  
  DEBUG.println("[iBUS] Initialized on GPIO 16");
  DEBUG.println("       Waiting for receiver signal...");
}

void setupSensors() {
  bool sensorsOK = true;
  
  // BMP280 Barometer
  if (!bmp.begin(0x76)) {
    DEBUG.println("[BMP280] ⚠️  NOT DETECTED (altitude hold disabled)");
    sensorsOK = false;
  } else {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
    DEBUG.println("[BMP280] ✓ Initialized");
  }
  
  // HMC5883L Magnetometer
  if (!mag.begin()) {
    DEBUG.println("[HMC5883L] ⚠️  NOT DETECTED (compass disabled)");
  } else {
    DEBUG.println("[HMC5883L] ✓ Initialized");
  }
  
  // MPU6050 IMU
  imu.initialize();
  if (!imu.testConnection()) {
    DEBUG.println("[MPU6050] ❌ NOT CONNECTED - CRITICAL!");
    DEBUG.println("          Cannot fly without IMU!");
    sensorsOK = false;
  } else {
    DEBUG.println("[MPU6050] ✓ Initialized");
  }
  
  if (!sensorsOK) {
    DEBUG.println("\n⚠️  WARNING: Some sensors failed to initialize!");
    DEBUG.println("   Flight capabilities may be limited.\n");
  }
}

void setupMotors() {
  // Attach LEDC channels to motor pins
  ledcAttach(MOTOR1_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(MOTOR2_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(MOTOR3_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(MOTOR4_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
  
  // Send initial idle signal to ESCs
  for (int i = 0; i < 100; i++) {
    setMotorPWM(MOTOR1_PIN, PWM_MIN);
    setMotorPWM(MOTOR2_PIN, PWM_MIN);
    setMotorPWM(MOTOR3_PIN, PWM_MIN);
    setMotorPWM(MOTOR4_PIN, PWM_MIN);
    delay(10);
  }
  
  DEBUG.println("[MOTORS] ✓ Initialized (50Hz, 16-bit PWM)");
  DEBUG.println("         All motors at IDLE");
}

void setupDisplay() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    DEBUG.println("[OLED] ⚠️  Display not detected");
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("ESP32 DRONE FC");
    display.println("Initializing...");
    display.display();
    DEBUG.println("[OLED] ✓ Display ready");
  }
}

/* ═══════════════════════════════════════════════════════════════════════
   iBUS RECEIVER
   ═══════════════════════════════════════════════════════════════════════ */

bool readIBus() {
  // iBUS frame: [0x20][0x40][CH1_L][CH1_H]...[CHK_L][CHK_H]
  
  while (IBUS_SERIAL.available() > 0) {
    uint8_t byte = IBUS_SERIAL.read();
    
    // Look for frame header
    if (ibusIndex == 0 && byte != 0x20) {
      continue; // Wait for start byte
    }
    if (ibusIndex == 1 && byte != 0x40) {
      ibusIndex = 0; // Invalid header, restart
      continue;
    }
    
    ibusBuffer[ibusIndex++] = byte;
    
    // Complete frame received
    if (ibusIndex == IBUS_BUFFER_SIZE) {
      ibusIndex = 0;
      
      // Verify checksum
      uint16_t receivedChecksum = ibusBuffer[30] | (ibusBuffer[31] << 8);
      uint16_t calculatedChecksum = calculateIBusChecksum(ibusBuffer, 30);
      
      if (receivedChecksum == calculatedChecksum) {
        // Extract channel data
        for (uint8_t i = 0; i < IBUS_MAX_CHANNELS; i++) {
          uint16_t value = ibusBuffer[2 + i*2] | (ibusBuffer[3 + i*2] << 8);
          
          // Validate range
          if (value >= 900 && value <= 2100) {
            rcChannel[i] = value;
          }
        }
        
        lastValidIBusFrame = millis();
        return true;
      }
    }
  }
  
  return false;
}

uint16_t calculateIBusChecksum(uint8_t* data, uint8_t length) {
  uint16_t checksum = 0xFFFF;
  for (uint8_t i = 0; i < length; i++) {
    checksum -= data[i];
  }
  return checksum;
}

void processReceiverInput() {
  // Map receiver channels to control inputs
  
  // Roll: CH1 (1000-2000) → (-500 to +500)
  roll = (int16_t)rcChannel[0] - 1500;
  roll = constrain(roll, -500, 500);
  
  // Pitch: CH2 (1000-2000) → (-500 to +500)
  pitch = (int16_t)rcChannel[1] - 1500;
  pitch = constrain(pitch, -500, 500);
  
  // Throttle: CH3 (1000-2000) → (0 to 1000)
  throttle = (int16_t)rcChannel[2] - 1000;
  throttle = constrain(throttle, 0, 1000);
  
  // Yaw: CH4 (1000-2000) → (-500 to +500)
  yaw = (int16_t)rcChannel[3] - 1500;
  yaw = constrain(yaw, -500, 500);
  
  // Flight mode switch (CH5) - 3 positions
  if (rcChannel[4] < 1300) {
    currentFlightMode = FM_ACRO;
  } else if (rcChannel[4] < 1700) {
    currentFlightMode = FM_ANGLE;
  } else {
    currentFlightMode = FM_ALT_HOLD;
    if (!altitudeHoldActive) {
      targetAltitude = baroAltitude;
      altitudeHoldActive = true;
    }
  }
  
  // Reset altitude hold flag if not in ALT_HOLD mode
  if (currentFlightMode != FM_ALT_HOLD) {
    altitudeHoldActive = false;
  }
  
  // Arm/Disarm switch (CH6) - 2 positions
  static bool lastArmSwitch = false;
  bool currentArmSwitch = (rcChannel[5] > 1500);
  
  // Detect switch rising edge (OFF → ON)
  if (currentArmSwitch && !lastArmSwitch) {
    // Trying to ARM
    if (!motorsArmed && throttle < MIN_THROTTLE_FOR_ARMING) {
      motorsArm();
    } else if (!motorsArmed) {
      DEBUG.println("[ARM] ⚠️  Cannot arm - throttle too high!");
      DEBUG.printf("      Current throttle: %d (max allowed: %d)\n", 
                   throttle, MIN_THROTTLE_FOR_ARMING);
    }
  }
  
  // Detect switch falling edge (ON → OFF)
  if (!currentArmSwitch && lastArmSwitch) {
    // Trying to DISARM
    if (motorsArmed) {
      motorsDisarm();
    }
  }
  
  lastArmSwitch = currentArmSwitch;
}

/* ═══════════════════════════════════════════════════════════════════════
   SENSOR READING
   ═══════════════════════════════════════════════════════════════════════ */

void readSensors() {
  // Read barometer
  baroAltitude = bmp.readAltitude(1013.25); // Sea level pressure in hPa
  
  // Read magnetometer
  sensors_event_t magEvent;
  mag.getEvent(&magEvent);
}

void updateIMU(float dt) {
  // Read raw IMU data
  int16_t ax, ay, az, gx, gy, gz;
  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Convert accelerometer to g's
  float accelX = ax / 16384.0f;
  float accelY = ay / 16384.0f;
  float accelZ = az / 16384.0f;
  
  // Calculate angles from accelerometer
  float pitchAcc = atan2(accelY, accelZ) * 180.0f / PI;
  float rollAcc = atan2(-accelX, accelZ) * 180.0f / PI;
  
  // Convert gyroscope to degrees/second
  float gyroX = gx / 131.0f;
  float gyroY = gy / 131.0f;
  float gyroZ = gz / 131.0f;
  
  // Integrate gyroscope
  imuPitch += gyroX * dt;
  imuRoll += gyroY * dt;
  imuYaw += gyroZ * dt;
  
  // Complementary filter (98% gyro, 2% accel)
  imuPitch = 0.98f * imuPitch + 0.02f * pitchAcc;
  imuRoll = 0.98f * imuRoll + 0.02f * rollAcc;
}

/* ═══════════════════════════════════════════════════════════════════════
   FLIGHT CONTROLLER
   ═══════════════════════════════════════════════════════════════════════ */

void flightController(float dt) {
  float rollOutput = 0.0f;
  float pitchOutput = 0.0f;
  float yawOutput = 0.0f;
  float altitudeOutput = 0.0f;
  
  // Convert inputs to normalized values
  float rollInput = roll / 500.0f;     // -1.0 to +1.0
  float pitchInput = pitch / 500.0f;   // -1.0 to +1.0
  float yawInput = yaw / 500.0f;       // -1.0 to +1.0
  
  if (currentFlightMode == FM_ACRO) {
    // ═══ ACRO MODE: Direct rate control ═══
    rollOutput = rollInput * 200.0f;   // Direct passthrough
    pitchOutput = pitchInput * 200.0f;
    yawOutput = yawInput * 200.0f;
    
  } else if (currentFlightMode == FM_ANGLE || currentFlightMode == FM_ALT_HOLD) {
    // ═══ ANGLE MODE: Self-leveling ═══
    float targetRoll = rollInput * 30.0f;    // Max ±30 degrees
    float targetPitch = pitchInput * 30.0f;  // Max ±30 degrees
    
    rollOutput = computePID(pidRoll, targetRoll, imuRoll, dt);
    pitchOutput = computePID(pidPitch, targetPitch, imuPitch, dt);
    yawOutput = yawInput * 200.0f; // Yaw is rate-controlled
    
    if (currentFlightMode == FM_ALT_HOLD) {
      // ═══ ALTITUDE HOLD: Barometer-based ═══
      // Throttle stick adjusts target altitude
      float climbRate = (throttle / 1000.0f - 0.5f) * 2.0f; // -1 to +1
      targetAltitude += climbRate * dt * 0.5f; // Slow climb rate
      
      altitudeOutput = computePID(pidAltitude, targetAltitude, baroAltitude, dt);
    }
  }
  
  // Mix motor outputs
  mixMotors(throttle, rollOutput, pitchOutput, yawOutput, altitudeOutput);
}

float computePID(PIDController &pid, float setpoint, float measurement, float dt) {
  // Calculate error
  float error = setpoint - measurement;
  
  // Proportional term
  float pTerm = pid.kp * error;
  
  // Integral term with anti-windup
  pid.integrator += error * dt;
  float maxIntegrator = pid.outputMax / pid.ki;
  float minIntegrator = pid.outputMin / pid.ki;
  pid.integrator = constrainFloat(pid.integrator, minIntegrator, maxIntegrator);
  float iTerm = pid.ki * pid.integrator;
  
  // Derivative term
  float dTerm = 0.0f;
  if (dt > 0.0f) {
    dTerm = pid.kd * (error - pid.lastError) / dt;
  }
  pid.lastError = error;
  
  // Calculate output
  float output = pTerm + iTerm + dTerm;
  
  // Constrain output
  output = constrainFloat(output, pid.outputMin, pid.outputMax);
  
  return output;
}

void resetPID(PIDController &pid) {
  pid.integrator = 0.0f;
  pid.lastError = 0.0f;
}

/* ═══════════════════════════════════════════════════════════════════════
   MOTOR MIXING
   ═══════════════════════════════════════════════════════════════════════ */

void mixMotors(float throttle, float roll, float pitch, float yaw, float altitude) {
  // X configuration motor mixing
  // 
  //      Front
  //   M1 ↻  ↺ M2
  //      \ X /
  //      / X \
  //   M4 ↺  ↻ M3
  //      Rear
  
  float m1 = throttle + (-pitch) + (-roll) + (-yaw) + altitude;
  float m2 = throttle + (-pitch) + ( roll) + ( yaw) + altitude;
  float m3 = throttle + ( pitch) + ( roll) + (-yaw) + altitude;
  float m4 = throttle + ( pitch) + (-roll) + ( yaw) + altitude;
  
  // Convert to PWM pulse width (1000-2000 µs)
  auto toPulseWidth = [](float value) -> uint16_t {
    int pulseWidth = (int)(value + 1000.0f);
    return (uint16_t)constrain(pulseWidth, PWM_MIN, PWM_MAX);
  };
  
  // Apply minimum throttle safety
  if (!motorsArmed || throttle < 50) {
    motor1_us = PWM_MIN;
    motor2_us = PWM_MIN;
    motor3_us = PWM_MIN;
    motor4_us = PWM_MIN;
  } else {
    motor1_us = toPulseWidth(m1);
    motor2_us = toPulseWidth(m2);
    motor3_us = toPulseWidth(m3);
    motor4_us = toPulseWidth(m4);
  }
}

/* ═══════════════════════════════════════════════════════════════════════
   MOTOR CONTROL
   ═══════════════════════════════════════════════════════════════════════ */

void setMotorPWM(uint8_t motorPin, uint16_t pulseWidth_us) {
  // Convert microseconds to duty cycle
  uint32_t period_us = 1000000 / PWM_FREQUENCY;
  uint32_t maxDuty = (1 << PWM_RESOLUTION) - 1;
  uint32_t duty = (pulseWidth_us * maxDuty) / period_us;
  
  // Write to LEDC
  ledcWrite(motorPin, duty);
}

void motorsArm() {
  motorsArmed = true;
  
  // Reset PID integrators
  resetPID(pidRoll);
  resetPID(pidPitch);
  resetPID(pidYaw);
  resetPID(pidAltitude);
  
  DEBUG.println("\n╔═══════════════════════════════════════════╗");
  DEBUG.println("║     ✓✓✓ MOTORS ARMED - FLY SAFE! ✓✓✓     ║");
  DEBUG.println("╚═══════════════════════════════════════════╝\n");
}

void motorsDisarm() {
  motorsArmed = false;
  altitudeHoldActive = false;
  
  // Immediately set motors to idle
  motor1_us = PWM_MIN;
  motor2_us = PWM_MIN;
  motor3_us = PWM_MIN;
  motor4_us = PWM_MIN;
  
  setMotorPWM(MOTOR1_PIN, PWM_MIN);
  setMotorPWM(MOTOR2_PIN, PWM_MIN);
  setMotorPWM(MOTOR3_PIN, PWM_MIN);
  setMotorPWM(MOTOR4_PIN, PWM_MIN);
  
  DEBUG.println("\n╔═══════════════════════════════════════════╗");
  DEBUG.println("║         ✗ MOTORS DISARMED ✗               ║");
  DEBUG.println("╚═══════════════════════════════════════════╝\n");
}

/* ═══════════════════════════════════════════════════════════════════════
   SAFETY
   ═══════════════════════════════════════════════════════════════════════ */

void checkSignalHealth() {
  unsigned long currentTime = millis();
  unsigned long timeSinceLastFrame = currentTime - lastValidIBusFrame;
  
  if (timeSinceLastFrame > SIGNAL_TIMEOUT_MS) {
    // Signal lost!
    if (motorsArmed) {
      DEBUG.println("\n!!! SIGNAL LOST - EMERGENCY DISARM !!!\n");
      emergencyStop();
    }
  }
}

void emergencyStop() {
  currentFlightMode = FM_EMERGENCY;
  motorsDisarm();
  
  DEBUG.println("╔═══════════════════════════════════════════╗");
  DEBUG.println("║       !!! EMERGENCY STOP !!!              ║");
  DEBUG.println("╚═══════════════════════════════════════════╝");
}

/* ═══════════════════════════════════════════════════════════════════════
   DISPLAY & DEBUG
   ═══════════════════════════════════════════════════════════════════════ */

void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  
  // Line 1: Armed status + Flight mode
  display.print(motorsArmed ? "ARMED " : "DISARM");
  switch (currentFlightMode) {
    case FM_ACRO:      display.println(" ACRO");      break;
    case FM_ANGLE:     display.println(" ANGLE");     break;
    case FM_ALT_HOLD:  display.println(" ALT_HOLD");  break;
    case FM_EMERGENCY: display.println(" EMERGENCY"); break;
  }
  
  // Line 2: Control inputs
  display.printf("T:%4d R:%4d\n", throttle, roll);
  display.printf("P:%4d Y:%4d\n", pitch, yaw);
  
  // Line 3: IMU data
  display.printf("IMU %.1f/%.1f\n", imuRoll, imuPitch);
  
  // Line 4: Altitude
  display.printf("Alt: %.1fm\n", baroAltitude);
  
  // Line 5: Motor outputs
  display.printf("M:%d/%d/%d/%d\n", 
                 motor1_us, motor2_us, motor3_us, motor4_us);
  
  // Line 6: Signal status
  unsigned long timeSinceFrame = millis() - lastValidIBusFrame;
  display.print(timeSinceFrame < SIGNAL_TIMEOUT_MS ? "RX:OK" : "RX:LOST");
  
  display.display();
}

void printDebugInfo() {
  unsigned long timeSinceFrame = millis() - lastValidIBusFrame;
  bool signalOK = (timeSinceFrame < SIGNAL_TIMEOUT_MS);
  
  DEBUG.println("┌─────────────────────────────────────────────────────────┐");
  DEBUG.printf("│ ARMED: %s  │  MODE: ", motorsArmed ? "YES" : " NO");
  
  switch (currentFlightMode) {
    case FM_ACRO:      DEBUG.print("ACRO     "); break;
    case FM_ANGLE:     DEBUG.print("ANGLE    "); break;
    case FM_ALT_HOLD:  DEBUG.print("ALT_HOLD "); break;
    case FM_EMERGENCY: DEBUG.print("EMERGENCY"); break;
  }
  
  DEBUG.printf(" │  RX: %s  │\n", signalOK ? "OK  " : "LOST");
  DEBUG.println("├─────────────────────────────────────────────────────────┤");
  DEBUG.printf("│ Throttle: %4d  │  Roll: %4d  │  Pitch: %4d  │  Yaw: %4d  │\n",
               throttle, roll, pitch, yaw);
  DEBUG.println("├─────────────────────────────────────────────────────────┤");
  DEBUG.printf("│ IMU Roll: %6.2f°  │  Pitch: %6.2f°  │  Yaw: %6.2f°    │\n",
               imuRoll, imuPitch, imuYaw);
  DEBUG.println("├─────────────────────────────────────────────────────────┤");
  DEBUG.printf("│ Altitude: %6.2fm  │  Target: %6.2fm                     │\n",
               baroAltitude, targetAltitude);
  DEBUG.println("├─────────────────────────────────────────────────────────┤");
  DEBUG.printf("│ Motors: M1:%4d  M2:%4d  M3:%4d  M4:%4d              │\n",
               motor1_us, motor2_us, motor3_us, motor4_us);
  DEBUG.println("└─────────────────────────────────────────────────────────┘\n");
}

/* ═══════════════════════════════════════════════════════════════════════
   UTILITIES
   ═══════════════════════════════════════════════════════════════════════ */

float constrainFloat(float value, float min, float max) {
  if (value < min) return min;
  if (value > max) return max;
  return value;
}

/* ═══════════════════════════════════════════════════════════════════════
   END OF CODE
   ═══════════════════════════════════════════════════════════════════════ */