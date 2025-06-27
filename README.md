# Gesture & Camera Car

A complete IoT project for controlling a robotic car using hand gestures, with real-time video streaming. The system is built around two ESP32-WROOM-U modules and one ESP32-CAM, with an MPU6050 motion sensor for gesture detection.

---

## Table of Contents
- [Project Overview](#project-overview)
- [Hardware Used](#hardware-used)
- [Project Structure](#project-structure)
- [System Architecture](#system-architecture)
- [Module Details](#module-details)
  - [Gesture Transmitter](#gesture-transmitter)
  - [Camera Transmitter](#camera-transmitter)
  - [Receiver](#receiver)
- [How It Works](#how-it-works)
- [Setup & Usage](#setup--usage)
- [Customization](#customization)
- [Diagrams](#diagrams)
- [License](#license)

---

## Project Overview

This project enables gesture-based wireless control of a robotic car, with live video feedback. It consists of three main modules:

- **Gesture Transmitter:** Reads hand gestures using an MPU6050 and sends commands via ESP-NOW.
- **Camera Transmitter:** Streams live video from the car and relays remote control commands.
- **Receiver:** Receives commands and controls the car's motors.

---

## Hardware Used

- 2x ESP32-WROOM-U modules
- 1x ESP32-CAM module (for video streaming)
- 1x MPU6050 6-axis motion sensor (for gesture detection)
- 2x L298N Motor driver (for motor controlcontrol)
- Car chassis with motors
- Power supplies (Lipo 2S or 3S)
- Optional: LEDs, buttons, etc.

---

## Project Structure

```
Gesture & Camera Car/
  ├── Camera_Transmitter/
  │     └── Camera_Transmitter.ino
  ├── Diagram/
  │     ├── GestureTransmitter_ESP32.png
  │     └── Receiver_ESP32.png
  ├── Gesture_Transmitter/
  │     └── Gesture_Transmitter.ino
  └── Receiver/
        └── Receiver.ino
```

---

## System Architecture

- **Gesture Transmitter** (ESP32-WROOM-U + MPU6050): Detects hand orientation and movement, maps them to control commands, and sends them wirelessly to the receiver using ESP-NOW. Also logs gesture data to Firebase.
- **Camera Transmitter** (ESP32-CAM): Streams live video via a web interface and relays remote control commands to the receiver.
- **Receiver** (ESP32-WROOM-U): Receives commands (from gesture or remote), processes them, and drives the car's motors accordingly.

---

## Module Details

### Gesture Transmitter

- **Code:** `Gesture_Transmitter/Gesture_Transmitter.ino`
- **Hardware:** ESP32-WROOM-U, MPU6050, button (for calibration/control toggle)
- **Features:**
  - Reads orientation (yaw, pitch, roll) from MPU6050 using DMP.
  - Calibrates neutral position via button hold.
  - Maps orientation to X/Y/Z values and sends as a data packet via ESP-NOW.
  - Logs gesture data to Firebase (if WiFi connected).
  - Button toggles control on/off and triggers calibration.
- **Communication:** ESP-NOW to Receiver, WiFi for Firebase logging.

### Camera Transmitter

- **Code:** `Camera_Transmitter/Camera_Transmitter.ino`
- **Hardware:** ESP32-CAM, onboard LED for lighting
- **Features:**
  - Hosts a web server for live video streaming and remote control UI.
  - Accepts remote control commands via WebSocket and relays them to the receiver via ESP-NOW.
  - Allows speed and light control via web interface.
- **Communication:** WiFi (web server), ESP-NOW to Receiver.

### Receiver

- **Code:** `Receiver/Receiver.ino`
- **Hardware:** ESP32-WROOM-U, motor driver (L298N), 4 motors (standard), status LED
- **Features:**
  - Receives and parses command packets from both gesture and remote sources.
  - Prioritizes gesture control unless overridden by remote for a set timeout.
  - Maps commands to motor actions (forward, backward, left, right, turns, diagonals, stop).
  - Handles signal loss and safety stop.
- **Communication:** ESP-NOW (from Gesture and Camera modules).

---

## How It Works

1. **Gesture Control:**  
   - User holds the gesture transmitter and moves their hand.
   - MPU6050 detects orientation; ESP32 maps this to movement commands.
   - Commands are sent wirelessly to the receiver, which drives the car.
2. **Video Streaming & Remote Control:**  
   - Camera transmitter streams live video via WiFi.
   - User can access the web interface to view video and send remote commands.
   - Remote commands are relayed to the receiver, temporarily overriding gesture control.
3. **Receiver:**  
   - Receives and executes commands, driving the car's motors.
   - Handles both gesture and remote sources, with priority logic.

---

## Setup & Usage

### 1. Hardware Assembly

- Connect the MPU6050 to the Gesture Transmitter ESP32-WROOM-U via I2C.
- Assemble the car chassis, connect motors to the motor driver, and connect the driver to the Receiver ESP32-WROOM-U.
- Connect the camera module to the ESP32-CAM.

### 2. Flashing the Code

- Use Arduino IDE.
- Install required libraries:
  - `esp_now`, `WiFi`, `WiFiManager`, `I2Cdev`, `MPU6050_6Axis_MotionApps20`, `ESPAsyncWebServer`, `ESPAsyncWiFiManager`, etc.
- Upload each `.ino` file to the corresponding ESP32 module.

### 3. WiFi Setup

- On first boot, each module will start a WiFiManager portal (e.g., "GestureSetup", "CameraSetup").
- Connect to the AP and enter your WiFi credentials.

### 4. Operation

- Power all modules.
- For gesture control: Hold the transmitter, press and hold the button to calibrate neutral position, then move your hand to drive the car.
- For remote control: Connect to the Camera Transmitter's web interface, view the video, and use the on-screen controls.
- The receiver will execute commands and drive the car accordingly.

---

## Customization

- **Gesture Mapping:** Adjust mapping logic in `Gesture_Transmitter.ino` for different gestures or sensitivity.
- **Remote UI:** Modify the HTML/JS in `Camera_Transmitter.ino` for custom controls or appearance.
- **Motor Logic:** Change motor pin assignments or movement logic in `Receiver.ino` to match your hardware.

---

## Diagrams

See the `Diagram/` folder for wiring and block diagrams:
- `GestureTransmitter_ESP32.png`
- `Receiver_ESP32.png`

---
