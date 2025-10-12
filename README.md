# ESP32-CAM Line-Following Robot
---

## Hardware Components

| Component | Description | Quantity |
|-----------|-------------|----------|
| ESP32-CAM | Microcontroller with integrated camera for image capture and processing | 1 |
| FTDI USB-to-UART Adapter | Used to program the ESP32-CAM (no onboard USB) | 1 |
| L298N Motor Driver | Controls DC motors using PWM signals from the ESP32-CAM | 1 |
| DC Motors with Wheels | Drive system for movement | 2 |
| Chassis | Base structure for mounting components | 1 |
| Li-ion Battery Pack (7.4V–11.1V) | Powers the motors and ESP32-CAM | 1 |
| Voltage Regulator (AMS1117 5V) | Provides stable 5V for ESP32-CAM | 1 |
| Power Switch | On/off control | 1 |
| Track Surface | Black line on a white background for the robot to follow | — |

---

## System Architecture

1. **Camera Input**: ESP32-CAM captures low-resolution images of the track.
2. **Image Processing**: The captured frame is processed to detect the position of the black line relative to the robot.
3. **Decision Making**: The robot calculates steering corrections based on line position.
4. **Motor Control**: PWM signals are sent to the L298N motor driver to adjust left and right motor speeds.
5. **Continuous Loop**: The robot continuously captures frames, processes them, and adjusts movement in real time.

---
