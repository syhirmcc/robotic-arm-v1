# Wireless Gesture-Controlled Robotic Arm (Beginner Version)

## 🔧 Overview
This is a beginner-friendly robotic arm project using wireless gesture control.  
The goal is to learn embedded systems fundamentals, wireless data transmission, servo control, and Git-based engineering workflow.

This project will be expanded later to include camera-based gesture tracking and inverse kinematics.

## 🧠 Learning Focus
- ESP32-to-ESP32 wireless communication
- IMU (inertial measurement unit) for gesture sensing
- Servo motor control and synchronization
- Embedded systems architecture and planning
- GitHub documentation and engineering habits

## 🛠 Hardware Used
- Microcontrollers: 2x ESP32
- IMU Sensor: MPU6050 (gesture input)
- Servo Motors: SG90 (x4)
- Power Supply: External 5V
- Communication: WiFi or ESP-NOW (wireless control)

## 📊 Block Diagram
                 +--------------------+
                 |    MPU6050 IMU     |
                 |  (Gesture Sensor)  |
                 +--------------------+
                           |
                        I2C (SDA, SCL)
                           |
                           v
            +-----------------------------+
            |     ESP32 - Transmitter     |
            |  ("Gesture Unit")           |
            |                             |
            | - Reads IMU data            |
            | - Sends via ESP-NOW or WiFi |
            +-----------------------------+
                           |
                      Wireless (2.4 GHz)
                           v
            +-----------------------------+
            |     ESP32 - Receiver        |
            |  ("Arm Controller Unit")    |
            |                             |
            | - Receives gesture data     |
            | - Maps to servo angles      |
            +-----------------------------+
                    |     |     |     |
                   PWM   PWM   PWM   PWM
                    |     |     |     |
                    v     v     v     v
               +-------+-------+-------+-------+
               |Servo 1|Servo 2|Servo 3|Servo 4|
               +-------+-------+-------+-------+

                     [Robotic Arm Joints]

                     +------------------+
                     |   5V Power Source |
                     +------------------+
                             |
                             v
                  Powers Servos and ESP32s

## 📁 Project Structure
/code        → ESP32 code for gesture input and servo control
/assets      → Photos, videos, diagrams
/docs        → Design notes, wiring, and planning

## ✅ Status
- [x] Basic wireless setup
- [x] IMU calibration
- [ ] Servo angle mapping from IMU data
- [ ] Multi-joint control logic
- [ ] Demo video
- [ ] README polish + documentation

## 📸 Demo / Images
(Upload photos/videos in `/assets` and add them here)

## 🔭 Planned Expansion
- Add inverse kinematics for precision control
- Use camera + computer vision (OpenCV or MediaPipe) for hand tracking
- Rebuild arm with custom CAD design (advanced mechanical version)

## 📚 License
MIT — free to use, modify, and build upon.
