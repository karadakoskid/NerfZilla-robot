# Nerfzilla – Face Tracking Nerf Gun Robot 🤖🔫

Nerfzilla is an autonomous face-tracking robot built using a Raspberry Pi and OpenCV. Its goal is to detect a human face, track its position, and fire a foam dart using servos and spinning DC motors. It's designed for fun and educational purposes.

---
### Authors

This project was developed by Dimitar Iliev and Damjan Karadakoski as part of an educational initiative.

---

## 🎯 Features

- Real-time face detection and tracking
- Horizontal (X) and vertical (Y) servo aiming
- Extra servo trigger mechanism for firing
- Dual DC motor flywheels for launching foam darts
- Visual feedback via OpenCV interface
- Fully 3D printed frame

---

## 🧠 Tech Stack

- **Hardware**: Raspberry Pi (tested on Pi 4), USB webcam, 3x MG90S servo motors, 2x 12V DC motors, L298N motor driver, 3D-printed components
- **Software**: Python, OpenCV, pigpio, NumPy, [cvzone](https://github.com/cvzone/cvzone) for face detection

---
## 📦 Requirements
Install the required Python libraries:
pip install opencv-python numpy cvzone pigpio
