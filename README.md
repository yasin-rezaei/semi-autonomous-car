# 🚗 Line Follower with Raspberry Pi & Arduino

This project uses **Raspberry Pi + Arduino** to build a semi-autonomous car that can:
- Follow a line using OpenCV
- Detect traffic signs (right, left, stop, tunnel, crosswalk)
- Control motors through serial communication

---

## 🧠 Features
- **Real-time camera stream** from Raspberry Pi
- **PID control** for smoother line following
- **Cascade classifiers** for traffic sign detection
- **Template matching** for static sign recognition
- Communication with **Arduino via Serial**

---

## ⚙️ Requirements
- Raspberry Pi 4 (or similar)
- Arduino Uno/Nano
- USB serial connection
- Python 3.11+

---

## 🧩 Installation
```bash
git clone https://github.com/YOUR_USERNAME/LineFollower-RPi-Arduino.git
cd LineFollower-RPi-Arduino
pip install -r requirements.txt
