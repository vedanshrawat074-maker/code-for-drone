# 🚁 Gesture Controlled Drone using ESP32 & MPU6050

## 📌 Overview
This project implements a gesture-controlled drone system using an ESP32 and MPU6050 sensor. 

Motion data is captured from the sensor, filtered using a Kalman filter, and transmitted via Bluetooth to control drone movement in real time.

---

## ⚙️ Tech Stack
- ESP32 (Embedded System)
- MPU6050 (Accelerometer + Gyroscope)
- Bluetooth Classic (Serial Communication)
- Python (Drone Control)
- Crazyflie API

---

## 🔄 System Workflow

1. MPU6050 captures motion (tilt, acceleration)
2. Kalman filter reduces noise and stabilizes readings
3. ESP32 sends processed data via Bluetooth
4. Python script receives data and interprets movement
5. Drone responds to gestures (roll, pitch, thrust)

---

## 🚀 Features
- Real-time gesture-based control  
- Sensor fusion using Kalman filter  
- Bluetooth communication between ESP32 and drone  
- Smooth thrust ramping for stable flight  
- Safety handling (auto stop, reconnect mechanism)  

---

## 🎮 Controls
- Tilt (X/Y) → Controls drone movement (roll & pitch)
- Light sensor (LDR) → Acts as trigger (start/stop)

---

## 🧑‍💻 My Contribution
- Performed real-time experimental tuning of drone control parameters (roll, pitch, and altitude)
- Improved flight stability through iterative testing and adjustments
- Reduced Bluetooth communication delay to enhance responsiveness
- Assisted in system testing and integration

---

## 📂 Project Structure
project/
├── esp32_code/
│ └── main.ino
├── python_control/
│ └── drone.py
├── README.md



---

## ▶️ How to Run

### ESP32
- Upload the Arduino code to ESP32
- Connect MPU6050 and LDR sensor
- Start Bluetooth (device name: ESP32_BT)

### Python
` ``bash
pip install cflib pyserial
python drone.py


⚠️ Challenges
Sensor noise affecting stability
Bluetooth latency in real-time control
Manual tuning required for stable flight


🔮 Future Improvements
Implement PID controller for better stability
Add obstacle detection
Reduce latency further
Develop mobile app interface


🙌 Author
Vedansh Rawat (Team Project)
