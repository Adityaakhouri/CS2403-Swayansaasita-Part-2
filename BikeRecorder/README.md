
# 📱 BikeRecorder iPad App

---

## 📌 Overview

The BikeRecorder app is an iPad-based application used to control and stream sensor data to the main recording system.

It acts as a **sensor node** in the multi-sensor pipeline.

---

## 🚀 Features

* 📡 Sends **depth data** to the main system
* 📐 Streams **pose (4x4 transformation matrices)**
* 📲 Communicates with Python server via TCP
* 🎮 Controls recording:

  * CHECK (detect cameras)
  * START (begin recording)
  * STOP (end recording)
* 🔋 Provides device status (battery, connectivity)

---

## 🗂️ Project Structure

Bikerecoder_fxcode/
│
├── (Xcode project files)
├── ViewControllers
├── Networking modules
└── Sensor handling logic

---

## ⚙️ Requirements

* iPad device
* Xcode (for building the app)
* Network connection to host system

---

## 🔌 Communication

The app communicates with the main system using:

* TCP → commands + depth stream
* UDP → IMU + GPS data

---

## 🧠 Role in System

The app provides:

* Depth maps (used similar to LiDAR)
* Pose estimation for scene understanding
* Control interface for recording sessions

---

## ▶️ How to Use

1.  Upload the app using Xcode
2. Connect to the host system IP
3. Use:

   * CHECK → detect cameras
   * START → begin recording
   * STOP → stop recording

---

## 📌 Notes

* Ensure both devices are on the same network
* Stable connection is required for real-time streaming
