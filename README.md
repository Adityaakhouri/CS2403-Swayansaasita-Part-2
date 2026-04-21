# 🚴 CS2403 Swayansaasita Part 2

### End-to-End Autonomous Driving Pipeline (Data Collection + AI Analysis)

---

## 📌 Project Overview

This project implements a **complete autonomous driving pipeline**, inspired by modern self-driving systems.
It combines:

* 📡 Real-world **multi-sensor data collection**
* 🧠 **AI-based perception and prediction models**

The system captures synchronized data from multiple sensors and uses it for downstream analysis and model development.

---

## 🚀 Key Components

### 🔹 1. Multi-Sensor Data Collection System

* 6-camera setup (Arducam)
* iPad-based depth + pose estimation
* IMU (motion data)
* GPS (location tracking)
* Time-synchronized recording

### 🔹 2. Data Processing & Storage

* Structured dataset generation
* Frame-level timestamps
* Sensor alignment for fusion tasks

### 🔹 3. AI / ML Pipeline

* Dataset used for training perception models
* Focus on:

  * Scene understanding
  * Pattern analysis
  * Predictive modeling

---

## ⚙️ Technologies Used

* **Python** → Core system logic
* **Socket Programming (TCP/UDP)** → Device communication
* **FFmpeg / FFprobe** → Multi-camera video recording
* **NumPy / Pandas** → Data processing
* **Xcode (iOS)** → iPad app development

---

## ▶️ How to Run

1. Clone the repository:

```bash
git clone https://github.com/your-username/CS2403-Swayansaasita-Part-2.git
cd CS2403-Swayansaasita-Part-2
```

2. Install dependencies:

```bash
pip install numpy pandas
```

3. Run the recorder:

```bash
python Master_Recorder.py
```

4. Launch the iPad app and connect to the system.

---

## 📊 Output Data

All recordings are stored in:

~/recordings_data/<scene_name>/

Includes:

* 🎥 Multi-camera video streams (.mp4)
* 🧠 Depth maps and poses (.npy)
* 📈 IMU logs (.csv)
* 📍 GPS logs (.csv)
* 📊 Timestamp synchronization files
* 📄 Session metadata

---

## 🧠 Use Case

* Autonomous driving research
* Sensor fusion experiments
* Multi-view perception systems
* Dataset creation for AI models

---

## 🔮 Future Work

* Real-time inference pipeline
* Sensor fusion model integration
* LiDAR + camera fusion
* Deployment for edge devices
