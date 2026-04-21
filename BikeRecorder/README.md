# 📱 BikeRecorder iOS App

---

## 📌 Description

This app works together with the main recording script (`Master_Recorder.py`).

* The **iOS app runs on the iPad/iPhone**
* The **master script must run on the system where all cameras are connected**

The app connects to the master system over the network to control recording and stream sensor data.

---

## ▶️ How to Run the App

### 1. Open the Project

* Extract `App.zip`
* Open the `.xcodeproj` file in Xcode

---

### 2. Connect Your Device

* Plug in your iPad/iPhone via USB
* Select your device in Xcode (top bar)

---

### 3. Enable Signing

* Go to **Signing & Capabilities**
* Select your Apple ID (Personal Team is fine)

---

### 4. Build & Install

* Click **Run (▶)** in Xcode
* The app will install on your device

---

### 5. Allow Permissions (Important)

On first launch, allow:

* Camera access
* Network access

---

## 🔌 Connect to Recorder

1. On your computer (camera system), run:

```bash
python Master_Recorder.py
```

2. Make sure:

* All cameras are connected to this system
* Both devices are on the same WiFi

3. In the app:

* Enter the **IP address of the computer**
* Connect

---

## 🎮 Basic Usage

* `CHECK` → Detect cameras
* `START` → Start recording
* `STOP` → Stop recording

---

## ⚠️ Notes

* The master script must run on the camera-connected system
* Keep both devices on the same network
* Stable WiFi is required for smooth data streaming
* If connection fails, check IP and firewall settings
