# Autonomous Waste Sorting Delta Robot with Edge AI (YOLOv8 & Jetson Nano)

An end-to-end industrial-grade robotics system featuring real-time object detection, stable tracking, and high-speed Delta robot manipulation.

## 🚀 Key Performance Metrics
- **Inference Latency:** 89ms (TensorRT INT8 Quantization)
- **Detection Throughput:** 11.2 FPS on NVIDIA Jetson Nano
- **Picking Accuracy:** ±3.8mm (Coordinate Transformation Error)
- **Success Rate:** 82% Autonomous Picking in uncontrolled environments
- **Cycle Time:** 1.4 seconds per object

## 🛠 Tech Stack
- **AI/CV:** YOLOv8 (Ultralytics), TensorRT, OpenCV, Kalman Filter.
- **Hardware:** NVIDIA Jetson Nano 4GB, Arduino Mega 2560, Delta Robot (3-Axis).
- **Communication:** High-speed JSON Serial Protocol (115200 baud).
- **Control:** Inverse Kinematics (IK), Dynamic Acceleration Profiling, EEPROM Configuration.

## 🌟 Advanced Features
- **Stable Tracking:** Implements a 1D Kalman Filter to suppress coordinate noise, ensuring smooth robot trajectories.
- **Edge AI Optimization:** Models are optimized with TensorRT to achieve 2x speedup compared to standard FP32 inference.
- **Dynamic Motion Profiling:** Distance-aware acceleration control to reduce mechanical vibration and improve motor longevity.
- **Workspace Protection:** Integrated 3D conical workspace violation checks to prevent mechanical collisions.

## 📂 Project Structure
- `/firmware`: Arduino source code with custom Inverse Kinematics solver.
- `/software`: Python-based GUI with multi-threaded inference pipeline.
- `/core`: Robot communication and coordinate mapping modules.
- `/vision`: YOLO detector and Kalman tracking modules.

## 🎥 Demo Video & Full Resources
- **[Click here to view Demo & Project Files (Google Drive)](https://drive.google.com/drive/u/1/folders/15FrvZ3mc-QcWxVCQaWBIcRBczue9rxv0)**
- Includes: Video demo, Research Poster, Thesis PDF, and supplementary datasets.

## 🏗 Setup & Installation
1. Flash the firmware to Arduino Mega using Arduino IDE (found in `/firmware`).
2. Install dependencies on Jetson Nano: `pip install -r requirements.txt`.
3. Run the system: `python3 main.py`.

---
*Developed by Tran Gia Bao as part of a University Thesis Project.*
