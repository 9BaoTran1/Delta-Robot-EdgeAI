# Autonomous Waste Sorting Delta Robot with Edge AI (YOLOv8 & Jetson Nano)

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg?style=flat-square)](CONTRIBUTING.md)
[![Python Version](https://img.shields.io/badge/Python-3.8%2B-blue.svg)](requirements.txt)
[![Platform](https://img.shields.io/badge/Platform-Windows%20%7C%20Linux%20%7C%20Jetson%20Nano-lightgrey.svg)]()

An end-to-end industrial-grade robotics system featuring real-time object detection, stable tracking, and high-speed Delta robot manipulation. This repository contains both the Python-based host system (GUI, computer vision, Kalman filters) and the Arduino Mega 2560 microcontroller firmware (inverse kinematics, stepper motor controls).

---

## 🚀 Key Performance Metrics

- **Inference Latency:** 89ms (TensorRT INT8 Quantization)
- **Detection Throughput:** 11.2 FPS on NVIDIA Jetson Nano
- **Picking Accuracy:** ±3.8mm (Coordinate Transformation Error)
- **Success Rate:** 82% Autonomous Picking in uncontrolled environments
- **Cycle Time:** 1.4 seconds per object

---

## 🛠 Tech Stack

- **AI/CV:** YOLOv8 (Ultralytics), TensorRT, OpenCV, Kalman Filter.
- **Hardware:** NVIDIA Jetson Nano 4GB, Arduino Mega 2560, Delta Robot (3-Axis).
- **Communication:** High-speed JSON Serial Protocol (115200 baud) via PySerial.
- **Control:** Inverse Kinematics (IK), Dynamic Acceleration Profiling, EEPROM Configuration.

---

## 🌟 Advanced Features

- **Stable Tracking:** Implements a 1D Kalman Filter to suppress coordinate noise, ensuring smooth robot trajectories.
- **Edge AI Optimization:** Models are optimized with TensorRT to achieve a 2x speedup compared to standard FP32 inference.
- **Dynamic Motion Profiling:** Distance-aware acceleration control to reduce mechanical vibration and improve motor longevity.
- **Workspace Protection:** Integrated 3D conical workspace violation checks to prevent mechanical collisions.

---

## 📂 Project Structure

- [`/firmware`](file:///C:/Users/Thinkpad/.gemini/antigravity/scratch/Delta-Robot-EdgeAI/firmware): Arduino source code with custom Inverse Kinematics solver.
- [`/ui`](file:///C:/Users/Thinkpad/.gemini/antigravity/scratch/Delta-Robot-EdgeAI/ui): Python-based Tkinter GUI with multi-threaded inference pipeline.
- [`/core`](file:///C:/Users/Thinkpad/.gemini/antigravity/scratch/Delta-Robot-EdgeAI/core): Robot communication and coordinate mapping modules.
- [`/vision`](file:///C:/Users/Thinkpad/.gemini/antigravity/scratch/Delta-Robot-EdgeAI/vision): YOLO detector and Kalman tracking modules.
- [`/docs`](file:///C:/Users/Thinkpad/.gemini/antigravity/scratch/Delta-Robot-EdgeAI/docs): Thesis documentation files and introductions.

---

## 🎥 Demo Video & Full Resources

- **[Click here to view Demo & Project Files (Google Drive)](https://drive.google.com/drive/u/1/folders/15FrvZ3mc-QcWxVCQaWBIcRBczue9rxv0)**
- Includes: Video demo, Research Poster, Thesis PDF, and supplementary datasets.

---

## 🏗 Setup & Installation

1. Flash the firmware to Arduino Mega using Arduino IDE (found in [`/firmware`](file:///C:/Users/Thinkpad/.gemini/antigravity/scratch/Delta-Robot-EdgeAI/firmware)).
2. Install dependencies on your machine or Jetson Nano:
   ```bash
   pip install -r requirements.txt
   ```
3. Run the system:
   ```bash
   python main.py
   ```

---

## 📄 License & Contributing
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details. Contributions are welcome! Please read [CONTRIBUTING.md](CONTRIBUTING.md) to get started.

---
*Developed by Tran Gia Bao as part of a University Thesis Project.*

