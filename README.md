# AI Vision Delta Robot Waste Sorting

Graduation thesis project for a vision-guided Delta Robot waste sorting prototype.
The system combines a Jetson Nano host for object detection with an Arduino Mega
2560 controller for inverse kinematics, serial communication, homing, and
stepper-motor motion control.

This repository is intended as a technical evidence package for embedded
software, robotics, and C/C++ fresher applications.

## What This Project Shows

- C++ firmware for Delta Robot inverse kinematics and motion sequencing.
- UART communication between a Jetson Nano host and Arduino Mega controller at
  115200 baud.
- Python-based host application for vision, tracking, serial commands, and GUI.
- YOLOv8/TensorRT object detection pipeline on embedded edge hardware.
- Hardware/software debugging across camera input, coordinate mapping, serial
  transfer, motors, limit switches, and vacuum pickup.

## Measured Prototype Results

These results are from controlled thesis/demo tests and should be treated as
prototype-level measurements, not production specifications.

| Area | Result |
| --- | --- |
| Detection model | YOLOv8n, 3 classes: metal, paper, plastic |
| Detection quality | mAP@0.5 around 0.91 in thesis testing |
| End-to-end demo speed | Around 8 FPS on Jetson Nano prototype setup |
| Robot repeatability | Around 2-3 mm in controlled tests |
| Host-to-controller link | UART serial command flow at 115200 baud |

## System Architecture

```text
Camera
  -> Jetson Nano host
      -> YOLOv8/TensorRT detection
      -> coordinate mapping and filtering
      -> serial command generation
  -> Arduino Mega 2560
      -> command parsing
      -> Delta inverse kinematics
      -> stepper motion control
      -> homing, workspace checks, vacuum pickup
```

## Repository Structure

| Path | Purpose |
| --- | --- |
| [`firmware/Delta_ver2`](firmware/Delta_ver2) | Arduino Mega firmware, Delta kinematics, motion profiles, homing, and command handling |
| [`core`](core) | Serial communication and coordinate mapping utilities |
| [`vision`](vision) | YOLO/TensorRT detector and tracking utilities |
| [`ui`](ui) | Tkinter GUI for operating and observing the prototype |
| [`docs`](docs) | Thesis-related technical notes |
| [`main.py`](main.py) | Host application entry point |

## Firmware Highlights

- Finite-state style robot control flow with idle, homing, moving, sorting, and
  error states.
- Separate motion profiles for homing, fast movement, precision movement, and
  smoother movement.
- Workspace boundary checks before executing robot movement.
- EEPROM-backed configuration for bin locations, pickup heights, and counters.
- Limit-switch based homing for the three robot axes.

Key files:

- [`firmware/Delta_ver2/Delta_ver2.ino`](firmware/Delta_ver2/Delta_ver2.ino)
- [`firmware/Delta_ver2/DeltaKinematics.cpp`](firmware/Delta_ver2/DeltaKinematics.cpp)
- [`firmware/Delta_ver2/DeltaKinematics.h`](firmware/Delta_ver2/DeltaKinematics.h)

## Host Software Highlights

- Serial controller wrapper for connect/disconnect, command sending, homing, and
  movement commands.
- TensorRT detector wrapper with preprocessing, inference, and postprocessing.
- GUI layer for running the prototype and monitoring system behavior.

Key files:

- [`core/robot_comm.py`](core/robot_comm.py)
- [`core/mapper.py`](core/mapper.py)
- [`vision/detector.py`](vision/detector.py)
- [`vision/tracker.py`](vision/tracker.py)
- [`ui/main_window.py`](ui/main_window.py)

## Demo and Thesis Materials

- Portfolio page: https://9baotran1.github.io/firmware-motion-control-portfolio/
- Demo and thesis resources: https://drive.google.com/drive/u/1/folders/15FrvZ3mc-QcWxVCQaWBIcRBczue9rxv0

The portfolio contains a short demo video, thesis poster, thesis report, and
project review material.

## Running the Host Application

The full system requires the physical robot, Arduino Mega firmware, camera, and
Jetson Nano environment. For code review, the repository can still be inspected
module by module.

Typical Jetson setup:

```bash
python -m pip install -r requirements.txt
python main.py
```

Notes:

- TensorRT Python bindings should match the JetPack/TensorRT installation on the
  Jetson device.
- The Arduino firmware is located in `firmware/Delta_ver2`.
- Serial communication defaults to 115200 baud.

## Author

Tran Gia Bao

Control Engineering and Automation, International University - VNU-HCM

GitHub: https://github.com/9BaoTran1
