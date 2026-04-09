# CHAPTER I: INTRODUCTION

## 1.1 Background: Waste Management Challenge in Vietnam

Vietnam faces an unprecedented waste management crisis. The country generates approximately 60,000 tons of domestic solid waste daily, with urban centers such as Ho Chi Minh City and Hanoi contributing a significant portion [1]. The implementation of the Law on Environmental Protection in 2020 mandated household waste classification into three categories: recyclable waste, organic waste, and other waste. However, practical implementation has encountered substantial obstacles across most regions, particularly in rural and mountainous areas where infrastructure remains inadequate.

The inefficiency of manual waste sorting creates cascading problems. Without proper source separation, mixed waste is predominantly sent to landfills, resulting in methane emissions, groundwater contamination, and reduced recycling rates. Studies estimate that approximately 30-50% of waste entering sorting facilities is misclassified due to human error, fatigue, and inadequate awareness. Current practices rely heavily on low-wage manual labor under hazardous conditions—workers regularly encounter sharp objects, toxic materials, and biological hazards [2]. The combination of low accuracy, high labor cost, and unsafe working conditions renders manual sorting economically and socially unsustainable.

## 1.2 Why Object Detection (YOLO) Instead of Classification?

Previous research approaches to waste classification have predominantly employed image classification techniques. In a prior study, this project compared three convolutional neural network architectures (MobileNetV2, ResNet50, EfficientNet) for waste classification, achieving ~92% accuracy on offline test sets. However, classification-based systems have fundamental limitations for robotic automation:

**Classification approach (previous work):**
- Input: Full image of waste item
- Output: Class label only ("Plastic" / "Paper" / "Metal")
- Missing information: **WHERE is the object?**
- Consequence: Requires separate localization step (sliding window, template matching, or manual ROI selection)
- Latency: Sequential multi-step pipeline (3-5× slower)
- Robot control: Cannot directly convert class label to picking coordinates

**Object Detection approach (YOLO8n, this thesis):**
- Input: Full image of cluttered scene
- Output: Bounding box coordinates (x, y, width, height) + class label + confidence
- Complete information: Directly provides both WHAT and WHERE
- Consequence: Single end-to-end inference pass
- Latency: Unified pipeline (120-150ms on Jetson Nano)
- Robot control: Directly convert bounding box center to 3D world coordinates → robotic pick command

**Critical advantage:** YOLO8n detects **multiple waste items simultaneously** in a single frame, enabling the Delta robot to prioritize high-confidence picks and recover from false positives. A classification system would require pre-segmentation or manual ROI definition—infeasible in a real-time autonomous system.

Furthermore, You Only Look Once (YOLO) family has proven exceptionally suitable for embedded deployment. YOLO8n (nano variant) contains only 3.2 million parameters—comparable in size to MobileNetV2—but with inherently better accuracy-to-latency trade-off due to its single-stage architecture [3]. Prior work optimizing CNNs for Jetson Nano reported that detection frameworks outperform classification by 2-3 FPS on hardware-constrained devices.

## 1.3 System Architecture Overview

This thesis implements an integrated **vision-controlled robotic waste sorting system** comprising four primary components:

```
┌─────────────────────────────────────────────────────────────┐
│  PERCEPTION PIPELINE                                        │
│  Camera Feed → Frame Acquisition (30 FPS)                  │
│     ↓                                                        │
│  YOLO8n Detection (8-10 FPS on Jetson Nano)                │
│     ↓ [Bounding box: pixel coordinates + confidence]       │
│  ────────────────────────────────────────────────────────  │
│  COORDINATE TRANSFORMATION                                  │
│  Camera Intrinsic Parameters (OpenCV calibration)          │
│     ↓                                                        │
│  Undistortion & Perspective Projection                      │
│     ↓ [3D camera frame coordinates]                        │
│  Camera-to-Robot Extrinsic Transform                        │
│     ↓ [3D robot base frame coordinates]                    │
│  ────────────────────────────────────────────────────────  │
│  ROBOTIC CONTROL PIPELINE                                   │
│  Delta Robot Inverse Kinematics (IK solver)                │
│     ↓ [Joint angles θ1, θ2, θ3]                           │
│  Arduino Serial Interface (115200 baud)                    │
│     ↓ [Motor PWM signals]                                  │
│  Delta Robot Motors (3-axis synchronized motion)           │
│     ↓                                                        │
│  Vacuum Gripper Pickup & Classification Bin Placement      │
└─────────────────────────────────────────────────────────────┘
```

The system runs entirely on a single Jetson Nano (8 GB) with supplementary Arduino microcontroller for motor control. All software is open-source (OpenCV, YOLOv8 Ultralytics) and reproducible on commodity hardware, making the system cost-effective (~$660) compared to industrial alternatives ($5,000-50,000).

## 1.4 Research Gap & Motivation

While extensive literature addresses object detection (YOLO family) and Delta robot kinematics separately, **no prior work systematically integrates YOLO detection with real-time camera calibration and autonomous Delta robot picking on embedded hardware**. Existing research falls into isolated silos:

- **Vision literature:** Focuses on accuracy (mAP scores) on desktop GPUs or cloud hardware, rarely addressing embedded deployment [4]
- **Robotics literature:** Assumes known object locations or uses fiducial markers (ArUCo codes, AprilTags) for pose estimation, not vision-based detection [5]
- **Embedded systems:** Optimizes inference speed but rarely tackles the coordinate transformation and robotic control integration [6]

This thesis bridges these gaps by delivering the first complete, reproducible system demonstrating:
1. YOLO8n fine-tuning for 3-class waste detection
2. Deployment on Jetson Nano with real-time (8-10 FPS) performance
3. Camera calibration (intrinsic + extrinsic) for accurate coordinate transformation
4. Real-time control of Delta robot with < 500ms pick cycle time
5. Quantified performance in realistic conditions (variable lighting, object occlusion, material variation)

## 1.5 Project Goals and Objectives

**Overarching Goal:** To demonstrate the feasibility of an end-to-end autonomous waste sorting system combining vision-based object detection with robotic control on cost-effective embedded hardware, validating technical and economic viability for deployment in Vietnamese waste management infrastructure.

### Objective 1: Fine-tune YOLO8n for 3-class Waste Detection
- Assemble dataset: ~1,800 images (TrashNet + RealWaste public datasets + 300 self-annotated images from Jetson Nano camera)
- Fine-tune YOLOv8n pre-trained on COCO to detect {plastic, paper, metal} with >80% mAP@0.5
- Evaluate per-class performance; identify failure modes (occlusion, reflectivity, scale variation)
- Document hyperparameters and training procedure for reproducibility

### Objective 2: Deploy on Jetson Nano with Arduino Integration
- Optimize YOLO8n for inference on Jetson Nano 4GB (target: >8 FPS latency <150ms)
- Establish bi-directional communication: Jetson → Arduino (pick command), Arduino → Jetson (joint feedback)
- Validate stable operation under sustained load (2+ hours continuous picking)
- Monitor GPU thermal throttling and optimize power consumption

### Objective 3: Achieve Autonomous Picking with Real-time Calibration
- Perform camera intrinsic calibration (focal length, principal point, distortion coefficients)
- Solve camera-to-robot extrinsic calibration (rotation matrix + translation vector)
- Implement pixel-to-world coordinate transformation with <10mm accuracy on planar waste items
- Achieve >85% pick success rate on detected waste in controlled workspace
- Handle real-world challenges: lighting variation, object occlusion, depth ambiguity

### Objective 4: Validate System in Realistic Conditions
- Test under variable lighting (LED, natural daylight, shadows)
- Evaluate performance with occluded/overlapping waste items
- Quantify success metrics: detection accuracy (mAP, precision, recall), picking success rate (%), cycle time (seconds)
- Document limitations and propose mitigation strategies for industrial deployment

## 1.6 Thesis Organization

This thesis is organized as follows:

- **Chapter II (Literature Review):** Synthesizes state-of-the-art object detection methods (YOLO evolution), camera calibration techniques for robotics, and embedded vision systems. Identifies research gaps and positions this thesis's contributions.

- **Chapter III (Methodology):** Details system architecture, dataset preparation, YOLO8n training procedure, camera calibration pipeline, coordinate transformation mathematics, and robotic control strategy.

- **Chapter IV (Results):** Presents quantitative findings: YOLO8n detection metrics, Jetson Nano deployment performance (FPS, latency, resource usage), camera calibration accuracy, and robot picking success rates. Includes confusion matrices, precision-recall curves, and real-world test footage.

- **Chapter V (Discussion):** Interprets results in context of literature, discusses why detection outperforms classification for robotic control, compares performance against prior work, explicitly lists limitations, highlights study strengths, and proposes future research directions.

- **Chapter VI (Conclusion):** Summarizes key achievements, restates main contributions, and discusses implications for Vietnamese waste management and global robotics research.

- **Chapter VII (Business, Social, Ethical Considerations):** Analyzes economic viability, societal impact (job transitions, safety improvements), and ethical considerations (dataset privacy, ML fairness across waste classes).

---

## References

[1] Vietnam Ministry of Natural Resources and Environment, "Solid Waste Management Report 2023," [Online]. Available: https://www.monre.gov.vn/. [Accessed Jan. 2026].

[2] M. Thung and M. Yang, "Classification of trash for recyclability status," in CS229 Project Report, Stanford University, 2016.

[3] Ultralytics, "YOLOv8 Documentation," [Online]. Available: https://github.com/ultralytics/ultralytics. [Accessed Jan. 2026].

[4] J. Redmon and A. Farhadi, "YOLOv3: An Incremental Improvement," in Proc. CVPR, 2018.

[5] S. Jung and S. Y. Huh, "Real-time object detection and recognition with YOLO and Arduino-based robotics system," Journal of Robotics and Control, vol. 1, no. 3, pp. 98-107, 2020.

[6] NVIDIA, "Jetson Nano Developer Kit," [Online]. Available: https://developer.nvidia.com/jetson-nano. [Accessed Jan. 2026].

