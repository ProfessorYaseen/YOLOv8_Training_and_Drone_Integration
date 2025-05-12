# YOLOv8 Training and Drone Integration

## Overview
This repository showcases a robust vehicle detection system, integrating YOLOv8 with ROS 2 and Gazebo on Ubuntu 22.04 LTS. Designed as a beginner-friendly project, it simulates a drone detecting vehicles in real-time, with applications in military drone scenarios such as autonomous surveillance. Built using free, open-source tools, this project serves as a compelling addition to a freelancing portfolio or a stepping stone to advanced AI and robotics development.

## Project Structure
### Step 1: Software Setup
- Installed ROS 2 Humble, Gazebo 11, Python 3.8, and key libraries (ultralytics, opencv-python, numpy).
- Configured development environment with VS Code, Git, and Kazam for coding and documentation.

### Step 2: Dataset Preparation and Model Training
- Curated a dataset from VisDrone2019, selecting 500 training and 100 validation images focused on cars and trucks.
- Trained a YOLOv8 nano model on Google Colab’s free GPU tier, producing the `vehicle_detection.pt` model for deployment.

### Step 3: Integration with ROS 2 and Gazebo
- Developed the `yolo_detector` ROS 2 package (located in `src/yolo_detector/`) to process drone camera feeds in real-time.
- Simulated an Iris drone and a Prius Hybrid car in Gazebo, achieving a detection accuracy of 81% in the simulated environment.
- Included are the core files: `yolo_detector.py` (detection script), `vehicle_detection.pt` (trained model), and package configurations.

## Repository Contents
- `src/yolo_detector/`:
  - `yolo_detector.py`: Python script for YOLOv8 inference on ROS 2 camera feeds.
  - `vehicle_detection.pt`: Trained YOLOv8 model (note: may need to be downloaded separately due to size limits).
  - `package.xml`, `setup.py`: ROS 2 package metadata and setup files.
- `install/`, `log/`: Build artifacts and logs from the ROS 2 workspace.
- `README.md`: This project documentation.

## How to Run
1. **Clone the Repository**:
   ```bash
   git clone https://github.com/ProfessorYaseen/YOLOv8_Training_and_Drone_Integration.git
   cd YOLOv8_Training_and_Drone_Integration/ros2_ws
