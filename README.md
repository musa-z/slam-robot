# SLAM Robot

## Overview
**University Project:** Implementation of SLAM (Simultaneous Localisation and Mapping) using a mobile robot and simple sensor/camera setup. Developed as part of undergraduate degree.

This project demonstrates hands on experience with robotics, Python programming, computer vision, and mapping algorithms.

---

## Features
- Real-time SLAM mapping on Raspberry Pi
- Obstacle detection and avoidance
- Indoor mapping of small scale environments
- Multiple SLAM configurations: Monocular, RGB-D and Stereo

---

## Code
- Written in Python
- Scripts located in the `code/` folder
- Includes algorithms for camera calibration, mapping, navigation and obstacle detection

---

## Media
- Screenshots of maps for each configuration
- Screenshot of feature matches and disparity
- Screenshot of camera calibration process

---

## Setup
- Requires installation of OpenCV, NumPy, PyYAML, FilterPy, PySerial and RPi.GPIO libraries
- SLAM configuration must be set in main.py before executing

```bash
# Clone the repo
git clone https://github.com/musa-z/slam-robot.git

# Navigate to code folder
cd slam-robot/code

# Run camera calibration script
python calibrate_mono.py

# Run main script
python main.py
