# Mobile Robot SLAM and Localisation

Python-based mobile robot SLAM/localisation project using monocular, RGB-D and stereo vision configurations. The project combines computer vision, camera calibration, mapping, feature extraction, basic robot control and sensor-based localisation for small indoor environments.

## Overview

This project was developed as part of my BEng Robotics Engineering degree and focused on building a low-cost mobile robot mapping/localisation system using accessible hardware and open-source Python libraries.

The system supports multiple SLAM/localisation configurations:

- Monocular camera-based mapping
- RGB-D/depth-based mapping
- Stereo camera feature matching and disparity estimation
- Basic serial-based mobile robot control
- Camera calibration and visualisation tools

## Key Features

- Implemented Python-based SLAM/localisation scripts for different camera configurations
- Used OpenCV for image processing, camera calibration, feature extraction and stereo/depth processing
- Integrated serial robot control with a simple Tkinter control interface
- Tested mapping/localisation behaviour in small indoor environments
- Documented practical limitations including sensor noise, feature mismatch, calibration accuracy and low-cost hardware constraints

## Technologies Used

- Python
- OpenCV
- NumPy
- PyYAML
- FilterPy
- PySerial
- RPi.GPIO
- Tkinter
- Raspberry Pi
- Camera/depth sensors

## Repository Structure

```text
slam-robot/
├── code/
│   ├── main.py
│   ├── monocular_slam.py
│   ├── rgbd_slam.py
│   ├── stereo_slam.py
│   ├── robot_control.py
│   └── calibration/
├── media/
│   ├── camera-calibration.png
│   ├── monocular.png
│   ├── rgb-d.png
│   ├── stereo.png
│   └── stereo-feature-disparity.png
├── README.md
└── LICENSE.txt
```

## System Pipeline

```text
Camera Input
     ↓
Camera Calibration
     ↓
Image Processing / Feature Extraction
     ↓
Depth, Stereo or Monocular Processing
     ↓
Mapping / Localisation Estimate
     ↓
Robot Control and Testing
```

## SLAM Modes

The project can be run in one of three modes by changing the `SLAM_MODE` variable in `main.py`:

```python
SLAM_MODE = "monocular"  # Options: "monocular", "rgbd", "stereo"
```

### Monocular Mode

Uses a single camera input for image-based mapping/localisation experiments.

### RGB-D Mode

Uses depth information to support indoor mapping and localisation.

### Stereo Mode

Uses two camera inputs for stereo processing, feature matching and disparity estimation.

## Robot Control

The `robot_control.py` script provides a simple Tkinter-based control interface and sends serial commands to the mobile robot. This was used to support manual testing during mapping/localisation experiments.

Supported movements include:

- Forward
- Backward
- Left turn
- Right turn
- Stop

## Results

### Camera Calibration

![Camera calibration](media/camera-calibration.png)

### Monocular Mapping

![Monocular SLAM output](media/monocular.png)

### RGB-D Mapping

![RGB-D SLAM output](media/rgb-d.png)

### Stereo Processing

![Stereo SLAM output](media/stereo.png)

### Stereo Feature Matching and Disparity

![Stereo feature disparity](media/stereo-feature-disparity.png)

## Setup

Clone the repository:

```bash
git clone https://github.com/musa-z/slam-robot.git
cd slam-robot/code
```

Install dependencies:

```bash
pip install opencv-python numpy pyyaml filterpy pyserial
```

For Raspberry Pi GPIO support:

```bash
pip install RPi.GPIO
```

Run camera calibration:

```bash
python calibrate_mono.py
```

Run the main program:

```bash
python main.py
```

## Limitations

This project was developed using low-cost hardware, so performance was affected by:

- Camera calibration quality
- Sensor noise
- Lighting variation
- Feature mismatch in low-texture environments
- Limited processing power on embedded hardware
- Drift and uncertainty during longer runs

## What I Learned

This project improved my practical understanding of:

- Mobile robot localisation
- Computer vision for robotics
- Camera calibration
- Stereo/depth processing
- Kalman-filter-based estimation
- Serial robot control
- Testing and debugging under noisy real-world conditions

## Future Improvements

- Add ROS2 integration
- Improve real-time visualisation
- Add saved map output
- Add quantitative localisation error analysis
- Improve sensor fusion between camera/depth data and wheel odometry
- Add automated navigation instead of manual robot control

