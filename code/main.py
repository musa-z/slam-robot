# main.py
import sys  # Imports sys library for exiting the code
from robot_control import RobotControl  # Imports RobotControl class

# Choose SLAM configuration
SLAM_MODE = "monocular" # "monocular", "rgbd", "stereo"

def main():
    robot = RobotControl() # Start robot control GUI and open serial port

    if SLAM_MODE == "monocular":    # Check SLAM mode is monocular
        from monocular_slam import MonocularSLAM    # Imports MonocularSLAM class
        slam = MonocularSLAM(calib_file="mono_calibration.yaml", cam_index=0)   # Imports calibration file for 1 camera
    elif SLAM_MODE == "rgbd":   # Check SLAM mode is rgbd
        from rgbd_slam import RGBDSLAM    # Imports RGBDSLAM class
        slam = RGBDSLAM(calib_file="mono_calibration.yaml", cam_index=0)   # Imports calibration file for 1 camera
    elif SLAM_MODE == "stereo": # Check SLAM mode is stereo
        from stereo_slam import StereoSLAM    # Imports StereoSLAM class
        slam = StereoSLAM(cam_left=0, cam_right=2)   # Imports calibration file for stereo cameras
    else:
        print(f"Invalid SLAM_MODE: {SLAM_MODE}")
        sys.exit(1) # Exits code if no SLAM mode is selcted

    print("[Main] Starting SLAM. Press 'q' to exit")
    slam.run()

    # When SLAM loop finishes shutdown program
    robot.shutdown()
    print("[Main] Shutdown")

if __name__ == "__main__":
    main()
