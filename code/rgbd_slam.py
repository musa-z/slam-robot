# rgbd_slam.py
import cv2  # Imports OpenCV for image processing
import numpy as np  # Imports numpy for matrix calculations
import yaml  # Imports yaml to read calibration files saved as .yaml
import math  # Imports math for math operations
import time  # Imports time for delays
import RPi.GPIO as GPIO  # Imports RPi.GPIO for raspberry pi GPIO
from filterpy.kalman import KalmanFilter  # Imports Kalman Filter from filterpy library
import robot_control  # Import to read robots current motion

class MyKalman2D:   # Kalman Filter class
    def __init__(self):
        self.pose = np.zeros(3, dtype=np.float32)   # Pose is [x, y, theta]

        # FilterPy Kalman Filter with 3 states and 3 measurement dims
        self.kf = KalmanFilter(dim_x=3, dim_z=3)

        self.kf.x = np.zeros(3, dtype=np.float32)   # Initialise to [0, 0, 0]
        # F, H, P, R, Q are standard Kalman Filter matrix identities
        self.kf.F = np.eye(3)   # Set to identity matrix
        self.kf.H = np.eye(3)   # Set to identity matrix
        self.kf.P *= 1000.0          # Large initial uncertainty
        self.kf.R = np.eye(3) * 0.5  # Measurement noise
        self.kf.Q = np.eye(3) * 0.1  # Process noise

    def predict(self):  # Kalman Filter to predict next state
        self.kf.predict()

    # Sets new values in Kalman Filter
    def update(self, delta):
        self.kf.update(delta)
        self.pose += self.kf.x
        self.kf.x = np.zeros(3, dtype=np.float32)

    # Returns the current 2D position of robot
    def get_pose(self):
        return self.pose.copy()


class RGBDSLAM:
    def __init__(self, calib_file='mono_calibration.yaml', cam_index=0):
        # Load camera calibration from .yaml file
        with open(calib_file, 'r') as fs:
            data = yaml.safe_load(fs)

        # Set variables from calibration
        self.mtx = np.array(data['camera_matrix'], dtype=np.float32)
        self.dist = np.array(data['distortion_coeffs'], dtype=np.float32)
        self.img_w = data['image_width']
        self.img_h = data['image_height']

        # Sets camera at given resolution and frame rate
        self.cap = cv2.VideoCapture(cam_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.img_w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.img_h)
        self.cap.set(cv2.CAP_PROP_FPS, 20)

        # Ultrasonic sensor pins
        self.TRIG = 23
        self.ECHO = 24
        # GPIO setup
        GPIO.setmode(GPIO.BCM)  
        GPIO.setup(self.TRIG, GPIO.OUT)
        GPIO.setup(self.ECHO, GPIO.IN)

        # Initialises ORB and BFMatcher for feature detection
        self.orb = cv2.ORB_create(nfeatures=500)
        self.bf  = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        # Sets Kalman Filter class to variable
        self.kf = MyKalman2D()

        # Initalise 2D map
        self.map_size = 600 # Map 600x600
        self.map_img  = 255*np.ones((self.map_size, self.map_size), dtype=np.uint8)
        self.map_origin = (self.map_size//2, self.map_size//2)  # Robot start in center
        self.scale = 50.0   # Scale from camera coords to map

        # Set previous frame data
        self.prev_gray  = None
        self.prev_color = None
        self.prev_kps   = None
        self.prev_desc  = None

        print("[RGBDSLAM] Initialized. Press 'q' to quit") # Message indiciating RGB-D SLAM is running

    def get_distance(self): # Gets distance from ultrasonic sensor
        try:
            GPIO.output(self.TRIG, True)    # Sets trig pin to high
            time.sleep(0.00001)     # Short delay
            GPIO.output(self.TRIG, False)   # Sets trig pin to low

            # Initialise variables
            start_t = time.time()
            stop_t  = start_t
            timeout = 0.05

            # Wait for echo to go high
            while GPIO.input(self.ECHO) == 0:   
                start_t = time.time()   # Update start time to time when pin goes high
                if (time.time()-stop_t) > timeout:  # If it takes too long then timeout and set distance 1000
                    return 1000

            # Wait for echo to go low
            while GPIO.input(self.ECHO) == 1:   
                stop_t = time.time()    # Update stop time to time when pin goes low
                if (time.time()-start_t) > timeout: # If it takes too long then timeout and set distance 1000
                    return 1000

            # Calculate distance from the time difference
            time_diff = stop_t - start_t
            dist_cm = (time_diff*34300)/2.0   # (time_diff * speed of sound) / 2
            return round(min(dist_cm,400),2)    # Rounds to 2dp with maximum range of sensor 4m
        except:
            return 1000 # Exception handling returns 1000 if it runs into an error

    def run(self):  # Function which runs the main SLAM loop
        while True:
            ret, frame = self.cap.read()    # Reads a frame from the camera
            if not ret:    # If camera is disconnected loop is exited
                break

            # Show raw camera feed
            cv2.imshow("Camera Feed", frame)

            # Undistort video using camera matrix and distortion coefficients
            undist = cv2.undistort(frame, self.mtx, self.dist)
            gray   = cv2.cvtColor(undist, cv2.COLOR_BGR2GRAY)   # Converts to grayscale

            # ORB detection for keypoints
            kps, des = self.orb.detectAndCompute(gray, None)

            # Read ultrasonic distance
            dist_cm = self.get_distance()
            print(f"Distance: {dist_cm} cm")    # Print distance

            match_img = None  # Store colour matches
            if self.prev_gray is not None and self.prev_desc is not None and des is not None:
                matches = self.bf.match(self.prev_desc, des)    # Matches features between previous and current frame
                if len(matches) > 8:    # Minimum 8 matches needed to calc essential matrix
                    # Keep top 50 best feature matches
                    matches = sorted(matches, key=lambda x: x.distance)[:50]
                    # Extracts coords and puts into arrays
                    pts_prev = np.float32([self.prev_kps[m.queryIdx].pt for m in matches])
                    pts_curr = np.float32([kps[m.trainIdx].pt for m in matches])

                    # Lumps all points
                    flow = np.mean(pts_curr - pts_prev, axis=0)  # [dx, dy]
                    dx, dy, dtheta = 0.0, 0.0, 0.0

                    # Read the motion from robot_control
                    motion = robot_control.current_motion
                    if motion in ["forward","back"]:
                        # If moving back then invert flow
                        if motion == "back":
                            flow = -flow
                        dx, dy = flow[0], flow[1]

                        # Kalman Filter
                        self.kf.predict()   # Predicts next state
                        self.kf.update([dx, dy, dtheta])    # Updates Kalman Filter

                    # Mark features on map
                    pose = self.kf.get_pose()  # Get updated 2D pose [x,y,theta]
                    for pt in pts_curr: # Loop through each keypoint matched
                        # Calcs coords of feature points to put on map taking scale into account
                        mx = int(self.map_origin[0] + (pose[0] + (pt[0]-self.img_w/2)/100.0)*self.scale)
                        my = int(self.map_origin[1] - (pose[1] + (pt[1]-self.img_h/2)/100.0)*self.scale)
                        # Mark on 2D map
                        if 0 <= mx < self.map_size and 0 <= my < self.map_size:
                            self.map_img[my,mx] = 0

                    # Displays previous and current frames with matched features
                    if self.prev_color is not None:
                        match_img = cv2.drawMatches(
                            self.prev_color, self.prev_kps,
                            undist, kps,
                            matches, None,
                            flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
                        )

            # Place a single obstacle if distance is less than 100cm using ultrasonic
            if dist_cm < 100:
                pose = self.kf.get_pose()
                # Interpret forward to negative y
                obs_x = pose[0]
                obs_y = pose[1] - (dist_cm/100.0)
                # Calcs coords of feature points to put on map taking scale into account
                mx = int(self.map_origin[0] + obs_x*self.scale)
                my = int(self.map_origin[1] - obs_y*self.scale)
                # Mark on 2D map
                if 0 <= mx < self.map_size and 0 <= my < self.map_size:
                    self.map_img[my,mx] = 0

            # Mark robot as red circle on map
            pose = self.kf.get_pose()
            px = int(self.map_origin[0] + pose[0]*self.scale)
            py = int(self.map_origin[1] - pose[1]*self.scale)
            color_map = cv2.cvtColor(self.map_img, cv2.COLOR_GRAY2BGR)
            if 0<=px<self.map_size and 0<=py<self.map_size:
                cv2.circle(color_map, (px, py), 4, (0,0,255), -1)

            # Show feature matches or keep blank if none
            if match_img is not None:
                cv2.imshow("Feature Matches", match_img)
            else:
                blank = np.zeros((self.img_h, self.img_w, 3), dtype=np.uint8)
                cv2.imshow("Feature Matches", blank)

            # Show the final 2D map
            cv2.imshow("SLAM Map", color_map)

            # Quit program when 'q' is pressed
            if cv2.waitKey(1)&0xFF==ord('q'):
                break

            # Store data for next iteration
            self.prev_gray  = gray
            self.prev_color = undist
            self.prev_kps   = kps
            self.prev_desc  = des

        # Close program and close windows
        self.cap.release()
        cv2.destroyAllWindows()
        GPIO.cleanup()
        print("[RGBDSLAM] Shutdown")
