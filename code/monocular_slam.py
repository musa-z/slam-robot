# monocular_slam.py
import cv2  # Imports OpenCV library for processing video data
import numpy as np  # Imports numpy for matrix calculations
import yaml # Imports yaml to read calibration files saved as .yaml
import math # Imports math for math operations
from filterpy.kalman import KalmanFilter  # Imports Kalman Filter from filterpy library

class MyKalman2D:   # Kalman Filter class
    def __init__(self):
        self.pose = np.zeros(3)  # Pose in 2D state [x, y, theta]

        # FilterPy Kalman Filter with 3 states and 3 measurement dims
        self.kf = KalmanFilter(dim_x=3, dim_z=3)
        self.kf.x = np.zeros(3) # Initialises to [0, 0, 0]
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
        self.kf.x = np.zeros(3)

    # Returns the current 2D position of robot
    def get_pose(self):
        return self.pose.copy()


class MonocularSLAM:
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

        # Initialises ORB and BFMatcher for feature detection
        self.orb = cv2.ORB_create(nfeatures=500)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        # Set previous frame data
        self.prev_frame_color = None
        self.prev_frame_gray  = None
        self.prev_kps = None
        self.prev_desc= None

        # Initalise 2D map
        self.map_size = 600 # Map 600x600
        self.map_img  = 255*np.ones((self.map_size, self.map_size), dtype=np.uint8)
        self.map_origin = (self.map_size//2, self.map_size//2)  # Robot start in center
        self.scale = 50.0   # Scale from camera coords to map

        # Sets Kalman Filter class to variable
        self.kf = MyKalman2D()

        print("[MonocularSLAM] Initialized. Press 'q' to quit")    # Message indiciating Monocular SLAM is running

    def run(self):  # Function which runs the main SLAM loop
        while True:
            ret, frame = self.cap.read()    # Reads a frame from the camera
            if not ret: # If camera is disconnected loop is exited
                break

            # Show raw camera feed
            cv2.imshow("Camera Feed", frame)

            # Undistort video using camera matrix and distortion coefficients
            undist = cv2.undistort(frame, self.mtx, self.dist)
            gray = cv2.cvtColor(undist, cv2.COLOR_BGR2GRAY) # Converts to grayscale

            # ORB detection for keypoints
            kps, des = self.orb.detectAndCompute(gray, None)

            # Feature matching with previous frame if there is one
            if self.prev_frame_gray is not None and self.prev_desc is not None and des is not None:
                matches = self.bf.match(self.prev_desc, des)    # Matches features between previous and current frame
                if len(matches) > 8:    # Minimum 8 matches needed to calc essential matrix
                    # Keep top 50 best feature matches
                    matches = sorted(matches, key=lambda x: x.distance)[:50]    # Sorts based on distance
                    # Extracts coords and puts into arrays
                    pts_prev = np.float32([self.prev_kps[m.queryIdx].pt for m in matches])
                    pts_curr = np.float32([kps[m.trainIdx].pt for m in matches])

                    # Essential matrix & recover pose
                    # Uses RANSAC method with 99.9% confidence and 1.0 pixel error tolerance
                    E, _ = cv2.findEssentialMat(pts_curr, pts_prev, self.mtx,
                                                method=cv2.RANSAC, prob=0.999, threshold=1.0)
                    if E is not None:   # Ensures essential matrix calc is completed
                        # Extracts rotation and translation matricies to estimate cameras motion between frames
                        _, R, t, _ = cv2.recoverPose(E, pts_curr, pts_prev, self.mtx)
                        dx = t[0][0]
                        dy = t[2][0]
                        dtheta = math.atan2(R[0,2], R[2,2]) # Yaw angle

                        # Kalman filter 
                        self.kf.predict()   # Predicts next state
                        self.kf.update(np.array([dx, dy, dtheta]))  # Updates Kalman Filter

                        # Mark features on map
                        pose = self.kf.get_pose()  # Get updated 2D pose [x, y, theta]
                        for pt in pts_curr: # Loop through each keypoint matched
                            # Calcs coords of feature points to put on map taking scale into account
                            mx = int(self.map_origin[0] + (pose[0] + (pt[0]-self.img_w/2)/100.0)*self.scale)
                            my = int(self.map_origin[1] - (pose[1] + (pt[1]-self.img_h/2)/100.0)*self.scale)
                            # Mark on 2D map
                            if 0 <= mx < self.map_size and 0 <= my < self.map_size:
                                self.map_img[my, mx] = 0

                        # Displays previous and current frame with matched features
                        match_img = cv2.drawMatches(self.prev_frame_color, self.prev_kps,
                                                    undist, kps,
                                                    matches, None,
                                                    flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
                        cv2.imshow("Feature Matches", match_img)

            # Store for next iteration
            self.prev_frame_color = undist
            self.prev_frame_gray  = gray
            self.prev_kps = kps
            self.prev_desc= des

            # Draw 2D map
            color_map = cv2.cvtColor(self.map_img, cv2.COLOR_GRAY2BGR)
            pose = self.kf.get_pose()  # Get [x, y, theta]
            # Calcs robots current position
            px = int(self.map_origin[0] + pose[0]*self.scale)
            py = int(self.map_origin[1] - pose[1]*self.scale)
            # Mark robot as red circle on map
            if 0 <= px < self.map_size and 0 <= py < self.map_size:
                cv2.circle(color_map, (px, py), 4, (0,0,255), -1)
            cv2.imshow("SLAM Map", color_map)

            # Quit program when 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break  

        # Close program and close windows
        self.cap.release()
        cv2.destroyAllWindows()
        print("[MonocularSLAM] Shutdown")
