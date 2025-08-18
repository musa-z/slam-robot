# stereo_slam.py 
import cv2  # Imports OpenCV library for processing video data
import numpy as np  # Imports numpy for matrix calculations
import math # Imports math for math operations
from filterpy.kalman import KalmanFilter  # Imports Kalman Filter from filterpy library
import robot_control  # Import to read robots current motion

class MyKalman2D:
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

class StereoSLAM:
    def __init__(self, cam_left, cam_right, width=320, height=240):
        # Open left and right cameras with index, resolution, frame rate set
        self.cap_left  = cv2.VideoCapture(cam_left)
        self.cap_right = cv2.VideoCapture(cam_right)
        self.width  = width
        self.height = height
        self.cap_left.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        self.cap_left.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap_right.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        self.cap_right.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap_left.set(cv2.CAP_PROP_FPS, 15)
        self.cap_right.set(cv2.CAP_PROP_FPS, 15)

        # Error messages if cameras cant be opened
        if not self.cap_left.isOpened():
            print(f"Could not open left camera index {cam_left}.")
            exit(1)
        if not self.cap_right.isOpened():
            print(f"Could not open right camera index {cam_right}.")
            exit(1)

        self.f = 300.0  # Focal lenght
        cx = width/2.0  # Principal point x
        cy = height/2.0  # Principal point y
        self.K1 = np.array([[self.f, 0., cx],
                            [0., self.f, cy],
                            [0., 0., 1.]], dtype=np.float64)  # Left camera intrinsic matrix
        self.D1 = np.zeros(5, dtype=np.float64)  # Assume zero distortion for left camera
        self.K2 = self.K1.copy()  # Right camera uses same intrinsic matrix
        self.D2 = np.zeros(5, dtype=np.float64)  # Zero distortion for right camera
        self.R = np.eye(3, dtype=np.float64)  # Assume identity rotation between cameras
        self.T = np.array([0.07, 0.0, 0.0], dtype=np.float64).reshape(3,1)  # Camera lens 7cm apart

        # Stereo rectification
        image_size = (width, height)
        R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
            self.K1, self.D1,
            self.K2, self.D2,
            image_size,
            self.R, self.T
        )  # Calculate rectification with parameters

        self.mapLx, self.mapLy = cv2.initUndistortRectifyMap(
            self.K1, self.D1, R1, P1, image_size, cv2.CV_32FC1
        )  # Left rectification map

        self.mapRx, self.mapRy = cv2.initUndistortRectifyMap(
            self.K2, self.D2, R2, P2, image_size, cv2.CV_32FC1
        )  # Right rectification map

        # For calulcating disparity
        self.stereo = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=16*3,
            blockSize=5,
            P1=8*3*(5**2),
            P2=32*3*(5**2),
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=32
        )  

        # Initialises ORB for feature detection
        self.orb = cv2.ORB_create(nfeatures=100)  # ORB feature

        # Set left and right frame data
        self.prev_gray_left  = None  
        self.prev_color_left = None  
        self.prev_kp = None  
        self.prev_des = None  

        self.global_pose = np.eye(4, dtype=np.float64)  # 4x4 global pose matrix
        self.translation_threshold = 0.02  # Threshold to accept a translation update
        self.ransac_threshold = 1.5  # RANSAC threshold for essential matrix
        self.min_inliers = 30  # Minimum number of inlier matches required
        self.ratio_thresh = 0.6  # Lowes ratio for filtering feature matches

        # Initialise 2D map
        self.map_size = 600  # Map 600x600
        self.map_img = 255 * np.ones((self.map_size, self.map_size), dtype=np.uint8) 
        self.map_origin = (self.map_size // 2, self.map_size // 2)  # Robot start in center
        self.scale = 50.0  # Scale from camera coords to map

        # Kalman filter for 2D pose (x, z, theta)
        self.kf = MyKalman2D()

        self.frame_count = 0  # Counter to keep track of processed frames
        self.skip_frames = 3  # Process heavy computations every 3 frames to reduce load

        print("[StereoSLAM] Initialized. Press 'q' to quit")

    def update_occupancy_grid(self, disparity): # Convert disparity map to 2D occupancy grid
        step = 20  # Process every 20 pixels
        fx = self.f  # Focal length
        cx = self.width / 2.0  # Image center x axis
        baseline = np.linalg.norm(self.T)   # Calculates distance between cameras
        if baseline < 1e-6: # Ensures it isnt 0
            baseline = 0.1

        for v in range(0, self.height, step):
            for u in range(0, self.width, step):    # Loop through pixels in disparity map
                disp_val = disparity[v, u]  # Extract disparity at pixel
                if disp_val <= 0:   # Skip if no disparities found
                    continue
                depth = (fx * baseline) / (disp_val + 1e-6) # Converts disparity to actual distance
                if depth > 3.0: # Limit to stay within 3 meters
                    continue
                # Calculated 3D coords
                Xc = (u - cx) * depth / fx  # X coord
                Zc = depth  # Z coord
                pt_cam = np.array([Xc, 0, Zc, 1], dtype=np.float64).reshape(4, 1)   # Convert to homogeneous
                pt_world = self.global_pose @ pt_cam  # Convert to world coordinates
                xw, zw = pt_world[0, 0], pt_world[2, 0] # Extract world coord from point
                # Convert world coordinates to map coordinates
                mx = int(self.map_origin[0] + xw * self.scale)
                my = int(self.map_origin[1] - zw * self.scale)
                # Mark obstacle in the map
                if 0 <= mx < self.map_size and 0 <= my < self.map_size:
                    self.map_img[my, mx] = 0  

    def run(self):  # Function which runs the main SLAM loop
        while True:
            retL, frameL = self.cap_left.read()  # Read a frame from the left camera
            retR, frameR = self.cap_right.read()  # Read a frame from the right camera
            # If camera is disconnected or no frame then loop is exited
            if not retL or frameL is None:  
                print("Failed to read left camera")
                break
            if not retR or frameR is None:
                print("Failed to read right camera")
                break

            # Rectify the frames using the calculated maps
            rect_left_color = cv2.remap(frameL, self.mapLx, self.mapLy, cv2.INTER_LINEAR)
            rect_right_color = cv2.remap(frameR, self.mapRx, self.mapRy, cv2.INTER_LINEAR)
            # Displays rectified camera feeds
            cv2.imshow("Left Camera", rect_left_color)
            cv2.imshow("Right Camera", rect_right_color)

            self.frame_count += 1  # Increment the frame counter
            do_heavy_stuff = (self.frame_count % self.skip_frames == 0) # Counts frames and decides when to do heavy calcs to reduce load
            if do_heavy_stuff:
                # Convert rectified images to grayscale for disparity
                grayL = cv2.cvtColor(rect_left_color, cv2.COLOR_BGR2GRAY)
                grayR = cv2.cvtColor(rect_right_color, cv2.COLOR_BGR2GRAY)
                # Calculate disparity
                disp = self.stereo.compute(grayL, grayR).astype(np.float32) / 16.0  # Calcs disparity between grayscale images
                disp_vis = cv2.normalize(disp, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)  # Normalise disparity for visualisation
                disp_vis = cv2.medianBlur(disp_vis, 5)  # Add median blur
                cv2.imshow("Disparity", disp_vis)   # Displays disparity

                # Detects ORB keypoints in grayscale image
                if self.prev_gray_left is not None and self.prev_des is not None:
                    kp, des = self.orb.detectAndCompute(grayL, None)
                    # Matches features between prev and current frames
                    if des is not None and len(des) >= 2:
                        bf = cv2.BFMatcher(cv2.NORM_HAMMING)
                        knn = bf.knnMatch(self.prev_des, des, k=2)  # Uses KNN method
                        good_matches = []
                        for m, n in knn:
                            if m.distance < self.ratio_thresh * n.distance: # Filters macthes based on Lowe's ratio test
                                good_matches.append(m)  # Append good matches to list
                        if len(good_matches) > self.min_inliers:    # Ensures enough good macthes for calcs
                            # Extracts coords of matched points from prev and current frames
                            pts_prev = np.float32([self.prev_kp[m.queryIdx].pt for m in good_matches])
                            pts_curr = np.float32([kp[m.trainIdx].pt for m in good_matches])
                            # Uses RANSAC method with 99.9% confidence and 1.5 pixel error tolerance
                            E, mask = cv2.findEssentialMat(
                                pts_curr, pts_prev,
                                self.K1,
                                method=cv2.RANSAC,
                                prob=0.999,
                                threshold=self.ransac_threshold
                            )
                            # Makes sure essential matrix is valid with reliable matches
                            if E is not None and mask is not None:
                                inliers = mask.sum()
                                if inliers > self.min_inliers:
                                    # Get rotation and translation from the essential matrix
                                    _, R, t, _ = cv2.recoverPose(E, pts_curr, pts_prev, self.K1)
                                    # Get robot translation and rotation increments from camera movement
                                    if np.linalg.norm(t) >= self.translation_threshold:
                                        dx = t[0][0]
                                        dz = t[2][0]
                                        dtheta = math.atan2(R[0,2], R[2,2])
                                        delta = np.array([dx, dz, dtheta])
                                        # Read the motion from robot_control
                                        motion = robot_control.current_motion
                                        if motion in ["forward", "back"]:
                                            # If moving back then invert flow
                                            if motion == "back":
                                                delta[0] = -delta[0]
                                                delta[1] = -delta[1]

                                            # Kalman Filter
                                            self.kf.predict()  # Predict next state
                                            self.kf.update(delta)  # Updates Kalman Filter

                                            pose = self.kf.get_pose()  # Get updated 2D pose [x, z, theta]
                                            # Update global matrix based on estimated pose
                                            c = math.cos(pose[2])
                                            s = math.sin(pose[2])
                                            self.global_pose = np.array([
                                                [c, 0, s, pose[0]],
                                                [0, 1, 0, 0],
                                                [-s, 0, c, pose[1]],
                                                [0, 0, 0, 1]
                                            ], dtype=np.float64)

                                        # Displays previous and current frames with matched features from left camera
                                        if self.prev_color_left is not None:
                                            match_img = cv2.drawMatches(
                                                self.prev_color_left, self.prev_kp,
                                                rect_left_color, kp,
                                                good_matches,
                                                None,
                                                flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
                                            )
                                            cv2.imshow("Feature Matches", match_img)    # Displays feature matches

                # Store for next iteration
                self.prev_gray_left = grayL
                self.prev_color_left = rect_left_color
                self.prev_kp, self.prev_des = self.orb.detectAndCompute(grayL, None)
                
                # Update the occupancy grid using the disparity map
                self.update_occupancy_grid(disp)
                map_bgr = cv2.cvtColor(self.map_img, cv2.COLOR_GRAY2BGR)
                pose = self.kf.get_pose()  # Get updated 2D pose [x, z, theta]
                # Convert coords to calc robots position on map
                px = int(self.map_origin[0] + pose[0] * self.scale)
                py = int(self.map_origin[1] - pose[1] * self.scale)
                if 0 <= px < self.map_size and 0 <= py < self.map_size:
                    cv2.circle(map_bgr, (px, py), 4, (0, 0, 255), -1)   # Mark robots position on map
                cv2.imshow("SLAM Map", map_bgr)  # Display updated map

            # Quit program when 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # Close program and close windows
        self.cap_left.release()
        self.cap_right.release()
        cv2.destroyAllWindows()
        print("[StereoSLAM] Shutdown")
