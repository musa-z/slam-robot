# calibrate_mono.py
import cv2  # Imports OpenCV to process image data and do corner detection and calibration
import numpy as np  # Imports numpy for matrix operations
import yaml # Imports yaml to create and write to .yaml files
import os   # Imports os to allow creation of files and folders

# Chess board variables
CHESSBOARD_SIZE = (10, 7)  # 10x7 inner corners
SQUARE_SIZE = 0.015 # Each square on chess board is 15mm

CAM_INDEX = 0   # Camera index on pi
# Camera resolution
FRAME_WIDTH = 320   
FRAME_HEIGHT = 240  

def save_mono_to_yaml(filename, camera_matrix, dist_coeffs, image_size):
    # Python dictionary of calibration data
    data = {
        'camera_model': 'pinhole',
        'image_width':  image_size[0],
        'image_height': image_size[1],
        'camera_matrix': camera_matrix.tolist(),
        'distortion_coeffs': dist_coeffs.tolist()
    }
    with open(filename, 'w') as f:
        yaml.dump(data, f)  # Writes to calibration file

def calibrate_camera():
    # Camera initalisation with index, resolution and frame rate
    cap = cv2.VideoCapture(CAM_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, 20)

    # Make 3D object points for chess board
    objp = np.zeros((CHESSBOARD_SIZE[0]*CHESSBOARD_SIZE[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE

    # Arrays to store detected points
    objpoints = []
    imgpoints = []

    # Creates a new folder for images if it doesnt exist already
    save_dir = "mono_calib_images"
    if not os.path.exists(save_dir):    
        os.makedirs(save_dir)

    # Messages printed once program is initialised and running
    print("Monocular Calibration:")
    print("Press 'c' to capture image. Press 'q' to finish")

    img_count = 0   # Keeps track of images taken for calibration
    while True: # Loop
        ret, frame = cap.read() # Takes frame from camera
        if not ret:
            print("Failed to grab frame")  
            break   # If unsuccessful in taking frame break out of loop

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret_cb, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)    # Locate chess board corners

        display = frame.copy()  # Copies to display to show corners if found
        if ret_cb:
            cv2.drawChessboardCorners(display, CHESSBOARD_SIZE, corners, ret_cb)    # Draws corners

        # Displays frame
        cv2.imshow("Monocular Calibration", display)
        key = cv2.waitKey(1) & 0xFF # Waits for input key
        if key == ord('c'): # When 'c' is inputted for capturing frame
            if ret_cb:  # If chess board is detected
                img_count += 1  # Increment
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)  # Define pixel criteria
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)    # Refine corner position
                # Append points to calibration lists
                objpoints.append(objp)
                imgpoints.append(corners2)
                # Save image of current frame
                filename = os.path.join(save_dir, f"mono_calib_{img_count:02d}.jpg")
                cv2.imwrite(filename, frame)
                print(f"Captured image #{img_count} -> {filename}")
            else:
                print("Chessboard not detected")   # No board message
        elif key == ord('q'):   # Exit loop
            break
    
    # Closes video feed window
    cap.release()
    cv2.destroyAllWindows()

    # If fewer than 10 images
    if len(objpoints) < 10:
        print("Not enough images for calibration \nExiting")    # Return a message saying not enough
        return
    
    # Runs a calibrateCamera function to return camera matrix and distortion coefficients
    ret_calib, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )

    print("Calibration successful")
    print("Camera matrix:\n", mtx)  # Camera matrix printed
    print("Distortion coefficients:\n", dist)   # Distortion coefficients printed

    # Stores camera matrix and distortion coefficients and saves to .yaml file
    save_mono_to_yaml("mono_calibration.yaml", mtx, dist, (FRAME_WIDTH, FRAME_HEIGHT))
    print("Saved calibration to 'mono_calibration.yaml'")  # Prints success message

if __name__ == "__main__":
    calibrate_camera()
