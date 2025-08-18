# robot_control.py
import serial          # Imports to allow sending serial commands for mobile robot
import time            # Imports time for adding delays
import tkinter as tk   # Imports Tkinter GUI library
import math            # Imports math to enable operations with radians and degrees
import threading       # Imports threading library

# Global variables for robots control
current_motion = "idle"        # "forward", "back", "left", "right", "idle"
robot_orientation = 0.0        # in radians
TURN_INCREMENT = math.radians(10)  # each left or right move rotates robot by 10 deg

class RobotControl: # RobotControl class
    def __init__(self, serial_port='/dev/ttyUSB0', baudrate=115200):    # Presets serial port variables with baud rate at 115200
        self.serial_port = serial_port
        # Attempts to open serial port
        try:
            self.ser = serial.Serial(serial_port, baudrate, timeout=1) # Opens serial port and sets variables
            print(f"[RobotControl] Serial port {serial_port} opened")  # Successful message
        except Exception as e:
            print(f"ERROR opening serial port: {e}")    # Error message
            self.ser = None

        # Open Tkinter GUI in a separate thread 
        self.gui_thread = threading.Thread(target=self.start_robot_gui, daemon=True)
        self.gui_thread.start()

    def send_command(self, cmd):    # Function to send a command to robot via serial port
        if self.ser:    # If serial is connected
            try:
                self.ser.write(cmd.encode())    # Send movement command
                time.sleep(0.05)  # Short delay
            except Exception as e:
                print(f"ERROR sending serial command: {e}") # Error message

    def start_robot_gui(self):
        global current_motion, robot_orientation    # Global variables defined at the beginning

        # Builds Tkinter GUI
        root = tk.Tk()
        root.title("Robot Controls")

        # Movement functions
        def move_forward(): # Forward movement
            global current_motion
            current_motion = "forward"  # Set motion to forward in global variable
            self.send_command("8")  # Send serial command for forward movement

        def move_backward(): # Backwards movement
            global current_motion
            current_motion = "back"  # Set motion to back in global variable
            self.send_command("2")  # Send serial command for back movement

        def turn_left(): # Left turn
            global current_motion, robot_orientation
            current_motion = "left"  # Set motion to left in global variable
            self.send_command("0")  # Send serial command for left movement
            robot_orientation -= TURN_INCREMENT # Adds orientation to global variable

        def turn_right(): # Right turn
            global current_motion, robot_orientation
            current_motion = "right"  # Set motion to right in global variable
            self.send_command(".")  # Send serial command for right movement
            robot_orientation += TURN_INCREMENT # Adds orientation to global variable

        def stop_movement(): # Stop
            global current_motion
            current_motion = "idle"  # Set motion to idle in global variable
            self.send_command("5")  # Send serial command to stop movement

        # Layout of the GUI with buttons
        frame = tk.Frame(root)
        frame.pack(pady=20)

        # Buttons
        btn_forward = tk.Button(frame, text="Forward", command=move_forward,
                                height=2, width=10)
        btn_forward.grid(row=0, column=1, pady=5)

        btn_left = tk.Button(frame, text="Left", command=turn_left,
                             height=2, width=10)
        btn_left.grid(row=1, column=0, padx=10)

        btn_stop = tk.Button(frame, text="STOP", command=stop_movement,
                             height=2, width=10, fg="white", bg="red")
        btn_stop.grid(row=1, column=1, padx=10, pady=5)

        btn_right = tk.Button(frame, text="Right", command=turn_right,
                              height=2, width=10)
        btn_right.grid(row=1, column=2, padx=10)

        btn_back = tk.Button(frame, text="Back", command=move_backward,
                             height=2, width=10)
        btn_back.grid(row=2, column=1, pady=5)

        # Closes the GUI
        root.protocol("WM_DELETE_WINDOW", lambda: root.quit())
        root.mainloop()

        # After the GUI closes stop movement and close serial
        stop_movement()
        if self.ser:
            self.ser.close()

    # Updates main function on robots current movement status when called
    def update(self):
        global current_motion
        print(f"[RobotControl] Current motion: {current_motion}")
        time.sleep(0.01)

    # Runs on program close
    def shutdown(self):
        print("[RobotControl] Shutdown")
