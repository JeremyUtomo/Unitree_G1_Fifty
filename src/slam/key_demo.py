#!/usr/bin/env python3

"""
Python rewrite of the Unitree SLAM keyDemo.cpp example.

This script demonstrates controlling SLAM and navigation functions
on a Unitree robot using keyboard commands.
"""

import sys
import os
import tty
import termios
import threading
import json

from unitree_sdk2py.rpc.client import Client
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_ as RosString
from unitree_sdk2py.core.channel import ChannelFactory, ChannelSubscriber


# --- Constants ---
SLAM_INFO_TOPIC = "rt/slam_info"
SLAM_KEY_INFO_TOPIC = "rt/slam_key_info"
TEST_SERVICE_NAME = "slam_operate"
TEST_API_VERSION = "1.0.0.1"

# API IDs
ROBOT_API_ID_STOP_NODE = 1901
ROBOT_API_ID_START_MAPPING_PL = 1801
ROBOT_API_ID_END_MAPPING_PL = 1802
ROBOT_API_ID_START_RELOCATION_PL = 1804
ROBOT_API_ID_POSE_NAV_PL = 1102
ROBOT_API_ID_PAUSE_NAV = 1201
ROBOT_API_ID_RESUME_NAV = 1202

# ANSI color codes for printing
GREEN = '\033[1;32m'
YELLOW = '\033[33m'
RESET = '\033[0m'

class PoseData:
    """A simple class to hold pose information."""
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.q_x = 0.0
        self.q_y = 0.0
        self.q_z = 0.0
        self.q_w = 1.0
        self.mode = 1
        # self.speed = 0.5  # Corresponds to commented-out C++ code

    def to_json_str(self):
        """Serializes the pose data to a JSON string."""
        data = {
            "data": {
                "targetPose": {
                    "x": self.x,
                    "y": self.y,
                    "z": self.z,
                    "q_x": self.q_x,
                    "q_y": self.q_y,
                    "q_z": self.q_z,
                    "q_w": self.q_w
                },
                "mode": self.mode
                # "speed": self.speed
            }
        }
        return json.dumps(data, indent=4)

    def print_info(self):
        """Prints the current pose to the console."""
        print(f"Pose added: x={self.x}, y={self.y}, z={self.z}, "
              f"q_x={self.q_x}, q_y={self.q_y}, q_z={self.q_z}, q_w={self.q_w}")

def getch():
    """
    Gets a single character from standard input without requiring Enter.
    (Linux/macOS specific)
    """
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

class TestClient(Client):
    """
    Main client class for SLAM keyboard demonstration.
    Inherits from unitree_sdk2.client.Client.
    """
    def __init__(self):
        super().__init__(TEST_SERVICE_NAME, False)
        self.sub_slam_info = None
        self.sub_slam_key_info = None
        
        # State variables
        self.cur_pose = PoseData()
        self.pose_list = []
        self.is_arrived = False
        
        # Threading control
        self.control_thread = None
        self.stop_task_event = threading.Event()
        self.arrived_event = threading.Event()
        
        self._print_menu()

    def _print_menu(self):
        """Prints the welcome menu and key commands."""
        print("*********************** Unitree SLAM Demo (Python) ***********************")
        print("---------------            q    w                -----------------")
        print("---------------            a    s   d   f        -----------------")
        print("---------------            z    x                -----------------")
        print("------------------------------------------------------------------")
        print("------------------ q: Start mapping            -------------------")
        print("------------------ w: End mapping              -------------------")
        print("------------------ a: Start relocation         -------------------")
        print("------------------ s: Add pose to task list    -------------------")
        print("------------------ d: Execute task list        -------------------")
        print("------------------ f: Clear task list          -------------------")
        print("------------------ z: Pause navigation         -------------------")
        print("------------------ x: Resume navigation        -------------------")
        print("---------------- Press any other key to stop SLAM ----------------")
        print("------------------------------------------------------------------")
        print("------------------------------------------------------------------")
        print("--------------- Press 'Ctrl + C' to exit the program -------------")
        print("------------------------------------------------------------------")
        print("------------------------------------------------------------------\n")

    def Init(self):
        """Initializes the client, registers APIs, and sets up subscribers."""
        # self.SetApiVersion(TEST_API_VERSION)
        
        # Register all API IDs
        self._RegistApi(ROBOT_API_ID_POSE_NAV_PL, 0)
        self._RegistApi(ROBOT_API_ID_PAUSE_NAV, 0)
        self._RegistApi(ROBOT_API_ID_RESUME_NAV, 0)
        self._RegistApi(ROBOT_API_ID_STOP_NODE, 0)
        self._RegistApi(ROBOT_API_ID_START_MAPPING_PL, 0)
        self._RegistApi(ROBOT_API_ID_END_MAPPING_PL, 0)
        self._RegistApi(ROBOT_API_ID_START_RELOCATION_PL, 0)

        # Initialize subscribers
        self.sub_slam_info = ChannelSubscriber(SLAM_INFO_TOPIC, RosString)
        self.sub_slam_info.Init(self._slam_info_handler, 1)
        
        self.sub_slam_key_info = ChannelSubscriber(SLAM_KEY_INFO_TOPIC, RosString)
        self.sub_slam_key_info.Init(self._slam_key_info_handler, 1)

    def _slam_info_handler(self, msg: RosString):
        """Callback for the SLAM_INFO_TOPIC."""
        try:
            data = json.loads(msg.data)
            
            if data.get("errorCode", 0) != 0:
                print(f"{YELLOW}{data.get('info', 'Unknown error')}{RESET}")
                return

            if data.get("type") == "pos_info":
                pose = data.get("data", {}).get("currentPose", {})
                self.cur_pose.x = pose.get("x", 0.0)
                self.cur_pose.y = pose.get("y", 0.0)
                self.cur_pose.z = pose.get("z", 0.0)
                self.cur_pose.q_x = pose.get("q_x", 0.0)
                self.cur_pose.q_y = pose.get("q_y", 0.0)
                self.cur_pose.q_z = pose.get("q_z", 0.0)
                self.cur_pose.q_w = pose.get("q_w", 1.0)
        except json.JSONDecodeError:
            print(f"{YELLOW}Failed to parse slam_info JSON{RESET}")

    def _slam_key_info_handler(self, msg: RosString):
        """Callback for the SLAM_KEY_INFO_TOPIC."""
        try:
            data = json.loads(msg.data)
            
            if data.get("errorCode", 0) != 0:
                print(f"{YELLOW}{data.get('info', 'Unknown error')}{RESET}")
                return

            if data.get("type") == "task_result":
                task_data = data.get("data", {})
                self.is_arrived = task_data.get("is_arrived", False)
                target_name = task_data.get("targetNodeName", "unknown")
                
                if self.is_arrived:
                    print(f"I arrived at {target_name}")
                    # Signal the task loop thread that it has arrived
                    self.arrived_event.set()
                else:
                    print(f"I did not arrive at {target_name}. Please help me!! (T_T)")
                    # Also signal, so the loop can continue (or stop)
                    self.arrived_event.set() 
        except json.JSONDecodeError:
            print(f"{YELLOW}Failed to parse slam_key_info JSON{RESET}")

    def _call_api(self, api_id, parameter_json_str):
        """Helper function to call an API and print results."""
        print(f"Calling API: {api_id} with params: {parameter_json_str}")
        try:
            status, data = self.Call(api_id, parameter_json_str)
            print(f"StatusCode: {status}")
            print(f"Response Data: {data}")
        except Exception as e:
            print(f"{YELLOW}Error calling API {api_id}: {e}{RESET}")

    # --- API Functions ---

    def stop_node(self):
        print("Stopping SLAM node...")
        param = json.dumps({"data": {}})
        self._call_api(ROBOT_API_ID_STOP_NODE, param)

    def start_mapping(self):
        print("Starting mapping...")
        param = json.dumps({"data": {"slam_type": "indoor"}})
        self._call_api(ROBOT_API_ID_START_MAPPING_PL, param)

    def end_mapping(self):
        print("Ending mapping...")
        # Note: Ensure /home/unitree exists or change the path
        param = json.dumps({"data": {"address": "/home/unitree/test.pcd"}})
        self._call_api(ROBOT_API_ID_END_MAPPING_PL, param)

    def start_relocation(self):
        print("Starting relocation...")
        param_data = {
            "data": {
                "x": 0.0, "y": 0.0, "z": 0.0,
                "q_x": 0.0, "q_y": 0.0, "q_z": 0.0, "q_w": 1.0,
                "address": "/home/unitree/test.pcd"  # Note: Ensure this file exists
            }
        }
        self._call_api(ROBOT_API_ID_START_RELOCATION_PL, json.dumps(param_data))

    def pause_nav(self):
        print("Pausing navigation...")
        param = json.dumps({"data": {}})
        self._call_api(ROBOT_API_ID_PAUSE_NAV, param)

    def resume_nav(self):
        print("Resuming navigation...")
        param = json.dumps({"data": {}})
        self._call_api(ROBOT_API_ID_RESUME_NAV, param)

    # --- Task Loop Thread Functions ---

    def task_thread_run(self):
        """Stops any existing task thread and starts a new one."""
        self.task_thread_stop()  # Stop previous thread if running
        
        if not self.pose_list:
            print("Task list is empty. Nothing to execute.")
            return

        print(f"Starting task thread with {len(self.pose_list)} poses.")
        self.stop_task_event.clear()
        self.control_thread = threading.Thread(target=self._task_loop_fun, daemon=True)
        self.control_thread.start()

    def _task_loop_fun(self):
        """The main logic for the navigation task loop."""
        current_pose_list = list(self.pose_list)
        
        while not self.stop_task_event.is_set():
            # Loop through poses
            for i, pose in enumerate(current_pose_list):
                if self.stop_task_event.is_set():
                    break
                
                print(f"Navigating to pose {i+1}/{len(current_pose_list)}...")
                self.is_arrived = False
                self.arrived_event.clear()
                
                # Call the navigation API
                self._call_api(ROBOT_API_ID_POSE_NAV_PL, pose.to_json_str())
                
                # Wait for the arrived_event (set by _slam_key_info_handler)
                # or for the stop_task_event.
                while not self.stop_task_event.is_set():
                    if self.arrived_event.wait(timeout=0.1): # Wait for 100ms
                        break # Event was set (arrived or failed)
                
                if self.stop_task_event.is_set():
                    break # Exit inner loop if stop is requested

            # After one full loop, reverse the list and loop again
            if not self.stop_task_event.is_set():
                print("Task list complete. Reversing and repeating.")
                current_pose_list.reverse()
            
        print("Task loop stopped.")

    def task_thread_stop(self):
        """Signals the task thread to stop and waits for it to join."""
        if self.control_thread and self.control_thread.is_alive():
            print("Stopping task thread...")
            self.stop_task_event.set()
            self.arrived_event.set() # Wake up a waiting thread
            self.control_thread.join(timeout=2.0)
            if self.control_thread.is_alive():
                print(f"{YELLOW}Task thread did not stop gracefully.{RESET}")
        self.control_thread = None

    def run_key_execute(self):
        """The main loop for detecting and executing key presses."""
        while True:
            try:
                ch = getch()
                print(f"{GREEN}Key '{ch}' pressed.{RESET}")

                if ch == 'q':
                    self.start_mapping()
                elif ch == 'w':
                    self.end_mapping()
                elif ch == 'a':
                    self.start_relocation()
                elif ch == 's':
                    print("Adding current pose to task list.")
                    new_pose = PoseData()
                    new_pose.__dict__.update(self.cur_pose.__dict__) # Deep copy
                    self.pose_list.append(new_pose)
                    new_pose.print_info()
                elif ch == 'd':
                    self.task_thread_run()
                elif ch == 'f':
                    self.pose_list.clear()
                    print("Cleared task list.")
                elif ch == 'z':
                    self.pause_nav()
                elif ch == 'x':
                    self.resume_nav()
                else:
                    # Any other key stops SLAM and breaks the loop
                    print("Stopping node due to key press...")
                    self.task_thread_stop()
                    self.stop_node()
                    # Note: C++ version breaks, but Ctrl+C is cleaner
                    # break 
            
            except KeyboardInterrupt: # Handle Ctrl+C
                print("\nCtrl+C detected. Exiting...")
                break

# --- Main execution ---

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} networkInterface")
        print(f"Example: {sys.argv[0]} eno1")
        sys.exit(-1)
        
    # Initialize the Unitree SDK ChannelFactory
    ChannelFactory().Init(0, sys.argv[1])
    tc = TestClient()
    tc.Init()
    tc.SetTimeout(10.0)
    
    try:
        # Run the main key detection loop
        tc.run_key_execute()
        
    except Exception as e:
        print(f"{YELLOW}An unexpected error occurred: {e}{RESET}")
        
    finally:
        # **Crucial Cleanup**
        # This replaces the C++ destructor logic
        print("\nPerforming final cleanup...")
        tc.task_thread_stop()
        tc.stop_node()
        print("Cleanup complete. Exiting.")