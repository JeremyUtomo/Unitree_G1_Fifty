#!/usr/bin/env python3

"""
This script monitors the 'rt/lowstate' topic from the Unitree G1 robot
and displays the status of all joint motors and the IMU in a clean,
refreshing terminal interface.

If a second argument (a joint ID) is provided, it will only monitor
that specific joint.

Usage:
    python3 monitor_low_state.py <network_interface>
    python3 monitor_low_state.py <network_interface> <joint_id>

Example:
    python3 monitor_low_state.py ens33
    python3 monitor_low_state.py ens33 5
"""

import sys
import os
import time
import threading

# --- Unitree SDK Imports ---
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_ # Failsafe import

# --- List of all 29 G1 motor names in their correct index order ---
JOINT_NAMES = [
    "LeftHipPitch",       # 0
    "LeftHipRoll",        # 1
    "LeftHipYaw",         # 2
    "LeftKnee",           # 3
    "LeftAnklePitch",     # 4
    "LeftAnkleRoll",      # 5
    "RightHipPitch",      # 6
    "RightHipRoll",       # 7
    "RightHipYaw",        # 8
    "RightKnee",          # 9
    "RightAnklePitch",    # 10
    "RightAnkleRoll",     # 11
    "WaistYaw",           # 12
    "WaistRoll",          # 13 (INVALID on some models)
    "WaistPitch",         # 14 (INVALID on some models)
    "LeftShoulderPitch",  # 15
    "LeftShoulderRoll",   # 16
    "LeftShoulderYaw",    # 17
    "LeftElbow",          # 18
    "LeftWristRoll",      # 19
    "LeftWristPitch",     # 20 (INVALID on 23dof)
    "LeftWristYaw",       # 21 (INVALID on 23dof)
    "RightShoulderPitch", # 22
    "RightShoulderRoll",  # 23
    "RightShoulderYaw",   # 24
    "RightElbow",         # 25
    "RightWristRoll",     # 26
    "RightWristPitch",    # 27 (INVALID on 23dof)
    "RightWristYaw",      # 28 (INVALID on 23dof)
]

class StateMonitor:
    """
    Subscribes to LowState and provides a method to print a clean dashboard.
    Can be initialized to monitor all joints or a single joint ID.
    """
    def __init__(self, joint_id_to_monitor=None):
        self.low_state = None
        self.lock = threading.Lock()
        self.joint_id_to_monitor = joint_id_to_monitor
        
        # Create subscriber
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self._low_state_handler, 10)

    def _low_state_handler(self, msg: LowState_):
        """
        Callback for the LowState subscriber.
        Uses a lock to prevent race conditions with the print thread.
        """
        with self.lock:
            self.low_state = msg

    def print_dashboard(self):
        """
        Clears the terminal and prints the latest state data.
        """
        # Clear the terminal
        os.system('clear' if os.name == 'posix' else 'cls')
        
        # --- Print Header ---
        if self.joint_id_to_monitor is not None:
            print(f"--- Unitree G1 LowState Monitor (Joint ID: {self.joint_id_to_monitor}) ---")
        else:
            print("--- Unitree G1 LowState Monitor (All Joints) ---")
        print(f"Press Ctrl+C to exit\n")

        with self.lock:
            if self.low_state is None:
                print("Waiting for first LowState message...")
                return

            # --- Always Print IMU Data ---
            print("="*75)
            print("== IMU State ==")
            imu = self.low_state.imu_state
            # Formatting to 3 decimal places
            rpy = f"Roll: {imu.rpy[0]:.3f}, Pitch: {imu.rpy[1]:.3f}, Yaw: {imu.rpy[2]:.3f}"
            acc = f"Acc (x,y,z): {imu.accelerometer[0]:.3f}, {imu.accelerometer[1]:.3f}, {imu.accelerometer[2]:.3f}"
            gyr = f"Gyro (x,y,z): {imu.gyroscope[0]:.3f}, {imu.gyroscope[1]:.3f}, {imu.gyroscope[2]:.3f}"
            print(rpy)
            print(acc)
            print(gyr)
            
            # --- Print Motor States ---
            print("\n" + "="*75)
            print("== Motor States ==")
            # Print table header
            print(f"{'ID':<3} | {'Joint Name':<20} | {'Mode':<5} | {'Pos (q)':<10} | {'Vel (dq)':<10} | {'Torque (tau_est)':<15}")
            print("-"*75)

            if self.joint_id_to_monitor is not None:
                # --- Single Joint Mode ---
                i = self.joint_id_to_monitor
                
                # Check for valid ID
                if i < 0 or i >= len(self.low_state.motor_state) or i >= len(JOINT_NAMES):
                    print(f"ERROR: Invalid Joint ID {i}. Must be between 0 and {len(JOINT_NAMES) - 1}.")
                    return

                motor = self.low_state.motor_state[i]
                name = JOINT_NAMES[i]
                mode = motor.mode
                q = motor.q
                dq = motor.dq
                tau = motor.tau_est # Estimated torque
                
                # Print the formatted row
                print(f"{i:<3} | {name:<20} | {mode:<5} | {q:<10.3f} | {dq:<10.3f} | {tau:<15.3f}")

            else:
                # --- All Joints Mode (Original logic) ---
                for i, motor in enumerate(self.low_state.motor_state):
                    # Stop if the state list is longer than our names list
                    if i >= len(JOINT_NAMES):
                        break
                    
                    name = JOINT_NAMES[i]
                    mode = motor.mode
                    q = motor.q
                    dq = motor.dq
                    tau = motor.tau_est
                    
                    # Print the formatted row
                    print(f"{i:<3} | {name:<20} | {mode:<5} | {q:<10.3f} | {dq:<10.3f} | {tau:<15.3f}")
        
        # Print footer
        print("\n" + "="*75)
        print("Mode: 1 = Enabled, 0 = Disabled/Idle")


if __name__ == '__main__':
    
    # Check for network interface argument
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <network_interface> [joint_id]")
        print(f"Example (all joints): {sys.argv[0]} ens33")
        print(f"Example (single joint): {sys.argv[0]} ens33 5")
        sys.exit(1)

    # Initialize the Channel Factory
    ChannelFactoryInitialize(0, sys.argv[1])
    
    # Check for optional joint_id argument
    joint_id_to_monitor = None
    if len(sys.argv) > 2:
        try:
            joint_id_to_monitor = int(sys.argv[2])
            print(f"Initializing state monitor for joint ID: {joint_id_to_monitor}")
        except ValueError:
            print(f"Invalid joint ID '{sys.argv[2]}'. Monitoring all joints.")
    
    if joint_id_to_monitor is None:
        print("Initializing state monitor for all joints...")
        
    monitor = StateMonitor(joint_id_to_monitor)
    
    try:
        # Run the dashboard at 10 Hz (refresh rate of 0.1s)
        while True:
            monitor.print_dashboard()
            time.sleep(0.1) 
    except KeyboardInterrupt:
        print("\nExiting monitor.")