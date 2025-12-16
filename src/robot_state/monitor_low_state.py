"""
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

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_

JOINT_NAMES = [
    "LeftHipPitch",
    "LeftHipRoll",
    "LeftHipYaw",
    "LeftKnee",
    "LeftAnklePitch",
    "LeftAnkleRoll",
    "RightHipPitch",
    "RightHipRoll",
    "RightHipYaw",
    "RightKnee",
    "RightAnklePitch",
    "RightAnkleRoll",
    "WaistYaw",
    "WaistRoll",
    "WaistPitch",
    "LeftShoulderPitch",
    "LeftShoulderRoll",
    "LeftShoulderYaw",
    "LeftElbow",
    "LeftWristRoll",
    "LeftWristPitch",
    "LeftWristYaw",
    "RightShoulderPitch",
    "RightShoulderRoll",
    "RightShoulderYaw",
    "RightElbow",
    "RightWristRoll",
    "RightWristPitch",
    "RightWristYaw",
]

class StateMonitor:
    def __init__(self, joint_id_to_monitor=None):
        self.low_state = None
        self.lock = threading.Lock()
        self.joint_id_to_monitor = joint_id_to_monitor
        
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self._low_state_handler, 10)

    def _low_state_handler(self, msg: LowState_):
        with self.lock:
            self.low_state = msg

    def print_dashboard(self):
        os.system('clear' if os.name == 'posix' else 'cls')
        
        if self.joint_id_to_monitor is not None:
            print(f"--- Unitree G1 LowState Monitor (Joint ID: {self.joint_id_to_monitor}) ---")
        else:
            print("--- Unitree G1 LowState Monitor (All Joints) ---")
        print(f"Press Ctrl+C to exit\n")

        with self.lock:
            if self.low_state is None:
                print("Waiting for first LowState message...")
                return

            print("="*75)
            print("== IMU State ==")
            imu = self.low_state.imu_state
            rpy = f"Roll: {imu.rpy[0]:.3f}, Pitch: {imu.rpy[1]:.3f}, Yaw: {imu.rpy[2]:.3f}"
            acc = f"Acc (x,y,z): {imu.accelerometer[0]:.3f}, {imu.accelerometer[1]:.3f}, {imu.accelerometer[2]:.3f}"
            gyr = f"Gyro (x,y,z): {imu.gyroscope[0]:.3f}, {imu.gyroscope[1]:.3f}, {imu.gyroscope[2]:.3f}"
            print(rpy)
            print(acc)
            print(gyr)
            
            print("\n" + "="*75)
            print("== Motor States ==")
            print(f"{'ID':<3} | {'Joint Name':<20} | {'Mode':<5} | {'Pos (q)':<10} | {'Vel (dq)':<10} | {'Torque (tau_est)':<15}")
            print("-"*75)

            if self.joint_id_to_monitor is not None:
                i = self.joint_id_to_monitor
                
                if i < 0 or i >= len(self.low_state.motor_state) or i >= len(JOINT_NAMES):
                    print(f"ERROR: Invalid Joint ID {i}. Must be between 0 and {len(JOINT_NAMES) - 1}.")
                    return

                motor = self.low_state.motor_state[i]
                name = JOINT_NAMES[i]
                mode = motor.mode
                q = motor.q
                dq = motor.dq
                tau = motor.tau_est
                
                print(f"{i:<3} | {name:<20} | {mode:<5} | {q:<10.3f} | {dq:<10.3f} | {tau:<15.3f}")

            else:
                for i, motor in enumerate(self.low_state.motor_state):
                    if i >= len(JOINT_NAMES):
                        break
                    
                    name = JOINT_NAMES[i]
                    mode = motor.mode
                    q = motor.q
                    dq = motor.dq
                    tau = motor.tau_est
                    
                    print(f"{i:<3} | {name:<20} | {mode:<5} | {q:<10.3f} | {dq:<10.3f} | {tau:<15.3f}")
        
        print("\n" + "="*75)
        print("Mode: 1 = Enabled, 0 = Disabled/Idle")


if __name__ == '__main__':
    
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <network_interface> [joint_id]")
        print(f"Example (all joints): {sys.argv[0]} ens33")
        print(f"Example (single joint): {sys.argv[0]} ens33 5")
        sys.exit(1)

    ChannelFactoryInitialize(0, sys.argv[1])
    
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
        while True:
            monitor.print_dashboard()
            time.sleep(0.1) 
    except KeyboardInterrupt:
        print("\nExiting monitor.")