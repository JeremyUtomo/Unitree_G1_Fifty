#!/usr/bin/env python3

"""
This script moves both arms in a 3-stage sequence WHILE the
robot's high-level balancing controller is active.

It does this by ONLY enabling and commanding the arm motors (15-28)
and leaving the leg/waist motors (0-14) in their default mode (mode=0)
so the balancing controller is not interrupted.

*** *** CRITICAL SAFETY WARNING ***
THIS SCRIPT IS ADVANCED. Moving the arms changes the robot's
center of mass. If the movement is too fast, the robot's
balancing controller may fail, and the robot WILL FALL.

BE PREPARED FOR AN EMERGENCY STOP.
***
"""

import sys
import time
import threading
import numpy as np

# --- Unitree SDK Imports ---
from unitree_sdk2py.core.channel import (
    ChannelSubscriber,
    ChannelPublisher,
    ChannelFactoryInitialize,
)
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.idl import default
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread

# --- Constants ---
G1_NUM_MOTOR = 29

# Kp and Kd values (only used for arm motors now)
Kp = [
    60, 60, 60, 100, 40, 40,   # legs
    60, 60, 60, 100, 40, 40,   # legs
    60, 40, 40,               # waist
    40, 40, 40, 40, 40, 40, 40, # arms
    40, 40, 40, 40, 40, 40, 40  # arms
]

Kd = [
    1, 1, 1, 2, 1, 1,   # legs
    1, 1, 1, 2, 1, 1,   # legs
    1, 1, 1,            # waist
    1, 1, 1, 1, 1, 1, 1, # arms
    1, 1, 1, 1, 1, 1, 1  # arms
]

# --- Joint IDs ---
LEFT_ARM_JOINT_IDS = [15, 16, 17, 18, 19, 20, 21]
RIGHT_ARM_JOINT_IDS = [22, 23, 24, 25, 26, 27, 28]

# --- STAGE 1 Target positions ---
TARGET_POSITIONS_1_RIGHT = [0.251, -0.277, 0.089, 0.791, -0.013, 0.003, -0.004]
TARGET_POSITIONS_1_LEFT =  [0.251,  0.277, -0.089, 0.791,  0.013, 0.003,  0.004]

# --- STAGE 2 Target positions ---
TARGET_POSITIONS_2_RIGHT = [-0.041, -0.072, -0.120, 0.418, 0.063, -0.646, 0.400]
TARGET_POSITIONS_2_LEFT =  [-0.041,  0.072,  0.120, 0.418, -0.063, -0.646, -0.400]

# --- STAGE 3 Target positions ---
# *** UPDATED POSITIONS ***
TARGET_POSITIONS_3_RIGHT = [
    0.041,  # 22: RightShoulderPitch
    -0.058, # 23: RightShoulderRoll
    0.478,  # 24: RightShoulderYaw
    0.455,  # 25: RightElbow
    0.070,  # 26: RightWristRoll
    -0.560, # 27: RightWristPitch
    -0.453  # 28: RightWristYaw
]
TARGET_POSITIONS_3_LEFT =  [
    0.041,  # 15: LeftShoulderPitch
    0.058,  # 16: LeftShoulderRoll
    -0.478, # 17: LeftShoulderYaw
    0.455,  # 18: LeftElbow
    -0.070, # 19: LeftWristRoll
    -0.560, # 20: LeftWristPitch
    0.453   # 21: LeftWristYaw
]


class ArmMover:
    def __init__(self, control_dt=0.002, move_duration=3.0):
        self.control_dt_ = control_dt
        self.move_duration_ = move_duration
        
        self.low_state = None
        self.low_cmd = default.unitree_hg_msg_dds__LowCmd_()
        self.crc = CRC()
        self.control_thread = None
        self.start_time = 0.0
        self.loop_counter = 0

        self.start_positions_left = [0.0] * len(LEFT_ARM_JOINT_IDS)
        self.start_positions_right = [0.0] * len(RIGHT_ARM_JOINT_IDS)
        
        self.first_state_received = threading.Event()
        self.position_reached_event = threading.Event()
        
        self.move_stage = 1
        self.lock = threading.Lock()

    def _low_state_handler(self, msg: LowState_):
        with self.lock:
            self.low_state = msg
            if not self.first_state_received.is_set():
                for i, joint_id in enumerate(LEFT_ARM_JOINT_IDS):
                    self.start_positions_left[i] = self.low_state.motor_state[joint_id].q
                for i, joint_id in enumerate(RIGHT_ARM_JOINT_IDS):
                    self.start_positions_right[i] = self.low_state.motor_state[joint_id].q
                self.first_state_received.set()

    def Init(self):
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self._low_state_handler, 10)
        self.lowcmd_publisher_ = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher_.Init()

    def Start(self):
        print("Waiting for the first LowState message to get current arm positions...")
        if not self.first_state_received.wait(timeout=5.0):
            print("Error: Did not receive LowState message.")
            print("Please ensure the robot is powered on and publishing 'rt/lowstate'.")
            return

        print("Got current arm states. Starting 500Hz control loop to move to Stage 1.")
        self.start_time = time.time()
        
        self.control_thread = RecurrentThread(
            interval=self.control_dt_, 
            target=self._control_loop, 
            name="arm_control"
        )
        self.control_thread.Start()

    def _update_start_positions(self):
        with self.lock:
            if self.low_state is None:
                print("Error: Lost LowState, cannot start next move.")
                return False
            for i, joint_id in enumerate(LEFT_ARM_JOINT_IDS):
                self.start_positions_left[i] = self.low_state.motor_state[joint_id].q
            for i, joint_id in enumerate(RIGHT_ARM_JOINT_IDS):
                self.start_positions_right[i] = self.low_state.motor_state[joint_id].q
        return True

    def MoveToStage2(self):
        print("Moving to Stage 2...")
        if not self._update_start_positions():
            return
        self.start_time = time.time()
        self.move_stage = 2
        self.position_reached_event.clear()

    def MoveToStage3(self):
        print("Moving to Stage 3...")
        if not self._update_start_positions():
            return
        self.start_time = time.time()
        self.move_stage = 3
        self.position_reached_event.clear()

    def _control_loop(self):
        """
        The high-frequency (500Hz) control loop.
        """
        with self.lock:
            if self.low_state is None:
                return
            start_pos_left = self.start_positions_left[:]
            start_pos_right = self.start_positions_right[:]
            # We MUST copy the machine mode from the state
            self.low_cmd.mode_machine = self.low_state.mode_machine

        current_time = time.time() - self.start_time
        ratio = min(current_time / self.move_duration_, 1.0)

        # --- THIS IS THE CRITICAL SAFETY CHANGE ---
        # Set default command for ALL motors
        self.low_cmd.mode_pr = 0 # Use PR (Pitch/Roll) mode
        
        for i in range(G1_NUM_MOTOR):
            motor_cmd = self.low_cmd.motor_cmd[i]
            
            # CHECK IF MOTOR IS AN ARM MOTOR
            if i in LEFT_ARM_JOINT_IDS or i in RIGHT_ARM_JOINT_IDS:
                # This is an arm motor, command it.
                motor_cmd.mode = 1  # 1 = Enable
                motor_cmd.dq = 0.0
                motor_cmd.kp = Kp[i]
                motor_cmd.kd = Kd[i]
                motor_cmd.tau = 0.0
            else:
                # THIS IS A LEG/WAIST MOTOR
                # SET MODE 0 TO "RELEASE" CONTROL to high-level controller
                motor_cmd.mode = 0  # 0 = Disable (Idle)
                # Zero out all commands just in case
                motor_cmd.q = 0.0
                motor_cmd.dq = 0.0
                motor_cmd.kp = 0.0
                motor_cmd.kd = 0.0
                motor_cmd.tau = 0.0
        # --- END OF SAFETY CHANGE ---


        # Select target positions based on stage
        if self.move_stage == 1:
            target_left = TARGET_POSITIONS_1_LEFT
            target_right = TARGET_POSITIONS_1_RIGHT
        elif self.move_stage == 2:
            target_left = TARGET_POSITIONS_2_LEFT
            target_right = TARGET_POSITIONS_2_RIGHT
        else: # self.move_stage == 3
            target_left = TARGET_POSITIONS_3_LEFT
            target_right = TARGET_POSITIONS_3_RIGHT

        # --- OVERRIDE commands for the LEFT Arm joints ---
        for i, joint_id in enumerate(LEFT_ARM_JOINT_IDS):
            start_q = start_pos_left[i]
            target_q = target_left[i]
            interp_q = start_q + (target_q - start_q) * ratio
            self.low_cmd.motor_cmd[joint_id].q = interp_q
        
        # --- OVERRIDE commands for the RIGHT Arm joints ---
        for i, joint_id in enumerate(RIGHT_ARM_JOINT_IDS):
            start_q = start_pos_right[i]
            target_q = target_right[i]
            interp_q = start_q + (target_q - start_q) * ratio
            self.low_cmd.motor_cmd[joint_id].q = interp_q
        
        # Finalize and send the command
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_publisher_.Write(self.low_cmd)
        
        # Print status occasionally
        self.loop_counter += 1
        if self.loop_counter % 500 == 0: # Print once per second
            if ratio < 1.0:
                print(f"Moving to position (Stage {self.move_stage})... {int(ratio * 100)}% complete.")
        
        # Handle stage transitions
        if ratio == 1.0:
            if not self.position_reached_event.is_set():
                if self.move_stage == 1:
                    print(f"Target position for Stage 1 reached. Holding.")
                    self.position_reached_event.set()
                elif self.move_stage == 2:
                    print(f"Target position for Stage 2 reached.")
                    self.position_reached_event.set()
                elif self.move_stage == 3:
                    print(f"Target position for Stage 3 reached. Holding.")
                    self.position_reached_event.set()

if __name__ == '__main__':
    
    print("="*80)
    print("⚠️  SAFETY WARNING - BALANCING MODE SCRIPT ⚠️")
    print("1. This script is designed to run WITH the high-level balancing controller.")
    print("2. Moving the arms will affect balance. A FALL IS STILL POSSIBLE.")
    print("3. Start with a gantry first. Only try on the ground if you are confident.")
    print("4. Be ready to press Ctrl+C or use an E-Stop.")
    print("="*80)
    
    try:
        input("Press Enter to begin moving to Stage 1...")
    except KeyboardInterrupt:
        print("\nAborted.")
        sys.exit(0)

    # Initialize the Channel Factory
    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        print(f"Usage: {sys.argv[0]} <network_interface>")
        print(f"Example: {sys.argv[0]} ens33")
        ChannelFactoryInitialize(0) # Fallback

    mover = ArmMover()
    mover.Init()
    mover.Start()

    try:
        # 1. Wait until Stage 1 position is reached
        mover.position_reached_event.wait()
        
        # 2. Ask user to continue to Stage 2
        print("\n" + "="*80)
        input("Stage 1 reached. Press Enter to move to new position (Stage 2)...")
        print("="*80)
        
        # 3. Trigger Stage 2
        mover.MoveToStage2()
        
        # 4. Wait until Stage 2 position is reached
        mover.position_reached_event.wait()
        
        # 5. IMMEDIATELY Trigger Stage 3 (as requested)
        print("Stage 2 reached. Moving automatically to Stage 3...")
        mover.MoveToStage3()

        # 6. Wait until Stage 3 position is reached
        mover.position_reached_event.wait()
        
        print("\nStage 3 reached. Holding position. Press Ctrl+C to stop.")
        
        while True:
            time.sleep(10)
            
    except KeyboardInterrupt:
        print("\nStopping control loop...")
        print("Script terminated.")