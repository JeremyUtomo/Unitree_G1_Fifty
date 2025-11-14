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
# 1. IMPORT THE *TYPES* (NOT from default)
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
# 2. IMPORT THE `default` MODULE ITSELF
from unitree_sdk2py.idl import default

from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread

# --- Constants ---
G1_NUM_MOTOR = 29 # The LowCmd_ message has 30 motors (0-29)

# Kp and Kd values
Kp_Arm = 40.0
Kd_Arm = 1.0

# --- Joint IDs ---
LEFT_ARM_JOINT_IDS = [15, 16, 17, 18, 19, 20, 21]
RIGHT_ARM_JOINT_IDS = [22, 23, 24, 25, 26, 27, 28]

TARGET_POSITIONS_STOP_RIGHT = [0.246, -0.281, 0.089, 0.768, 0.000, 0.000, 0.000 ]

# --- STAGE 1 Target positions ---
TARGET_POSITIONS_1_RIGHT = [0.246, -0.281, 0.089, 0.768, 0.000, 0.000, 0.000]
TARGET_POSITIONS_1_LEFT =  [0.246,  0.281, -0.089, 0.768,  0.000, 0.000,  0.000]

# --- STAGE 2 Target positions ---
TARGET_POSITIONS_2_RIGHT = [-0.041, -0.072, -0.120, 0.418, 0.063, -0.646, 0.400]
TARGET_POSITIONS_2_LEFT =  [-0.041,  0.072,  0.120, 0.418, -0.063, -0.646, -0.400]

# --- STAGE 3 Target positions ---
# *** UPDATED POSITIONS ***
TARGET_POSITIONS_3_RIGHT = [0.041, -0.058, 0.478, 0.455, 0.070, -0.560, -0.453]
TARGET_POSITIONS_3_LEFT =  [0.041, 0.058,  -0.478, 0.455, -0.070, -0.560, 0.453]

# --- STAGE 4 Target positions ---
TARGET_POSITIONS_4_RIGHT = [-1.551, -0.075, -0.168, -0.067, 0.080, -0.156, -0.048]
TARGET_POSITIONS_4_LEFT  = [-1.551,  0.075,  0.168, -0.067, -0.080, -0.156,  0.048]

# --- STAGE 5 Target positions ---
TARGET_POSITIONS_5_RIGHT = [-2.410, -0.437, -0.085, -0.047, -0.094, -0.159, -0.061]
TARGET_POSITIONS_5_LEFT  = [-2.410,  0.437,  0.085, -0.047,  0.094, -0.159,  0.061]

# --- STAGE 6 Target positions ---
TARGET_POSITIONS_6_RIGHT = [-2.751, -0.491, 0.109, -0.593, -0.094, -0.161, -0.085]
TARGET_POSITIONS_6_LEFT  = [-2.751,  0.491, -0.109, -0.593,  0.094, -0.161,  0.085]

# --- STAGE 7 Target positions ---
TARGET_POSITIONS_7_RIGHT = [-2.751, -0.491, -0.563, -0.593, -0.094, -0.161, -0.085]
TARGET_POSITIONS_7_LEFT  = [-2.751,  0.491, 0.563, -0.593,  0.094, -0.161,  0.085]

class ArmMover:
    # --- CHANGED: control_dt_ is now 0.02 (50Hz) to match examples ---
    def __init__(self, control_dt=0.02, move_duration=3.0):
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
        self.is_stopped_event = threading.Event() # --- NEW: To signal exit ---
        
        # State machine for movement
        # 1 = Moving to STAGE 1
        # 2 = Moving to STAGE 2
        # 3 = Moving to STAGE 3
        # 4 = Moving to STAGE 4
        # 5 = Moving to STAGE 5
        # 6 = Moving to STAGE 6
        # 7 = Moving to STAGE 7
        # 8 = Returning to STAGE 1 (before stop)
        # 9 = Releasing control
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
        
        # Publish to the 'rt/arm_sdk' topic
        self.lowcmd_publisher_ = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.lowcmd_publisher_.Init()

    def Start(self):
        print("Waiting for the first LowState message to get current arm positions...")
        if not self.first_state_received.wait(timeout=5.0):
            print("Error: Did not receive LowState message.")
            return

        print(f"Got current arm states. Starting {1.0/self.control_dt_}Hz control loop to move to Stage 1.")
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

    def MoveToStage4(self):
        print("Moving to Stage 4...")
        if not self._update_start_positions():
            return
        self.start_time = time.time()
        self.move_stage = 4
        self.position_reached_event.clear()

    def MoveToStage5(self):
        print("Moving to Stage 5...")
        if not self._update_start_positions():
            return
        self.start_time = time.time()
        self.move_stage = 5
        self.position_reached_event.clear()

    def MoveToStage6(self):
        print("Moving to Stage 6...")
        if not self._update_start_positions():
            return
        self.start_time = time.time()
        self.move_stage = 6
        self.position_reached_event.clear()

    def MoveToStage7(self):
        print("Moving to Stage 7...")
        if not self._update_start_positions():
            return
        self.start_time = time.time()
        self.move_stage = 7
        self.position_reached_event.clear()

    # --- NEW: Function to handle graceful stop ---
    def Stop(self):
        """
        First returns to Stage 1 position, then begins the fade-out process to release control.
        """
        print("Returning to Stage 1 position before releasing control...")
        if not self._update_start_positions():
            return
        self.start_time = time.time()
        self.move_duration_ = 3 # Set duration for returning to Stage 1
        self.move_stage = 8 # Enter the "return to Stage 1" stage
        self.position_reached_event.clear() # Re-use this event

    def _control_loop(self):
        """
        The high-frequency control loop.
        Based on the new working examples.
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

        # 1. Set all motors to mode 0 (Idle) by default.
        for i in range(G1_NUM_MOTOR):
            self.low_cmd.motor_cmd[i].mode = 0
            self.low_cmd.motor_cmd[i].q = 0.0
            self.low_cmd.motor_cmd[i].dq = 0.0
            self.low_cmd.motor_cmd[i].kp = 0.0
            self.low_cmd.motor_cmd[i].kd = 0.0
            self.low_cmd.motor_cmd[i].tau = 0.0
        
        # 2. Set mode_pr (pitch/roll). 0 is the default and correct.
        self.low_cmd.mode_pr = 0
        
        # --- Handle logic based on the current stage ---
        
        if self.move_stage < 8:
            # --- STAGES 1, 2, 3, 4, 5, 6: Moving arms ---
            
            # Set the "weight" of our command (Index 29)
            self.low_cmd.motor_cmd[29].q = 1.0 
            
            # Select target positions based on stage
            if self.move_stage == 1:
                target_left = TARGET_POSITIONS_1_LEFT
                target_right = TARGET_POSITIONS_1_RIGHT
            elif self.move_stage == 2:
                target_left = TARGET_POSITIONS_2_LEFT
                target_right = TARGET_POSITIONS_2_RIGHT
            elif self.move_stage == 3:
                target_left = TARGET_POSITIONS_3_LEFT
                target_right = TARGET_POSITIONS_3_RIGHT
            elif self.move_stage == 4:
                target_left = TARGET_POSITIONS_4_LEFT
                target_right = TARGET_POSITIONS_4_RIGHT
            elif self.move_stage == 5:
                target_left = TARGET_POSITIONS_5_LEFT
                target_right = TARGET_POSITIONS_5_RIGHT
            elif self.move_stage == 6: # self.move_stage == 6
                target_left = TARGET_POSITIONS_6_LEFT
                target_right = TARGET_POSITIONS_6_RIGHT
            else:
                target_left = TARGET_POSITIONS_7_LEFT
                target_right = TARGET_POSITIONS_7_RIGHT

            # Set commands for the LEFT Arm joints (15-21)
            for i, joint_id in enumerate(LEFT_ARM_JOINT_IDS):
                start_q = start_pos_left[i]
                target_q = target_left[i]
                interp_q = start_q + (target_q - start_q) * ratio
                
                motor_cmd = self.low_cmd.motor_cmd[joint_id]
                # --- CRITICAL: DO NOT SET MODE = 1 ---
                motor_cmd.q = interp_q
                motor_cmd.dq = 0.0
                motor_cmd.kp = Kp_Arm
                motor_cmd.kd = Kd_Arm
                motor_cmd.tau = 0.0
            
            # Set commands for the RIGHT Arm joints (22-28)
            for i, joint_id in enumerate(RIGHT_ARM_JOINT_IDS):
                start_q = start_pos_right[i]
                target_q = target_right[i]
                interp_q = start_q + (target_q - start_q) * ratio
                
                motor_cmd = self.low_cmd.motor_cmd[joint_id]
                # --- CRITICAL: DO NOT SET MODE = 1 ---
                motor_cmd.q = interp_q
                motor_cmd.dq = 0.0
                motor_cmd.kp = Kp_Arm
                motor_cmd.kd = Kd_Arm
                motor_cmd.tau = 0.0

        elif self.move_stage == 8:
            # --- STAGE 8: Returning to Stage 1 position ---
            # Set the "weight" of our command (Index 29)
            self.low_cmd.motor_cmd[29].q = 1.0
            
            # Target is Stage 1 position
            target_left = TARGET_POSITIONS_1_LEFT
            target_right = TARGET_POSITIONS_1_RIGHT
            
            # Set commands for the LEFT Arm joints (15-21)
            for i, joint_id in enumerate(LEFT_ARM_JOINT_IDS):
                start_q = start_pos_left[i]
                target_q = target_left[i]
                interp_q = start_q + (target_q - start_q) * ratio
                
                motor_cmd = self.low_cmd.motor_cmd[joint_id]
                motor_cmd.q = interp_q
                motor_cmd.dq = 0.0
                motor_cmd.kp = Kp_Arm
                motor_cmd.kd = Kd_Arm
                motor_cmd.tau = 0.0
            
            # Set commands for the RIGHT Arm joints (22-28)
            for i, joint_id in enumerate(RIGHT_ARM_JOINT_IDS):
                start_q = start_pos_right[i]
                target_q = target_right[i]
                interp_q = start_q + (target_q - start_q) * ratio
                
                motor_cmd = self.low_cmd.motor_cmd[joint_id]
                motor_cmd.q = interp_q
                motor_cmd.dq = 0.0
                motor_cmd.kp = Kp_Arm
                motor_cmd.kd = Kd_Arm
                motor_cmd.tau = 0.0
        
        elif self.move_stage == 9:
            # --- STAGE 8: Releasing control ---
            # Fade weight from 1.0 down to 0.0 while maintaining Stage 1 position
            self.low_cmd.motor_cmd[29].q = 1.0 - ratio
            
            # Hold Stage 1 position during fade-out
            target_left = TARGET_POSITIONS_1_LEFT
            target_right = TARGET_POSITIONS_1_RIGHT
            
            # Set commands for the LEFT Arm joints (15-21)
            for i, joint_id in enumerate(LEFT_ARM_JOINT_IDS):
                motor_cmd = self.low_cmd.motor_cmd[joint_id]
                motor_cmd.q = target_left[i]  # Hold at Stage 1 position
                motor_cmd.dq = 0.0
                motor_cmd.kp = Kp_Arm
                motor_cmd.kd = Kd_Arm
                motor_cmd.tau = 0.0
            
            # Set commands for the RIGHT Arm joints (22-28)
            for i, joint_id in enumerate(RIGHT_ARM_JOINT_IDS):
                motor_cmd = self.low_cmd.motor_cmd[joint_id]
                motor_cmd.q = target_right[i]  # Hold at Stage 1 position
                motor_cmd.dq = 0.0
                motor_cmd.kp = Kp_Arm
                motor_cmd.kd = Kd_Arm
                motor_cmd.tau = 0.0
        
        # Finalize and send the command
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_publisher_.Write(self.low_cmd)
        
        # Print status occasionally
        self.loop_counter += 1
        if self.loop_counter % 25 == 0: # Print every 0.5 sec
            if ratio < 1.0:
                if self.move_stage == 8:
                    print(f"Returning to Stage 1... {int(ratio * 100)}% complete.")
                elif self.move_stage == 9:
                    print(f"Releasing control... {int((1.0 - ratio) * 100)}% weight remaining.")
                else:
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
                elif self.move_stage == 4:
                    print(f"Target position for Stage 4 reached. Holding.")
                    self.position_reached_event.set()
                elif self.move_stage == 5:
                    print(f"Target position for Stage 5 reached. Holding.")
                    self.position_reached_event.set()
                elif self.move_stage == 6:
                    print(f"Target position for Stage 6 reached. Holding.")
                    self.position_reached_event.set()
                elif self.move_stage == 7:
                    print(f"Target position for Stage 7 reached. Holding.")
                    self.position_reached_event.set()
                elif self.move_stage == 8:
                    # Returned to Stage 1, now fade out control
                    print("Returned to Stage 1. Now releasing arm control...")
                    # Update start positions to current (Stage 1) positions before fading
                    with self.lock:
                        if self.low_state is not None:
                            for i, joint_id in enumerate(LEFT_ARM_JOINT_IDS):
                                self.start_positions_left[i] = self.low_state.motor_state[joint_id].q
                            for i, joint_id in enumerate(RIGHT_ARM_JOINT_IDS):
                                self.start_positions_right[i] = self.low_state.motor_state[joint_id].q
                    self.start_time = time.time()
                    self.move_duration_ = 1.0  # 2 seconds to fade out
                    self.move_stage = 9  # Move to release stage
                    self.position_reached_event.clear()
                elif self.move_stage == 9:
                    # Final stop
                    print("Arm control released. Stopping thread.")
                    self.position_reached_event.set()
                    self.is_stopped_event.set() # Signal main thread
                    self.control_thread.Stop() # Stop this recurrent thread

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
        
        # 7. Ask user to continue to Stage 4
        print("\n" + "="*80)
        input("Stage 3 reached. Press Enter to move to Stage 4...")
        print("="*80)
        
        # 8. Trigger Stage 4
        mover.MoveToStage4()
        
        # 9. Wait until Stage 4 position is reached
        mover.position_reached_event.wait()
        
        # 10. Ask user to continue to Stage 5
        print("Stage 4 reached. Moving automatically to Stage 5...")
        
        # 11. Trigger Stage 5
        mover.MoveToStage5()
        
        # 12. Wait until Stage 5 position is reached
        mover.position_reached_event.wait()
        
        # 13. Ask user to continue to Stage 6
        print("Stage 5 reached. Moving automatically to Stage 6...")
        
        # 14. Trigger Stage 6
        mover.MoveToStage6()
        
        # 15. Wait until Stage 6 position is reached
        mover.position_reached_event.wait()

        print("Stage 6 reached. Moving automatically to Stage 7...")
        
        # 14. Trigger Stage 7
        mover.MoveToStage7()
        
        # 15. Wait until Stage 7 position is reached
        mover.position_reached_event.wait()
        
        print("\nStage 7 reached. Holding position. Press Ctrl+C to stop.")
        
        # Keep the main thread alive while the control thread holds
        while True:
            time.sleep(10)
            
    except KeyboardInterrupt:
        # --- UPDATED: Handle graceful exit ---
        print("\nStopping control loop... Fading out arm control.")
        mover.Stop()
        
        # Wait for the fade-out to complete
        mover.is_stopped_event.wait(timeout=10.0)
        
        print("Script terminated. Robot is back in walking mode.")