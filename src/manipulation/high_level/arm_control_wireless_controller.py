import sys
import time
import threading
import numpy as np
import struct
from enum import Enum

# --- Unitree SDK Imports ---
from unitree_sdk2py.core.channel import (
    ChannelSubscriber,
    ChannelPublisher,
    ChannelFactoryInitialize,
)
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.idl import default
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread

# --- Constants ---
G1_NUM_MOTOR = 29
Kp_Arm = 40.0
Kd_Arm = 1.0

# --- Joint IDs ---
LEFT_ARM_JOINT_IDS = [15, 16, 17, 18, 19, 20, 21]
RIGHT_ARM_JOINT_IDS = [22, 23, 24, 25, 26, 27, 28]

TARGET_POSITIONS_1_RIGHT = [0.246, -0.281, 0.089, 0.768, 0.000, 0.000, 0.000]
TARGET_POSITIONS_1_LEFT =  [0.246,  0.281, -0.089, 0.768,  0.000, 0.000,  0.000]

TARGET_POSITIONS_2_RIGHT = [-0.041, -0.072, -0.120, 0.418, 0.063, -0.646, 0.400]
TARGET_POSITIONS_2_LEFT =  [-0.041,  0.072,  0.120, 0.418, -0.063, -0.646, -0.400]

TARGET_POSITIONS_3_RIGHT = [0.041, -0.058, 0.478, 0.455, 0.070, -0.560, -0.453]
TARGET_POSITIONS_3_LEFT =  [0.041,  0.058, -0.478, 0.455, -0.070, -0.560,  0.453]

TARGET_POSITIONS_4_RIGHT = [-1.551, -0.075, -0.168, -0.067, 0.080, -0.156, -0.048]
TARGET_POSITIONS_4_LEFT  = [-1.551,  0.075,  0.168, -0.067, -0.080, -0.156,  0.048]

TARGET_POSITIONS_5_RIGHT = [-2.410, -0.437, -0.085, -0.047, -0.094, -0.159, -0.061]
TARGET_POSITIONS_5_LEFT  = [-2.410,  0.437,  0.085, -0.047,  0.094, -0.159,  0.061]

TARGET_POSITIONS_6_RIGHT = [-2.751, -0.491, 0.109, -0.593, -0.094, -0.161, -0.085]
TARGET_POSITIONS_6_LEFT  = [-2.751,  0.491, -0.109, -0.593,  0.094, -0.161,  0.085]

TARGET_POSITIONS_7_RIGHT = [-2.751, -0.491, -0.563, -0.593, -0.094, -0.161, -0.085]
TARGET_POSITIONS_7_LEFT  = [-2.751,  0.491,  0.563, -0.593,  0.094, -0.161,  0.085]


class MoveState(Enum):
    STAGE_1 = 1
    STAGE_2 = 2
    STAGE_3 = 3
    STAGE_4 = 4
    STAGE_5 = 5
    STAGE_6 = 6
    STAGE_7 = 7
    RETURNING = 8
    RELEASING = 9


class ArmMover:
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
        self.is_stopped_event = threading.Event()

        self.move_stage = MoveState.STAGE_1
        self.lock = threading.Lock()

        self.TARGET_STAGES = [
            (TARGET_POSITIONS_1_LEFT, TARGET_POSITIONS_1_RIGHT),
            (TARGET_POSITIONS_2_LEFT, TARGET_POSITIONS_2_RIGHT),
            (TARGET_POSITIONS_3_LEFT, TARGET_POSITIONS_3_RIGHT),
            (TARGET_POSITIONS_4_LEFT, TARGET_POSITIONS_4_RIGHT),
            (TARGET_POSITIONS_5_LEFT, TARGET_POSITIONS_5_RIGHT),
            (TARGET_POSITIONS_6_LEFT, TARGET_POSITIONS_6_RIGHT),
            (TARGET_POSITIONS_7_LEFT, TARGET_POSITIONS_7_RIGHT),
        ]

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
        self.lowcmd_publisher_ = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.lowcmd_publisher_.Init()

    def Start(self):
        print("Waiting for the first LowState message to get current arm positions...")
        if not self.first_state_received.wait(timeout=5.0):
            print("Error: Did not receive LowState message.")
            return

        print(f"Got current arm states. Starting {1.0/self.control_dt_}Hz control loop to move to {self.move_stage.name}.")
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

    def MoveToNextStage(self):
        """Advances to the next sequential stage (e.g., 2 -> 3)."""
        if self.move_stage.value >= MoveState.STAGE_7.value: # Can't go past 7
            print("Already at final stage.")
            return

        next_stage_val = self.move_stage.value + 1
        self.move_stage = MoveState(next_stage_val)

        print(f"Moving to {self.move_stage.name}...")

        if not self._update_start_positions():
            print(f"Error: Could not move to {self.move_stage.name}, no LowState.")
            self.move_stage = MoveState(self.move_stage.value - 1) # Revert
            return

        self.start_time = time.time()
        self.position_reached_event.clear()

    def Stop(self):
        """
        First returns to Stage 1 position, then begins the fade-out process.
        """
        print("Returning to Stage 1 position before releasing control...")
        if not self._update_start_positions():
            return
        self.start_time = time.time()
        self.move_duration_ = 3.0 # Set duration for returning to Stage 1
        self.move_stage = MoveState.RETURNING
        self.position_reached_event.clear()

    def _control_loop(self):
        """
        The high-frequency control loop.
        """
        with self.lock:
            if self.low_state is None:
                return
            start_pos_left = self.start_positions_left[:]
            start_pos_right = self.start_positions_right[:]
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

        self.low_cmd.mode_pr = 0

        # --- Handle logic based on the current stage ---

        if self.move_stage.value <= MoveState.STAGE_7.value:
            # --- STAGES 1-7: Moving arms ---
            self.low_cmd.motor_cmd[29].q = 1.0

            stage_index = self.move_stage.value - 1 # (e.g., STAGE_1 is index 0)
            target_left, target_right = self.TARGET_STAGES[stage_index]

        elif self.move_stage == MoveState.RETURNING:
            # --- STAGE 8: Returning to Stage 1 position ---
            self.low_cmd.motor_cmd[29].q = 1.0
            target_left, target_right = self.TARGET_STAGES[0] # Target is Stage 1

        elif self.move_stage == MoveState.RELEASING:
            # --- STAGE 9: Releasing control ---
            self.low_cmd.motor_cmd[29].q = 1.0 - ratio # Fade weight
            target_left, target_right = self.TARGET_STAGES[0] # Hold at Stage 1


        # --- Set Arm Commands (if not releasing) ---
        if self.move_stage != MoveState.RELEASING:
            # Set commands for the LEFT Arm joints (15-21)
            for i, joint_id in enumerate(LEFT_ARM_JOINT_IDS):
                interp_q = start_pos_left[i] + (target_left[i] - start_pos_left[i]) * ratio
                motor_cmd = self.low_cmd.motor_cmd[joint_id]
                motor_cmd.q = interp_q
                motor_cmd.dq = 0.0
                motor_cmd.kp = Kp_Arm
                motor_cmd.kd = Kd_Arm
                motor_cmd.tau = 0.0

            # Set commands for the RIGHT Arm joints (22-28)
            for i, joint_id in enumerate(RIGHT_ARM_JOINT_IDS):
                interp_q = start_pos_right[i] + (target_right[i] - start_pos_right[i]) * ratio
                motor_cmd = self.low_cmd.motor_cmd[joint_id]
                motor_cmd.q = interp_q
                motor_cmd.dq = 0.0
                motor_cmd.kp = Kp_Arm
                motor_cmd.kd = Kd_Arm
                motor_cmd.tau = 0.0

        # --- Set Arm Commands (if releasing, just hold Stage 1) ---
        elif self.move_stage == MoveState.RELEASING:
            for i, joint_id in enumerate(LEFT_ARM_JOINT_IDS):
                motor_cmd = self.low_cmd.motor_cmd[joint_id]
                motor_cmd.q = target_left[i] # Hold Stage 1
                motor_cmd.dq = 0.0
                motor_cmd.kp = Kp_Arm
                motor_cmd.kd = Kd_Arm
                motor_cmd.tau = 0.0
            for i, joint_id in enumerate(RIGHT_ARM_JOINT_IDS):
                motor_cmd = self.low_cmd.motor_cmd[joint_id]
                motor_cmd.q = target_right[i] # Hold Stage 1
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
                if self.move_stage == MoveState.RETURNING:
                    print(f"Returning to Stage 1... {int(ratio * 100)}% complete.")
                elif self.move_stage == MoveState.RELEASING:
                    print(f"Releasing control... {int((1.0 - ratio) * 100)}% weight remaining.")
                else:
                    print(f"Moving to {self.move_stage.name}... {int(ratio * 100)}% complete.")

        # Handle stage transitions
        if ratio == 1.0:
            if not self.position_reached_event.is_set():
                print(f"Target position for {self.move_stage.name} reached. Holding.")
                self.position_reached_event.set()

                if self.move_stage == MoveState.RETURNING:
                    # Returned to Stage 1, now fade out control
                    print("Returned to Stage 1. Now releasing arm control...")
                    self._update_start_positions() # Read current pos
                    self.start_time = time.time()
                    self.move_duration_ = 1.0  # 1 second to fade out
                    self.move_stage = MoveState.RELEASING
                    self.position_reached_event.clear()

                elif self.move_stage == MoveState.RELEASING:
                    # Final stop
                    print("Arm control released. Stopping thread.")
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

        # --- REFACTORED: Use MoveToNextStage() ---

        # 3. Trigger Stage 2
        mover.MoveToNextStage() # Becomes Stage 2
        mover.position_reached_event.wait()

        # 4. Auto-Trigger Stage 3
        print("Stage 2 reached. Moving automatically to Stage 3...")
        mover.MoveToNextStage() # Becomes Stage 3
        mover.position_reached_event.wait()

        # 5. Ask user to continue to Stage 4
        print("\n" + "="*80)
        input("Stage 3 reached. Press Enter to move to Stage 4...")
        print("="*80)

        # 6. Trigger Stage 4
        mover.MoveToNextStage() # Becomes Stage 4
        mover.position_reached_event.wait()

        # 7. Auto-Trigger Stage 5
        print("Stage 4 reached. Moving automatically to Stage 5...")
        mover.MoveToNextStage() # Becomes Stage 5
        mover.position_reached_event.wait()

        # 8. Auto-Trigger Stage 6
        print("Stage 5 reached. Moving automatically to Stage 6...")
        mover.MoveToNextStage() # Becomes Stage 6
        mover.position_reached_event.wait()

        # 9. Auto-Trigger Stage 7
        print("Stage 6 reached. Moving automatically to Stage 7...")
        mover.MoveToNextStage() # Becomes Stage 7
        mover.position_reached_event.wait()

        print("\nStage 7 reached.")

        # 10. Auto-Trigger Stop (which moves to Stage 8: RETURNING)
        mover.Stop()

        # 11. Wait for the fade-out to complete (which is Stage 9: RELEASING)
        mover.is_stopped_event.wait(timeout=10.0) # Wait for return (3s) + fade (3s)

        print("\nScript has finished the sequence and released control.")
        print("Robot is back in walking mode. Script will now exit.")

        # --- CHANGED: Script will now exit automatically ---
        sys.exit(0)

    except KeyboardInterrupt:
        # Handle graceful exit
        print("\nStopping control loop... Returning to Stage 1 and fading out arm control.")
        mover.Stop()

        # Wait for the fade-out to complete
        mover.is_stopped_event.wait(timeout=10.0)

        print("Script terminated. Robot is back in walking mode.")