import sys
import time
import threading
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.idl import default
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread

# Joint indices for arms
LEFT_ARM_JOINT_IDS = [15, 16, 17, 18, 19, 20, 21]
RIGHT_ARM_JOINT_IDS = [22, 23, 24, 25, 26, 27, 28]

# Starting position (67 configuration)
START_POSITION_RIGHT = [-0.052, 0.000, 0.120, 0.292, 0.000, -0.000, -0.000]
START_POSITION_LEFT = [-0.052, -0.000, -0.120, 0.292, -1.544, -0.000, 0.000]  # Mirrored

# Return position (from pick_up_arm_sequence.py Stage 1)
RETURN_POSITION_RIGHT = [0.246, -0.281, 0.089, 0.768, 0.000, 0.000, 0.000]
RETURN_POSITION_LEFT = [0.246, 0.281, -0.089, 0.768, 0.000, 0.000, 0.000]

TARGET_POSITION_STAGE_1_RIGHT = [-0.079, 0.043, 0.052, 0.455, 1.597, 0.019, 0.121]
TARGET_POSITION_STAGE_1_LEFT = [-0.027, 0.022, -0.119, -0.527, -1.483, 0.013, -0.076]

TARGET_POSITION_STAGE_2_RIGHT = [-0.154, 0.046, -0.005, -0.287, 1.598, -0.040, -0.089]
TARGET_POSITION_STAGE_2_LEFT = [0.055, 0.019, -0.032, 0.380, -1.484, 0.012, -0.153]

# Number of motor slots in LowCmd
G1_NUM_MOTOR = 29

# Gains
Kp_Arm = 40.0
Kd_Arm = 1.0


class ArmSequence67:
    """Arm sequence starting from 67 configuration."""
    
    def __init__(self, control_dt: float = 0.02, stage_duration: float = 2.0):
        self.control_dt_ = control_dt
        self.move_duration_ = stage_duration

        self.low_state = None
        self.low_cmd = default.unitree_hg_msg_dds__LowCmd_()
        self.crc = CRC()
        self.control_thread = None

        self.current_positions_left = [0.0] * len(LEFT_ARM_JOINT_IDS)
        self.current_positions_right = [0.0] * len(RIGHT_ARM_JOINT_IDS)

        self.is_running = False
        self.is_returning = False
        self.is_releasing = False
        self.current_stage = 1  # Track which stage we're at (1 or 2)
        self.cycle_count = 0  # Track number of complete cycles
        self.max_cycles = 2  # Maximum number of cycles before stopping
        self.lock = threading.Lock()

    def Init(self):
        """Initialize the publisher."""
        self.lowcmd_publisher_ = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.lowcmd_publisher_.Init()

    def set_low_state(self, state: LowState_):
        """Update low state from subscriber."""
        with self.lock:
            self.low_state = state

    def Start(self, initial_low_state: LowState_):
        """Start the sequence - move to 67 position."""
        with self.lock:
            if initial_low_state is None:
                print("ArmSequence67.Start: no low state provided")
                return False
            
            # Record current positions
            for i, joint_id in enumerate(LEFT_ARM_JOINT_IDS):
                self.current_positions_left[i] = initial_low_state.motor_state[joint_id].q
            for i, joint_id in enumerate(RIGHT_ARM_JOINT_IDS):
                self.current_positions_right[i] = initial_low_state.motor_state[joint_id].q

        self.is_running = True
        self.current_stage = 1
        self.cycle_count = 0
        self.start_time = time.time()

        self.control_thread = RecurrentThread(
            interval=self.control_dt_, 
            target=self._control_loop, 
            name="arm_sequence_67"
        )
        self.control_thread.Start()
        print(f"Moving to Stage 1... (Cycle 1/{self.max_cycles})")
        return True

    def Stop(self):
        """Stop the sequence gracefully."""
        if not self.is_running:
            return
        
        print("Stopping arm sequence...")
        self.is_running = False
        if self.control_thread:
            self.control_thread.Stop()
        print("Stopped")
    
    def ReturnToStart(self):
        """Return to starting position (from pick_up_arm_sequence Stage 1)."""
        with self.lock:
            if self.low_state is None:
                print("Cannot return: no low state available")
                return False
            
            # Record current positions
            for i, joint_id in enumerate(LEFT_ARM_JOINT_IDS):
                self.current_positions_left[i] = self.low_state.motor_state[joint_id].q
            for i, joint_id in enumerate(RIGHT_ARM_JOINT_IDS):
                self.current_positions_right[i] = self.low_state.motor_state[joint_id].q
        
        self.is_returning = True
        self.is_running = True
        self.start_time = time.time()
        self.move_duration_ = 3.0  # 3 seconds to return
        
        self.control_thread = RecurrentThread(
            interval=self.control_dt_,
            target=self._return_loop,
            name="arm_return_67"
        )
        self.control_thread.Start()
        print("Returning to starting position...")
        return True
    
    def _start_release(self):
        """Start release phase - gradually reduce weight to return control to walking mode."""
        with self.lock:
            if self.low_state is None:
                return
            
            # Record current positions at Stage 1
            for i, joint_id in enumerate(LEFT_ARM_JOINT_IDS):
                self.current_positions_left[i] = self.low_state.motor_state[joint_id].q
            for i, joint_id in enumerate(RIGHT_ARM_JOINT_IDS):
                self.current_positions_right[i] = self.low_state.motor_state[joint_id].q
        
        self.is_releasing = True
        self.start_time = time.time()
        self.move_duration_ = 1  # 1 second to release
        
        if self.control_thread:
            self.control_thread.Stop()
        
        self.control_thread = RecurrentThread(
            interval=self.control_dt_,
            target=self._release_loop,
            name="arm_release_67"
        )
        self.control_thread.Start()
        print("Releasing to walking mode...")
    
    def _release_loop(self):
        """Control loop for releasing control back to walking mode."""
        if not self.is_running or not self.is_releasing:
            return
        
        with self.lock:
            current_low_state = self.low_state
        
        if current_low_state is None:
            return
        
        # Calculate interpolation ratio
        elapsed = time.time() - self.start_time
        ratio = min(1.0, elapsed / self.move_duration_)
        
        # Hold at Stage 1 positions while reducing weight
        for i, joint_id in enumerate(LEFT_ARM_JOINT_IDS):
            self.low_cmd.motor_cmd[joint_id].q = RETURN_POSITION_LEFT[i]
            self.low_cmd.motor_cmd[joint_id].dq = 0.0
            self.low_cmd.motor_cmd[joint_id].kp = Kp_Arm
            self.low_cmd.motor_cmd[joint_id].kd = Kd_Arm
            self.low_cmd.motor_cmd[joint_id].tau = 0.0
        
        for i, joint_id in enumerate(RIGHT_ARM_JOINT_IDS):
            self.low_cmd.motor_cmd[joint_id].q = RETURN_POSITION_RIGHT[i]
            self.low_cmd.motor_cmd[joint_id].dq = 0.0
            self.low_cmd.motor_cmd[joint_id].kp = Kp_Arm
            self.low_cmd.motor_cmd[joint_id].kd = Kd_Arm
            self.low_cmd.motor_cmd[joint_id].tau = 0.0
        
        # Hold waist yaw (joint 12)
        self.low_cmd.motor_cmd[12].q = current_low_state.motor_state[12].q
        self.low_cmd.motor_cmd[12].dq = 0.0
        self.low_cmd.motor_cmd[12].kp = 50.0
        self.low_cmd.motor_cmd[12].kd = 2.0
        self.low_cmd.motor_cmd[12].tau = 0.0
        
        # Gradually reduce weight command from 1.0 to 0.0
        self.low_cmd.motor_cmd[29].q = 1.0 - ratio
        
        # CRC and publish
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_publisher_.Write(self.low_cmd)
        
        if ratio >= 1.0:
            print("Released to walking mode")
            self.is_releasing = False
            self.is_running = False
            if self.control_thread:
                self.control_thread.Stop()
    
    def _return_loop(self):
        """Control loop for returning to starting position."""
        if not self.is_running or not self.is_returning:
            return
        
        with self.lock:
            current_low_state = self.low_state
        
        if current_low_state is None:
            return
        
        # Calculate interpolation ratio
        elapsed = time.time() - self.start_time
        ratio = min(1.0, elapsed / self.move_duration_)
        
        # Interpolate to return positions
        for i, joint_id in enumerate(LEFT_ARM_JOINT_IDS):
            start = self.current_positions_left[i]
            target = RETURN_POSITION_LEFT[i]
            interpolated = start + (target - start) * ratio
            
            self.low_cmd.motor_cmd[joint_id].q = interpolated
            self.low_cmd.motor_cmd[joint_id].dq = 0.0
            self.low_cmd.motor_cmd[joint_id].kp = Kp_Arm
            self.low_cmd.motor_cmd[joint_id].kd = Kd_Arm
            self.low_cmd.motor_cmd[joint_id].tau = 0.0
        
        for i, joint_id in enumerate(RIGHT_ARM_JOINT_IDS):
            start = self.current_positions_right[i]
            target = RETURN_POSITION_RIGHT[i]
            interpolated = start + (target - start) * ratio
            
            self.low_cmd.motor_cmd[joint_id].q = interpolated
            self.low_cmd.motor_cmd[joint_id].dq = 0.0
            self.low_cmd.motor_cmd[joint_id].kp = Kp_Arm
            self.low_cmd.motor_cmd[joint_id].kd = Kd_Arm
            self.low_cmd.motor_cmd[joint_id].tau = 0.0
        
        # Hold waist yaw (joint 12)
        self.low_cmd.motor_cmd[12].q = current_low_state.motor_state[12].q
        self.low_cmd.motor_cmd[12].dq = 0.0
        self.low_cmd.motor_cmd[12].kp = 50.0
        self.low_cmd.motor_cmd[12].kd = 2.0
        self.low_cmd.motor_cmd[12].tau = 0.0
        
        # Set weight command
        self.low_cmd.motor_cmd[29].q = 1.0
        
        # CRC and publish
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_publisher_.Write(self.low_cmd)
        
        if ratio >= 1.0:
            print("Returned to starting position")
            self.is_returning = False
            self._start_release()

    def _control_loop(self):
        """Control loop running at specified frequency."""
        if not self.is_running:
            return

        with self.lock:
            current_low_state = self.low_state

        if current_low_state is None:
            return

        # Calculate interpolation ratio
        elapsed = time.time() - self.start_time
        ratio = min(1.0, elapsed / self.move_duration_)

        # Select target based on current stage
        if self.current_stage == 1:
            target_left = TARGET_POSITION_STAGE_1_LEFT
            target_right = TARGET_POSITION_STAGE_1_RIGHT
        else:  # stage 2
            target_left = TARGET_POSITION_STAGE_2_LEFT
            target_right = TARGET_POSITION_STAGE_2_RIGHT

        # Interpolate to target positions
        for i, joint_id in enumerate(LEFT_ARM_JOINT_IDS):
            start = self.current_positions_left[i]
            target = target_left[i]
            interpolated = start + (target - start) * ratio
            
            self.low_cmd.motor_cmd[joint_id].q = interpolated
            self.low_cmd.motor_cmd[joint_id].dq = 0.0
            self.low_cmd.motor_cmd[joint_id].kp = Kp_Arm
            self.low_cmd.motor_cmd[joint_id].kd = Kd_Arm
            self.low_cmd.motor_cmd[joint_id].tau = 0.0

        for i, joint_id in enumerate(RIGHT_ARM_JOINT_IDS):
            start = self.current_positions_right[i]
            target = target_right[i]
            interpolated = start + (target - start) * ratio
            
            self.low_cmd.motor_cmd[joint_id].q = interpolated
            self.low_cmd.motor_cmd[joint_id].dq = 0.0
            self.low_cmd.motor_cmd[joint_id].kp = Kp_Arm
            self.low_cmd.motor_cmd[joint_id].kd = Kd_Arm
            self.low_cmd.motor_cmd[joint_id].tau = 0.0

        # Hold waist yaw (joint 12)
        self.low_cmd.motor_cmd[12].q = current_low_state.motor_state[12].q
        self.low_cmd.motor_cmd[12].dq = 0.0
        self.low_cmd.motor_cmd[12].kp = 50.0
        self.low_cmd.motor_cmd[12].kd = 2.0
        self.low_cmd.motor_cmd[12].tau = 0.0

        # Set weight command
        self.low_cmd.motor_cmd[29].q = 1.0

        # CRC and publish
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_publisher_.Write(self.low_cmd)

        # Check if position reached - then switch to next stage
        if ratio >= 1.0:
            # Update current positions to where we are now
            with self.lock:
                if self.low_state is not None:
                    for i, joint_id in enumerate(LEFT_ARM_JOINT_IDS):
                        self.current_positions_left[i] = self.low_state.motor_state[joint_id].q
                    for i, joint_id in enumerate(RIGHT_ARM_JOINT_IDS):
                        self.current_positions_right[i] = self.low_state.motor_state[joint_id].q
            
            if self.current_stage == 1:
                self.current_stage = 2
                print(f"Moving to Stage 2... (Cycle {self.cycle_count + 1}/{self.max_cycles})")
            else:
                self.current_stage = 1
                self.cycle_count += 1
                
                if self.cycle_count >= self.max_cycles:
                    print(f"Completed {self.max_cycles} cycles. Returning to start...")
                    self.Stop()
                    time.sleep(0.2)
                    self.ReturnToStart()
                    return
                
                print(f"Moving to Stage 1... (Cycle {self.cycle_count + 1}/{self.max_cycles})")
            
            self.start_time = time.time()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} <network_interface>")
        print(f"Example: python3 {sys.argv[0]} eth0")
        sys.exit(1)

    print("Unitree G1 - Arm Sequence 67")
    print("WARNING: Ensure no obstacles around the robot!")

    network_interface = sys.argv[1]
    ChannelFactoryInitialize(0, network_interface)

    # Create arm sequence
    arm_seq = ArmSequence67(control_dt=0.02, stage_duration=1.0)
    arm_seq.Init()

    # Subscribe to low state
    lowstate_sub = ChannelSubscriber("rt/lf/lowstate", LowState_)
    lowstate_sub.Init(arm_seq.set_low_state, 10)

    print("Waiting for low state...")
    time.sleep(0.5)

    if arm_seq.low_state is None:
        print("No low state received. Exiting.")
        sys.exit(1)

    arm_seq.Start(arm_seq.low_state)

    try:
        print("\nRunning alternating sequence. Press Ctrl+C to stop early...\n")
        while arm_seq.is_running:
            time.sleep(0.1)
        
        print("\nWaiting for return and release sequence to complete...")
        while arm_seq.is_running:
            time.sleep(0.1)
        
        print("Sequence complete!")
        print("Closing in 5 seconds...")
        time.sleep(3.5)
        
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        arm_seq.Stop()
        time.sleep(0.2)
        
        if arm_seq.ReturnToStart():
            while arm_seq.is_running:
                time.sleep(0.1)
        time.sleep(0.3)
        
        print("Closing in 5 seconds...")
        time.sleep(5.0)

    print("Goodbye!")
