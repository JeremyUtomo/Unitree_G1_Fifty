import time
import threading
from unitree_sdk2py.core.channel import ChannelPublisher
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.idl import default
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread

# Joint indices for arms
LEFT_ARM_JOINT_IDS = [15, 16, 17, 18, 19, 20, 21]
RIGHT_ARM_JOINT_IDS = [22, 23, 24, 25, 26, 27, 28]

# Stage targets (copied from examples)
TARGET_POSITIONS_1_RIGHT = [0.246, -0.281, 0.089, 0.768, 0.000, 0.000, 0.000]
TARGET_POSITIONS_1_LEFT =  [0.246,  0.281, -0.089, 0.768,  0.000, 0.000,  0.000]
TARGET_POSITIONS_2_RIGHT = [-0.041, -0.072, -0.120, 0.418, 0.063, -0.646, 0.400]
TARGET_POSITIONS_2_LEFT =  [-0.041,  0.072,  0.120, 0.418, -0.063, -0.646, -0.400]
TARGET_POSITIONS_3_RIGHT = [0.041, -0.058, 0.478, 0.455, 0.070, -0.560, -0.453]
TARGET_POSITIONS_3_LEFT =  [0.041, 0.058,  -0.478, 0.455, -0.070, -0.560, 0.453]
TARGET_POSITIONS_4_RIGHT = [-1.551, -0.075, -0.168, -0.067, 0.080, -0.156, -0.048]
TARGET_POSITIONS_4_LEFT  = [-1.551,  0.075,  0.168, -0.067, -0.080, -0.156,  0.048]
TARGET_POSITIONS_5_RIGHT = [-2.410, -0.437, -0.085, -0.047, -0.094, -0.159, -0.061]
TARGET_POSITIONS_5_LEFT  = [-2.410,  0.437,  0.085, -0.047,  0.094, -0.159,  0.061]
TARGET_POSITIONS_6_RIGHT = [-2.751, -0.491, 0.109, -0.593, -0.094, -0.161, -0.085]
TARGET_POSITIONS_6_LEFT  = [-2.751,  0.491, -0.109, -0.593,  0.094, -0.161,  0.085]
TARGET_POSITIONS_7_RIGHT = [-2.751, -0.491, -0.563, -0.593, -0.094, -0.161, -0.085]
TARGET_POSITIONS_7_LEFT  = [-2.751,  0.491, 0.563, -0.593,  0.094, -0.161,  0.085]

# Number of motor slots in LowCmd
G1_NUM_MOTOR = 29

# Gains
Kp_Arm = 40.0
Kd_Arm = 1.0

class ArmSequence:
    def __init__(self, control_dt: float = 0.02, stage_duration: float = 2.0):
        self.control_dt_ = control_dt
        self.move_duration_ = stage_duration

        self.low_state = None
        self.low_cmd = default.unitree_hg_msg_dds__LowCmd_()
        self.crc = CRC()
        self.control_thread = None

        self.start_positions_left = [0.0] * len(LEFT_ARM_JOINT_IDS)
        self.start_positions_right = [0.0] * len(RIGHT_ARM_JOINT_IDS)

        self.position_reached_event = threading.Event()
        self.is_stopped_event = threading.Event()
        self.is_running = False

        self.move_stage = 1
        self.lock = threading.Lock()

    def Init(self):
        self.lowcmd_publisher_ = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.lowcmd_publisher_.Init()

    def set_low_state(self, state: LowState_):
        with self.lock:
            self.low_state = state

    def Start(self, initial_low_state: LowState_):
        with self.lock:
            if initial_low_state is None:
                print("ArmSequence.Start: no low state provided")
                return False
            for i, joint_id in enumerate(LEFT_ARM_JOINT_IDS):
                self.start_positions_left[i] = initial_low_state.motor_state[joint_id].q
            for i, joint_id in enumerate(RIGHT_ARM_JOINT_IDS):
                self.start_positions_right[i] = initial_low_state.motor_state[joint_id].q

        self.is_running = True
        self.move_stage = 1
        self.move_duration_ = 2.0
        self.start_time = time.time()
        self.position_reached_event.clear()
        self.is_stopped_event.clear()

        self.control_thread = RecurrentThread(interval=self.control_dt_, target=self._control_loop, name="arm_sequence")
        self.control_thread.Start()
        return True

    def MoveToNextStage(self):
        if self.move_stage == 1:
            self._update_start_positions()
            self.move_duration_ = 2.0
            self.start_time = time.time()
            self.move_stage = 2
            print("ArmSequence: moving to Stage 2")

    def Stop(self):
        if not self.is_running:
            return
        if not self._update_start_positions():
            return
        self.start_time = time.time()
        self.move_duration_ = 3.0
        self.move_stage = 8
        self.position_reached_event.clear()

    def _update_start_positions(self):
        with self.lock:
            if self.low_state is None:
                return False
            for i, joint_id in enumerate(LEFT_ARM_JOINT_IDS):
                self.start_positions_left[i] = self.low_state.motor_state[joint_id].q
            for i, joint_id in enumerate(RIGHT_ARM_JOINT_IDS):
                self.start_positions_right[i] = self.low_state.motor_state[joint_id].q
        return True

    def _control_loop(self):
        with self.lock:
            if self.low_state is None:
                return
            start_pos_left = self.start_positions_left[:]
            start_pos_right = self.start_positions_right[:]
            self.low_cmd.mode_machine = self.low_state.mode_machine

        current_time = time.time() - self.start_time
        ratio = min(current_time / self.move_duration_, 1.0)

        # Reset all motors
        for i in range(G1_NUM_MOTOR):
            self.low_cmd.motor_cmd[i].mode = 0
            self.low_cmd.motor_cmd[i].q = 0.0
            self.low_cmd.motor_cmd[i].dq = 0.0
            self.low_cmd.motor_cmd[i].kp = 0.0
            self.low_cmd.motor_cmd[i].kd = 0.0
            self.low_cmd.motor_cmd[i].tau = 0.0

        self.low_cmd.mode_pr = 0

        if self.move_stage < 8:
            self.low_cmd.motor_cmd[29].q = 1.0

            if self.move_stage == 1:
                target_left, target_right = TARGET_POSITIONS_1_LEFT, TARGET_POSITIONS_1_RIGHT
            elif self.move_stage == 2:
                target_left, target_right = TARGET_POSITIONS_2_LEFT, TARGET_POSITIONS_2_RIGHT
            elif self.move_stage == 3:
                target_left, target_right = TARGET_POSITIONS_3_LEFT, TARGET_POSITIONS_3_RIGHT
            elif self.move_stage == 4:
                target_left, target_right = TARGET_POSITIONS_4_LEFT, TARGET_POSITIONS_4_RIGHT
            elif self.move_stage == 5:
                target_left, target_right = TARGET_POSITIONS_5_LEFT, TARGET_POSITIONS_5_RIGHT
            elif self.move_stage == 6:
                target_left, target_right = TARGET_POSITIONS_6_LEFT, TARGET_POSITIONS_6_RIGHT
            else:
                target_left, target_right = TARGET_POSITIONS_7_LEFT, TARGET_POSITIONS_7_RIGHT

            for i, joint_id in enumerate(LEFT_ARM_JOINT_IDS):
                interp_q = start_pos_left[i] + (target_left[i] - start_pos_left[i]) * ratio
                self.low_cmd.motor_cmd[joint_id].q = interp_q
                self.low_cmd.motor_cmd[joint_id].kp = Kp_Arm
                self.low_cmd.motor_cmd[joint_id].kd = Kd_Arm

            for i, joint_id in enumerate(RIGHT_ARM_JOINT_IDS):
                interp_q = start_pos_right[i] + (target_right[i] - start_pos_right[i]) * ratio
                self.low_cmd.motor_cmd[joint_id].q = interp_q
                self.low_cmd.motor_cmd[joint_id].kp = Kp_Arm
                self.low_cmd.motor_cmd[joint_id].kd = Kd_Arm

        elif self.move_stage == 8:
            self.low_cmd.motor_cmd[29].q = 1.0
            target_left, target_right = TARGET_POSITIONS_1_LEFT, TARGET_POSITIONS_1_RIGHT
            for i, joint_id in enumerate(LEFT_ARM_JOINT_IDS):
                interp_q = start_pos_left[i] + (target_left[i] - start_pos_left[i]) * ratio
                self.low_cmd.motor_cmd[joint_id].q = interp_q
                self.low_cmd.motor_cmd[joint_id].kp = Kp_Arm
                self.low_cmd.motor_cmd[joint_id].kd = Kd_Arm
            for i, joint_id in enumerate(RIGHT_ARM_JOINT_IDS):
                interp_q = start_pos_right[i] + (target_right[i] - start_pos_right[i]) * ratio
                self.low_cmd.motor_cmd[joint_id].q = interp_q
                self.low_cmd.motor_cmd[joint_id].kp = Kp_Arm
                self.low_cmd.motor_cmd[joint_id].kd = Kd_Arm

        elif self.move_stage == 9:
            self.low_cmd.motor_cmd[29].q = 1.0 - ratio
            target_left, target_right = TARGET_POSITIONS_1_LEFT, TARGET_POSITIONS_1_RIGHT
            for i, joint_id in enumerate(LEFT_ARM_JOINT_IDS):
                self.low_cmd.motor_cmd[joint_id].q = target_left[i]
                self.low_cmd.motor_cmd[joint_id].kp = Kp_Arm
                self.low_cmd.motor_cmd[joint_id].kd = Kd_Arm
            for i, joint_id in enumerate(RIGHT_ARM_JOINT_IDS):
                self.low_cmd.motor_cmd[joint_id].q = target_right[i]
                self.low_cmd.motor_cmd[joint_id].kp = Kp_Arm
                self.low_cmd.motor_cmd[joint_id].kd = Kd_Arm

        # Ensure WaistYaw (joint id 12) holds at 0.0 with active gains
        try:
            # Actively hold WaistYaw (joint id 12) at 0.0
            # Use PD gains without forcing position mode to avoid conflict with balancing controller.
            self.low_cmd.motor_cmd[12].q = 0.0
            self.low_cmd.motor_cmd[12].dq = 0.0
            self.low_cmd.motor_cmd[12].kp = 50.0
            self.low_cmd.motor_cmd[12].kd = 2.0
            self.low_cmd.motor_cmd[12].tau = 0.0
        except Exception:
            pass

        # Periodic debug info for waist position (every ~1s)
        try:
            if hasattr(self, 'loop_counter'):
                if self.loop_counter % 50 == 0:
                    actual = None
                    if self.low_state is not None:
                        actual = self.low_state.motor_state[12].q
                    print(f"[waist debug] cmd.q=0.0 kp=50 kd=2 actual={actual}")
        except Exception:
            pass

        # Send command
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_publisher_.Write(self.low_cmd)

        # Stage transitions
        if ratio >= 1.0:
            if self.move_stage < 7:
                self.move_stage += 1
                self.move_duration_ = 2.0
                self.start_time = time.time()
                self._update_start_positions()
            elif self.move_stage == 7:
                # go back to stage 1 automatically
                with self.lock:
                    if self.low_state is not None:
                        for i, joint_id in enumerate(LEFT_ARM_JOINT_IDS):
                            self.start_positions_left[i] = self.low_state.motor_state[joint_id].q
                        for i, joint_id in enumerate(RIGHT_ARM_JOINT_IDS):
                            self.start_positions_right[i] = self.low_state.motor_state[joint_id].q
                self.start_time = time.time()
                self.move_duration_ = 3.0
                self.move_stage = 8
            elif self.move_stage == 8:
                with self.lock:
                    if self.low_state is not None:
                        for i, joint_id in enumerate(LEFT_ARM_JOINT_IDS):
                            self.start_positions_left[i] = self.low_state.motor_state[joint_id].q
                        for i, joint_id in enumerate(RIGHT_ARM_JOINT_IDS):
                            self.start_positions_right[i] = self.low_state.motor_state[joint_id].q
                self.start_time = time.time()
                self.move_duration_ = 1.0
                self.move_stage = 9
            elif self.move_stage == 9:
                self.is_stopped_event.set()
                self.is_running = False
                if self.control_thread:
                    self.control_thread.Stop()


if __name__ == '__main__':
    import sys
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    
    print("="*80)
    print("🤖 Unitree G1 - Manual Arm Sequence Control")
    print("="*80)
    print("📋 This runs the full arm sequence automatically:")
    print("   Stage 1 → Stage 2 → ... → Stage 7 → Return → Release")
    print("="*80)
    print("\n⚠️  SAFETY WARNING:")
    print("   - Ensure no obstacles around the robot")
    print("   - Robot must be in balancing/walking mode")
    print("   - Press Ctrl+C to emergency stop")
    print("="*80)

    if len(sys.argv) < 2:
        print("\nUsage: python3 arm_pick_up_package.py <network_interface>")
        print("Example: python3 arm_pick_up_package.py eth0")
        sys.exit(1)

    network_interface = sys.argv[1]
    ChannelFactoryInitialize(0, network_interface)
    
    print(f"\n✅ Initialized with network interface: {network_interface}")
    
    # Create arm sequence instance
    arm = ArmSequence(control_dt=0.02, stage_duration=2.0)
    arm.Init()
    
    # Subscribe to lowstate to feed arm
    def lowstate_handler(msg: LowState_):
        arm.set_low_state(msg)
    
    lowstate_sub = ChannelSubscriber("rt/lf/lowstate", LowState_)
    lowstate_sub.Init(lowstate_handler, 10)
    
    # Wait for first state
    print("⏳ Waiting for robot state...")
    time.sleep(1.0)
    
    if arm.low_state is None:
        print("❌ Error: No robot state received. Check network connection.")
        sys.exit(1)
    
    print("✅ Robot state received")
    
    try:
        input("\nPress Enter to start the sequence...")
    except KeyboardInterrupt:
        print("\n👋 Aborted.")
        sys.exit(0)
    
    print("\n🚀 Starting sequence...")
    arm.Start(arm.low_state)
    
    # Wait a moment then trigger full sequence
    time.sleep(0.5)
    print("▶️  Running full sequence (Stages 2-7 with auto-return)")
    arm.MoveToNextStage()
    
    try:
        # Wait for completion
        arm.is_stopped_event.wait()
        print("\n✅ Sequence complete!")
        time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n\n🛑 Emergency stop requested...")
        arm.Stop()
        arm.is_stopped_event.wait(timeout=10.0)
        print("✅ Stopped safely")
    
    print("👋 Goodbye!")
