import time
import sys
import struct
import threading

from unitree_sdk2py.core.channel import (
    ChannelSubscriber,
    ChannelPublisher,
    ChannelFactoryInitialize,
)

# Uncomment the following two lines when using Go2、Go2-W、B2、B2-W、H1 robot
# from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_
# from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_

# Uncomment the following two lines when using G1、H1-2 robot
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_, unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, LowCmd_
from unitree_sdk2py.idl import default

from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread

# --- Constants ---
G1_NUM_MOTOR = 29

# Kp and Kd values
Kp_Arm = 40.0
Kd_Arm = 1.0

# --- Joint IDs ---
LEFT_ARM_JOINT_IDS = [15, 16, 17, 18, 19, 20, 21]
RIGHT_ARM_JOINT_IDS = [22, 23, 24, 25, 26, 27, 28]

# --- STAGE 1 Target positions ---
TARGET_POSITIONS_1_RIGHT = [0.246, -0.281, 0.089, 0.768, 0.000, 0.000, 0.000]
TARGET_POSITIONS_1_LEFT =  [0.246,  0.281, -0.089, 0.768,  0.000, 0.000,  0.000]

# --- STAGE 2 Target positions ---
TARGET_POSITIONS_2_RIGHT = [-0.041, -0.072, -0.120, 0.418, 0.063, -0.646, 0.400]
TARGET_POSITIONS_2_LEFT =  [-0.041,  0.072,  0.120, 0.418, -0.063, -0.646, -0.400]

# --- STAGE 3 Target positions ---
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

class unitreeRemoteController:
    def __init__(self):
        # key
        self.Lx = 0           
        self.Rx = 0            
        self.Ry = 0            
        self.Ly = 0

        # button
        self.L1 = 0
        self.L2 = 0
        self.R1 = 0
        self.R2 = 0
        self.A = 0
        self.B = 0
        self.X = 0
        self.Y = 0
        self.Up = 0
        self.Down = 0
        self.Left = 0
        self.Right = 0
        self.Select = 0
        self.F1 = 0
        self.F3 = 0
        self.Start = 0
       
    def parse_botton(self,data1,data2):
        self.R1 = (data1 >> 0) & 1
        self.L1 = (data1 >> 1) & 1
        self.Start = (data1 >> 2) & 1
        self.Select = (data1 >> 3) & 1
        self.R2 = (data1 >> 4) & 1
        self.L2 = (data1 >> 5) & 1
        self.F1 = (data1 >> 6) & 1
        self.F3 = (data1 >> 7) & 1
        self.A = (data2 >> 0) & 1
        self.B = (data2 >> 1) & 1
        self.X = (data2 >> 2) & 1
        self.Y = (data2 >> 3) & 1
        self.Up = (data2 >> 4) & 1
        self.Right = (data2 >> 5) & 1
        self.Down = (data2 >> 6) & 1
        self.Left = (data2 >> 7) & 1

    def parse_key(self,data):
        lx_offset = 4
        self.Lx = struct.unpack('<f', data[lx_offset:lx_offset + 4])[0]
        rx_offset = 8
        self.Rx = struct.unpack('<f', data[rx_offset:rx_offset + 4])[0]
        ry_offset = 12
        self.Ry = struct.unpack('<f', data[ry_offset:ry_offset + 4])[0]
        L2_offset = 16
        L2 = struct.unpack('<f', data[L2_offset:L2_offset + 4])[0] # Placeholder，unused
        ly_offset = 20
        self.Ly = struct.unpack('<f', data[ly_offset:ly_offset + 4])[0]


    def parse(self,remoteData):
        self.parse_key(remoteData)
        self.parse_botton(remoteData[2],remoteData[3])

class ArmMover:
    def __init__(self, control_dt=0.02, move_duration=3.0):
        self.control_dt_ = control_dt
        self.move_duration_ = move_duration
        
        self.low_cmd = default.unitree_hg_msg_dds__LowCmd_()
        self.crc = CRC()
        self.control_thread = None
        self.start_time = 0.0
        self.loop_counter = 0

        self.start_positions_left = [0.0] * len(LEFT_ARM_JOINT_IDS)
        self.start_positions_right = [0.0] * len(RIGHT_ARM_JOINT_IDS)
        
        self.position_reached_event = threading.Event()
        self.is_stopped_event = threading.Event()
        self.is_running = False
        
        # State machine: 1-7 = movement stages, 8 = return to Stage 1, 9 = release control
        self.move_stage = 1
        self.lock = threading.Lock()
        self.low_state_for_arm = None  # Separate reference for arm control

    def set_low_state(self, state):
        """Called from main low state handler"""
        with self.lock:
            self.low_state_for_arm = state

    def Init(self):
        # Publish to the 'rt/arm_sdk' topic
        self.lowcmd_publisher_ = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.lowcmd_publisher_.Init()

    def Start(self, current_low_state):
        """Start the arm control sequence"""
        if self.is_running:
            print("⚠️  Arm control already running!")
            return False
            
        print("\n" + "="*80)
        print("🚀 Starting arm movement sequence...")
        print("="*80 + "\n")
        
        with self.lock:
            if current_low_state is None:
                print("❌ Error: No robot state available")
                return False
            
            self.low_state_for_arm = current_low_state
            # Get initial arm positions
            for i, joint_id in enumerate(LEFT_ARM_JOINT_IDS):
                self.start_positions_left[i] = current_low_state.motor_state[joint_id].q
            for i, joint_id in enumerate(RIGHT_ARM_JOINT_IDS):
                self.start_positions_right[i] = current_low_state.motor_state[joint_id].q

        self.is_running = True
        self.move_stage = 1
        self.start_time = time.time()
        self.position_reached_event.clear()
        self.is_stopped_event.clear()
        
        self.control_thread = RecurrentThread(
            interval=self.control_dt_, 
            target=self._control_loop, 
            name="arm_control"
        )
        self.control_thread.Start()
        print("✅ Arm control started - Moving to Stage 1")
        return True

    def Stop(self):
        """Stop the arm control sequence and return to Stage 1"""
        if not self.is_running:
            return
            
        print("\n" + "="*80)
        print("🛑 Stopping arm control - Returning to Stage 1...")
        print("="*80 + "\n")
        
        if not self._update_start_positions():
            return
        self.start_time = time.time()
        self.move_duration_ = 3.0
        self.move_stage = 8  # Return to Stage 1
        self.position_reached_event.clear()

    def _update_start_positions(self):
        with self.lock:
            if self.low_state_for_arm is None:
                return False
            for i, joint_id in enumerate(LEFT_ARM_JOINT_IDS):
                self.start_positions_left[i] = self.low_state_for_arm.motor_state[joint_id].q
            for i, joint_id in enumerate(RIGHT_ARM_JOINT_IDS):
                self.start_positions_right[i] = self.low_state_for_arm.motor_state[joint_id].q
        return True

    def _control_loop(self):
        """High-frequency control loop"""
        with self.lock:
            if self.low_state_for_arm is None:
                return
            start_pos_left = self.start_positions_left[:]
            start_pos_right = self.start_positions_right[:]
            self.low_cmd.mode_machine = self.low_state_for_arm.mode_machine

        current_time = time.time() - self.start_time
        ratio = min(current_time / self.move_duration_, 1.0)

        # Set all motors to idle
        for i in range(G1_NUM_MOTOR):
            self.low_cmd.motor_cmd[i].mode = 0
            self.low_cmd.motor_cmd[i].q = 0.0
            self.low_cmd.motor_cmd[i].dq = 0.0
            self.low_cmd.motor_cmd[i].kp = 0.0
            self.low_cmd.motor_cmd[i].kd = 0.0
            self.low_cmd.motor_cmd[i].tau = 0.0
        
        self.low_cmd.mode_pr = 0
        
        # Handle different stages
        if self.move_stage < 8:
            # Movement stages 1-7
            self.low_cmd.motor_cmd[29].q = 1.0
            
            # Select target based on stage
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
            else:  # stage 7
                target_left, target_right = TARGET_POSITIONS_7_LEFT, TARGET_POSITIONS_7_RIGHT

            # Set arm joint commands
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
            # Return to Stage 1
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
            # Release control
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
        
        # Send command
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_publisher_.Write(self.low_cmd)
        
        # Handle stage transitions
        if ratio >= 1.0:
            if self.move_stage < 7:
                # Auto-advance through stages 1-7
                print(f"✅ Stage {self.move_stage} complete - Moving to Stage {self.move_stage + 1}")
                self.move_stage += 1
                self.start_time = time.time()
                self._update_start_positions()
            elif self.move_stage == 7:
                # Reached final stage, hold
                print("✅ Stage 7 complete - Holding position (Press SELECT+X to stop)")
                self.move_stage = 10  # Hold state
            elif self.move_stage == 8:
                # Returned to Stage 1, now release
                print("✅ Returned to Stage 1 - Releasing control...")
                with self.lock:
                    if self.low_state_for_arm is not None:
                        for i, joint_id in enumerate(LEFT_ARM_JOINT_IDS):
                            self.start_positions_left[i] = self.low_state_for_arm.motor_state[joint_id].q
                        for i, joint_id in enumerate(RIGHT_ARM_JOINT_IDS):
                            self.start_positions_right[i] = self.low_state_for_arm.motor_state[joint_id].q
                self.start_time = time.time()
                self.move_duration_ = 1.0
                self.move_stage = 9
            elif self.move_stage == 9:
                # Finished releasing
                print("✅ Control released - Back to walking mode")
                self.is_stopped_event.set()
                self.is_running = False
                if self.control_thread:
                    self.control_thread.Stop()

        
class Custom:
    def __init__(self, network_interface):
        self.low_state = None 
        self.remoteController = unitreeRemoteController()
        self.network_interface = network_interface
        self.arm_mover = ArmMover()
        
        # Button press tracking
        self.select_a_was_pressed = False
        self.select_x_was_pressed = False

    def Init(self):
        self.lowstate_subscriber = ChannelSubscriber("rt/lf/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateMessageHandler, 10)
        self.arm_mover.Init()

    
    def LowStateMessageHandler(self, msg: LowState_):
        self.low_state = msg
        wireless_remote_data = self.low_state.wireless_remote
        self.remoteController.parse(wireless_remote_data)
        
        # Update arm mover with latest state
        self.arm_mover.set_low_state(msg)
        
        # Check for SELECT + A (start sequence)
        if self.remoteController.Select == 1 and self.remoteController.A == 1:
            if not self.select_a_was_pressed:
                self.select_a_was_pressed = True
                if not self.arm_mover.is_running:
                    self.arm_mover.Start(msg)
        else:
            self.select_a_was_pressed = False
        
        # Check for SELECT + X (stop sequence)
        if self.remoteController.Select == 1 and self.remoteController.X == 1:
            if not self.select_x_was_pressed:
                self.select_x_was_pressed = True
                if self.arm_mover.is_running:
                    self.arm_mover.Stop()
        else:
            self.select_x_was_pressed = False


if __name__ == '__main__':

    print("="*80)
    print("🤖 Unitree G1 Wireless Controller - Arm Control")
    print("="*80)
    print("📋 Controls:")
    print("   SELECT + A : Start arm movement sequence (Stages 1-7)")
    print("   SELECT + X : Stop sequence and return to walking mode")
    print("="*80)
    print("\n⚠️  SAFETY WARNING:")
    print("   - Ensure no obstacles around the robot")
    print("   - Robot must be in balancing/walking mode")
    print("   - Be ready for emergency stop")
    print("="*80)
    input("\nPress Enter to start...\n")

    # Get network interface from command line argument
    if len(sys.argv) > 1:
        network_interface = sys.argv[1]
        ChannelFactoryInitialize(0, network_interface)
    else:
        print("Usage: python3 custom_wireless_controller.py <network_interface>")
        print("Example: python3 custom_wireless_controller.py ens33")
        sys.exit(1)

    print(f"✅ Initialized with network interface: {network_interface}\n")
    print("🎮 Wireless controller active - waiting for commands...\n")

    custom = Custom(network_interface)
    custom.Init()

    try:
        while True:   
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n\n" + "="*80)
        print("🛑 Shutting down...")
        
        # Stop arm control if running
        if custom.arm_mover.is_running:
            print("   Stopping arm control...")
            custom.arm_mover.Stop()
            custom.arm_mover.is_stopped_event.wait(timeout=10.0)
        
        print("="*80)
        print("👋 Goodbye!")
        sys.exit(0)