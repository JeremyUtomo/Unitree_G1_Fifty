import time
import sys
import signal
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.idl import default
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread

# Joint indices for arms only
LEFT_ARM_JOINT_IDS = [15, 16, 17, 18, 19, 20, 21]   # Left shoulder to wrist
RIGHT_ARM_JOINT_IDS = [22, 23, 24, 25, 26, 27, 28]  # Right shoulder to wrist

# Total number of motors
G1_NUM_MOTOR = 35

# Damping parameters for arms
Kp_DAMP = 0.0      # Zero stiffness = damping mode
Kd_DAMP = 2.0      # Light damping coefficient

# Target positions for arms when transitioning to walking mode
TARGET_POSITIONS_1_RIGHT = [0.246, -0.281, 0.089, 0.768, 0.000, 0.000, 0.000]
TARGET_POSITIONS_1_LEFT =  [0.246,  0.281, -0.089, 0.768,  0.000, 0.000,  0.000]

# Control parameters for position movement
Kp_ARM = 40.0
Kd_ARM = 1.0

# Stages for state machine
STAGE_DAMPING = 'damping'
STAGE_MOVE_TO_POSITION = 'move_to_position'
STAGE_RELEASE_CONTROL = 'release_control'


class ArmDamper:
    """Applies damping to arms only, leaves legs and waist untouched"""
    
    def __init__(self, control_dt: float = 0.02):
        self.control_dt = control_dt
        self.low_state = None
        self.low_cmd = default.unitree_hg_msg_dds__LowCmd_()
        self.crc = CRC()
        self.is_running = False
        self.control_thread = None
        
        # State machine variables
        self.current_stage = STAGE_DAMPING
        self.start_positions_left = [0.0] * len(LEFT_ARM_JOINT_IDS)
        self.start_positions_right = [0.0] * len(RIGHT_ARM_JOINT_IDS)
        self.move_duration = 3.0
        self.start_time = 0.0
        
    def Init(self):
        """Initialize publisher and subscriber"""
        self.lowcmd_publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.lowcmd_publisher.Init()
        print("✅ Publisher initialized")
        
    def set_low_state(self, state: LowState_):
        """Update current robot state"""
        self.low_state = state
    
    def start_damping(self):
        """Start damping the arms"""
        if self.low_state is None:
            print("❌ Error: No robot state available")
            return False
            
        self.is_running = True
        self.control_thread = RecurrentThread(
            interval=self.control_dt,
            target=self._control_loop,
            name="arm_damping"
        )
        self.control_thread.Start()
        print("🔧 Arm damping started (arms loose, legs/waist unchanged)")
        return True
        
    def stop(self):
        """Stop damping and transition to walking mode"""
        if self.current_stage == STAGE_DAMPING:
            print("🔄 Transitioning to walking mode...")
            self._capture_current_positions()
            self.current_stage = STAGE_MOVE_TO_POSITION
            self.start_time = time.time()
            # Keep running to execute transition
        elif self.current_stage == STAGE_RELEASE_CONTROL:
            self.is_running = False
            if self.control_thread:
                self.control_thread.Stop()
            print("🛑 Control released")
    
    def force_stop(self):
        """Force immediate stop without transition"""
        self.is_running = False
        if self.control_thread:
            self.control_thread.Stop()
        print("🛑 Damping stopped")
    
    def _capture_current_positions(self):
        """Capture current joint positions as start positions"""
        if self.low_state is None:
            return False
        for i, joint_id in enumerate(LEFT_ARM_JOINT_IDS):
            self.start_positions_left[i] = self.low_state.motor_state[joint_id].q
        for i, joint_id in enumerate(RIGHT_ARM_JOINT_IDS):
            self.start_positions_right[i] = self.low_state.motor_state[joint_id].q
        return True
    
    def _control_loop(self):
        """Main control loop - state machine for damping and transition"""
        if self.low_state is None:
            return
            
        mode_machine = self.low_state.mode_machine
        
        # Reset all motor commands to default (no control)
        for i in range(G1_NUM_MOTOR):
            self.low_cmd.motor_cmd[i].mode = 0
            self.low_cmd.motor_cmd[i].q = 0.0
            self.low_cmd.motor_cmd[i].dq = 0.0
            self.low_cmd.motor_cmd[i].kp = 0.0
            self.low_cmd.motor_cmd[i].kd = 0.0
            self.low_cmd.motor_cmd[i].tau = 0.0
            
        self.low_cmd.mode_machine = mode_machine
        self.low_cmd.mode_pr = 0
        
        # Enable arm SDK control (must be set before waist control)
        if self.current_stage != STAGE_RELEASE_CONTROL:
            self.low_cmd.motor_cmd[29].q = 1.0
        else:
            # Fade out control during release
            elapsed = time.time() - self.start_time
            ratio = min(elapsed / self.move_duration, 1.0)
            self.low_cmd.motor_cmd[29].q = 1.0 - ratio
        
        # Hold WaistYaw (joint id 12) at 0.0 with active gains
        self.low_cmd.motor_cmd[12].q = 0.0
        self.low_cmd.motor_cmd[12].dq = 0.0
        self.low_cmd.motor_cmd[12].kp = 50.0
        self.low_cmd.motor_cmd[12].kd = 2.0
        self.low_cmd.motor_cmd[12].tau = 0.0
        
        if self.current_stage == STAGE_DAMPING:
            # Apply damping to left arm
            for joint_id in LEFT_ARM_JOINT_IDS:
                self.low_cmd.motor_cmd[joint_id].q = self.low_state.motor_state[joint_id].q
                self.low_cmd.motor_cmd[joint_id].kp = Kp_DAMP
                self.low_cmd.motor_cmd[joint_id].kd = Kd_DAMP
                
            # Apply damping to right arm
            for joint_id in RIGHT_ARM_JOINT_IDS:
                self.low_cmd.motor_cmd[joint_id].q = self.low_state.motor_state[joint_id].q
                self.low_cmd.motor_cmd[joint_id].kp = Kp_DAMP
                self.low_cmd.motor_cmd[joint_id].kd = Kd_DAMP
        
        elif self.current_stage == STAGE_MOVE_TO_POSITION:
            # Calculate interpolation ratio
            elapsed = time.time() - self.start_time
            ratio = min(elapsed / self.move_duration, 1.0)
            
            # Interpolate left arm to target position
            for i, joint_id in enumerate(LEFT_ARM_JOINT_IDS):
                interp_q = self.start_positions_left[i] + (TARGET_POSITIONS_1_LEFT[i] - self.start_positions_left[i]) * ratio
                self.low_cmd.motor_cmd[joint_id].q = interp_q
                self.low_cmd.motor_cmd[joint_id].kp = Kp_ARM
                self.low_cmd.motor_cmd[joint_id].kd = Kd_ARM
            
            # Interpolate right arm to target position
            for i, joint_id in enumerate(RIGHT_ARM_JOINT_IDS):
                interp_q = self.start_positions_right[i] + (TARGET_POSITIONS_1_RIGHT[i] - self.start_positions_right[i]) * ratio
                self.low_cmd.motor_cmd[joint_id].q = interp_q
                self.low_cmd.motor_cmd[joint_id].kp = Kp_ARM
                self.low_cmd.motor_cmd[joint_id].kd = Kd_ARM
            
            # Check if movement complete
            if ratio >= 1.0:
                print("✅ Target position reached - releasing control...")
                self.current_stage = STAGE_RELEASE_CONTROL
                self.start_time = time.time()
                self.move_duration = 1.0
        
        elif self.current_stage == STAGE_RELEASE_CONTROL:
            # Calculate interpolation ratio (already handled above for motor_cmd[29])
            elapsed = time.time() - self.start_time
            ratio = min(elapsed / self.move_duration, 1.0)
            
            # Hold target positions with gains while motor_cmd[29] fades out
            for i, joint_id in enumerate(LEFT_ARM_JOINT_IDS):
                self.low_cmd.motor_cmd[joint_id].q = TARGET_POSITIONS_1_LEFT[i]
                self.low_cmd.motor_cmd[joint_id].kp = Kp_ARM
                self.low_cmd.motor_cmd[joint_id].kd = Kd_ARM
            
            for i, joint_id in enumerate(RIGHT_ARM_JOINT_IDS):
                self.low_cmd.motor_cmd[joint_id].q = TARGET_POSITIONS_1_RIGHT[i]
                self.low_cmd.motor_cmd[joint_id].kp = Kp_ARM
                self.low_cmd.motor_cmd[joint_id].kd = Kd_ARM
            
            # Check if release complete
            if ratio >= 1.0:
                print("✅ Walking mode restored")
                self.is_running = False
                if self.control_thread:
                    self.control_thread.Stop()
                return
        
        # Send command
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_publisher.Write(self.low_cmd)


def main():
    print("=" * 80)
    print("🤖 Unitree G1 - Arm Damping Only")
    print("=" * 80)
    print("This script damps both arms while keeping legs and waist unchanged.")
    print("Arms will become loose and movable by hand.")
    print("=" * 80)
    print("\n⚠️  SAFETY:")
    print("   - Arms will drop under gravity when damped")
    print("   - Support arms manually if needed")
    print("   - Press Ctrl+C to stop damping")
    print("=" * 80)
    
    if len(sys.argv) < 2:
        print("\nUsage: python3 arm_damp.py <network_interface>")
        print("Example: python3 arm_damp.py eth0")
        sys.exit(1)
        
    network_interface = sys.argv[1]
    ChannelFactoryInitialize(0, network_interface)
    print(f"\n✅ Network initialized on {network_interface}")
    
    # Create damper
    damper = ArmDamper(control_dt=0.02)
    damper.Init()
    
    # Subscribe to robot state
    def state_handler(msg: LowState_):
        damper.set_low_state(msg)
        
    state_sub = ChannelSubscriber("rt/lowstate", LowState_)
    state_sub.Init(state_handler, 10)
    
    # Wait for first state
    print("⏳ Waiting for robot state...")
    max_wait = 10.0
    wait_start = time.time()
    
    while damper.low_state is None and (time.time() - wait_start) < max_wait:
        time.sleep(0.1)
    
    if damper.low_state is None:
        print("❌ Error: No robot state received after 10 seconds")
        print("   Check network connection and robot status")
        sys.exit(1)
        
    print("✅ Robot state received")
    
    # Setup signal handler for graceful shutdown
    def signal_handler(sig, frame):
        print("\n\n⚠️  Interrupt received - transitioning to walking mode...")
        print("⏳ Press Ctrl+C again within 10 seconds to force stop")
        damper.stop()
        
        # Wait up to 10 seconds for user confirmation or transition completion
        wait_start = time.time()
        max_wait = 10.0
        
        try:
            while damper.is_running and (time.time() - wait_start) < max_wait:
                time.sleep(0.1)
            
            # If transition completed within 10 seconds
            if not damper.is_running:
                time.sleep(1.0)
                print("👋 Goodbye!")
                sys.exit(0)
            else:
                # Timeout - continue transition
                print("⏳ Continuing transition to walking mode...")
                while damper.is_running:
                    time.sleep(0.1)
                time.sleep(1.0)
                print("👋 Goodbye!")
                sys.exit(0)
                
        except KeyboardInterrupt:
            # Second Ctrl+C - force stop
            print("\n⚠️  Force stop - immediate exit")
            damper.force_stop()
            time.sleep(0.2)
            print("👋 Goodbye!")
            sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        input("\n▶️  Press Enter to start damping arms...")
    except KeyboardInterrupt:
        print("\n👋 Cancelled")
        sys.exit(0)
        
    # Start damping
    if not damper.start_damping():
        sys.exit(1)
    
    print("\n💡 Arms are now damped. Press Ctrl+C to transition to walking mode.")
    
    # Keep running until interrupted or sequence completes
    try:
        while damper.is_running:
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    
    # If still running, wait for completion
    while damper.is_running:
        time.sleep(0.1)
    
    time.sleep(0.5)
    print("👋 Goodbye!")


if __name__ == '__main__':
    main()
