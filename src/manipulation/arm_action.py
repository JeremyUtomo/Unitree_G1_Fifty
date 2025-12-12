#!/usr/bin/env python3
"""Unitree G1 Left Arm - Custom 4-Position Sequence
Moves through: Starting Position → Position 1 → Position 2 → Position 3 → Hand Open → Position 1 → Starting Position
"""
import time
import sys
import threading
import subprocess
import os
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_, HandState_
from unitree_sdk2py.idl import default
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from hand_controller import HandController

# Left arm joint indices
LEFT_ARM_JOINT_IDS = [15, 16, 17, 18, 19, 20, 21]

# Right arm joint indices
RIGHT_ARM_JOINT_IDS = [22, 23, 24, 25, 26, 27, 28]

# Right arm hold position (captured from monitor)
RIGHT_ARM_HOLD_POSITION = [0.294, -0.229, 0.018, 0.977, -0.132, 0.028, -0.012]

# Target positions from your data
STARTING_POSITION = [0.256, 0.280, -0.079, 0.829, 0.005, 0.012, -0.001]
POSITION_1 = [0.844, 0.247, -0.041, -0.991, -0.305, 0, 0.017]
POSITION_2 = [-0.255, 0.221, 0.312, 0.016, 0.010, 0, -0.087]
POSITION_3 = [-0.442, 0.004, -0.058, 0.183, -0.112, 0.002, 0.014]
POSITION_4 = [-0.253, 0.200, 0, -0.743, 0, 0.75, 0.081]

# Stage names for clarity
STAGE_START = 0
STAGE_POSITION_1 = 1
STAGE_POSITION_2 = 2
STAGE_POSITION_3 = 3
STAGE_POSITION_4 = 4
STAGE_RETURN_TO_POSITION_1 = 5
STAGE_RETURN_START = 6
STAGE_RETURN_TO_NEUTRAL = 'return_to_neutral'  # Return to neutral position
STAGE_RELEASE_CONTROL = 'release_control'      # Release arm SDK control
STAGE_INTERRUPT_RETURN = 'interrupt_return'    # Interrupt handler - return to start

# Put-down sequence stages
STAGE_PUTDOWN_TO_POS3 = 'putdown_to_pos3'      # Return to Position 3
STAGE_PUTDOWN_OPEN_HAND = 'putdown_open_hand'  # Open hand at Position 3
STAGE_PUTDOWN_TO_POS2 = 'putdown_to_pos2'      # Move to Position 2
STAGE_PUTDOWN_TO_POS1 = 'putdown_to_pos1'      # Move to Position 1
STAGE_PUTDOWN_TO_START = 'putdown_to_start'    # Move to Starting position

# Control parameters
G1_NUM_MOTOR = 35
Kp_ARM = 40.0
Kd_ARM = 1.0


class LeftArmSequence:
    """Controls left arm through a 3-position sequence"""
    
    PRESSURE_THRESHOLD = 150000  # Pressure detection threshold (15.0 scaled)
    
    def __init__(self, control_dt: float = 0.02):
        self.control_dt = control_dt
        self.low_state = None
        self.hand_state = None
        self.low_cmd = default.unitree_hg_msg_dds__LowCmd_()
        self.crc = CRC()
        
        self.start_positions = [0.0] * len(LEFT_ARM_JOINT_IDS)
        self.target_positions = STARTING_POSITION.copy()
        
        self.current_stage = STAGE_START
        self.move_duration = 3.0
        self.start_time = 0.0
        
        self.is_running = False
        self.control_thread = None
        self.lock = threading.Lock()
        
        self.hand_opened = False
        self.pressure_detected = False
        self.interrupt_requested = False
        self.position_4_hold_printed = False
        self.put_down_requested = False
        
        # Loco client for FSM control
        self.loco_client = None
        
    def Init(self):
        """Initialize publishers and hand controller"""
        self.lowcmd_publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.lowcmd_publisher.Init()
        
        # Initialize hand controller
        self.hand_controller = HandController()
        self.hand_controller.init_left_hand()
        
        # Initialize loco client for FSM control
        try:
            self.loco_client = LocoClient()
            self.loco_client.SetTimeout(10.0)
            self.loco_client.Init()
            time.sleep(0.5)  # Give loco client time to initialize
            print("✅ LocoClient initialized for FSM control")
        except Exception as e:
            print(f"⚠️  Warning: Failed to initialize LocoClient: {e}")
            self.loco_client = None
        
        print("✅ Publishers initialized")
        
    def set_low_state(self, state: LowState_):
        """Update current robot state"""
        with self.lock:
            self.low_state = state
    
    def set_hand_state(self, state: HandState_):
        """Update hand state"""
        with self.lock:
            self.hand_state = state
    
    def _check_pressure(self):
        """Check if any fingertip has detected pressure"""
        if self.hand_state is None:
            return False
        
        if not hasattr(self.hand_state, 'press_sensor_state'):
            return False
        
        # Check all fingertip sensors
        for sensor in self.hand_state.press_sensor_state:
            pressures = list(sensor.pressure)
            if any(p >= self.PRESSURE_THRESHOLD for p in pressures):
                return True
        return False
            
    def _capture_current_positions(self):
        """Capture current joint positions as start positions"""
        with self.lock:
            if self.low_state is None:
                return False
            for i, joint_id in enumerate(LEFT_ARM_JOINT_IDS):
                self.start_positions[i] = self.low_state.motor_state[joint_id].q
        return True
        
    def start_sequence(self):
        """Start the full sequence from current position"""
        if not self._capture_current_positions():
            print("❌ Error: No robot state available")
            return False
            
        self.current_stage = STAGE_START
        self.target_positions = STARTING_POSITION.copy()
        self.move_duration = 0.5
        self.start_time = time.time()
        self.is_running = True
        
        # Start control loop
        self.control_thread = RecurrentThread(
            interval=self.control_dt,
            target=self._control_loop,
            name="left_arm_control"
        )
        self.control_thread.Start()
        print("🚀 Sequence started")
        return True
        
    def stop(self):
        """Stop the sequence immediately"""
        self.is_running = False
        if self.control_thread:
            self.control_thread.Stop()
        print("🛑 Sequence stopped")
    
    def start_put_down(self):
        """Start put-down sequence from Position 4"""
        if not self.is_running:
            print("❌ Error: Sequence not running")
            return False
        
        if self.current_stage != STAGE_POSITION_4:
            print(f"❌ Error: Can only put down from Position 4 (currently at stage {self.current_stage})")
            return False
        
        print("🔽 Starting put-down sequence...")
        self.put_down_requested = True
        
        # Use Position 4 as start position (don't capture - we're already holding it)
        self.start_positions = POSITION_4.copy()
        self.target_positions = POSITION_3.copy()
        self.current_stage = STAGE_PUTDOWN_TO_POS3
        self.move_duration = 2.0
        self.start_time = time.time()
        
        return True
    
    def graceful_stop(self):
        """Gracefully return to starting position before stopping"""
        if not self.is_running:
            return
        
        # Set interrupt flag to stop any ongoing operations
        self.interrupt_requested = True
        
        # Return through Position 1 to Starting Position
        print("🔄 Interrupt received - returning to starting position...")
        
        if self.current_stage == STAGE_POSITION_4:
            # From Position 4, go to Position 1 first - capture actual positions
            if not self._capture_current_positions():
                # Fallback to known Position 4 if capture fails
                self.start_positions = POSITION_4.copy()
            self.target_positions = POSITION_1.copy()
            self.current_stage = STAGE_RETURN_TO_POSITION_1
            self.move_duration = 5.0  # Slower transition: 5 seconds
            self.start_time = time.time()
        elif self.current_stage == STAGE_POSITION_3:
            # From Position 3, go to Position 1 first - capture actual positions
            if not self._capture_current_positions():
                # Fallback to known Position 3 if capture fails
                self.start_positions = POSITION_3.copy()
            self.target_positions = POSITION_1.copy()
            self.current_stage = STAGE_RETURN_TO_POSITION_1
            self.move_duration = 5.0  # Slower transition: 5 seconds
            self.start_time = time.time()
        else:
            # From any other stage, go directly to starting position
            self._capture_current_positions()
            self.target_positions = STARTING_POSITION.copy()
            self.current_stage = STAGE_INTERRUPT_RETURN
            self.move_duration = 3.0
            self.start_time = time.time()
    
    def release_to_walking_mode(self):
        """Release arm control and return to walking mode"""
        if not self.is_running:
            # Need to start control thread to release
            self._capture_current_positions()
            self.is_running = True
            self.control_thread = RecurrentThread(
                interval=self.control_dt,
                target=self._control_loop,
                name="left_arm_release"
            )
            self.control_thread.Start()
        
        # Return to neutral position before releasing control
        self._capture_current_positions()
        self.target_positions = STARTING_POSITION.copy()
        self.current_stage = STAGE_RETURN_TO_NEUTRAL
        self.move_duration = 3.0
        self.start_time = time.time()
        print("🔄 Returning to starting position...")
        
    def _control_loop(self):
        """Main control loop - runs at fixed frequency"""
        with self.lock:
            if self.low_state is None:
                return
            mode_machine = self.low_state.mode_machine
            
        # Calculate interpolation ratio
        elapsed = time.time() - self.start_time
        ratio = min(elapsed / self.move_duration, 1.0)
        
        # Reset all motor commands
        for i in range(G1_NUM_MOTOR):
            self.low_cmd.motor_cmd[i].mode = 0
            self.low_cmd.motor_cmd[i].q = 0.0
            self.low_cmd.motor_cmd[i].dq = 0.0
            self.low_cmd.motor_cmd[i].kp = 0.0
            self.low_cmd.motor_cmd[i].kd = 0.0
            self.low_cmd.motor_cmd[i].tau = 0.0
            
        self.low_cmd.mode_machine = mode_machine
        self.low_cmd.mode_pr = 0
        
        # Enable/disable arm control based on stage
        if self.current_stage == STAGE_RELEASE_CONTROL:
            # Gradually release control (1.0 -> 0.0)
            self.low_cmd.motor_cmd[29].q = 1.0 - ratio
        else:
            # All other stages: arm control enabled
            self.low_cmd.motor_cmd[29].q = 1.0
        
        # Interpolate left arm positions
        for i, joint_id in enumerate(LEFT_ARM_JOINT_IDS):
            interp_q = self.start_positions[i] + (self.target_positions[i] - self.start_positions[i]) * ratio
            self.low_cmd.motor_cmd[joint_id].q = interp_q
            self.low_cmd.motor_cmd[joint_id].kp = Kp_ARM
            self.low_cmd.motor_cmd[joint_id].kd = Kd_ARM
        
        # Hold right arm at fixed position throughout sequence
        for i, joint_id in enumerate(RIGHT_ARM_JOINT_IDS):
            self.low_cmd.motor_cmd[joint_id].q = RIGHT_ARM_HOLD_POSITION[i]
            self.low_cmd.motor_cmd[joint_id].kp = Kp_ARM
            self.low_cmd.motor_cmd[joint_id].kd = Kd_ARM
            
        # Hold waist at 0
        self.low_cmd.motor_cmd[12].q = 0.0
        self.low_cmd.motor_cmd[12].kp = 50.0
        self.low_cmd.motor_cmd[12].kd = 2.0
        
        # Send command
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_publisher.Write(self.low_cmd)
        
        # Check for stage completion and transition
        if ratio >= 1.0:
            if self.current_stage == STAGE_START:
                # Move to Position 1
                self.start_positions = STARTING_POSITION.copy()
                self.target_positions = POSITION_1.copy()
                self.current_stage = STAGE_POSITION_1
                self.move_duration = 2.0
                self.start_time = time.time()
                print("▶️  Moving to Position 1...")
                
            elif self.current_stage == STAGE_POSITION_1:
                # Move to Position 2
                self.start_positions = POSITION_1.copy()
                self.target_positions = POSITION_2.copy()
                self.current_stage = STAGE_POSITION_2
                self.move_duration = 2.0
                self.start_time = time.time()
                print("▶️  Moving to Position 2...")
                
            elif self.current_stage == STAGE_POSITION_2:
                # Open hand after position 2
                if not self.hand_opened:
                    print("🖐️  Opening hand...")
                    self.hand_controller.open_left_hand()
                    time.sleep(0.5)  # Wait for hand to open
                    self.hand_opened = True
                
                # Move to Position 3
                self.start_positions = POSITION_2.copy()
                self.target_positions = POSITION_3.copy()
                self.current_stage = STAGE_POSITION_3
                self.move_duration = 2.0
                self.start_time = time.time()
                print("▶️  Moving to Position 3...")
                
            elif self.current_stage == STAGE_POSITION_3:
                # Check if interrupt was requested during transition
                if self.interrupt_requested:
                    return  # Exit immediately, graceful_stop has already set up return path
                
                # Close hand after position 3
                print("🤏 Closing hand until pressure detected...")
                self.hand_controller.close_left_hand()
                self.pressure_detected = False
                
                # Monitor pressure while closing
                close_start = time.time()
                max_close_time = 3.0
                
                while (time.time() - close_start) < max_close_time:
                    # Check for interrupt during pressure monitoring
                    if self.interrupt_requested:
                        print("⚠️  Interrupt during hand closing - aborting")
                        return
                    
                    if self._check_pressure():
                        print("✓ Pressure detected - holding position!")
                        self.hand_controller.hold_left_hand_position()
                        self.pressure_detected = True
                        break
                    time.sleep(0.05)  # Check at 20Hz
                
                if not self.pressure_detected and not self.interrupt_requested:
                    print("⚠️  No pressure detected - hand fully closed")
                
                # Move to Position 4 after grasping
                if not self.interrupt_requested:
                    self.start_positions = POSITION_3.copy()
                    self.target_positions = POSITION_4.copy()
                    self.current_stage = STAGE_POSITION_4
                    self.move_duration = 2.0
                    self.start_time = time.time()
                    print("▶️  Moving to Position 4...")
            
            elif self.current_stage == STAGE_POSITION_4:
                # Check if interrupt was requested during transition
                if self.interrupt_requested:
                    return  # Exit immediately, graceful_stop has already set up return path
                
                # Hold position 4 indefinitely - wait for user interrupt
                if not self.interrupt_requested and not self.position_4_hold_printed:
                    print("✅ Position 4 reached - holding position until Ctrl+C...")
                    self.position_4_hold_printed = True
                # Don't transition to next stage - stay in STAGE_POSITION_4
            elif self.current_stage == STAGE_RETURN_TO_POSITION_1:
                # Return to starting position
                self.start_positions = POSITION_1.copy()
                self.target_positions = STARTING_POSITION.copy()
                self.current_stage = STAGE_RETURN_START
                self.move_duration = 4.0
                self.start_time = time.time()
                print("▶️  Returning to starting position...")
                
            elif self.current_stage == STAGE_RETURN_START:
                # Returned to starting - now release control
                print("✅ Returned to starting position")
                self.start_positions = STARTING_POSITION.copy()
                self.current_stage = STAGE_RETURN_TO_NEUTRAL
                self.move_duration = 3.0
                self.start_time = time.time()
            
            elif self.current_stage == STAGE_INTERRUPT_RETURN:
                # Interrupt return complete - now release
                print("✅ Returned to starting position")
                self.start_positions = STARTING_POSITION.copy()
                self.current_stage = STAGE_RETURN_TO_NEUTRAL
                self.move_duration = 3.0
                self.start_time = time.time()
            
            elif self.current_stage == STAGE_RETURN_TO_NEUTRAL:
                # Neutral position reached - now release arm control
                self.start_positions = STARTING_POSITION.copy()
                self.current_stage = STAGE_RELEASE_CONTROL
                self.move_duration = 1.0
                self.start_time = time.time()
                print("🔓 Releasing arm control...")
            
            elif self.current_stage == STAGE_PUTDOWN_TO_POS3:
                # Reached Position 3 - open hand
                print("✅ Position 3 reached - opening hand...")
                self.hand_controller.open_left_hand()
                time.sleep(2.5)  # Wait for hand to open
                
                # Move to Position 2
                self.start_positions = POSITION_3.copy()
                self.target_positions = POSITION_2.copy()
                self.current_stage = STAGE_PUTDOWN_TO_POS2
                self.move_duration = 2.0
                self.start_time = time.time()
                print("▶️  Moving to Position 2...")
            
            elif self.current_stage == STAGE_PUTDOWN_TO_POS2:
                # Close hand and move to Position 1
                print("🤏 Closing hand...")
                self.hand_controller.close_left_hand()
                time.sleep(2.0)  # Wait for hand to close
                
                self.start_positions = POSITION_2.copy()
                self.target_positions = POSITION_1.copy()
                self.current_stage = STAGE_PUTDOWN_TO_POS1
                self.move_duration = 2.0
                self.start_time = time.time()
                print("▶️  Moving to Position 1...")
            
            elif self.current_stage == STAGE_PUTDOWN_TO_POS1:
                # Move to Starting position
                self.start_positions = POSITION_1.copy()
                self.target_positions = STARTING_POSITION.copy()
                self.current_stage = STAGE_PUTDOWN_TO_START
                self.move_duration = 4.0
                self.start_time = time.time()
                print("▶️  Returning to starting position...")
            
            elif self.current_stage == STAGE_PUTDOWN_TO_START:
                # Reached starting position - release control first, then switch FSM
                print("✅ Reached starting position - releasing arm control...")
                self.start_positions = STARTING_POSITION.copy()
                self.current_stage = STAGE_RELEASE_CONTROL
                self.move_duration = 1.0
                self.start_time = time.time()
            
            elif self.current_stage == STAGE_RELEASE_CONTROL:
                # Control released - now switch FSM from 500 to 801
                print("✅ Arm control released")
                
                if self.loco_client and self.put_down_requested:
                    print("🔧 Switching FSM from 500 (balance) to 801 (walking)...")
                    try:
                        # Stop all movement first
                        self.loco_client.Move(0, 0, 0)
                        time.sleep(0.5)
                        
                        # Set FSM to 801 
                        self.loco_client.SetFsmId(801)
                        time.sleep(2)
                        
                        print("✅ Walking mode (801) active")
                    except Exception as e:
                        print(f"❌ Error setting FSM ID: {e}")
                elif not self.loco_client:
                    print("⚠️  Warning: LocoClient not available, cannot set FSM ID")
                
                self.stop()


def main():
    print("=" * 80)
    print("🤖 Unitree G1 - Left Arm Custom Sequence")
    print("=" * 80)
    print("📋 Sequence: Start → Pos1 → Pos2 → Pos3 → Hand → Pos4 → Hold")
    print("=" * 80)
    print("\n⚠️  SAFETY WARNING:")
    print("   - Ensure no obstacles near left arm")
    print("   - Robot must be in stable stance")
    print("   - Press Ctrl+C for emergency stop")
    print("=" * 80)
    
    if len(sys.argv) < 2:
        print("\nUsage: python3 arm_action.py <network_interface>")
        print("Example: python3 arm_action.py ens33")
        sys.exit(1)
        
    network_interface = sys.argv[1]
    ChannelFactoryInitialize(0, network_interface)
    print(f"\n✅ Network initialized on {network_interface}")
    
    # Create controller
    controller = LeftArmSequence(control_dt=0.02)
    controller.Init()
    
    # Subscribe to robot state
    def state_handler(msg: LowState_):
        controller.set_low_state(msg)
        
    state_sub = ChannelSubscriber("rt/lf/lowstate", LowState_)
    state_sub.Init(state_handler, 10)
    
    # Subscribe to hand state for pressure monitoring
    def hand_state_handler(msg: HandState_):
        controller.set_hand_state(msg)
    
    hand_state_sub = ChannelSubscriber("rt/dex3/left/state", HandState_)
    hand_state_sub.Init(hand_state_handler, 10)
    
    # Wait for first state
    print("⏳ Waiting for robot state...")
    max_wait = 10.0
    wait_start = time.time()
    
    while controller.low_state is None and (time.time() - wait_start) < max_wait:
        time.sleep(0.1)
    
    if controller.low_state is None:
        print("❌ Error: No robot state received after 10 seconds")
        print("   Check network connection and robot status")
        sys.exit(1)
        
    print("✅ Robot state received")
    
    # Display current position
    print("\n📍 Current left arm position:")
    for i, joint_id in enumerate(LEFT_ARM_JOINT_IDS):
        joint_names = ['ShoulderPitch', 'ShoulderRoll', 'ShoulderYaw', 'Elbow', 
                       'WristRoll', 'WristPitch', 'WristYaw']
        q = controller.low_state.motor_state[joint_id].q
        print(f"   {joint_names[i]:15s}: {q:7.3f}")
    
    # Check if running programmatically (called from script_controller)
    programmatic = '--no-confirm' in sys.argv
    
    if not programmatic:
        try:
            input("\n▶️  Press Enter to start sequence...")
        except KeyboardInterrupt:
            print("\n👋 Cancelled")
            sys.exit(0)
    else:
        print("\n🤖 Starting sequence programmatically...")
        
    # Start sequence
    controller.start_sequence()
    
    try:
        # Wait until sequence completes (including walking mode release)
        while controller.is_running:
            time.sleep(0.1)
        print("\n🎉 All done!")
        time.sleep(0.5)
        
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupt received - gracefully stopping...")
        controller.graceful_stop()
        
        # Wait for graceful stop and walking mode release to complete
        while controller.is_running:
            time.sleep(0.1)
        
        time.sleep(0.5)
        
    print("👋 Goodbye!")


if __name__ == '__main__':
    main()