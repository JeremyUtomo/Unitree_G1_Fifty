#!/usr/bin/env python3
"""
Unitree G1 Left Arm - Custom 3-Position Sequence
Moves through: Starting Position → Position 1 → Position 2 → Hand Open
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
from hand_controller import HandController

# Left arm joint indices
LEFT_ARM_JOINT_IDS = [15, 16, 17, 18, 19, 20, 21]

# Target positions from your data
STARTING_POSITION = [0.256, 0.280, -0.079, 0.829, 0.005, 0.012, -0.001]
POSITION_1 = [0.944, 0.269, -0.101, -0.746, 0.002, -0.238, 0.028]
POSITION_2 = [-0.348, -0.090, -0.452, 0.360, -0.152, -0.007, 0.224]

# Stage names for clarity
STAGE_START = 0
STAGE_POSITION_1 = 1
STAGE_POSITION_2 = 2
STAGE_RETURN_START = 3
STAGE_RETURN_TO_NEUTRAL = 'return_to_neutral'  # Return to neutral position
STAGE_RELEASE_CONTROL = 'release_control'      # Release arm SDK control
STAGE_INTERRUPT_RETURN = 'interrupt_return'    # Interrupt handler - return to start

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
        
    def Init(self):
        """Initialize publishers and hand controller"""
        self.lowcmd_publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.lowcmd_publisher.Init()
        
        # Initialize hand controller
        self.hand_controller = HandController()
        self.hand_controller.init_left_hand()
        
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
        self.move_duration = 3.0
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
    
    def graceful_stop(self):
        """Gracefully return to starting position before stopping"""
        if not self.is_running:
            return
        
        # Capture current position and set target to starting position
        self._capture_current_positions()
        self.target_positions = STARTING_POSITION.copy()
        self.current_stage = STAGE_INTERRUPT_RETURN
        self.move_duration = 3.0
        self.start_time = time.time()
        print("🔄 Interrupt received - returning to starting position...")
    
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
                self._capture_current_positions()
                self.target_positions = POSITION_1.copy()
                self.current_stage = STAGE_POSITION_1
                self.move_duration = 4.0
                self.start_time = time.time()
                print("▶️  Moving to Position 1...")
                
            elif self.current_stage == STAGE_POSITION_1:
                # Move to Position 2
                self._capture_current_positions()
                self.target_positions = POSITION_2.copy()
                self.current_stage = STAGE_POSITION_2
                self.move_duration = 4.0
                self.start_time = time.time()
                print("▶️  Moving to Position 2...")
                
            elif self.current_stage == STAGE_POSITION_2:
                # Open and close hand after position 2
                if not self.hand_opened:
                    print("🖐️  Opening hand...")
                    self.hand_controller.open_left_hand()
                    time.sleep(2.5)  # Wait for hand to open
                    
                    print("🤏 Closing hand until pressure detected...")
                    self.hand_controller.close_left_hand()
                    self.pressure_detected = False
                    
                    # Monitor pressure while closing
                    close_start = time.time()
                    max_close_time = 3.0
                    
                    while (time.time() - close_start) < max_close_time:
                        if self._check_pressure():
                            print("✓ Pressure detected - holding position!")
                            self.hand_controller.hold_left_hand_position()
                            self.pressure_detected = True
                            break
                        time.sleep(0.05)  # Check at 20Hz
                    
                    if not self.pressure_detected:
                        print("⚠️  No pressure detected - hand fully closed")
                    
                    time.sleep(1.0)  # Hold grasp briefly
                    self.hand_opened = True
                
                # Return to starting position
                self._capture_current_positions()
                self.target_positions = STARTING_POSITION.copy()
                self.current_stage = STAGE_RETURN_START
                self.move_duration = 4.0
                self.start_time = time.time()
                print("▶️  Returning to starting position...")
                
            elif self.current_stage == STAGE_RETURN_START:
                # Sequence complete - return to neutral and release control
                print("✅ Sequence complete!")
                self._capture_current_positions()
                self.current_stage = STAGE_RETURN_TO_NEUTRAL
                self.move_duration = 3.0
                self.start_time = time.time()
                print("🔄 Returning to neutral position...")
            
            elif self.current_stage == STAGE_RETURN_TO_NEUTRAL:
                # Neutral position reached - now release arm control
                self._capture_current_positions()
                self.current_stage = STAGE_RELEASE_CONTROL
                self.move_duration = 1.0
                self.start_time = time.time()
                print("🔓 Releasing arm control...")
            
            elif self.current_stage == STAGE_RELEASE_CONTROL:
                # Control released - walking mode restored
                print("✅ Walking mode restored")
                self.stop()
            
            elif self.current_stage == STAGE_INTERRUPT_RETURN:
                # Interrupt return complete - now return to neutral and release
                print("✅ Returned to starting position")
                self._capture_current_positions()
                self.current_stage = STAGE_RETURN_TO_NEUTRAL
                self.move_duration = 3.0
                self.start_time = time.time()


def main():
    print("=" * 80)
    print("🤖 Unitree G1 - Left Arm Custom Sequence")
    print("=" * 80)
    print("📋 Sequence: Start → Position 1 → Position 2 → Start")
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
    
    try:
        input("\n▶️  Press Enter to start sequence...")
    except KeyboardInterrupt:
        print("\n👋 Cancelled")
        sys.exit(0)
        
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