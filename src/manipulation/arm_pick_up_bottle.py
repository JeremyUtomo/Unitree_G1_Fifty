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

LEFT_ARM_JOINT_IDS = [15, 16, 17, 18, 19, 20, 21]
RIGHT_ARM_JOINT_IDS = [22, 23, 24, 25, 26, 27, 28]
RIGHT_ARM_HOLD_POSITION = [0.294, -0.229, 0.018, 0.977, -0.132, 0.028, -0.012]
STARTING_POSITION = [0.256, 0.280, -0.079, 0.829, 0.005, 0.012, -0.001]
POSITION_1 = [0.844, 0.247, -0.041, -0.991, -0.305, 0, 0.017]
POSITION_2 = [-0.255, 0.221, 0.312, 0.016, 0.010, 0, -0.087]
# 4-position sequence for approach (hand closes at 3D)
POSITION_3A = [-0.243, 0.198, 0.215, 0.182, -0.148, 0.085, -0.172]
POSITION_3B = [-0.259, 0.015, 0.027, 0.183, -0.029, 0.031, 0.053]
POSITION_3C = [-0.295, -0.067, -0.171, 0.292, 0.006, -0.025, 0.279]
POSITION_3D = [-0.405, -0.111, -0.472, 0.450, -0.108, -0.113, 0.574]
# Hold position (lift with bottle)
POSITION_7 = [-0.253, 0.200, 0, -0.743, 0, 0.75, 0.081]

STAGE_START = 0
STAGE_POSITION_1 = 1
STAGE_POSITION_2 = 2
STAGE_POSITION_3A = '3a'
STAGE_POSITION_3B = '3b'
STAGE_POSITION_3C = '3c'
STAGE_POSITION_3D = '3d'
STAGE_POSITION_7 = 7
STAGE_RETURN_TO_POSITION_1 = 5
STAGE_RETURN_START = 6
STAGE_RETURN_TO_NEUTRAL = 'return_to_neutral'
STAGE_RELEASE_CONTROL = 'release_control'
STAGE_INTERRUPT_RETURN = 'interrupt_return'
STAGE_PUTDOWN_TO_POS3D = 'putdown_to_pos3d'
STAGE_PUTDOWN_TO_POS3C = 'putdown_to_pos3c'
STAGE_PUTDOWN_TO_POS3B = 'putdown_to_pos3b'
STAGE_PUTDOWN_TO_POS3A = 'putdown_to_pos3a'
STAGE_PUTDOWN_OPEN_HAND = 'putdown_open_hand'
STAGE_PUTDOWN_TO_POS2 = 'putdown_to_pos2'
STAGE_PUTDOWN_TO_POS1 = 'putdown_to_pos1'
STAGE_PUTDOWN_TO_START = 'putdown_to_start'

G1_NUM_MOTOR = 35
Kp_ARM = 40.0
Kd_ARM = 1.0


class LeftArmSequence:
    """Controls left arm through a 3-position sequence"""
    
    PRESSURE_THRESHOLD = 150000
    
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
        self.hand_closed_at_3f = False
        self.pressure_detected = False
        self.interrupt_requested = False
        self.position_4_hold_printed = False
        self.put_down_requested = False
        
        self.loco_client = None
        
    def Init(self):
        """Initialize publishers and hand controller"""
        self.lowcmd_publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.lowcmd_publisher.Init()
        
        self.hand_controller = HandController()
        self.hand_controller.init_left_hand()
        
        try:
            self.loco_client = LocoClient()
            self.loco_client.SetTimeout(10.0)
            self.loco_client.Init()
            time.sleep(0.5)
            print("LocoClient initialized for FSM control")
        except Exception as e:
            print(f"Warning: Failed to initialize LocoClient: {e}")
            self.loco_client = None
        
        print("Publishers initialized")
        
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
            print("Error: No robot state available")
            return False
            
        self.current_stage = STAGE_START
        self.target_positions = STARTING_POSITION.copy()
        self.move_duration = 0.5
        self.start_time = time.time()
        self.is_running = True
        
        self.control_thread = RecurrentThread(
            interval=self.control_dt,
            target=self._control_loop,
            name="left_arm_control"
        )
        self.control_thread.Start()
        print("Sequence started")
        return True
        
    def stop(self):
        """Stop the sequence immediately"""
        self.is_running = False
        if self.control_thread:
            try:
                self.control_thread.Stop()
            except AttributeError:
                # RecurrentThread might not have Stop() method
                pass
        print("Sequence stopped")
    
    def start_put_down(self):
        """Start put-down sequence from Position 7"""
        if not self.is_running:
            print("Error: Sequence not running")
            return False
        
        if self.current_stage != STAGE_POSITION_7:
            print(f"Error: Can only put down from Position 7 (currently at stage {self.current_stage})")
            return False
        
        print("Starting put-down sequence...")
        self.put_down_requested = True
        self.start_positions = POSITION_7.copy()
        self.target_positions = POSITION_3D.copy()
        self.current_stage = STAGE_PUTDOWN_TO_POS3D
        self.move_duration = 2.0
        self.start_time = time.time()
        
        return True
    
    def graceful_stop(self):
        """Gracefully return to starting position before stopping"""
        if not self.is_running:
            return
        
        self.interrupt_requested = True
        
        print("Interrupt received - returning to starting position...")
        
        if self.current_stage == STAGE_POSITION_7:
            if not self._capture_current_positions():
                self.start_positions = POSITION_7.copy()
            self.target_positions = POSITION_1.copy()
            self.current_stage = STAGE_RETURN_TO_POSITION_1
            self.move_duration = 5.0
            self.start_time = time.time()
        elif self.current_stage in [STAGE_POSITION_3A, STAGE_POSITION_3B, STAGE_POSITION_3C, 
                                     STAGE_POSITION_3D]:
            if not self._capture_current_positions():
                self.start_positions = POSITION_3A.copy()
            self.target_positions = POSITION_1.copy()
            self.current_stage = STAGE_RETURN_TO_POSITION_1
            self.move_duration = 5.0
            self.start_time = time.time()
        else:
            self._capture_current_positions()
            self.target_positions = STARTING_POSITION.copy()
            self.current_stage = STAGE_INTERRUPT_RETURN
            self.move_duration = 3.0
            self.start_time = time.time()
    
    def release_to_walking_mode(self):
        """Release arm control and return to walking mode"""
        if not self.is_running:
            self._capture_current_positions()
            self.is_running = True
            self.control_thread = RecurrentThread(
                interval=self.control_dt,
                target=self._control_loop,
                name="left_arm_release"
            )
            self.control_thread.Start()
        
        self._capture_current_positions()
        self.target_positions = STARTING_POSITION.copy()
        self.current_stage = STAGE_RETURN_TO_NEUTRAL
        self.move_duration = 3.0
        self.start_time = time.time()
        print("Returning to starting position...")
        
    def _control_loop(self):
        """Main control loop - runs at fixed frequency"""
        with self.lock:
            if self.low_state is None:
                return
            mode_machine = self.low_state.mode_machine
            
        elapsed = time.time() - self.start_time
        ratio = min(elapsed / self.move_duration, 1.0)
        for i in range(G1_NUM_MOTOR):
            self.low_cmd.motor_cmd[i].mode = 0
            self.low_cmd.motor_cmd[i].q = 0.0
            self.low_cmd.motor_cmd[i].dq = 0.0
            self.low_cmd.motor_cmd[i].kp = 0.0
            self.low_cmd.motor_cmd[i].kd = 0.0
            self.low_cmd.motor_cmd[i].tau = 0.0
            
        self.low_cmd.mode_machine = mode_machine
        self.low_cmd.mode_pr = 0
        
        if self.current_stage == STAGE_RELEASE_CONTROL:
            self.low_cmd.motor_cmd[29].q = 1.0 - ratio
        else:
            self.low_cmd.motor_cmd[29].q = 1.0
        
        for i, joint_id in enumerate(LEFT_ARM_JOINT_IDS):
            interp_q = self.start_positions[i] + (self.target_positions[i] - self.start_positions[i]) * ratio
            self.low_cmd.motor_cmd[joint_id].q = interp_q
            self.low_cmd.motor_cmd[joint_id].kp = Kp_ARM
            self.low_cmd.motor_cmd[joint_id].kd = Kd_ARM
        
        for i, joint_id in enumerate(RIGHT_ARM_JOINT_IDS):
            self.low_cmd.motor_cmd[joint_id].q = RIGHT_ARM_HOLD_POSITION[i]
            self.low_cmd.motor_cmd[joint_id].kp = Kp_ARM
            self.low_cmd.motor_cmd[joint_id].kd = Kd_ARM
            
        self.low_cmd.motor_cmd[12].q = 0.0
        self.low_cmd.motor_cmd[12].kp = 50.0
        self.low_cmd.motor_cmd[12].kd = 2.0
        
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_publisher.Write(self.low_cmd)
        
        if ratio >= 1.0:
            if self.current_stage == STAGE_START:
                self.start_positions = STARTING_POSITION.copy()
                self.target_positions = POSITION_1.copy()
                self.current_stage = STAGE_POSITION_1
                self.move_duration = 2.0
                self.start_time = time.time()
                print("Moving to Position 1...")
                
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
                    print("Opening hand...")
                    self.hand_controller.open_left_hand()
                    time.sleep(0.5)  # Wait for hand to open
                    self.hand_opened = True
                
                # Move to Position 3A (first of 6 approach positions)
                self.start_positions = POSITION_2.copy()
                self.target_positions = POSITION_3A.copy()
                self.current_stage = STAGE_POSITION_3A
                self.move_duration = 1.5
                self.start_time = time.time()
                print("Moving to Position 3A...")
            
            elif self.current_stage == STAGE_POSITION_3A:
                self.start_positions = POSITION_3A.copy()
                self.target_positions = POSITION_3B.copy()
                self.current_stage = STAGE_POSITION_3B
                self.move_duration = 1.0
                self.start_time = time.time()
                print("Moving to Position 3B...")
            
            elif self.current_stage == STAGE_POSITION_3B:
                self.start_positions = POSITION_3B.copy()
                self.target_positions = POSITION_3C.copy()
                self.current_stage = STAGE_POSITION_3C
                self.move_duration = 1.0
                self.start_time = time.time()
                print("Moving to Position 3C...")
            
            elif self.current_stage == STAGE_POSITION_3C:
                self.start_positions = POSITION_3C.copy()
                self.target_positions = POSITION_3D.copy()
                self.current_stage = STAGE_POSITION_3D
                self.move_duration = 1.0
                self.start_time = time.time()
                print("Moving to Position 3D...")
            
            elif self.current_stage == STAGE_POSITION_3D:
                # Stay at position 3D until hand has closed
                if not self.hand_closed_at_3f:  # Reusing flag name
                    # Check if interrupt was requested during transition
                    if self.interrupt_requested:
                        return  # Exit immediately, graceful_stop has already set up return path
                    
                    # Close hand after reaching final approach position (3D)
                    print("At Position 3D - closing hand until pressure detected...")
                    self.hand_controller.close_left_hand()
                    self.pressure_detected = False
                    
                    # Monitor pressure while closing
                    close_start = time.time()
                    max_close_time = 3.0
                    
                    while (time.time() - close_start) < max_close_time:
                        # Check for interrupt during pressure monitoring
                        if self.interrupt_requested:
                            print("Interrupt during hand closing - aborting")
                            return
                        
                        if self._check_pressure():
                            print("Pressure detected - holding position!")
                            self.hand_controller.hold_left_hand_position()
                            self.pressure_detected = True
                            break
                        time.sleep(0.05)  # Check at 20Hz
                    
                    if not self.pressure_detected and not self.interrupt_requested:
                        print("No pressure detected - hand fully closed")
                    
                    # Mark hand as closed so we can proceed
                    self.hand_closed_at_3f = True
                
                # Now that hand has closed, move to Position 7 (hold position)
                if self.hand_closed_at_3f and not self.interrupt_requested:
                    self.start_positions = POSITION_3D.copy()
                    self.target_positions = POSITION_7.copy()
                    self.current_stage = STAGE_POSITION_7
                    self.move_duration = 2.0
                    self.start_time = time.time()
                    print("Moving to Position 7 (hold position)...")
            
            elif self.current_stage == STAGE_POSITION_7:
                # Check if interrupt was requested during transition
                if self.interrupt_requested:
                    return  # Exit immediately, graceful_stop has already set up return path
                
                # Hold position 7 indefinitely - wait for user interrupt or put_down command
                if not self.interrupt_requested and not self.position_4_hold_printed:
                    print("Position 7 reached - holding position until Ctrl+C or put-down command...")
                    self.position_4_hold_printed = True
                # Don't transition to next stage - stay in STAGE_POSITION_7
                
            elif self.current_stage == STAGE_RETURN_TO_POSITION_1:
                # Return to starting position
                self.start_positions = POSITION_1.copy()
                self.target_positions = STARTING_POSITION.copy()
                self.current_stage = STAGE_RETURN_START
                self.move_duration = 4.0
                self.start_time = time.time()
                print("Returning to starting position...")
                
            elif self.current_stage == STAGE_RETURN_START:
                print("Returned to starting position")
                self.start_positions = STARTING_POSITION.copy()
                self.current_stage = STAGE_RETURN_TO_NEUTRAL
                self.move_duration = 3.0
                self.start_time = time.time()
            
            elif self.current_stage == STAGE_INTERRUPT_RETURN:
                print("Returned to starting position")
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
                print("Releasing arm control...")
            
            # Put-down sequence: reverse through positions 3D -> 3C -> 3B -> 3A -> 2 -> 1 -> start
            elif self.current_stage == STAGE_PUTDOWN_TO_POS3D:
                # Reached Position 3D - open hand to release bottle
                print("Position 3D reached - opening hand...")
                self.hand_controller.open_left_hand()
                time.sleep(2.5)  # Wait for hand to open
                
                # Move to Position 3C
                self.start_positions = POSITION_3D.copy()
                self.target_positions = POSITION_3C.copy()
                self.current_stage = STAGE_PUTDOWN_TO_POS3C
                self.move_duration = 1.0
                self.start_time = time.time()
                print("Moving to Position 3C...")
            
            elif self.current_stage == STAGE_PUTDOWN_TO_POS3C:
                self.start_positions = POSITION_3C.copy()
                self.target_positions = POSITION_3B.copy()
                self.current_stage = STAGE_PUTDOWN_TO_POS3B
                self.move_duration = 1.0
                self.start_time = time.time()
                print("Moving to Position 3B...")
            
            elif self.current_stage == STAGE_PUTDOWN_TO_POS3B:
                self.start_positions = POSITION_3B.copy()
                self.target_positions = POSITION_3A.copy()
                self.current_stage = STAGE_PUTDOWN_TO_POS3A
                self.move_duration = 1.0
                self.start_time = time.time()
                print("Moving to Position 3A...")
            
            elif self.current_stage == STAGE_PUTDOWN_TO_POS3A:
                # Close hand and move to Position 2
                print("Closing hand...")
                self.hand_controller.close_left_hand()
                time.sleep(2.0)  # Wait for hand to close
                
                self.start_positions = POSITION_3A.copy()
                self.target_positions = POSITION_2.copy()
                self.current_stage = STAGE_PUTDOWN_TO_POS2
                self.move_duration = 1.5
                self.start_time = time.time()
                print("Moving to Position 2...")
            
            elif self.current_stage == STAGE_PUTDOWN_TO_POS2:
                # Move to Position 1
                self.start_positions = POSITION_2.copy()
                self.target_positions = POSITION_1.copy()
                self.current_stage = STAGE_PUTDOWN_TO_POS1
                self.move_duration = 2.0
                self.start_time = time.time()
                print("Moving to Position 1...")
            
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
                print("Reached starting position - releasing arm control...")
                self.start_positions = STARTING_POSITION.copy()
                self.current_stage = STAGE_RELEASE_CONTROL
                self.move_duration = 1.0
                self.start_time = time.time()
            
            elif self.current_stage == STAGE_RELEASE_CONTROL:
                # Control released - arm sequence complete
                # Note: FSM switching is handled by integrated_controller.py
                print("Arm control released")
                print("Put-down sequence complete")
                
                self.stop()


def main():
    print("Unitree G1 - Left Arm Custom Sequence")
    print("Sequence: Start → Pos1 → Pos2 → 3A → 3B → 3C → 3D → Hand → Pos7 (Hold)")
    print("\nSAFETY WARNING:")
    print("   - Ensure no obstacles near left arm")
    print("   - Robot must be in stable stance")
    print("   - Press Ctrl+C for emergency stop")
    
    if len(sys.argv) < 2:
        print("\nUsage: python3 arm_pick_up_bottle.py <network_interface>")
        print("Example: python3 arm_pick_up_bottle.py ens33")
        sys.exit(1)
        
    network_interface = sys.argv[1]
    ChannelFactoryInitialize(0, network_interface)
    print(f"\nNetwork initialized on {network_interface}")
    
    controller = LeftArmSequence(control_dt=0.02)
    controller.Init()
    
    def state_handler(msg: LowState_):
        controller.set_low_state(msg)
        
    state_sub = ChannelSubscriber("rt/lf/lowstate", LowState_)
    state_sub.Init(state_handler, 10)
    
    def hand_state_handler(msg: HandState_):
        controller.set_hand_state(msg)
    
    hand_state_sub = ChannelSubscriber("rt/dex3/left/state", HandState_)
    hand_state_sub.Init(hand_state_handler, 10)
    
    print("Waiting for robot state...")
    max_wait = 10.0
    wait_start = time.time()
    
    while controller.low_state is None and (time.time() - wait_start) < max_wait:
        time.sleep(0.1)
    
    if controller.low_state is None:
        print("Error: No robot state received after 10 seconds")
        print("   Check network connection and robot status")
        sys.exit(1)
        
    print("Robot state received")
    
    print("\nCurrent left arm position:")
    for i, joint_id in enumerate(LEFT_ARM_JOINT_IDS):
        joint_names = ['ShoulderPitch', 'ShoulderRoll', 'ShoulderYaw', 'Elbow', 
                       'WristRoll', 'WristPitch', 'WristYaw']
        q = controller.low_state.motor_state[joint_id].q
        print(f"   {joint_names[i]:15s}: {q:7.3f}")
    
    programmatic = '--no-confirm' in sys.argv
    
    if not programmatic:
        try:
            input("\nPress Enter to start sequence...")
        except KeyboardInterrupt:
            print("\nCancelled")
            sys.exit(0)
    else:
        print("\nStarting sequence programmatically...")
    controller.start_sequence()
    
    try:
        while controller.is_running:
            time.sleep(0.1)
        print("\nAll done!")
        time.sleep(0.5)
        
    except KeyboardInterrupt:
        print("\n\nInterrupt received - gracefully stopping...")
        controller.graceful_stop()
        
        while controller.is_running:
            time.sleep(0.1)
        
        time.sleep(0.5)
        
    print("Goodbye!")


if __name__ == '__main__':
    main()