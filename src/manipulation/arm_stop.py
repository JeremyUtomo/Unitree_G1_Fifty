import time
import sys
import threading
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.idl import default
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread

LEFT_ARM_JOINT_IDS = [15, 16, 17, 18, 19, 20, 21]
RIGHT_ARM_JOINT_IDS = [22, 23, 24, 25, 26, 27, 28]

NEUTRAL_LEFT = [0.256, 0.280, -0.079, 0.829, 0.005, 0.012, -0.001]
NEUTRAL_RIGHT = [0.253, -0.294, 0.088, 0.788, 0.003, 0.003, 0.001]

G1_NUM_MOTOR = 35
Kp_ARM = 40.0
Kd_ARM = 1.0


class ArmStopSequence:
    """Returns arms to neutral and releases SDK control"""
    
    def __init__(self, control_dt: float = 0.02):
        self.control_dt_ = control_dt
        self.move_duration_ = 3.0
        
        self.low_state = None
        self.low_cmd = default.unitree_hg_msg_dds__LowCmd_()
        self.crc = CRC()
        self.control_thread = None
        
        self.start_positions_left = [0.0] * len(LEFT_ARM_JOINT_IDS)
        self.start_positions_right = [0.0] * len(RIGHT_ARM_JOINT_IDS)
        
        self.is_stopped_event = threading.Event()
        self.is_running = False
        
        self.move_stage = 8
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
                print("Error: No low state provided")
                return False
            for i, joint_id in enumerate(LEFT_ARM_JOINT_IDS):
                self.start_positions_left[i] = initial_low_state.motor_state[joint_id].q
            for i, joint_id in enumerate(RIGHT_ARM_JOINT_IDS):
                self.start_positions_right[i] = initial_low_state.motor_state[joint_id].q
        
        self.is_running = True
        self.move_stage = 8
        self.move_duration_ = 3.0
        self.start_time = time.time()
        self.is_stopped_event.clear()
        
        self.control_thread = RecurrentThread(
            interval=self.control_dt_,
            target=self._control_loop,
            name="arm_stop_sequence"
        )
        self.control_thread.Start()
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
        
        for i in range(G1_NUM_MOTOR):
            self.low_cmd.motor_cmd[i].mode = 0
            self.low_cmd.motor_cmd[i].q = 0.0
            self.low_cmd.motor_cmd[i].dq = 0.0
            self.low_cmd.motor_cmd[i].kp = 0.0
            self.low_cmd.motor_cmd[i].kd = 0.0
            self.low_cmd.motor_cmd[i].tau = 0.0
        
        self.low_cmd.mode_pr = 0
        
        if self.move_stage == 8:
            self.low_cmd.motor_cmd[29].q = 1.0
            
            for i, joint_id in enumerate(LEFT_ARM_JOINT_IDS):
                interp_q = start_pos_left[i] + (NEUTRAL_LEFT[i] - start_pos_left[i]) * ratio
                self.low_cmd.motor_cmd[joint_id].q = interp_q
                self.low_cmd.motor_cmd[joint_id].kp = Kp_ARM
                self.low_cmd.motor_cmd[joint_id].kd = Kd_ARM
            
            for i, joint_id in enumerate(RIGHT_ARM_JOINT_IDS):
                interp_q = start_pos_right[i] + (NEUTRAL_RIGHT[i] - start_pos_right[i]) * ratio
                self.low_cmd.motor_cmd[joint_id].q = interp_q
                self.low_cmd.motor_cmd[joint_id].kp = Kp_ARM
                self.low_cmd.motor_cmd[joint_id].kd = Kd_ARM
        
        elif self.move_stage == 9:
            self.low_cmd.motor_cmd[29].q = 1.0 - ratio
            
            for i, joint_id in enumerate(LEFT_ARM_JOINT_IDS):
                self.low_cmd.motor_cmd[joint_id].q = NEUTRAL_LEFT[i]
                self.low_cmd.motor_cmd[joint_id].kp = Kp_ARM
                self.low_cmd.motor_cmd[joint_id].kd = Kd_ARM
            
            for i, joint_id in enumerate(RIGHT_ARM_JOINT_IDS):
                self.low_cmd.motor_cmd[joint_id].q = NEUTRAL_RIGHT[i]
                self.low_cmd.motor_cmd[joint_id].kp = Kp_ARM
                self.low_cmd.motor_cmd[joint_id].kd = Kd_ARM
        
        self.low_cmd.motor_cmd[12].q = 0.0
        self.low_cmd.motor_cmd[12].kp = 50.0
        self.low_cmd.motor_cmd[12].kd = 2.0
        
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_publisher_.Write(self.low_cmd)
        
        if ratio >= 1.0:
            if self.move_stage == 8:
                with self.lock:
                    if self.low_state is not None:
                        for i, joint_id in enumerate(LEFT_ARM_JOINT_IDS):
                            self.start_positions_left[i] = self.low_state.motor_state[joint_id].q
                        for i, joint_id in enumerate(RIGHT_ARM_JOINT_IDS):
                            self.start_positions_right[i] = self.low_state.motor_state[joint_id].q
                self.start_time = time.time()
                self.move_duration_ = 1.0
                self.move_stage = 9
                print("Releasing arm control...")
            
            elif self.move_stage == 9:
                print("Walking mode restored")
                self.is_stopped_event.set()
                self.is_running = False
                if self.control_thread:
                    self.control_thread.Stop()


if __name__ == '__main__':
    print("Unitree G1 - Return to Walking Mode")
    print("Returns arms to neutral position and releases SDK control")
    
    if len(sys.argv) < 2:
        print("\nUsage: python3 arm_stop.py <network_interface>")
        print("Example: python3 arm_stop.py ens33")
        sys.exit(1)
    
    network_interface = sys.argv[1]
    ChannelFactoryInitialize(0, network_interface)
    print(f"Network initialized on {network_interface}")
    
    arm_stop = ArmStopSequence(control_dt=0.02)
    arm_stop.Init()
    
    def lowstate_handler(msg: LowState_):
        arm_stop.set_low_state(msg)
    
    lowstate_sub = ChannelSubscriber("rt/lf/lowstate", LowState_)
    lowstate_sub.Init(lowstate_handler, 10)
    
    print("Waiting for robot state...")
    max_wait = 10.0
    wait_start = time.time()
    
    while arm_stop.low_state is None and (time.time() - wait_start) < max_wait:
        time.sleep(0.1)
    
    if arm_stop.low_state is None:
        print("Error: No robot state received after 10 seconds")
        sys.exit(1)
    
    print("Robot state received")
    print("\nReturning to walking mode...")
    time.sleep(0.5)
    
    arm_stop.Start(arm_stop.low_state)
    
    try:
        arm_stop.is_stopped_event.wait()
        time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nInterrupted")
    
    print("Goodbye!")


def graceful_stop(arm_sequence):

    if arm_sequence is None:
        return
    try:
        arm_sequence.Stop()
    except Exception:
        print("Warning: failed to request stop on arm_sequence")
