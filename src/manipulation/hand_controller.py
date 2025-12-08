#!/usr/bin/env python3
"""
Unitree G1 - Dexterous Hand Controller
Provides functions for controlling the left and right hands
"""
import time
import sys
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_
from unitree_sdk2py.idl import default

# Hand motor count (7 DOF per hand)
HAND_MOTOR_COUNT = 7

# Hand positions (calibrated from actual robot state)
# Order: thumb_0, thumb_1, thumb_2, middle_0, middle_1, index_0, index_1
LEFT_HAND_OPEN = [0.645, -0.682, -0.083, -0.112, -0.038, -0.064, -0.058]
LEFT_HAND_CLOSE = [0.645, 0.987, 1.501, -1.586, -1.758, -1.621, -1.808]
RIGHT_HAND_OPEN = [-0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5]
RIGHT_HAND_CLOSE = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]

# Control gains (lower values = slower movement)
KP_HAND = 0.8
KD_HAND = 0.15


class HandController:
    """Controls Unitree G1 dexterous hands"""
    
    def __init__(self):
        self.left_hand_publisher = None
        self.right_hand_publisher = None
        self.left_hand_state_sub = None
        self.current_left_positions = [0.0] * HAND_MOTOR_COUNT
        self.current_right_positions = [0.0] * HAND_MOTOR_COUNT
    
    def init_left_hand(self):
        """Initialize left hand publisher and subscriber"""
        self.left_hand_publisher = ChannelPublisher("rt/dex3/left/cmd", HandCmd_)
        self.left_hand_publisher.Init()
        
        # Subscribe to hand state to track current positions
        def left_state_handler(msg: HandState_):
            for i in range(min(HAND_MOTOR_COUNT, len(msg.motor_state))):
                self.current_left_positions[i] = msg.motor_state[i].q
        
        self.left_hand_state_sub = ChannelSubscriber("rt/dex3/left/state", HandState_)
        self.left_hand_state_sub.Init(left_state_handler, 10)
        
        # Wait for first state update
        print("⏳ Waiting for left hand state...")
        wait_start = time.time()
        while all(p == 0.0 for p in self.current_left_positions) and (time.time() - wait_start) < 2.0:
            time.sleep(0.1)
        
        print("✅ Left hand initialized")
        print(f"   Current positions: {[f'{p:.3f}' for p in self.current_left_positions]}")
    
    def init_right_hand(self):
        """Initialize right hand publisher"""
        self.right_hand_publisher = ChannelPublisher("rt/dex3/right/cmd", HandCmd_)
        self.right_hand_publisher.Init()
        print("✅ Right hand publisher initialized")
    
    def _send_hand_command(self, publisher, positions, kp=KP_HAND, kd=KD_HAND):
        """
        Send command to hand
        
        Args:
            publisher: Hand command publisher
            positions: List of 7 target positions for each motor
            kp: Position gain
            kd: Damping gain
        """
        if publisher is None:
            print("❌ Error: Hand publisher not initialized")
            return False
        
        hand_cmd = default.unitree_hg_msg_dds__HandCmd_()
        
        for i in range(HAND_MOTOR_COUNT):
            # Set motor mode
            mode = 0
            mode |= (i & 0x0F)           # Motor ID (lower 4 bits)
            mode |= (0x01 & 0x07) << 4   # Status = 0x01 (FOC mode)
            mode |= (0x01 & 0x01) << 7   # Timeout enabled
            
            hand_cmd.motor_cmd[i].mode = mode
            hand_cmd.motor_cmd[i].q = positions[i]
            hand_cmd.motor_cmd[i].dq = 0.0
            hand_cmd.motor_cmd[i].tau = 0.0
            hand_cmd.motor_cmd[i].kp = kp
            hand_cmd.motor_cmd[i].kd = kd
        
        publisher.Write(hand_cmd)
        return True
    
    def open_left_hand(self):
        """Open the left hand"""
        if self._send_hand_command(self.left_hand_publisher, LEFT_HAND_OPEN):
            print("✋ Left hand opening...")
            return True
        return False
    
    def close_left_hand(self):
        """Close the left hand"""
        if self._send_hand_command(self.left_hand_publisher, LEFT_HAND_CLOSE):
            print("✊ Left hand closing...")
            return True
        return False
    
    def open_right_hand(self):
        """Open the right hand"""
        if self._send_hand_command(self.right_hand_publisher, RIGHT_HAND_OPEN):
            print("✋ Right hand opening...")
            return True
        return False
    
    def close_right_hand(self):
        """Close the right hand"""
        if self._send_hand_command(self.right_hand_publisher, RIGHT_HAND_CLOSE):
            print("✊ Right hand closing...")
            return True
        return False
    
    def set_left_hand_position(self, positions, kp=KP_HAND, kd=KD_HAND):
        """
        Set custom left hand position
        
        Args:
            positions: List of 7 target positions [thumb_0, thumb_1, thumb_2, middle_0, middle_1, index_0, index_1]
            kp: Position gain (default: 1.5)
            kd: Damping gain (default: 0.1)
        """
        if len(positions) != HAND_MOTOR_COUNT:
            print(f"❌ Error: Expected {HAND_MOTOR_COUNT} positions, got {len(positions)}")
            return False
        return self._send_hand_command(self.left_hand_publisher, positions, kp, kd)
    
    def set_right_hand_position(self, positions, kp=KP_HAND, kd=KD_HAND):
        """
        Set custom right hand position
        
        Args:
            positions: List of 7 target positions [thumb_0, thumb_1, thumb_2, middle_0, middle_1, index_0, index_1]
            kp: Position gain (default: 1.5)
            kd: Damping gain (default: 0.1)
        """
        if len(positions) != HAND_MOTOR_COUNT:
            print(f"❌ Error: Expected {HAND_MOTOR_COUNT} positions, got {len(positions)}")
            return False
        return self._send_hand_command(self.right_hand_publisher, positions, kp, kd)
    
    def hold_left_hand_position(self):
        """Hold current left hand position with active gains"""
        # Use current positions with normal gains to hold position
        if self._send_hand_command(self.left_hand_publisher, self.current_left_positions, kp=KP_HAND, kd=KD_HAND):
            print("🤲 Left hand holding position")
            return True
        return False
    
    def stop_left_hand(self):
        """Stop left hand motors"""
        zero_positions = [0.0] * HAND_MOTOR_COUNT
        if self._send_hand_command(self.left_hand_publisher, zero_positions, kp=0.0, kd=0.0):
            print("🛑 Left hand stopped")
            return True
        return False
    
    def stop_right_hand(self):
        """Stop right hand motors"""
        zero_positions = [0.0] * HAND_MOTOR_COUNT
        if self._send_hand_command(self.right_hand_publisher, zero_positions, kp=0.0, kd=0.0):
            print("🛑 Right hand stopped")
            return True
        return False


def main():
    """Demo script for hand controller"""
    print("=" * 80)
    print("🤖 Unitree G1 - Hand Controller Demo")
    print("=" * 80)
    print("This demonstrates opening and closing the left hand")
    print("=" * 80)
    
    if len(sys.argv) < 2:
        print("\nUsage: python3 hand_controller.py <network_interface>")
        print("Example: python3 hand_controller.py ens33")
        sys.exit(1)
    
    network_interface = sys.argv[1]
    ChannelFactoryInitialize(0, network_interface)
    print(f"✅ Network initialized on {network_interface}\n")
    
    # Create controller
    controller = HandController()
    controller.init_left_hand()
    
    try:
        # Demo sequence
        print("\n1️⃣  Opening left hand...")
        controller.open_left_hand()
        time.sleep(2.0)
        
        print("\n2️⃣  Closing left hand...")
        controller.close_left_hand()
        time.sleep(2.0)
        
        print("\n3️⃣  Opening left hand again...")
        controller.open_left_hand()
        time.sleep(2.0)
        
        print("\n✅ Demo complete!")
        
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted - stopping hand...")
        controller.stop_left_hand()
    
    print("👋 Goodbye!")


if __name__ == '__main__':
    main()
