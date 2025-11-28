#!/usr/bin/env python3
"""
Unitree G1 - Hand Damp Mode
Sets hand motors to zero torque/damping mode (relaxed state)
"""
import time
import sys
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_
from unitree_sdk2py.idl import default

# Hand motor count (7 DOF per hand)
HAND_MOTOR_COUNT = 7


def create_damp_command():
    """Create a hand command with zero gains for damp mode"""
    hand_cmd = default.unitree_hg_msg_dds__HandCmd_()
    
    for i in range(HAND_MOTOR_COUNT):
        # Set motor mode to disable/damp (mode = 0)
        hand_cmd.motor_cmd[i].mode = 0
        hand_cmd.motor_cmd[i].q = 0.0
        hand_cmd.motor_cmd[i].dq = 0.0
        hand_cmd.motor_cmd[i].tau = 0.0
        hand_cmd.motor_cmd[i].kp = 0.0
        hand_cmd.motor_cmd[i].kd = 0.0
    
    return hand_cmd


def main():
    """Damp hand motors - keeps running to maintain damp mode"""
    print("=" * 80)
    print("🤖 Unitree G1 - Hand Damp Mode")
    print("=" * 80)
    
    if len(sys.argv) < 3:
        print("\nUsage: python3 hand_damp.py <network_interface> <hand>")
        print("  hand: 'left', 'right', or 'both'")
        print("\nExample: python3 hand_damp.py ens33 left")
        print("Example: python3 hand_damp.py ens33 both")
        sys.exit(1)
    
    network_interface = sys.argv[1]
    hand_arg = sys.argv[2].lower()
    
    # Validate hand argument
    if hand_arg not in ['left', 'right', 'both']:
        print(f"❌ Invalid hand argument '{hand_arg}'. Must be 'left', 'right', or 'both'")
        sys.exit(1)
    
    # Initialize network
    ChannelFactoryInitialize(0, network_interface)
    print(f"✅ Network initialized on {network_interface}\n")
    
    # Create publishers
    left_publisher = None
    right_publisher = None
    
    if hand_arg in ['left', 'both']:
        left_publisher = ChannelPublisher("rt/dex3/left/cmd", HandCmd_)
        left_publisher.Init()
        print("✅ Left hand publisher initialized")
    
    if hand_arg in ['right', 'both']:
        right_publisher = ChannelPublisher("rt/dex3/right/cmd", HandCmd_)
        right_publisher.Init()
        print("✅ Right hand publisher initialized")
    
    # Create damp command
    damp_cmd = create_damp_command()
    
    print("\n🔧 Damping hand motors (sending commands continuously)...")
    print("   Press Ctrl+C to stop\n")
    
    try:
        # Continuously send damp commands at 50Hz
        while True:
            if left_publisher:
                left_publisher.Write(damp_cmd)
            if right_publisher:
                right_publisher.Write(damp_cmd)
            time.sleep(0.02)  # 50Hz
            
    except KeyboardInterrupt:
        print("\n\n🛑 Stopped - hand motors will return to their last commanded state")
        print("👋 Done!")


if __name__ == '__main__':
    main()
