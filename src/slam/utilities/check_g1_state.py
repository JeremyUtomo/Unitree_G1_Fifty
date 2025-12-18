#!/usr/bin/env python3
"""
Check G1 robot state and connection.
Helps diagnose why Move commands might not work.

Usage:
    python3.10 check_g1_state.py <network_interface>
    python3.10 check_g1_state.py enp49s0
"""

import sys
import time
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

def main():
    if len(sys.argv) < 2:
        print("Usage: python3.10 check_g1_state.py <network_interface>")
        print("Example: python3.10 check_g1_state.py enp49s0")
        sys.exit(1)
    
    network_interface = sys.argv[1]
    
    print("=" * 70)
    print("G1 Robot State Check")
    print("=" * 70)
    print(f"Network interface: {network_interface}\n")
    
    # Initialize SDK
    print("Initializing SDK...")
    try:
        ChannelFactoryInitialize(0, network_interface)
        print("SDK initialized\n")
    except Exception as e:
        print(f"Failed to initialize SDK: {e}")
        sys.exit(1)
    
    # Initialize LocoClient
    print("Initializing LocoClient...")
    try:
        loco_client = LocoClient()
        loco_client.SetTimeout(10.0)
        loco_client.Init()
        print("LocoClient initialized\n")
    except Exception as e:
        print(f"Failed to initialize LocoClient: {e}")
        sys.exit(1)
    
    # Try to get robot state
    print("Checking robot state...")
    print("-" * 70)
    
    # Try different state queries
    try:
        fsm_id = loco_client.GetFsmId()
        print(f"FSM ID: {fsm_id}")
    except Exception as e:
        print(f"Could not get FSM ID: {e}")
    
    try:
        fsm_mode = loco_client.GetFsmMode()
        print(f"FSM Mode: {fsm_mode}")
    except Exception as e:
        print(f"Could not get FSM Mode: {e}")
    
    try:
        balance_mode = loco_client.GetBalanceMode()
        print(f"Balance Mode: {balance_mode}")
    except Exception as e:
        print(f"Could not get Balance Mode: {e}")
    
    try:
        stand_height = loco_client.GetStandHeight()
        print(f"Stand Height: {stand_height}")
    except Exception as e:
        print(f"Could not get Stand Height: {e}")
    
    try:
        swing_height = loco_client.GetSwingHeight()
        print(f"Swing Height: {swing_height}")
    except Exception as e:
        print(f"Could not get Swing Height: {e}")
    
    print("-" * 70)
    
    # Test basic commands
    print("\nDo you want to test basic movement? (y/n): ", end='')
    response = input().strip().lower()
    
    if response == 'y':
        print("\nTesting movement commands...")
        print("Make sure robot has clear space!\n")
        
        print("1. Calling Start()...")
        try:
            loco_client.Start()
            print("   Start() successful")
            time.sleep(1)
        except Exception as e:
            print(f"   Start() failed: {e}")
        
        print("\n2. Sending Move(0.2, 0, 0) - slow forward...")
        try:
            loco_client.Move(0.2, 0, 0)
            print("   Move() command sent")
            print("   Watch the robot for 3 seconds...")
            time.sleep(3)
        except Exception as e:
            print(f"   Move() failed: {e}")
        
        print("\n3. Stopping movement...")
        try:
            loco_client.StopMove()
            print("   StopMove() successful")
            time.sleep(0.5)
        except Exception as e:
            print(f"   StopMove() failed: {e}")
        
        print("\nDid the robot move?")
    
    print("\n" + "=" * 70)
    print("Check complete!")
    print("=" * 70)

if __name__ == '__main__':
    main()
