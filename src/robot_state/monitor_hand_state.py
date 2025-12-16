"""
Usage:
    python3 monitor_hand_state.py <network_interface> <hand>
    
Arguments:
    hand: 'left', 'right', or 'both'

Example:
    python3 monitor_hand_state.py ens33 left
    python3 monitor_hand_state.py ens33 both
"""

import sys
import os
import time
import threading

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandState_

HAND_MOTOR_NAMES = [
    "Thumb_0",
    "Thumb_1",
    "Thumb_2",
    "Middle_0",
    "Middle_1",
    "Index_0",
    "Index_1",
]


class HandStateMonitor:
    def __init__(self, monitor_left=True, monitor_right=False):
        self.left_hand_state = None
        self.right_hand_state = None
        self.monitor_left = monitor_left
        self.monitor_right = monitor_right
        self.lock = threading.Lock()
        
        if monitor_left:
            self.left_subscriber = ChannelSubscriber("rt/dex3/left/state", HandState_)
            self.left_subscriber.Init(self._left_hand_handler, 10)
            
        if monitor_right:
            self.right_subscriber = ChannelSubscriber("rt/dex3/right/state", HandState_)
            self.right_subscriber.Init(self._right_hand_handler, 10)

    def _left_hand_handler(self, msg: HandState_):
        with self.lock:
            self.left_hand_state = msg

    def _right_hand_handler(self, msg: HandState_):
        with self.lock:
            self.right_hand_state = msg

    def print_dashboard(self):
        os.system('clear' if os.name == 'posix' else 'cls')
        
        hands = []
        if self.monitor_left:
            hands.append("Left")
        if self.monitor_right:
            hands.append("Right")
        hands_str = " & ".join(hands)
        
        print(f"--- Unitree G1 Hand State Monitor ({hands_str} Hand) ---")
        print(f"Press Ctrl+C to exit\n")

        with self.lock:
            if self.monitor_left:
                print("="*90)
                print("== LEFT HAND ==")
                
                if self.left_hand_state is None:
                    print("Waiting for left hand state message...")
                else:
                    self._print_hand_state(self.left_hand_state, "Left")
            
            if self.monitor_right:
                if self.monitor_left:
                    print()
                    
                print("="*90)
                print("== RIGHT HAND ==")
                
                if self.right_hand_state is None:
                    print("Waiting for right hand state message...")
                else:
                    self._print_hand_state(self.right_hand_state, "Right")
        
        print("\n" + "="*90)

    def _print_hand_state(self, hand_state, hand_name):
        print(f"{'ID':<3} | {'Motor Name':<12} | {'Mode':<6} | {'Pos (q)':<10} | {'Vel (dq)':<10} | {'Torque (tau)':<12} | {'Temp (°C)':<10}")
        print("-"*90)
        
        for i in range(len(HAND_MOTOR_NAMES)):
            motor = hand_state.motor_state[i]
            name = HAND_MOTOR_NAMES[i]
            
            mode_raw = motor.mode
            motor_id = mode_raw & 0x0F
            motor_status = (mode_raw >> 4) & 0x07
            
            q = motor.q
            dq = motor.dq
            tau = motor.tau_est
            temp = motor.temperature[0] if isinstance(motor.temperature, list) else motor.temperature
            
            print(f"{i:<3} | {name:<12} | {motor_status:<6} | {q:<10.3f} | {dq:<10.3f} | {tau:<12.3f} | {temp:<10.1f}")


if __name__ == '__main__':
    
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} <network_interface> <hand>")
        print(f"  hand: 'left', 'right', or 'both'")
        print(f"\nExample: {sys.argv[0]} ens33 left")
        print(f"Example: {sys.argv[0]} ens33 both")
        sys.exit(1)

    hand_arg = sys.argv[2].lower()
    monitor_left = hand_arg in ['left', 'both']
    monitor_right = hand_arg in ['right', 'both']
    
    if not (monitor_left or monitor_right):
        print(f"Invalid hand argument '{sys.argv[2]}'. Must be 'left', 'right', or 'both'")
        sys.exit(1)

    ChannelFactoryInitialize(0, sys.argv[1])
    
    hands = []
    if monitor_left:
        hands.append("left")
    if monitor_right:
        hands.append("right")
    print(f"Initializing hand state monitor for {' and '.join(hands)} hand(s)...")
        
    monitor = HandStateMonitor(monitor_left, monitor_right)
    
    try:
        while True:
            monitor.print_dashboard()
            time.sleep(0.1) 
    except KeyboardInterrupt:
        print("\nExiting monitor.")
