#!/usr/bin/env python3
"""
Monitor tactile pressure sensors on Unitree G1 dexterous hand.

The Dex3-1 hand has 6 fingertip pressure sensors (3x4 array each = 12 pressure points).
Sensor locations: Thumb tip, Thumb pad, Index tip, Index pad, Middle tip, Middle pad
"""

import sys
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandState_
import time

class HandPressureMonitor:
    FINGER_NAMES = [
        "Thumb Tip",   # ID 0
        "Thumb Pad",   # ID 1
        "Index Tip",   # ID 2
        "Index Pad",   # ID 3
        "Middle Tip",  # ID 4
        "Middle Pad"   # ID 5
    ]
    
    VALID_THRESHOLD = 150000  # Actual pressure detection >= 150000 (15.0 scaled)
    BASELINE_VALUE = 100000   # Baseline no-pressure reading ~100000 (10.0 scaled)
    INVALID_VALUE = 30000     # Invalid/disconnected sensor = 30000 (3.0 scaled)
    SCALE_FACTOR = 10000.0    # Scale down 100000 to 10.0 for display
    
    def __init__(self, network_interface, hand_side="left", finger_id_filter=None):
        self.network_interface = network_interface
        self.hand_side = hand_side
        self.finger_id_filter = finger_id_filter  # None = show all, or specific ID (0-5)
        self.current_state = None
        self.last_print_time = 0
        self.print_interval = 0.5  # Minimum 0.5 seconds between prints
        
        # Setup subscriber
        state_topic = f"rt/dex3/{hand_side}/state"
        self.state_sub = ChannelSubscriber(state_topic, HandState_)
        self.state_sub.Init(self._state_handler, 10)
        
        print(f"\n{'='*80}")
        if finger_id_filter is not None:
            print(f"Hand Pressure Monitor - {hand_side.upper()} Hand - {self.FINGER_NAMES[finger_id_filter]} Only")
        else:
            print(f"Hand Pressure Sensor Monitor - {hand_side.upper()} Hand - All Fingers")
        print(f"{'='*80}")
        print("\nSensor Layout (3x4 grid per fingertip):")
        print("  Pressure detected: >= 15.0 (baseline ~10.0)")
        print("  Invalid/No sensor: 3.0")
        print(f"\n{'='*80}\n")
    
    def _state_handler(self, msg: HandState_):
        """Process state updates"""
        self.current_state = msg
    
    def display_pressure(self):
        """Display pressure sensor data only when pressure is detected"""
        if self.current_state is None:
            return
        
        msg = self.current_state
        
        # Check if pressure sensors are available
        if not hasattr(msg, 'press_sensor_state') or len(msg.press_sensor_state) == 0:
            return
        
        # Check if any finger has pressure
        has_pressure = False
        pressure_data = []
        
        for finger_id, sensor in enumerate(msg.press_sensor_state):
            if finger_id >= len(self.FINGER_NAMES):
                continue
            
            # Skip if filtering and this isn't the target finger
            if self.finger_id_filter is not None and finger_id != self.finger_id_filter:
                continue
            
            data = list(sensor.pressure)
            if len(data) < 12:
                continue
            
            # Check if this finger has any significant pressure (valid reading >= 100000)
            max_val = max(data)
            if max_val >= self.VALID_THRESHOLD:
                has_pressure = True
                finger_name = self.FINGER_NAMES[finger_id]
                temps = list(sensor.temperature)
                avg_temp = sum(temps) / len(temps) if temps else 0
                pressure_data.append((finger_id, finger_name, data, avg_temp, max_val))
        
        # Only print if pressure detected and enough time has passed
        current_time = time.time()
        if has_pressure and (current_time - self.last_print_time) >= self.print_interval:
            self.last_print_time = current_time
            
            # Clear terminal using standard clear command
            import os
            os.system('clear')
            
            print(f"{'='*80}")
            print(f"🤚 PRESSURE DETECTED - {self.hand_side.upper()} Hand | Time: {current_time:.2f}")
            print(f"{'='*80}")
            
            for finger_id, finger_name, data, avg_temp, max_val in pressure_data:
                # Only show fingers with pressure
                if max_val < self.VALID_THRESHOLD:
                    continue
                
                # Scale max value for display
                scaled_max = max_val / self.SCALE_FACTOR
                
                print(f"\n  {finger_name} (ID: {finger_id}) | Temp: {avg_temp:.1f}°C | Max: {scaled_max:.2f}")
                print(f"  {'-'*60}")
                
                # Display as 3x4 grid (scaled values)
                print("  Pressure Grid (3 rows x 4 columns):")
                for row in range(3):
                    row_str = "    "
                    for col in range(4):
                        idx = row * 4 + col
                        val = data[idx]
                        scaled_val = val / self.SCALE_FACTOR
                        
                        # Format with color coding (on scaled values)
                        if scaled_val >= 20.0:
                            row_str += f"\033[91m{scaled_val:7.2f}\033[0m "  # Red for high pressure
                        elif scaled_val >= 15.0:
                            row_str += f"\033[92m{scaled_val:7.2f}\033[0m "  # Green for detected pressure
                        else:
                            row_str += f"\033[90m{scaled_val:7.2f}\033[0m "  # Gray for baseline/no contact
                    
                    print(row_str)
                
                # Show max pressure point
                max_idx = data.index(max_val)
                max_row = max_idx // 4
                max_col = max_idx % 4
                print(f"  Peak at [row={max_row}, col={max_col}]: {scaled_max:.2f}")
            
            print(f"{'='*80}\n")

def main():
    if len(sys.argv) < 3:
        print("Usage: python3 hand_pressure_monitor.py <network_interface> <left|right> [finger_id]")
        print("Example: python3 hand_pressure_monitor.py ens33 left")
        print("         python3 hand_pressure_monitor.py ens33 left 2  # Only Index Tip")
        print("\nFinger IDs:")
        print("  0 = Thumb Tip    1 = Thumb Pad")
        print("  2 = Index Tip    3 = Index Pad")
        print("  4 = Middle Tip   5 = Middle Pad")
        sys.exit(1)
    
    network_interface = sys.argv[1]
    hand_side = sys.argv[2].lower()
    
    # Optional finger ID filter
    finger_id_filter = None
    if len(sys.argv) >= 4:
        try:
            finger_id_filter = int(sys.argv[3])
            if finger_id_filter < 0 or finger_id_filter > 5:
                print("Error: finger_id must be between 0 and 5")
                sys.exit(1)
        except ValueError:
            print("Error: finger_id must be an integer (0-5)")
            sys.exit(1)
    
    if hand_side not in ["left", "right"]:
        print("Error: hand_side must be 'left' or 'right'")
        sys.exit(1)
    
    # Initialize DDS
    ChannelFactoryInitialize(0, network_interface)
    
    # Initialize monitor
    monitor = HandPressureMonitor(network_interface, hand_side, finger_id_filter)
    
    print("\n📊 Monitoring pressure sensors... (Ctrl+C to exit)")
    print("Waiting for pressure contact... (will only print when pressure detected)\n")
    
    try:
        while True:
            monitor.display_pressure()
            time.sleep(0.2)  # Update at 5Hz
    except KeyboardInterrupt:
        print("\n\nShutting down pressure monitor...")

if __name__ == "__main__":
    main()
