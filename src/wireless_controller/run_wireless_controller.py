#!/usr/bin/env python3
"""
General-purpose wireless controller runner.

Integrates:
- Arm control (arm_pick_up_package.py) via F1+A and F1+X
- Voice/TTS control via F1+Y
"""

import sys
import time
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

from custom_wireless_controller import WirelessController
from manipulation.arm_pick_up_package import ArmSequence
from manipulation.arm_stop import graceful_stop
from vui.voice_input import VoiceController

# Import 67.py as a module
import importlib.util
import os
spec = importlib.util.spec_from_file_location("arm67", os.path.join(os.path.dirname(__file__), "manipulation", "67.py"))
arm67_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(arm67_module)
ArmSequence67 = arm67_module.ArmSequence67


class WirelessControlApp:
    """General wireless control application with arm control and voice."""
    
    def __init__(self):
        self.wireless = WirelessController(lowstate_topic="rt/lf/lowstate")
        self.voice = VoiceController()
        self.arm = ArmSequence(control_dt=0.02, stage_duration=2.0)
        self.arm67 = ArmSequence67(control_dt=0.02, stage_duration=1.0)
        self.lowstate_sub = None
        
        # Track button press state for arm sequence
        self.first_press_done = False
        self.intro_done = False  # Track if introduction has been done
        
    def Init(self):
        # Initialize arm sequences
        self.arm.Init()
        self.arm67.Init()
        print("✅ Arm sequences initialized")
        
        # Initialize voice controller
        self.voice.Init()
        
        # Wire wireless controller callbacks
        self.wireless.on_start = self._on_start
        self.wireless.on_next = self._on_next
        self.wireless.on_stop = self._on_stop
        self.wireless.on_say_hello = self._on_say_hello
        self.wireless.Init()
        
        # Subscribe to lowstate to feed arm sequences
        self.lowstate_sub = ChannelSubscriber("rt/lf/lowstate", LowState_)
        self.lowstate_sub.Init(self._feed_arm_lowstate, 10)
        
    def _feed_arm_lowstate(self, msg: LowState_):
        """Feed lowstate to arm sequences for position tracking."""
        self.arm.set_low_state(msg)
        self.arm67.set_low_state(msg)
        
    def _on_start(self):
        """F1 + A: Start arm sequence or advance to next stage."""
        if not self.arm.is_running:
            # Start arm sequence - move to Stage 1
            if self.arm.low_state is not None:
                print("🚀 Starting arm sequence - Moving to Stage 1")
                self.arm.Start(self.arm.low_state)
                self.first_press_done = True
        elif self.arm.move_stage == 1 and self.first_press_done:
            # Second press at Stage 1: trigger full sequence
            print("▶️  Triggering full sequence (Stages 2-7 with auto-stop)")
            self.arm.MoveToNextStage()
            self.first_press_done = False
        
    def _on_next(self):
        """F1 + A (while running): handled in on_start."""
        pass
        
    def _on_stop(self):
        """F1 + X: Stop arm sequences."""
        if self.arm67.is_running:
            print("🛑 Emergency stop requested - Stopping 67 sequence")
            self.arm67.Stop()
            time.sleep(0.2)
            if self.arm67.ReturnToStart():
                print("⏳ Returning to start position...")
        elif self.arm.is_running:
            print("🛑 Emergency stop requested - Stopping arm sequence")
            graceful_stop(self.arm)
        else:
            print("⚠️  No arm sequence running")
    
    def _on_say_hello(self):
        """F1 + Y: Say hello and start 67 sequence."""
        print("🔊 F1 + Y pressed: Saying hello and starting sequence...")
        self.voice.introduce()
        
        # Start 67 sequence immediately after introduction
        if not self.arm67.is_running and self.arm67.low_state is not None:
            print("🚀 Starting 67 alternating sequence...")
            self.arm67.Start(self.arm67.low_state)
        else:
            print("⚠️  67 sequence already running or no state available")

if __name__ == '__main__':
    print("="*80)
    print("🤖 Unitree G1 - Wireless Controller Hub")
    print("="*80)
    print("📋 Controls:")
    print("   F1 + A : Arm Control (Start → Stage 1, press again → Full Sequence)")
    print("   F1 + X : Stop Arm Sequence (Emergency Stop)")
    print("   F1 + Y : Say Hello (TTS) + Start 67 Alternating Sequence")
    print("="*80)
    print("\n⚠️  SAFETY WARNING:")
    print("   - Ensure no obstacles around the robot")
    print("   - Robot must be in balancing/walking mode")
    print("   - Be ready for emergency stop")
    print("="*80)

    # Get network interface from command line argument
    if len(sys.argv) < 2:
        print("\nUsage: python3 run_wireless_controller.py <network_interface>")
        print("Example: python3 run_wireless_controller.py eth0")
        sys.exit(1)
    
    network_interface = sys.argv[1]
    ChannelFactoryInitialize(0, network_interface)
    
    print(f"\n✅ Initialized with network interface: {network_interface}")
    print("🎮 Wireless controller active - waiting for commands...\n")

    app = WirelessControlApp()
    app.Init()

    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n\n" + "="*80)
        print("🛑 Shutting down...")
        
        # Stop arm control if running
        if app.arm.is_running:
            print("   Stopping arm control...")
            graceful_stop(app.arm)
            app.arm.is_stopped_event.wait(timeout=10.0)
        
        print("="*80)
        print("👋 Goodbye!")
        sys.exit(0)
