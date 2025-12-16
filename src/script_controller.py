import subprocess
import sys
import time
import signal
import os
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent / "manipulation"))

from arm_pick_up_bottle import LeftArmSequence
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, HandState_


class ScriptController:
    """Orchestrates the full pick and place sequence"""
    
    def __init__(self, network_interface='eth0', client_ip='192.168.123.222'):
        self.network_interface = network_interface
        self.client_ip = client_ip
        self.auto_center_process = None
        self.arm_controller = None
        
    def start_auto_center(self):
        """Start auto_center_bottle.py and monitor for full alignment"""
        print("PHASE 1: Auto-centering bottle")
        
        cmd = [
            sys.executable,
            "auto_center_bottle.py",
            "--client-ip", self.client_ip,
            "--network-interface", self.network_interface
        ]
        
        # Start process with unbuffered output
        self.auto_center_process = subprocess.Popen(
            cmd,
            cwd=Path(__file__).parent,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            universal_newlines=True
        )
        
        print(f"Auto-center process started (PID: {self.auto_center_process.pid})")
        print("Auto-center output:")
        
        alignment_detected = False
        
        try:
            for line in self.auto_center_process.stdout:
                sys.stdout.write(line)
                sys.stdout.flush()
                
                if "BOTTLE FULLY ALIGNED" in line and "✓" in line:
                    if not alignment_detected:
                        print("Full alignment detected - waiting 0.1 seconds to confirm...")
                        time.sleep(0.1)
                        alignment_detected = True
                        break
                        
        except KeyboardInterrupt:
            print("\nAuto-centering interrupted by user")
            self.stop_auto_center()
            return False
            
        self.stop_auto_center()
        
        if alignment_detected:
            print("Phase 1 complete - bottle aligned!")
            return True
        else:
            print("Auto-centering ended without full alignment")
            return False
    
    def stop_auto_center(self):
        """Stop the auto-center process"""
        if self.auto_center_process and self.auto_center_process.poll() is None:
            print("\nStopping auto-center process...")
            self.auto_center_process.send_signal(signal.SIGINT)
            
            # Wait for graceful shutdown
            try:
                self.auto_center_process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                print("Force killing auto-center process...")
                self.auto_center_process.kill()
                self.auto_center_process.wait()
            
            print("Auto-center stopped")
    
    def start_arm_pickup(self):
        """Start arm pickup sequence programmatically"""
        print("PHASE 2: Executing pickup sequence")
        
        ChannelFactoryInitialize(0, self.network_interface)
        print(f"Network initialized on {self.network_interface}")
        
        self.arm_controller = LeftArmSequence(control_dt=0.02)
        self.arm_controller.Init()
        
        def state_handler(msg: LowState_):
            self.arm_controller.set_low_state(msg)
        
        state_sub = ChannelSubscriber("rt/lf/lowstate", LowState_)
        state_sub.Init(state_handler, 10)
        
        def hand_state_handler(msg: HandState_):
            self.arm_controller.set_hand_state(msg)
        
        hand_state_sub = ChannelSubscriber("rt/dex3/left/state", HandState_)
        hand_state_sub.Init(hand_state_handler, 10)
        
        print("Waiting for robot state...")
        max_wait = 10.0
        wait_start = time.time()
        
        while self.arm_controller.low_state is None and (time.time() - wait_start) < max_wait:
            time.sleep(0.1)
        
        if self.arm_controller.low_state is None:
            print("Error: No robot state received")
            return False
        
        print("Robot state received")
        
        print("Starting pickup sequence...")
        if not self.arm_controller.start_sequence():
            print("Failed to start sequence")
            return False
        
        print("Waiting for Position 4...")
        while self.arm_controller.is_running:
            if self.arm_controller.current_stage == 4 and self.arm_controller.position_4_hold_printed:
                print("Position 4 reached and holding!")
                break
            time.sleep(0.1)
        
        print("Phase 2 complete - bottle grasped at Position 4!")
        return True
    
    def execute_put_down(self):
        """Execute put-down sequence"""
        print("PHASE 3: Executing put-down sequence")
        
        if not self.arm_controller or not self.arm_controller.is_running:
            print("Error: Arm controller not active")
            return False
        
        print("Starting put-down sequence...")
        
        self.arm_controller.start_put_down()
        
        while self.arm_controller.is_running:
            time.sleep(0.1)
        
        print("Phase 3 complete - put-down sequence finished!")
        return True
    
    def run_full_sequence(self):
        """Run the complete pick and place sequence"""
        print("UNITREE G1 - AUTONOMOUS BOTTLE PICK AND PLACE")
        print("Full Sequence:")
        print("   1. Auto-center bottle")
        print("   2. Pick up bottle")
        print("   3. Put down bottle")
        print("   4. Return to walking mode")
        
        try:
            if not self.start_auto_center():
                print("\nFailed at Phase 1: Auto-centering")
                return False
            
            print("\nWaiting 0.5 seconds before arm movement...")
            time.sleep(0.5)
            
            if not self.start_arm_pickup():
                print("\nFailed at Phase 2: Pickup")
                return False
            
            print("\nHolding at Position 4 for 3 seconds...")
            time.sleep(3.0)
            
            if not self.execute_put_down():
                print("\nFailed at Phase 3: Put-down")
                return False
            
            print("\nFULL SEQUENCE COMPLETE!")
            return True
            
        except KeyboardInterrupt:
            print("\n\nSequence interrupted by user")
            self.cleanup()
            return False
        except Exception as e:
            print(f"\nError during sequence: {e}")
            import traceback
            traceback.print_exc()
            self.cleanup()
            return False
    
    def cleanup(self):
        """Clean up resources"""
        print("\nCleaning up...")
        
        self.stop_auto_center()
        
        if self.arm_controller and self.arm_controller.is_running:
            print("Stopping arm controller...")
            self.arm_controller.graceful_stop()
            while self.arm_controller.is_running:
                time.sleep(0.1)
        
        print("Cleanup complete")


def main():
    if len(sys.argv) < 2:
        print("\nUsage: python3 script_controller.py <network_interface> [client_ip]")
        print("Example: python3 script_controller.py eth0 192.168.123.222")
        sys.exit(1)
    
    network_interface = sys.argv[1]
    client_ip = sys.argv[2] if len(sys.argv) > 2 else "192.168.123.222"
    
    controller = ScriptController(network_interface, client_ip)
    
    try:
        success = controller.run_full_sequence()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\nGoodbye!")
        controller.cleanup()
        sys.exit(0)


if __name__ == "__main__":
    main()
