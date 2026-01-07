#!/usr/bin/env python3
"""
Integrated Controller for Unitree G1 - Full Pick and Place with Navigation

5-Phase Sequence:
1. Navigate to first goal (set in RViz) → when reached, proceed
2. SSH to robot and run auto_center_bottle.py → wait for alignment
3. Execute pickup sequence
4. Navigate to second goal (set in RViz) → when reached, proceed
5. Execute put-down sequence and return to FSM 801

Usage:
    python3 integrated_controller.py <network_interface>
    
Example:
    python3 integrated_controller.py enp49s0
"""

import sys
import time
import subprocess
import threading
import signal
from pathlib import Path

# ROS2 imports
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

# Unitree SDK imports
sys.path.insert(0, str(Path(__file__).parent / "manipulation"))
from arm_pick_up_bottle import LeftArmSequence
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, HandState_
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient


class IntegratedController(Node):
    """Orchestrates the full navigation + pick and place sequence"""
    
    def __init__(self, network_interface='eth0'):
        super().__init__('integrated_controller')
        self.network_interface = network_interface
        
        # Phase tracking
        self.current_phase = 0
        self.phase_complete = threading.Event()
        
        # Phase 1 & 4: Navigation
        self.nav_process = None
        self.goal_reached = False
        self.current_position = None
        self.goal_position = None
        self.goal_tolerance = 0.4
        
        # Phase 2: Auto-centering via SSH
        self.ssh_process = None
        self.bottle_aligned = False
        
        # Phase 3 & 5: Arm control (programmatic via SDK)
        self.arm_controller = None
        self.loco_client = None
        self.state_sub = None
        self.hand_state_sub = None
        
        # ROS2 subscriptions
        self.odom_sub = self.create_subscription(
            Odometry,
            '/Odometry',
            self.odom_callback,
            10
        )
        
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        self.alignment_sub = self.create_subscription(
            Bool,
            '/bottle_alignment_status',
            self.alignment_callback,
            10
        )
        
        print(f"✓ Integrated controller initialized")
        print(f"  Network interface: {network_interface}")
    
    def odom_callback(self, msg):
        """Track current robot position"""
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
    
    def goal_callback(self, msg):
        """Receive goal from RViz 2D Goal Pose"""
        self.goal_position = (
            msg.pose.position.x,
            msg.pose.position.y
        )
        
        if self.current_phase == 1:
            print(f"\n[Phase 1] Goal received: ({self.goal_position[0]:.2f}, {self.goal_position[1]:.2f})")
        elif self.current_phase == 4:
            print(f"\n[Phase 4] Goal received: ({self.goal_position[0]:.2f}, {self.goal_position[1]:.2f})")
    
    def alignment_callback(self, msg):
        """Receive bottle alignment status"""
        if msg.data and not self.bottle_aligned:
            self.bottle_aligned = True
            print(f"\n" + "="*60)
            print("✓ RECEIVED ALIGNMENT TOPIC FROM ROBOT!")
            print("="*60)
            print(f"[Phase 2] ✓ Bottle alignment confirmed!")
            self.phase_complete.set()
    
    def check_goal_reached(self):
        """Check if robot reached the goal"""
        if self.current_position is None or self.goal_position is None:
            return False
        
        dx = self.current_position[0] - self.goal_position[0]
        dy = self.current_position[1] - self.goal_position[1]
        distance = (dx**2 + dy**2)**0.5
        
        return distance < self.goal_tolerance
    
    # ========== PHASE 1: Navigate to First Goal ==========
    
    def phase1_navigate_to_first_goal(self):
        """Phase 1: Start navigation and wait for user to set goal in RViz"""
        print("\n" + "="*60)
        print("PHASE 1: Navigate to First Goal")
        print("="*60)
        
        # Start RViz navigation node
        print("Starting RViz navigation with obstacle avoidance...")
        nav_script = Path(__file__).parent / "slam" / "navigation" / "rviz_navigation_obstacle_avoidance.py"
        
        cmd = [
            sys.executable,
            str(nav_script),
            self.network_interface,
            "--speed", "0.3"
        ]
        
        self.nav_process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1
        )
        
        print(f"✓ Navigation node started (PID: {self.nav_process.pid})")
        print("\nWaiting for goal in RViz...")
        print("  → Open RViz2")
        print("  → Click '2D Goal Pose' button")
        print("  → Click and drag on map to set goal\n")
        
        # Wait for goal to be set
        while self.goal_position is None and self.nav_process.poll() is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        if self.nav_process.poll() is not None:
            print("✗ Navigation node terminated unexpectedly")
            return False
        
        print("✓ Goal received, navigating...")
        
        # Monitor navigation progress
        while not self.check_goal_reached() and self.nav_process.poll() is None:
            if self.current_position:
                dx = self.goal_position[0] - self.current_position[0]
                dy = self.goal_position[1] - self.current_position[1]
                distance = (dx**2 + dy**2)**0.5
                print(f"Distance to goal: {distance:.2f}m        ", end='\r')
            
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        if self.nav_process.poll() is not None:
            print("\n✗ Navigation node terminated unexpectedly")
            return False
        
        print(f"\n✓ First goal reached!")
        print("Phase 1 complete\n")
        return True
    
    # ========== PHASE 2: Auto-Center Bottle via SSH ==========
    
    def phase2_auto_center_bottle(self):
        """Phase 2: SSH to robot and run auto_center_bottle.py"""
        print("\n" + "="*60)
        print("PHASE 2: Auto-Center Bottle (via SSH)")
        print("="*60)
        
        # Note: auto_center_bottle.py will switch to FSM 500 itself
        # when table edge is detected
        
        robot_host = "unitree@192.168.123.164"
        robot_script = "/home/unitree/Unitree_G1_Fifty/src/center_bottle/auto_center_bottle.py"
        robot_network_interface = "eth0"  # Robot uses eth0
        
        print(f"\nConnecting to robot via SSH: {robot_host}")
        print(f"Running: {robot_script} --network-interface {robot_network_interface}\n")
        
        # SSH command (requires SSH keys to be set up)
        ssh_cmd = [
            "ssh",
            "-o", "StrictHostKeyChecking=no",
            robot_host,
            f"cd /home/unitree/Unitree_G1_Fifty && python3 {robot_script} --network-interface {robot_network_interface}"
        ]
        
        self.ssh_process = subprocess.Popen(
            ssh_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1
        )
        
        print(f"✓ SSH process started (PID: {self.ssh_process.pid})")
        print("=" * 60)
        print("SSH OUTPUT FROM ROBOT:")
        print("=" * 60)
        
        # Monitor SSH output for alignment message
        self.bottle_aligned = False
        timeout = 120.0  # 2 minutes timeout
        start_time = time.time()
        
        print("\n" + "="*60)
        print("Monitoring for bottle alignment...")
        print("Looking for: 'BOTTLE_ALIGNED' in SSH output")
        print("="*60 + "\n")
        
        try:
            for line in self.ssh_process.stdout:
                # Print with prefix to distinguish from local output
                print(f"[ROBOT] {line}", end='')
                
                # Check for alignment confirmation message
                if "BOTTLE_ALIGNED" in line:
                    self.bottle_aligned = True
                    print("\n[Controller] ✓ Alignment detected from robot!")
                    break
                
                # Check for timeout
                if time.time() - start_time > timeout:
                    print("\n" + "="*60)
                    print("✗ Timeout waiting for bottle alignment (120 seconds)")
                    print("="*60)
                    self.stop_ssh_process()
                    return False
            
            # If loop exits naturally (EOF from stdout), process has terminated
            if not self.bottle_aligned:
                print("\n" + "="*60)
                print("✗ Auto-center process terminated before alignment")
                print(f"   Process exit code: {self.ssh_process.poll()}")
                print("="*60)
                self.stop_ssh_process()
                return False
                    
        except Exception as e:
            print(f"\n[Controller] Error reading SSH output: {e}")
            self.stop_ssh_process()
            return False
        
        print("\n" + "="*60)
        print("✓ Bottle aligned!")
        print("="*60)
        
        # Keep SSH process running for streaming, but continue to next phase
        print("Phase 2 complete (auto-center still running for camera stream)\n")
        return True
    
    # ========== PHASE 3: Pickup Sequence ==========
    
    def phase3_pickup_bottle(self):
        """Phase 3: Execute arm pickup sequence programmatically"""
        print("\n" + "="*60)
        print("PHASE 3: Pickup Bottle")
        print("="*60)
        print(f"✓ SUCCESSFULLY TRANSITIONED TO PHASE 3!")
        print(f"   Alignment was confirmed and Phase 2 completed successfully!")
        print("="*60)
        
        # Initialize SDK if not already done
        if self.arm_controller is None:
            print(f"Initializing SDK on {self.network_interface}...")
            ChannelFactoryInitialize(0, self.network_interface)
            print(f"✓ Network initialized on {self.network_interface}")
            
            self.arm_controller = LeftArmSequence(control_dt=0.02)
            self.arm_controller.Init()
            
            # Set up state subscribers
            def state_handler(msg: LowState_):
                self.arm_controller.set_low_state(msg)
            
            self.state_sub = ChannelSubscriber("rt/lf/lowstate", LowState_)
            self.state_sub.Init(state_handler, 10)
            
            def hand_state_handler(msg: HandState_):
                self.arm_controller.set_hand_state(msg)
            
            self.hand_state_sub = ChannelSubscriber("rt/dex3/left/state", HandState_)
            self.hand_state_sub.Init(hand_state_handler, 10)
            
            print("Waiting for robot state...")
            max_wait = 10.0
            wait_start = time.time()
            
            while self.arm_controller.low_state is None and (time.time() - wait_start) < max_wait:
                time.sleep(0.1)
            
            if self.arm_controller.low_state is None:
                print("✗ Error: No robot state received")
                return False
            
            print("✓ Robot state received")
        
        print("=" * 60)
        print("ARM PICKUP SEQUENCE:")
        print("=" * 60)
        
        print("\n[ARM] Starting pickup sequence...")
        if not self.arm_controller.start_sequence():
            print("✗ Failed to start sequence")
            return False
        
        print("[ARM] Waiting for Position 4...")
        
        # Monitor arm controller status
        last_stage = -1
        while self.arm_controller.is_running:
            # Print stage changes
            if self.arm_controller.current_stage != last_stage:
                stage_names = {
                    0: "Starting Position",
                    1: "Position 1",
                    2: "Position 2 (Opening Hand)",
                    3: "Position 3 (Closing Hand)",
                    4: "Position 4 (Holding Bottle)"
                }
                if self.arm_controller.current_stage in stage_names:
                    print(f"[ARM] Stage: {stage_names[self.arm_controller.current_stage]}")
                last_stage = self.arm_controller.current_stage
            
            if self.arm_controller.current_stage == 4 and self.arm_controller.position_4_hold_printed:
                print("\n[Controller] ✓ Position 4 reached and holding!")
                break
            time.sleep(0.1)
        
        print("=" * 60)
        
        print("\n" + "="*60)
        print("✓ Bottle grasped at Position 4!")
        print("Phase 3 complete")
        print("="*60 + "\n")
        return True
    
    # ========== PHASE 4: Navigate to Second Goal ==========
    
    def phase4_navigate_to_second_goal(self):
        """Phase 4: Wait for user to set second goal in RViz and navigate"""
        print("\n" + "="*60)
        print("PHASE 4: Navigate to Second Goal")
        print("="*60)
        
        # Navigation process should still be running from Phase 1
        if self.nav_process is None or self.nav_process.poll() is not None:
            print("✗ Navigation node not running")
            return False
        
        print("Navigation node already running")
        print("\nWaiting for second goal in RViz...")
        print("  → Click '2D Goal Pose' button")
        print("  → Click and drag on map to set goal\n")
        
        # Reset goal tracking
        self.goal_position = None
        
        # Wait for new goal to be set
        while self.goal_position is None and self.nav_process.poll() is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        if self.nav_process.poll() is not None:
            print("✗ Navigation node terminated unexpectedly")
            return False
        
        print("✓ Goal received, navigating...")
        
        # Monitor navigation progress
        while not self.check_goal_reached() and self.nav_process.poll() is None:
            if self.current_position:
                dx = self.goal_position[0] - self.current_position[0]
                dy = self.goal_position[1] - self.current_position[1]
                distance = (dx**2 + dy**2)**0.5
                print(f"Distance to goal: {distance:.2f}m        ", end='\r')
            
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        if self.nav_process.poll() is not None:
            print("\n✗ Navigation node terminated unexpectedly")
            return False
        
        print(f"\n✓ Second goal reached!")
        print("Phase 4 complete\n")
        return True
    
    # ========== PHASE 5: Put Down Bottle ==========
    
    def phase5_put_down_bottle(self):
        """Phase 5: Execute put-down sequence programmatically"""
        print("\n" + "="*60)
        print("PHASE 5: Put Down Bottle")
        print("="*60)
        
        if not self.arm_controller or not self.arm_controller.is_running:
            print("✗ Error: Arm controller not active")
            return False
        
        print("=" * 60)
        print("ARM PUT-DOWN SEQUENCE:")
        print("=" * 60)
        
        print("\n[ARM] Starting put-down sequence...")
        
        self.arm_controller.start_put_down()
        
        # Monitor put-down stages
        last_stage = self.arm_controller.current_stage
        putdown_stages = {
            'STAGE_PUTDOWN_TO_POS3': "Moving to Position 3",
            'STAGE_PUTDOWN_OPEN_HAND': "Opening Hand (Releasing Bottle)",
            'STAGE_PUTDOWN_TO_POS2': "Moving to Position 2",
            'STAGE_PUTDOWN_TO_POS1': "Moving to Position 1",
            'STAGE_PUTDOWN_TO_START': "Returning to Starting Position"
        }
        
        while self.arm_controller.is_running:
            # Monitor stage changes
            if hasattr(self.arm_controller, 'current_stage') and self.arm_controller.current_stage != last_stage:
                stage_str = str(self.arm_controller.current_stage)
                if stage_str in putdown_stages:
                    print(f"[ARM] {putdown_stages[stage_str]}")
                last_stage = self.arm_controller.current_stage
            time.sleep(0.1)
        
        print("=" * 60)
        
        print("\n" + "="*60)
        print("✓ Put-down sequence complete!")
        print("="*60)
        
        # Give FSM time to fully transition to 801
        print("\nWaiting 3 seconds for FSM to stabilize in mode 801...")
        time.sleep(3.0)
        
        print("\n" + "="*60)
        print("Phase 5 complete")
        print("="*60 + "\n")
        return True
    
    # ========== Main Sequence ==========
    
    def run_full_sequence(self):
        """Run the complete 5-phase sequence"""
        print("\n" + "="*60)
        print("UNITREE G1 - INTEGRATED PICK AND PLACE")
        print("="*60)
        print("\nFull Sequence:")
        print("  1. Navigate to first goal (RViz)")
        print("  2. Auto-center bottle (SSH)")
        print("  3. Pick up bottle")
        print("  4. Navigate to second goal (RViz)")
        print("  5. Put down bottle → FSM 801")
        print()
        
        # Set initial FSM to 801
        print("Setting initial FSM ID to 801 (balance running mode)...")
        ChannelFactoryInitialize(0, self.network_interface)
        self.loco_client = LocoClient()
        self.loco_client.SetTimeout(10.0)
        self.loco_client.Init()
        self.loco_client.SetFsmId(801)
        print("✓ Initial FSM ID set to 801\n")
        
        try:
            # Phase 1: Navigate to first goal
            self.current_phase = 1
            if not self.phase1_navigate_to_first_goal():
                print("\n✗ Phase 1 failed")
                return False
            
            # Wait for robot to finish final rotation alignment
            print("\nWaiting 2 seconds for robot to stabilize after rotation...")
            time.sleep(2.0)
            
            # Phase 2: Auto-center bottle
            self.current_phase = 2
            if not self.phase2_auto_center_bottle():
                print("\n✗ Phase 2 failed")
                return False
            
            # Phase 3: Pickup
            self.current_phase = 3
            if not self.phase3_pickup_bottle():
                print("\n✗ Phase 3 failed")
                return False
            
            # Phase 4: Navigate to second goal
            self.current_phase = 4
            if not self.phase4_navigate_to_second_goal():
                print("\n✗ Phase 4 failed")
                return False
            
            # Wait for robot to finish final rotation alignment
            print("\nWaiting 2 seconds for robot to stabilize after rotation...")
            time.sleep(2.0)
            
            # Phase 5: Put down
            self.current_phase = 5
            if not self.phase5_put_down_bottle():
                print("\n✗ Phase 5 failed")
                return False
            
            print("\n" + "="*60)
            print("✓ ALL PHASES COMPLETE!")
            print("="*60)
            print("\nRobot is now in FSM 801 (balance running mode)")
            return True
            
        except KeyboardInterrupt:
            print("\n\n⚠ User interrupted - stopping gracefully...")
            return False
        except Exception as e:
            print(f"\n\n✗ Error: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def stop_ssh_process(self):
        """Stop SSH process to robot - kill remote script first"""
        if self.ssh_process and self.ssh_process.poll() is None:
            print("Stopping auto_center_bottle.py on robot...")
            
            # Send command via SSH to kill the Python script on the robot
            kill_cmd = [
                "ssh",
                "-o", "StrictHostKeyChecking=no",
                "unitree@192.168.123.164",
                "pkill -SIGINT -f 'python3.*auto_center_bottle.py'"
            ]
            
            try:
                print("Sending kill command to remote script...")
                subprocess.run(kill_cmd, timeout=2)
                time.sleep(1)  # Give remote script time to cleanup
                
                # Terminate the SSH process
                print("Terminating SSH connection...")
                self.ssh_process.terminate()
                self.ssh_process.wait(timeout=3)
                print("✓ SSH process stopped")
            except subprocess.TimeoutExpired:
                print("Force killing SSH connection...")
                self.ssh_process.kill()
                self.ssh_process.wait()
            except Exception as e:
                print(f"Error stopping SSH: {e}")
                # Force kill if anything goes wrong
                if self.ssh_process.poll() is None:
                    self.ssh_process.kill()
                    self.ssh_process.wait()
    
    def stop_nav_process(self):
        """Stop navigation process"""
        if self.nav_process and self.nav_process.poll() is None:
            print("Stopping navigation node...")
            self.nav_process.terminate()
            try:
                self.nav_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.nav_process.kill()
    
    def cleanup(self):
        "Clean up resources"
        print("\nCleaning up...")
        
        # If arm is active and holding bottle (Phase 3 complete, Phase 5 not complete)
        # Return arm gracefully to starting position
        if self.arm_controller and self.arm_controller.is_running:
            current_stage = self.arm_controller.current_stage
            
            # If holding at Position 4 (Phase 4 - navigating with bottle)
            if current_stage == 4:
                print("\n⚠ Arm is holding bottle - returning to starting position gracefully...")
                self.arm_controller.graceful_stop()
                
                # Wait for graceful return to complete
                while self.arm_controller.is_running:
                    time.sleep(0.1)
                
                print("✓ Arm returned to starting position")
            # If in middle of put-down sequence, let it complete naturally
            elif str(current_stage).startswith('STAGE_PUTDOWN'):
                print("\n⚠ Put-down sequence in progress - waiting for completion...")
                while self.arm_controller.is_running:
                    time.sleep(0.1)
                print("✓ Put-down sequence completed")
        
        self.stop_ssh_process()
        self.stop_nav_process()
        
        print("✓ Cleanup complete (robot remains in current FSM state)")


def main():
    if len(sys.argv) < 2:
        print("\nUsage: python3 integrated_controller.py <network_interface>")
        print("Example: python3 integrated_controller.py enp49s0")
        print("\nPrerequisites:")
        print("  1. FAST-LIO localization running")
        print("  2. SSH keys set up to robot (unitree@192.168.123.164)")
        print("  3. Robot in standing position")
        sys.exit(1)
    
    network_interface = sys.argv[1]
    
    # Initialize ROS2
    rclpy.init()
    
    controller = IntegratedController(network_interface)
    
    try:
        success = controller.run_full_sequence()
        controller.cleanup()  # Always cleanup after sequence
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n⚠ Interrupted by user")
        controller.cleanup()
        sys.exit(0)
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
