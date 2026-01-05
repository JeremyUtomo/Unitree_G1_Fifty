#!/usr/bin/env python3
"""
Test script to verify /bottle_alignment_status topic is working.
Run this on the laptop to test if the topic is being received.

Usage:
    python3 test_alignment_topic.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import sys

class AlignmentTopicTester(Node):
    def __init__(self):
        super().__init__('alignment_topic_tester')
        self.subscription = self.create_subscription(
            Bool,
            '/bottle_alignment_status',
            self.callback,
            10
        )
        self.received_count = 0
        print("="*60)
        print("Testing /bottle_alignment_status topic")
        print("="*60)
        print("Waiting for messages...")
        print("(Make sure auto_center_bottle.py is running on robot)")
        print()
    
    def callback(self, msg):
        self.received_count += 1
        print(f"✓ Received message #{self.received_count}: data={msg.data}")
        if msg.data:
            print("  → Bottle is ALIGNED!")
        else:
            print("  → Bottle is NOT aligned")

def main():
    rclpy.init()
    tester = AlignmentTopicTester()
    
    try:
        print("Spinning... (Ctrl+C to exit)\n")
        rclpy.spin(tester)
    except KeyboardInterrupt:
        print("\n\nStopping test...")
    finally:
        tester.destroy_node()
        rclpy.shutdown()
        print(f"Total messages received: {tester.received_count}")

if __name__ == '__main__':
    main()
