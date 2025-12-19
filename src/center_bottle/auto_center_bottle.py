"""
Auto-Center Bottle Detection and Rotation
Detects bottles and rotates the G1 robot to center them in the camera view

Usage:
    python3 auto_center_bottle.py --client-ip 192.168.123.222

Camera in use error (on G1):
-----------------------------
pgrep -af auto_center_bottle.py
kill <process id>
"""

from __future__ import annotations

import argparse
import sys
import time
import threading
import numpy as np
import cv2
import pyrealsense2 as rs
from ultralytics import YOLO

# ROS2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

# Unitree SDK
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

# GStreamer for streaming
import gi
gi.require_version("Gst", "1.0")
gi.require_version("GstApp", "1.0")
from gi.repository import Gst, GstApp

# Can detection class IDs from COCO dataset
CAN_CLASS_IDS = [39, 41, 42, 43, 44]  # bottle, cup, wine glass, mug, vase

# Camera and control parameters
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CENTER_X = CAMERA_WIDTH // 2
CENTER_THRESHOLD = 30  # Pixels - larger margin for centered alignment
MAX_YAW_SPEED = 0.5  # rad/s - maximum angular velocity (matches loco_client)
MIN_YAW_SPEED = 0.03  # rad/s - extremely slow minimum speed for perfect alignment
MIN_CONFIDENCE = 0.5  # Minimum detection confidence


def gst_pipeline(client_ip: str, w: int, h: int, fps: int) -> tuple[Gst.Pipeline, GstApp.AppSrc, GstApp.AppSrc]:
    """Create the GStreamer pipeline and return (pipeline, src_rgb, src_depth)."""
    Gst.init(None)

    rgb_caps = f"video/x-raw,format=BGR,width={w},height={h},framerate={fps}/1"
    depth_caps = f"video/x-raw,format=BGR,width={w},height={h},framerate={fps}/1"

    launch_description = (
        f"appsrc name=src_rgb is-live=true do-timestamp=true format=time caps={rgb_caps} ! "
        "videoconvert ! nvvidconv ! nvv4l2h264enc bitrate=4000000 insert-sps-pps=true idrinterval=15 ! "
        f"rtph264pay config-interval=1 pt=96 ! udpsink host={client_ip} port=5600 sync=false "
        f"appsrc name=src_depth is-live=true do-timestamp=true format=time caps={depth_caps} ! "
        "videoconvert ! nvvidconv ! nvv4l2h264enc bitrate=2000000 insert-sps-pps=true idrinterval=15 ! "
        f"rtph264pay config-interval=1 pt=97 ! udpsink host={client_ip} port=5602 sync=false"
    )

    pipeline = Gst.parse_launch(launch_description)
    src_rgb = pipeline.get_by_name("src_rgb")
    src_depth = pipeline.get_by_name("src_depth")

    return pipeline, src_rgb, src_depth


def reset_usb_device_by_product():
    """
    Reset RealSense camera via USB without unplugging
    Searches for Intel RealSense device by vendor ID (8086)
    
    Returns:
        True if reset successful, False otherwise
    """
    import subprocess
    import fcntl
    
    try:
        # Find the RealSense device - Intel vendor ID is 8086
        result = subprocess.run(["lsusb"], capture_output=True, text=True)
        realsense_line = None
        for line in result.stdout.split('\n'):
            # Search for Intel vendor ID and RealSense in device name
            if '8086:' in line and 'intel' in line.lower():
                print(f"Found Intel device: {line.strip()}")
                realsense_line = line
                break
        
        if not realsense_line:
            print("Intel RealSense device not found in lsusb")
            print("Trying to list all USB devices:")
            print(result.stdout)
            return False
        
        # Parse bus and device numbers from: "Bus 001 Device 003: ID 8086:0b64 Intel Corp."
        parts = realsense_line.split()
        bus_num = parts[1]
        dev_num = parts[3].rstrip(':')
        
        dev_path = f"/dev/bus/usb/{bus_num}/{dev_num}"
        print(f"Resetting USB device at {dev_path}...")
        
        # Open device and send reset ioctl
        USBDEVFS_RESET = ord('U') << (4*2) | 20
        with open(dev_path, 'wb') as f:
            fcntl.ioctl(f, USBDEVFS_RESET, 0)
        
        print("USB reset successful - waiting 3s for device re-enumeration...")
        time.sleep(3)
        return True
        
    except PermissionError:
        print("ERROR: Permission denied. Try running with sudo")
        return False
    except Exception as e:
        print(f"USB reset failed: {e}")
        return False


def detect_cans(model: YOLO, image: np.ndarray, confidence_threshold: float = 0.5):
    """
    Detect cans in image and return annotated image with detections
    
    Returns:
        annotated_image: Image with bounding boxes
        detections: List of detection dicts with bbox, center, confidence
    """
    results = model(image, verbose=False)[0]
    
    annotated = image.copy()
    detections = []
    
    for box in results.boxes:
        class_id = int(box.cls[0])
        confidence = float(box.conf[0])
        
        if class_id in CAN_CLASS_IDS and confidence >= confidence_threshold:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            
            detections.append({
                'bbox': (x1, y1, x2, y2),
                'center': (cx, cy),
                'confidence': confidence,
                'class_name': results.names[class_id]
            })
            
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(annotated, (cx, cy), 5, (0, 255, 0), -1)
            
            label = f"{results.names[class_id]} {confidence:.2f}"
            cv2.putText(annotated, label, (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    cv2.line(annotated, (CENTER_X, 0), (CENTER_X, CAMERA_HEIGHT), (255, 0, 0), 2)
    
    cv2.line(annotated, (0, 250), (CAMERA_WIDTH, 250), (0, 255, 255), 2)
    cv2.putText(annotated, "Y=250 (min depth)", (10, 245),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
    
    cv2.line(annotated, (0, 300), (CAMERA_WIDTH, 300), (0, 255, 255), 2)
    cv2.putText(annotated, "Y=300 (max depth)", (10, 295),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
    
    cv2.putText(annotated, f"Bottles: {len(detections)}", (10, 25),
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    
    return annotated, detections


def get_depth_at_point(depth_frame, x: int, y: int) -> float:
    """Get depth in meters at pixel coordinates"""
    if not isinstance(depth_frame, rs.depth_frame):
        depth_frame = depth_frame.as_depth_frame()
    return depth_frame.get_distance(x, y)


class BottleCenteringController(Node):
    """Controls robot side-stepping to center detected bottles after table edge alignment"""
    
    def __init__(self, network_interface='eth0'):
        super().__init__('bottle_centering_controller')
        
        # ROS2 publisher for alignment status
        self.alignment_publisher = self.create_publisher(Bool, '/bottle_alignment_status', 10)
        
        self.loco_client = None
        self.network_interface = network_interface
        self.is_centering = False
        self.target_bottle = None
        self.centered = False
        self.lock = threading.Lock()
        self.last_known_error = 0  # Track last known position for when bottle is lost
        self.frames_without_detection = 0
        self.bottle_lost = False  # Track if bottle is currently lost
        self.error_history = []  # Smooth error over multiple frames
        self.max_history = 5  # Number of frames to average
        self.lost_threshold = 20  # Frames without detection before considering bottle truly lost
        self.table_edge_detected = False  # Track if table edge has been detected
        self.last_step_time = 0  # Track time of last side-step for delay
        self.step_delay = 1  # 0.3 second delay between steps
        self.forward_walking = False  # Track if we're walking forward
        self.depth_aligned = False  # Track if depth alignment is complete
        self.centering_complete_time = 0  # Track when horizontal centering was achieved
        self.post_center_wait = 0.5  # Wait 0.3 second after centering before Y-axis alignment
        self.last_mode_switch_time = 0  # Track last time we switched between side-step and forward/backward
        self.mode_switch_delay = 1.0  # Wait 1 second when switching modes
        self.alignment_confirmation_time = 0  # Track when full alignment was first detected
        self.alignment_confirmation_delay = 0.1 # Wait 0.1 second to confirm alignment with clear image
        self.alignment_confirmed = False  # Track if alignment has been confirmed and FSM switched
        
    def init(self):
        """Initialize loco client"""
        ChannelFactoryInitialize(0, self.network_interface)
        self.loco_client = LocoClient()
        self.loco_client.SetTimeout(10.0)
        self.loco_client.Init()
        time.sleep(0.5)
        print("Loco client initialized")
        print("Note: Robot must be in balance/stand mode for side-stepping to work")
        
    def set_table_edge_detected(self):
        """Enable side-stepping alignment after table edge is detected"""
        if not self.table_edge_detected:
            self.table_edge_detected = True
            self.is_centering = True
            self.centered = False
            self.last_known_error = 0
            self.frames_without_detection = 0
            self.bottle_lost = False
            self.error_history = []
            self.last_step_time = 0
            print(f"\nTABLE EDGE ALIGNED - Starting side-step alignment")
            
            if self.loco_client:
                print("Setting FSM ID to 500 (balance mode)")
                self.loco_client.SetFsmId(500)
    
    def stop_centering(self):
        """Stop centering and halt robot (only called manually)"""
        with self.lock:
            self.is_centering = False
            self.centered = False
            if self.loco_client:
                self.loco_client.Move(0, 0, 0)
            print("Tracking stopped")
    
    def update(self, detections):
        """Update centering based on current detections - continuously tracks bottle"""
        if not self.is_centering:
            return
        
        if not detections:
            self.frames_without_detection += 1
            
            if not self.bottle_lost:
                print(f"\nBottle out of frame - stopping movement")
                self.bottle_lost = True
                if self.loco_client:
                    self.loco_client.Move(0, 0, 0)
            
            print(f"Waiting for bottle ({self.frames_without_detection} frames)        ", end='\r')
            return
        else:
            if self.bottle_lost:
                print(f"\nBottle found again - resuming tracking")
                self.bottle_lost = False
            
            self.frames_without_detection = 0
            bottle = detections[0]
            
            x1, y1, x2, y2 = bottle['bbox']
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            
            raw_error = cx - CENTER_X
            
            self.error_history.append(raw_error)
            if len(self.error_history) > self.max_history:
                self.error_history.pop(0)
            
            error = int(sum(self.error_history) / len(self.error_history))
            self.last_known_error = error
        
        if abs(error) < CENTER_THRESHOLD:
            if not self.centered:
                print(f"\nHorizontally centered! (error: {error:+d} px) - Waiting 0.3s for clear image...")
                self.centered = True
                self.centering_complete_time = time.time()
            
            current_time = time.time()
            time_since_centered = current_time - self.centering_complete_time
            
            if time_since_centered < self.post_center_wait:
                wait_remaining = self.post_center_wait - time_since_centered
                print(f"Waiting {wait_remaining:.1f}s for clear image after centering...        ", end='\r')
                if self.loco_client:
                    self.loco_client.Move(0, 0, 0)
                return
            
            if cy < 250:
                if not self.forward_walking:
                    current_time = time.time()
                    if self.last_mode_switch_time == 0 or (current_time - self.last_mode_switch_time) >= self.mode_switch_delay:
                        print(f"\nWalking forward - Bottle Y={cy} (target: 250-300)")
                        self.forward_walking = True
                        self.depth_aligned = False
                        self.last_mode_switch_time = current_time
                    else:
                        wait_remaining = self.mode_switch_delay - (current_time - self.last_mode_switch_time)
                        print(f"Waiting {wait_remaining:.1f}s before walking forward...        ", end='\r')
                        if self.loco_client:
                            self.loco_client.Move(0, 0, 0)
                        return
                
                distance_to_target = 250 - cy
                vx_speed = 0.15
                
                print(f"Walking forward (vx={vx_speed:.2f}, dist={distance_to_target}px) - Bottle Y={cy} (target: 250-300)        ", end='\r')
                if self.loco_client:
                    self.loco_client.Move(vx_speed, 0, 0)
            elif cy > 300:
                if self.forward_walking or not self.depth_aligned:
                    current_time = time.time()
                    if self.last_mode_switch_time == 0 or (current_time - self.last_mode_switch_time) >= self.mode_switch_delay:
                        print(f"\nStepping backward - Bottle Y={cy} (target: 250-300)")
                        self.forward_walking = False
                        self.depth_aligned = False
                        self.last_mode_switch_time = current_time
                    else:
                        wait_remaining = self.mode_switch_delay - (current_time - self.last_mode_switch_time)
                        print(f"Waiting {wait_remaining:.1f}s before stepping backward...        ", end='\r')
                        if self.loco_client:
                            self.loco_client.Move(0, 0, 0)
                        return
                
                distance_over_target = cy - 300
                vx_speed = -0.15
                
                print(f"Stepping backward (vx={vx_speed:.2f}, dist={distance_over_target}px) - Bottle Y={cy} (target: 250-300)        ", end='\r')
                if self.loco_client:
                    self.loco_client.Move(vx_speed, 0, 0)
            else:
                if self.forward_walking or not self.depth_aligned:
                    print(f"\nDEPTH ALIGNED - Bottle Y={cy} (target: 250-300)")
                    self.forward_walking = False
                    self.depth_aligned = True
                    self.alignment_confirmation_time = time.time()
                
                current_time = time.time()
                time_since_aligned = current_time - self.alignment_confirmation_time
                
                if time_since_aligned < self.alignment_confirmation_delay:
                    wait_remaining = self.alignment_confirmation_delay - time_since_aligned
                    print(f"Confirming alignment... {wait_remaining:.1f}s remaining - X={cx}, Y={cy}        ", end='\r')
                    if self.loco_client:
                        self.loco_client.Move(0, 0, 0)
                else:
                    if not self.alignment_confirmed:
                        print(f"\nALIGNMENT CONFIRMED - Setting FSM ID to 801 (walking mode)")
                        self.alignment_confirmed = True
                        
                        # Publish alignment status to ROS2 topic
                        msg = Bool()
                        msg.data = True
                        self.alignment_publisher.publish(msg)
                        self.get_logger().info('Published alignment status: True')
                    
                    # Continue publishing alignment status every frame
                    msg = Bool()
                    msg.data = True
                    self.alignment_publisher.publish(msg)
                    
                    print(f"BOTTLE FULLY ALIGNED - X={cx}, Y={cy} ✓        ", end='\r')
                    if self.loco_client:
                        self.loco_client.Move(0, 0, 0)
        else:
            if self.forward_walking or self.depth_aligned:
                current_time = time.time()
                if self.last_mode_switch_time == 0 or (current_time - self.last_mode_switch_time) >= self.mode_switch_delay:
                    print(f"\nLost horizontal alignment - stopping forward walk, resuming side-step")
                    self.forward_walking = False
                    self.depth_aligned = False
                    self.last_mode_switch_time = current_time
                else:
                    wait_remaining = self.mode_switch_delay - (current_time - self.last_mode_switch_time)
                    print(f"Waiting {wait_remaining:.1f}s before resuming side-step...        ", end='\r')
                    if self.loco_client:
                        self.loco_client.Move(0, 0, 0)
                    return
            
            if self.centered:
                print(f"\nBottle moved - resuming side-stepping...")
                self.centered = False
            
            if abs(error) < CENTER_THRESHOLD:
                print(f"Bottle aligned (error: {error:+d} px) - no step needed        ", end='\r')
                if self.loco_client:
                    self.loco_client.Move(0, 0, 0)
                return
            
            print(f"Side-step: X={cx} (target={CENTER_X}), Error: {error:+d} px → aligning...        ", end='\r')
            
            step_dir = -1.0 if error > 0 else 1.0
            vy_speed = step_dir * 0.15
            current_time = time.time()
            
            if self.last_step_time == 0:
                self.last_step_time = current_time
                print(f"\nWaiting 0.3s for clear image before first step...")
                if self.loco_client:
                    self.loco_client.Move(0, 0, 0)
                return
            
            time_since_last_step = current_time - self.last_step_time
            
            initial_wait = 0.3
            step_duration = 0.5
            total_cycle = initial_wait + step_duration + self.step_delay
            
            if time_since_last_step < initial_wait:
                wait_remaining = initial_wait - time_since_last_step
                print(f"Waiting {wait_remaining:.1f}s for clear image before step...        ", end='\r')
                if self.loco_client:
                    self.loco_client.Move(0, 0, 0)
            elif time_since_last_step < initial_wait + step_duration:
                print(f"\nSIDE-STEP: error={error}, vy_speed={vy_speed:.3f}, dir={'RIGHT' if step_dir < 0 else 'LEFT'}")
                if self.loco_client:
                    self.loco_client.Move(0, vy_speed, 0)
            elif time_since_last_step < total_cycle:
                wait_remaining = total_cycle - time_since_last_step
                print(f"Waiting {wait_remaining:.1f}s for stable image...        ", end='\r')
                if self.loco_client:
                    self.loco_client.Move(0, 0, 0)
            else:
                self.last_step_time = current_time
                print(f"\nWaiting 0.3s for clear image before next step...")
                if self.loco_client:
                    self.loco_client.Move(0, 0, 0)
    
    def stop(self):
        """Stop all motion"""
        if self.loco_client:
            self.loco_client.Move(0, 0, 0)


def main():
    ap = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--client-ip", default="192.168.123.222", help="Laptop IP address for streaming")
    ap.add_argument("--network-interface", default="eth0", help="Network interface for robot communication")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=15)
    ap.add_argument("--confidence", type=float, default=0.5, help="Detection confidence threshold")
    ap.add_argument("--model", type=str, default="yolov8n.pt", help="YOLO model path")
    ap.add_argument("--skip-frames", type=int, default=3, help="Process every Nth frame for detection")
    args = ap.parse_args()

    # Initialize ROS2
    rclpy.init()

    print(f"Loading YOLO model: {args.model}")
    model = YOLO(args.model)
    print("Model loaded!")

    print("\nInitializing robot controller...")
    controller = BottleCenteringController(network_interface=args.network_interface)
    controller.init()

    pipeline, src_rgb, src_depth = gst_pipeline(args.client_ip, args.width, args.height, args.fps)

    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)
    cfg.enable_stream(rs.stream.depth, args.width, args.height, rs.format.z16, args.fps)

    pipe = rs.pipeline()
    ctx = rs.context()
    
    # Debug: Check available devices
    print("\n=== Camera Debug Info ===")
    devices = ctx.query_devices()
    print(f"Number of RealSense devices found: {len(devices)}")
    for i, dev in enumerate(devices):
        print(f"  Device {i}: {dev.get_info(rs.camera_info.name)}")
        print(f"    Serial: {dev.get_info(rs.camera_info.serial_number)}")
        print(f"    Firmware: {dev.get_info(rs.camera_info.firmware_version)}")
        print(f"    USB Type: {dev.get_info(rs.camera_info.usb_type_descriptor)}")
    
    if len(devices) == 0:
        print("ERROR: No RealSense cameras detected!")
        print("Try: sudo dmesg | tail -20  (check for USB errors)")
        print("Try: lsusb | grep Intel  (check if camera is enumerated)")
        sys.exit(1)
    print("========================\n")
    
    import subprocess
    import os
    max_retries = 3
    
    for attempt in range(max_retries):
        try:
            print(f"Camera start attempt {attempt + 1}/{max_retries}...")
            profile = pipe.start(cfg)
            print(f"✅ Camera started successfully")
            break
        except RuntimeError as e:
            error_msg = str(e)
            if ("Device or resource busy" in error_msg or "already opened" in error_msg or "No device connected" in error_msg or "No such file or directory" in error_msg) and attempt < max_retries - 1:
                print(f"Camera busy (attempt {attempt + 1}/{max_retries}), trying USB reset...")
                
                if reset_usb_device_by_product():
                    print("USB reset complete, recreating pipeline...")
                    pipe = rs.pipeline()
                    ctx = rs.context()
                    print("Pipeline recreated, retrying camera...")
                else:
                    print("USB reset failed, retrying anyway...")
                    time.sleep(2)
            else:
                raise

    depth_stream = profile.get_stream(rs.stream.depth)
    intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()
    print(f"Camera intrinsics: fx={intrinsics.fx:.2f}, fy={intrinsics.fy:.2f}")

    temp_filter = rs.temporal_filter()
    align = rs.align(rs.stream.color)

    pipeline.set_state(Gst.State.PLAYING)
    duration = Gst.util_uint64_scale_int(1, Gst.SECOND, args.fps)

    print(f"\nStreaming to {args.client_ip}:5600 (RGB) and {args.client_ip}:5602 (depth)")
    print("Waiting for table edge alignment...\n")

    frame_count = 0
    last_detections = []
    auto_center_started = False
    initial_check_done = False
    
    try:
        while True:
            frames = pipe.wait_for_frames()
            aligned_frames = align.process(frames)
            
            colour_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            
            if not colour_frame or not depth_frame:
                continue
            
            depth_frame = temp_filter.process(depth_frame)
            
            colour = np.asanyarray(colour_frame.get_data())
            depth16 = np.asanyarray(depth_frame.get_data())

            frame_count += 1
            
            if frame_count % args.skip_frames == 0:
                annotated_rgb, detections = detect_cans(model, colour, args.confidence)
                last_detections = detections
            else:
                annotated_rgb = colour.copy()
                detections = last_detections
                
                for det in detections:
                    x1, y1, x2, y2 = det['bbox']
                    cx, cy = det['center']
                    cv2.rectangle(annotated_rgb, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(annotated_rgb, (cx, cy), 5, (0, 255, 0), -1)
                    label = f"{det['class_name']} {det['confidence']:.2f}"
                    cv2.putText(annotated_rgb, label, (x1, y1 - 10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                cv2.line(annotated_rgb, (CENTER_X, 0), (CENTER_X, CAMERA_HEIGHT), (255, 0, 0), 2)
                cv2.putText(annotated_rgb, f"Bottles: {len(detections)}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            for det in detections:
                cx, cy = det['center']
                depth_m = get_depth_at_point(depth_frame, cx, cy)
                det['depth_m'] = depth_m
                
                x1, y1, _, _ = det['bbox']
                depth_text = f"{depth_m:.2f}m"
                cv2.putText(annotated_rgb, depth_text, (x1, y1 - 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            
            if detections and not auto_center_started:
                print(f"Bottle detected - waiting for table edge alignment...        ", end='\r')
            
            if controller.is_centering:
                controller.update(detections)
                
                # Process ROS2 callbacks to publish messages
                rclpy.spin_once(controller, timeout_sec=0)
                
                if controller.centered:
                    status = "CENTERED - TRACKING"
                    color = (0, 255, 0)
                else:
                    status = "SIDE-STEPPING..."
                    color = (0, 255, 255)
                
                cv2.putText(annotated_rgb, status, (10, 70),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            
            depth_clip = np.clip(depth16, 0, 6000)
            depth8 = cv2.convertScaleAbs(depth_clip, alpha=255.0 / 6000.0)
            depth_bgr = cv2.applyColorMap(depth8, cv2.COLORMAP_PLASMA)
            
            lower_half = depth16[CAMERA_HEIGHT//2:, :]
            valid_depths = lower_half[(lower_half > 200) & (lower_half < 5000)]
            
            if len(valid_depths) > 100:
                hist, bin_edges = np.histogram(valid_depths, bins=50)
                peak_idx = np.argmax(hist)
                table_depth = (bin_edges[peak_idx] + bin_edges[peak_idx + 1]) / 2
                table_depth_m = table_depth / 1000.0
                
                
                table_mask = np.abs(depth16.astype(float) - table_depth) < 100
                depth_bgr[table_mask] = cv2.addWeighted(depth_bgr[table_mask], 0.5, 
                                                        np.full_like(depth_bgr[table_mask], (0, 255, 255)), 0.5, 0)
                
                cv2.putText(depth_bgr, f"Table: {table_depth_m:.2f}m", (10, 25),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                check_y = 350
                cv2.line(depth_bgr, (0, check_y), (CAMERA_WIDTH, check_y), (255, 0, 0), 2)
                
                if check_y < CAMERA_HEIGHT:
                    line_pixels = table_mask[check_y, :]
                    yellow_count = np.sum(line_pixels)
                    
                    if yellow_count > CAMERA_WIDTH * 0.3:
                        if 0.45 <= table_depth_m <= 0.65:
                            if not auto_center_started:
                                if frame_count <= 30:
                                    print(f"\nTABLE ALREADY ALIGNED at startup (frame {frame_count}, depth: {table_depth_m:.2f}m)")
                                else:
                                    print(f"\nTABLE EDGE DETECTED at line y={check_y} (depth: {table_depth_m:.2f}m)")
                                
                                print("Starting bottle centering immediately...")
                                controller.set_table_edge_detected()
                                auto_center_started = True
                        else:
                            if not hasattr(main, '_table_line_detected') or not main._table_line_detected:
                                print(f"\nTABLE DETECTED at line y={check_y} (depth: {table_depth_m:.2f}m)")
                                main._table_line_detected = True
                            if hasattr(main, '_table_edge_detected'):
                                main._table_edge_detected = False
                    else:
                        if hasattr(main, '_table_line_detected'):
                            main._table_line_detected = False
                        if hasattr(main, '_table_edge_detected'):
                            main._table_edge_detected = False
                    
                    if not initial_check_done and frame_count >= 30:
                        initial_check_done = True

            
            for det in detections:
                cx, cy = det['center']
                cv2.circle(depth_bgr, (cx, cy), 5, (0, 255, 0), -1)
            
            cv2.line(depth_bgr, (CENTER_X, 0), (CENTER_X, CAMERA_HEIGHT), (255, 255, 255), 2)

            buf_rgb = Gst.Buffer.new_allocate(None, annotated_rgb.nbytes, None)
            buf_rgb.fill(0, annotated_rgb.tobytes())
            buf_rgb.duration = duration
            src_rgb.emit("push-buffer", buf_rgb)

            buf_d = Gst.Buffer.new_allocate(None, depth_bgr.nbytes, None)
            buf_d.fill(0, depth_bgr.tobytes())
            buf_d.duration = duration
            src_depth.emit("push-buffer", buf_d)

    except KeyboardInterrupt:
        print("\n\nInterrupted – shutting down...")
    finally:
        controller.stop()
        for s in (src_rgb, src_depth):
            s.emit("end-of-stream")
        pipeline.set_state(Gst.State.NULL)
        pipe.stop()
        controller.destroy_node()
        rclpy.shutdown()
        print("Shutdown complete - robot still in balance mode")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print("Error:", exc)
        import traceback
        traceback.print_exc()
        sys.exit(1)
