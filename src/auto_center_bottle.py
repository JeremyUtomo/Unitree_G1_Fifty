#!/usr/bin/env python3
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
CENTER_THRESHOLD = 35  # Pixels - larger margin for centered alignment
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
            
            # Draw bounding box
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(annotated, (cx, cy), 5, (0, 255, 0), -1)
            
            label = f"{results.names[class_id]} {confidence:.2f}"
            cv2.putText(annotated, label, (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    # Draw center line
    cv2.line(annotated, (CENTER_X, 0), (CENTER_X, CAMERA_HEIGHT), (255, 0, 0), 2)
    cv2.putText(annotated, f"Bottles: {len(detections)}", (10, 25),
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    
    return annotated, detections


def get_depth_at_point(depth_frame, x: int, y: int) -> float:
    """Get depth in meters at pixel coordinates"""
    if not isinstance(depth_frame, rs.depth_frame):
        depth_frame = depth_frame.as_depth_frame()
    return depth_frame.get_distance(x, y)


class BottleCenteringController:
    """Controls robot side-stepping to center detected bottles after table edge alignment"""
    
    def __init__(self, network_interface='eth0'):
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
        self.step_delay = 1.0  # 1 second delay between steps
        
    def init(self):
        """Initialize loco client"""
        ChannelFactoryInitialize(0, self.network_interface)
        self.loco_client = LocoClient()
        self.loco_client.SetTimeout(10.0)
        self.loco_client.Init()
        time.sleep(0.5)
        print("✅ Loco client initialized")
        print("⚠️  Note: Robot must be in balance/stand mode for side-stepping to work")
        
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
            print(f"\n🎯 TABLE EDGE ALIGNED - Starting side-step alignment")
    
    def stop_centering(self):
        """Stop centering and halt robot (only called manually)"""
        with self.lock:
            self.is_centering = False
            self.centered = False
            if self.loco_client:
                # Stop rotation by sending zero velocity instead of damping
                self.loco_client.Move(0, 0, 0)
            print("⚠️  Tracking stopped")
    
    def update(self, detections):
        """Update centering based on current detections - continuously tracks bottle"""
        if not self.is_centering:
            return
        
        if not detections:
            # Lost the bottle - stop movement immediately
            self.frames_without_detection += 1
            
            if not self.bottle_lost:
                print(f"\n⚠️  Bottle out of frame - stopping movement")
                self.bottle_lost = True
                if self.loco_client:
                    self.loco_client.Move(0, 0, 0)
            
            print(f"⚠️  Waiting for bottle ({self.frames_without_detection} frames)        ", end='\r')
            return
        else:
            # Bottle detected
            if self.bottle_lost:
                print(f"\n✅ Bottle found again - resuming tracking")
                self.bottle_lost = False
            
            self.frames_without_detection = 0
            bottle = detections[0]  # Use first/closest detection
            
            # Calculate the middle of the bounding box (not just top-left to bottom-right center)
            x1, y1, x2, y2 = bottle['bbox']
            cx = (x1 + x2) // 2  # Horizontal center of bounding box
            cy = (y1 + y2) // 2  # Vertical center of bounding box
            
            raw_error = cx - CENTER_X
            
            # Smooth error using moving average
            self.error_history.append(raw_error)
            if len(self.error_history) > self.max_history:
                self.error_history.pop(0)
            
            error = int(sum(self.error_history) / len(self.error_history))
            self.last_known_error = error
        
        # Check if centered
        if abs(error) < CENTER_THRESHOLD:
            # Mark as centered but continue tracking
            if not self.centered:
                print(f"\n✅ Centered! (error: {error:+d} px) - Continuing to track...")
                self.centered = True
            
            # Check bottle depth for final positioning
            bottle_depth = bottle.get('depth_m', 0)
            if 0.48 <= bottle_depth <= 0.52:
                print(f"🎯 BOTTLE CENTERED AND ALIGNED - Depth: {bottle_depth:.2f}m ✓        ", end='\r')
            else:
                print(f"📍 Side-step: X={cx} (target={CENTER_X}), Error: {error:+d} px ✓ ALIGNED (depth: {bottle_depth:.2f}m)        ", end='\r')
            
            # Stop movement when centered
            if self.loco_client:
                self.loco_client.Move(0, 0, 0)
        else:
            # No longer centered - resume alignment
            if self.centered:
                print(f"\n🔄 Bottle moved - resuming side-stepping...")
                self.centered = False
            
            # Check if bottle is aligned (within threshold) before allowing step
            if abs(error) < CENTER_THRESHOLD:
                print(f"✅ Bottle aligned (error: {error:+d} px) - no step needed        ", end='\r')
                if self.loco_client:
                    self.loco_client.Move(0, 0, 0)
                return
            
            print(f"📍 Side-step: X={cx} (target={CENTER_X}), Error: {error:+d} px → aligning...        ", end='\r')
            
            # SIDE-STEP MODE: Move left/right to center bottle
            # Positive error = bottle on right side of center line
            # Need to step RIGHT (negative vy per loco_client.py) to bring bottle toward center
            # Negative error = bottle on left side of center line
            # Need to step LEFT (positive vy per loco_client.py) to bring bottle toward center
            step_dir = -1.0 if error > 0 else 1.0
            
            # Use side-step speed (loco_client uses 0.3 for side-step)
            vy_speed = step_dir * 0.2
            
            # Check if we should step or pause
            current_time = time.time()
            
            if self.last_step_time == 0:
                # First step - wait 1 second for clear image before starting
                self.last_step_time = current_time
                print(f"\n📸 Waiting 1s for clear image before first step...")
                if self.loco_client:
                    self.loco_client.Move(0, 0, 0)
                return
            
            time_since_last_step = current_time - self.last_step_time
            
            # Wait 1 second for clear image, step for 0.5 seconds, then pause for 1 second
            initial_wait = 1.0
            step_duration = 0.5
            total_cycle = initial_wait + step_duration + self.step_delay
            
            if time_since_last_step < initial_wait:
                # Initial wait for clear image
                wait_remaining = initial_wait - time_since_last_step
                print(f"📸 Waiting {wait_remaining:.1f}s for clear image before step...        ", end='\r')
                if self.loco_client:
                    self.loco_client.Move(0, 0, 0)
            elif time_since_last_step < initial_wait + step_duration:
                # Active stepping phase
                print(f"\n🔧 SIDE-STEP: error={error}, vy_speed={vy_speed:.3f}, dir={'RIGHT' if step_dir < 0 else 'LEFT'}")
                if self.loco_client:
                    self.loco_client.Move(0, vy_speed, 0)
            elif time_since_last_step < total_cycle:
                # Pause phase after step
                wait_remaining = total_cycle - time_since_last_step
                print(f"⏳ Waiting {wait_remaining:.1f}s for stable image...        ", end='\r')
                if self.loco_client:
                    self.loco_client.Move(0, 0, 0)
            else:
                # Start new cycle
                self.last_step_time = current_time
                print(f"\n📸 Waiting 1s for clear image before next step...")
                if self.loco_client:
                    self.loco_client.Move(0, 0, 0)
    
    def stop(self):
        """Stop all motion"""
        if self.loco_client:
            # Stop movement, keep balance mode active
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

    # Load YOLO model
    print(f"Loading YOLO model: {args.model}")
    model = YOLO(args.model)
    print("Model loaded!")

    # Initialize centering controller
    print("\n🔧 Initializing robot controller...")
    controller = BottleCenteringController(network_interface=args.network_interface)
    controller.init()

    # Initialize streaming pipeline
    pipeline, src_rgb, src_depth = gst_pipeline(args.client_ip, args.width, args.height, args.fps)

    # Initialize RealSense with retry logic
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)
    cfg.enable_stream(rs.stream.depth, args.width, args.height, rs.format.z16, args.fps)

    pipe = rs.pipeline()
    
    # Try to start camera with retries
    import subprocess
    import os
    max_retries = 3
    my_pid = os.getpid()
    
    for attempt in range(max_retries):
        try:
            profile = pipe.start(cfg)
            print(f"✅ Camera started successfully")
            break
        except RuntimeError as e:
            error_msg = str(e)
            if ("Device or resource busy" in error_msg or "already opened" in error_msg or "No device connected" in error_msg) and attempt < max_retries - 1:
                print(f"⚠️  Camera busy (attempt {attempt + 1}/{max_retries}), killing blocking processes...")
                
                # Kill videohub aggressively and prevent restart
                subprocess.run(["sudo", "pkill", "-9", "videohub"], stderr=subprocess.DEVNULL)
                subprocess.run(["sudo", "killall", "-9", "videohub_pc4"], stderr=subprocess.DEVNULL)
                
                # Stop any systemd services that might restart videohub
                subprocess.run(["sudo", "systemctl", "stop", "videohub"], stderr=subprocess.DEVNULL)
                subprocess.run(["sudo", "systemctl", "stop", "videohub_pc4"], stderr=subprocess.DEVNULL)
                
                # Find and kill ALL processes using video devices
                for vid_dev in ["/dev/video0", "/dev/video1", "/dev/video2", "/dev/video3", "/dev/video4", "/dev/video5"]:
                    result = subprocess.run(["sudo", "fuser", vid_dev], 
                                          capture_output=True, text=True)
                    output = result.stdout + result.stderr
                    if output:
                        pids = output.strip().split()
                        for pid in pids:
                            try:
                                pid_num = int(pid.strip())
                                if pid_num != my_pid:  # Don't kill ourselves
                                    print(f"  Killing PID {pid_num} using {vid_dev}")
                                    subprocess.run(["sudo", "kill", "-9", str(pid_num)], stderr=subprocess.DEVNULL)
                            except ValueError:
                                pass
                
                time.sleep(4)
            else:
                raise

    depth_stream = profile.get_stream(rs.stream.depth)
    intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()
    print(f"Camera intrinsics: fx={intrinsics.fx:.2f}, fy={intrinsics.fy:.2f}")

    temp_filter = rs.temporal_filter()
    align = rs.align(rs.stream.color)

    # Start streaming
    pipeline.set_state(Gst.State.PLAYING)
    duration = Gst.util_uint64_scale_int(1, Gst.SECOND, args.fps)

    print(f"\n🎥 Streaming to {args.client_ip}:5600 (RGB) and {args.client_ip}:5602 (depth)")
    print("🔍 Waiting for table edge alignment...\n")

    frame_count = 0
    last_detections = []
    auto_center_started = False
    
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
            
            # Run detection
            if frame_count % args.skip_frames == 0:
                annotated_rgb, detections = detect_cans(model, colour, args.confidence)
                last_detections = detections
            else:
                annotated_rgb = colour.copy()
                detections = last_detections
                
                # Redraw detections
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
            
            # Add depth info
            for det in detections:
                cx, cy = det['center']
                depth_m = get_depth_at_point(depth_frame, cx, cy)
                det['depth_m'] = depth_m
                
                x1, y1, _, _ = det['bbox']
                depth_text = f"{depth_m:.2f}m"
                cv2.putText(annotated_rgb, depth_text, (x1, y1 - 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            
            # Auto-centering logic - only after table edge detected
            if detections and not auto_center_started:
                # Bottle detected but waiting for table edge alignment
                print(f"📦 Bottle detected - waiting for table edge alignment...        ", end='\r')
            
            if controller.is_centering:
                # Update centering control - side-step mode
                controller.update(detections)
                
                # Draw centering status
                if controller.centered:
                    status = "CENTERED - TRACKING"
                    color = (0, 255, 0)
                else:
                    status = "SIDE-STEPPING..."
                    color = (0, 255, 255)
                
                cv2.putText(annotated_rgb, status, (10, 70),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            
            # Create depth visualization with table surface detection
            depth_clip = np.clip(depth16, 0, 6000)
            depth8 = cv2.convertScaleAbs(depth_clip, alpha=255.0 / 6000.0)
            depth_bgr = cv2.applyColorMap(depth8, cv2.COLORMAP_PLASMA)
            
            # Detect table surface (find most common depth in lower half of image)
            lower_half = depth16[CAMERA_HEIGHT//2:, :]
            valid_depths = lower_half[(lower_half > 200) & (lower_half < 5000)]  # 0.2m to 5m range
            
            if len(valid_depths) > 100:  # Need enough samples
                # Find the most common depth (table surface) using histogram
                hist, bin_edges = np.histogram(valid_depths, bins=50)
                peak_idx = np.argmax(hist)
                table_depth = (bin_edges[peak_idx] + bin_edges[peak_idx + 1]) / 2
                table_depth_m = table_depth / 1000.0
                
                
                # Highlight table surface (within ±10cm of detected depth)
                table_mask = np.abs(depth16.astype(float) - table_depth) < 100  # 10cm tolerance
                depth_bgr[table_mask] = cv2.addWeighted(depth_bgr[table_mask], 0.5, 
                                                        np.full_like(depth_bgr[table_mask], (0, 255, 255)), 0.5, 0)
                
                # Draw table depth info
                cv2.putText(depth_bgr, f"Table: {table_depth_m:.2f}m", (10, 25),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                # Draw horizontal line at y=380 (80% down the image)
                check_y = 350
                cv2.line(depth_bgr, (0, check_y), (CAMERA_WIDTH, check_y), (255, 0, 0), 2)
                
                # Check if the line intersects with yellow (table) pixels
                if check_y < CAMERA_HEIGHT:
                    line_pixels = table_mask[check_y, :]
                    yellow_count = np.sum(line_pixels)
                    
                    # If more than 30% of the line is yellow (table detected)
                    if yellow_count > CAMERA_WIDTH * 0.3:
                        # Check if table depth is in edge range (0.60m - 0.65m)
                        if 0.60 <= table_depth_m <= 0.65:
                            if not hasattr(main, '_table_edge_detected') or not main._table_edge_detected:
                                print(f"\n🎯 TABLE EDGE DETECTED at line y={check_y} (depth: {table_depth_m:.2f}m)")
                                main._table_edge_detected = True
                                # Start side-step alignment mode
                                controller.set_table_edge_detected()
                                auto_center_started = True
                        else:
                            if not hasattr(main, '_table_line_detected') or not main._table_line_detected:
                                print(f"\n📋 TABLE DETECTED at line y={check_y} (depth: {table_depth_m:.2f}m)")
                                main._table_line_detected = True
                            if hasattr(main, '_table_edge_detected'):
                                main._table_edge_detected = False
                    else:
                        if hasattr(main, '_table_line_detected'):
                            main._table_line_detected = False
                        if hasattr(main, '_table_edge_detected'):
                            main._table_edge_detected = False

            
            for det in detections:
                cx, cy = det['center']
                cv2.circle(depth_bgr, (cx, cy), 5, (0, 255, 0), -1)
            
            cv2.line(depth_bgr, (CENTER_X, 0), (CENTER_X, CAMERA_HEIGHT), (255, 255, 255), 2)

            # Push streams
            buf_rgb = Gst.Buffer.new_allocate(None, annotated_rgb.nbytes, None)
            buf_rgb.fill(0, annotated_rgb.tobytes())
            buf_rgb.duration = duration
            src_rgb.emit("push-buffer", buf_rgb)

            buf_d = Gst.Buffer.new_allocate(None, depth_bgr.nbytes, None)
            buf_d.fill(0, depth_bgr.tobytes())
            buf_d.duration = duration
            src_depth.emit("push-buffer", buf_d)

    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted – shutting down...")
    finally:
        # Stop motion but don't damp - keep robot in standing mode
        controller.stop()
        for s in (src_rgb, src_depth):
            s.emit("end-of-stream")
        pipeline.set_state(Gst.State.NULL)
        pipe.stop()
        print("👋 Shutdown complete - robot still in balance mode")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print("Error:", exc)
        import traceback
        traceback.print_exc()
        sys.exit(1)
