"""jetson_detect_and_stream.py – run on the G1 Jetson NX

Captures RGB + depth from the on-board D435i, runs YOLO can detection,
and sends annotated streams to your laptop via RTP/UDP.

Usage (on Jetson)
-----------------
python3 jetson_detect_and_stream.py --client-ip 192.168.123.222 \
        --width 640 --height 480 --fps 30

Install runtime once:
    sudo apt install -y python3-gi gstreamer1.0-tools gstreamer1.0-plugins-{good,bad} gstreamer1.0-libav
    python3 -m pip install --user pyrealsense2 numpy opencv-python ultralytics
"""

from __future__ import annotations

import argparse
import sys

import numpy as np
import cv2

import pyrealsense2 as rs
from ultralytics import YOLO

# GStreamer --------------------------------------------------------------
import gi

gi.require_version("Gst", "1.0")
from gi.repository import Gst, GstApp


# Can detection class IDs from COCO dataset
CAN_CLASS_IDS = [39, 41, 42, 43, 44]  # bottle, cup, wine glass, mug, vase


def gst_pipeline(client_ip: str, w: int, h: int, fps: int) -> tuple[Gst.Pipeline, GstApp.AppSrc, GstApp.AppSrc]:
    """Create the GStreamer pipeline and return (pipeline, src_rgb, src_depth)."""

    Gst.init(None)

    rgb_caps = f"video/x-raw,format=BGR,width={w},height={h},framerate={fps}/1"
    depth_caps = f"video/x-raw,format=BGR,width={w},height={h},framerate={fps}/1"

    launch_description = (
        # RGB with detections -------------------------------------------------
        f"appsrc name=src_rgb is-live=true do-timestamp=true format=time caps={rgb_caps} ! "
        "videoconvert ! nvvidconv ! nvv4l2h264enc bitrate=4000000 insert-sps-pps=true idrinterval=15 ! "
        f"rtph264pay config-interval=1 pt=96 ! udpsink host={client_ip} port=5600 sync=false "
        # Depth ---------------------------------------------------------------
        f"appsrc name=src_depth is-live=true do-timestamp=true format=time caps={depth_caps} ! "
        "videoconvert ! nvvidconv ! nvv4l2h264enc bitrate=2000000 insert-sps-pps=true idrinterval=15 ! "
        f"rtph264pay config-interval=1 pt=97 ! udpsink host={client_ip} port=5602 sync=false"
    )

    pipeline = Gst.parse_launch(launch_description)
    src_rgb = pipeline.get_by_name("src_rgb")  # type: ignore
    src_depth = pipeline.get_by_name("src_depth")  # type: ignore

    return pipeline, src_rgb, src_depth


def detect_cans(model: YOLO, image: np.ndarray, confidence_threshold: float = 0.5):
    """
    Detect cans in image and return annotated image with detections
    
    Returns:
        annotated_image: Image with bounding boxes
        detections: List of detection dicts with bbox, center, confidence, depth
    """
    results = model(image, verbose=False)[0]
    
    annotated = image.copy()
    detections = []
    
    for box in results.boxes:
        class_id = int(box.cls[0])
        confidence = float(box.conf[0])
        
        # Filter for can-like objects
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
            
            # Draw center point
            cv2.circle(annotated, (cx, cy), 5, (0, 255, 0), -1)
            
            # Draw label
            label = f"{results.names[class_id]} {confidence:.2f}"
            cv2.putText(annotated, label, (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    # Draw detection count
    cv2.putText(annotated, f"Cans detected: {len(detections)}", (10, 30),
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    return annotated, detections


def get_depth_at_point(depth_frame: rs.depth_frame, x: int, y: int) -> float:
    """Get depth in meters at pixel coordinates"""
    return depth_frame.get_distance(x, y)


def main() -> None:
    ap = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--client-ip", required=True, help="Laptop IP address")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--confidence", type=float, default=0.5, help="Detection confidence threshold")
    ap.add_argument("--model", type=str, default="yolov8n.pt", help="YOLO model path")
    args = ap.parse_args()

    # Load YOLO model
    print(f"Loading YOLO model: {args.model}")
    model = YOLO(args.model)
    print("Model loaded!")

    pipeline, src_rgb, src_depth = gst_pipeline(args.client_ip, args.width, args.height, args.fps)

    # Initialise RealSense -------------------------------------------------
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)
    cfg.enable_stream(rs.stream.depth, args.width, args.height, rs.format.z16, args.fps)

    pipe = rs.pipeline()
    profile = pipe.start(cfg)

    # Get depth intrinsics
    depth_stream = profile.get_stream(rs.stream.depth)
    intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()
    print(f"Camera intrinsics: fx={intrinsics.fx:.2f}, fy={intrinsics.fy:.2f}")

    # Post-processing filters
    temp_filter = rs.temporal_filter()
    align = rs.align(rs.stream.color)

    # Start streaming ------------------------------------------------------
    pipeline.set_state(Gst.State.PLAYING)

    duration = Gst.util_uint64_scale_int(1, Gst.SECOND, args.fps)

    print(f"\nStreaming to {args.client_ip}:5600 (RGB) and {args.client_ip}:5602 (depth)")
    print("Press Ctrl+C to stop\n")

    frame_count = 0
    try:
        while True:
            frames = pipe.wait_for_frames()
            aligned_frames = align.process(frames)
            
            colour_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            
            if not colour_frame or not depth_frame:
                continue
            
            depth_frame = temp_filter.process(depth_frame)
            
            colour = np.asanyarray(colour_frame.get_data())  # (H,W,3) uint8 BGR
            depth16 = np.asanyarray(depth_frame.get_data())  # (H,W) uint16

            # Run can detection
            annotated_rgb, detections = detect_cans(model, colour, args.confidence)
            
            # Add depth info to detections
            for det in detections:
                cx, cy = det['center']
                depth_m = get_depth_at_point(depth_frame, cx, cy)
                det['depth_m'] = depth_m
                
                # Draw depth on image
                x1, y1, _, _ = det['bbox']
                depth_text = f"{depth_m:.2f}m"
                cv2.putText(annotated_rgb, depth_text, (x1, y1 - 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            
            # Print detections every 30 frames
            frame_count += 1
            if detections and frame_count % 30 == 0:
                print(f"\n=== Frame {frame_count} ===")
                for i, det in enumerate(detections):
                    print(f"Can {i+1}: {det['class_name']} ({det['confidence']:.2f}) "
                          f"at ({det['center'][0]}, {det['center'][1]}) depth={det['depth_m']:.2f}m")

            # Create depth visualization
            depth_clip = np.clip(depth16, 0, 6000)
            depth8 = cv2.convertScaleAbs(depth_clip, alpha=255.0 / 6000.0)
            depth_bgr = cv2.applyColorMap(depth8, cv2.COLORMAP_PLASMA)
            
            # Draw detection centers on depth image too
            for det in detections:
                cx, cy = det['center']
                cv2.circle(depth_bgr, (cx, cy), 5, (0, 255, 0), -1)

            # Push RGB with detections --------------------------------------
            buf_rgb = Gst.Buffer.new_allocate(None, annotated_rgb.nbytes, None)
            buf_rgb.fill(0, annotated_rgb.tobytes())
            buf_rgb.duration = duration
            src_rgb.emit("push-buffer", buf_rgb)

            # Push depth ----------------------------------------------------
            buf_d = Gst.Buffer.new_allocate(None, depth_bgr.nbytes, None)
            buf_d.fill(0, depth_bgr.tobytes())
            buf_d.duration = duration
            src_depth.emit("push-buffer", buf_d)

    except KeyboardInterrupt:
        print("\nInterrupted – shutting down …")
    finally:
        for s in (src_rgb, src_depth):
            s.emit("end-of-stream")
        pipeline.set_state(Gst.State.NULL)
        pipe.stop()


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print("Error:", exc)
        import traceback
        traceback.print_exc()
        sys.exit(1)
