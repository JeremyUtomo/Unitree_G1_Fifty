"""laptop_view_detections.py – run on your laptop

Receives and displays the RGB + depth streams with can detections from G1 Jetson.

Usage (on laptop)
-----------------
python3 laptop_view_detections.py

Install dependencies:
    sudo apt install -y gstreamer1.0-tools gstreamer1.0-plugins-{good,bad,ugly} gstreamer1.0-libav
    pip install opencv-python numpy
"""

from __future__ import annotations

import sys
import threading
import time
from queue import Queue
from datetime import datetime

import cv2
import numpy as np

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib


class DualStreamReceiver:
    """Receive RGB and depth RTP streams from Jetson via GStreamer"""
    
    def __init__(self, rgb_port: int = 5600, depth_port: int = 5602):
        Gst.init(None)
        
        self.rgb_queue = Queue(maxsize=2)
        self.depth_queue = Queue(maxsize=2)
        
        # RGB pipeline
        rgb_pipeline = (
            f"udpsrc port={rgb_port} "
            "caps=\"application/x-rtp, media=video, clock-rate=90000, encoding-name=H264\" ! "
            "rtph264depay ! avdec_h264 ! videoconvert ! "
            "video/x-raw,format=BGR ! appsink name=sink_rgb emit-signals=true sync=false"
        )
        
        # Depth pipeline
        depth_pipeline = (
            f"udpsrc port={depth_port} "
            "caps=\"application/x-rtp, media=video, clock-rate=90000, encoding-name=H264\" ! "
            "rtph264depay ! avdec_h264 ! videoconvert ! "
            "video/x-raw,format=BGR ! appsink name=sink_depth emit-signals=true sync=false"
        )
        
        self.rgb_pipe = Gst.parse_launch(rgb_pipeline)
        self.depth_pipe = Gst.parse_launch(depth_pipeline)
        
        # Connect callbacks
        rgb_sink = self.rgb_pipe.get_by_name("sink_rgb")
        rgb_sink.connect("new-sample", self._on_rgb_sample)
        
        depth_sink = self.depth_pipe.get_by_name("sink_depth")
        depth_sink.connect("new-sample", self._on_depth_sample)
        
        self.running = False
        self.rgb_fps = 0
        self.depth_fps = 0
        self._rgb_frame_count = 0
        self._depth_frame_count = 0
        self._last_fps_time = time.time()
    
    def _on_rgb_sample(self, sink):
        """Callback for new RGB frame"""
        sample = sink.emit("pull-sample")
        if sample:
            buf = sample.get_buffer()
            caps = sample.get_caps()
            
            structure = caps.get_structure(0)
            width = structure.get_value("width")
            height = structure.get_value("height")
            
            result, mapinfo = buf.map(Gst.MapFlags.READ)
            if result:
                frame = np.ndarray(
                    shape=(height, width, 3),
                    dtype=np.uint8,
                    buffer=mapinfo.data
                )
                
                if not self.rgb_queue.full():
                    self.rgb_queue.put(frame.copy())
                
                self._rgb_frame_count += 1
                buf.unmap(mapinfo)
        
        return Gst.FlowReturn.OK
    
    def _on_depth_sample(self, sink):
        """Callback for new depth frame"""
        sample = sink.emit("pull-sample")
        if sample:
            buf = sample.get_buffer()
            caps = sample.get_caps()
            
            structure = caps.get_structure(0)
            width = structure.get_value("width")
            height = structure.get_value("height")
            
            result, mapinfo = buf.map(Gst.MapFlags.READ)
            if result:
                frame = np.ndarray(
                    shape=(height, width, 3),
                    dtype=np.uint8,
                    buffer=mapinfo.data
                )
                
                if not self.depth_queue.full():
                    self.depth_queue.put(frame.copy())
                
                self._depth_frame_count += 1
                buf.unmap(mapinfo)
        
        return Gst.FlowReturn.OK
    
    def _update_fps(self):
        """Update FPS counters"""
        while self.running:
            time.sleep(1.0)
            now = time.time()
            elapsed = now - self._last_fps_time
            
            self.rgb_fps = self._rgb_frame_count / elapsed
            self.depth_fps = self._depth_frame_count / elapsed
            
            self._rgb_frame_count = 0
            self._depth_frame_count = 0
            self._last_fps_time = now
    
    def start(self):
        """Start receiving streams"""
        self.running = True
        self.rgb_pipe.set_state(Gst.State.PLAYING)
        self.depth_pipe.set_state(Gst.State.PLAYING)
        
        # Run GLib main loop in separate thread
        self.loop = GLib.MainLoop()
        self.loop_thread = threading.Thread(target=self.loop.run, daemon=True)
        self.loop_thread.start()
        
        # Start FPS counter thread
        self.fps_thread = threading.Thread(target=self._update_fps, daemon=True)
        self.fps_thread.start()
        
        print("Waiting for streams from G1...")
    
    def get_frames(self):
        """Get latest RGB and depth frames"""
        rgb = None
        depth = None
        
        if not self.rgb_queue.empty():
            rgb = self.rgb_queue.get()
        
        if not self.depth_queue.empty():
            depth = self.depth_queue.get()
        
        return rgb, depth
    
    def stop(self):
        """Stop receiving streams"""
        self.running = False
        self.rgb_pipe.set_state(Gst.State.NULL)
        self.depth_pipe.set_state(Gst.State.NULL)
        self.loop.quit()


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="View can detection stream from G1")
    parser.add_argument("--rgb-port", type=int, default=5600, help="RGB stream port")
    parser.add_argument("--depth-port", type=int, default=5602, help="Depth stream port")
    parser.add_argument("--record", action="store_true", help="Record video to file")
    args = parser.parse_args()
    
    # Set OpenCV to use simpler backend to avoid Qt threading issues
    import os
    os.environ['QT_QPA_PLATFORM'] = 'xcb'
    
    # Create single window before starting threads
    cv2.namedWindow("G1 Camera Stream", cv2.WINDOW_NORMAL)
    # Set window size for stacked view (640 width, 480*2 = 960 height for RGB on top, depth on bottom)
    cv2.resizeWindow("G1 Camera Stream", 640, 960)
    
    # Start stream receiver
    receiver = DualStreamReceiver(args.rgb_port, args.depth_port)
    receiver.start()
    
    print("\n=== G1 Can Detection Viewer ===")
    print("Press 'q' to quit")
    print("Press 's' to save current frame")
    print("Press 'r' to start/stop recording")
    print("Press 'f' to toggle fullscreen\n")
    
    video_writer = None
    recording = False
    fullscreen = False
    save_counter = 0
    
    # Give streams time to start
    time.sleep(1)
    
    try:
        while True:
            rgb, depth = receiver.get_frames()
            
            combined = None
            
            if rgb is not None:
                # Add FPS overlay to RGB
                cv2.putText(rgb, f"RGB FPS: {receiver.rgb_fps:.1f}", (10, rgb.shape[0] - 40),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(rgb, f"Depth FPS: {receiver.depth_fps:.1f}", (10, rgb.shape[0] - 15),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                if recording:
                    cv2.circle(rgb, (rgb.shape[1] - 30, 30), 10, (0, 0, 255), -1)
                    cv2.putText(rgb, "REC", (rgb.shape[1] - 80, 35),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                
                # Stack RGB on top and depth on bottom
                if depth is not None:
                    # Make sure both images have the same width
                    if rgb.shape[1] != depth.shape[1]:
                        depth = cv2.resize(depth, (rgb.shape[1], depth.shape[0]))
                    
                    # Draw horizontal reference lines on RGB
                    cv2.line(rgb, (0, 250), (rgb.shape[1], 250), (255, 0, 0), 2)
                    cv2.line(rgb, (0, 300), (rgb.shape[1], 300), (0, 255, 0), 2)
                    
                    # Draw horizontal reference lines on depth
                    cv2.line(depth, (0, 250), (depth.shape[1], 250), (255, 0, 0), 2)
                    cv2.line(depth, (0, 300), (depth.shape[1], 300), (0, 255, 0), 2)
                    
                    combined = cv2.vconcat([rgb, depth])
                else:
                    combined = rgb
                
                # Show combined window
                if fullscreen:
                    cv2.setWindowProperty("G1 Camera Stream", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
                else:
                    cv2.setWindowProperty("G1 Camera Stream", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)
                
                cv2.imshow("G1 Camera Stream", combined)
                
                # Record if enabled (record the combined view)
                if recording and video_writer is not None:
                    video_writer.write(combined)
            
            # Keyboard controls
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                break
            elif key == ord('s') and combined is not None:
                save_counter += 1
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f'detection_{timestamp}_{save_counter}.png'
                cv2.imwrite(filename, combined)
                print(f"✓ Saved: {filename}")
            
            elif key == ord('r') and combined is not None:
                if not recording:
                    # Start recording
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    filename = f'detection_video_{timestamp}.avi'
                    fourcc = cv2.VideoWriter_fourcc(*'XVID')
                    video_writer = cv2.VideoWriter(filename, fourcc, 20.0, 
                                                   (combined.shape[1], combined.shape[0]))
                    recording = True
                    print(f"● Started recording: {filename}")
                else:
                    # Stop recording
                    if video_writer is not None:
                        video_writer.release()
                        video_writer = None
                    recording = False
                    print("■ Stopped recording")
            
            elif key == ord('f'):
                fullscreen = not fullscreen
                print(f"Fullscreen: {'ON' if fullscreen else 'OFF'}")
            
            time.sleep(0.01)
    
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        # Cleanup
        if video_writer is not None:
            video_writer.release()
        receiver.stop()
        cv2.destroyAllWindows()
        print("\nShutdown complete")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"Error: {exc}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
