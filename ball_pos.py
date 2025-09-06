#!/usr/bin/python
import atexit
import signal
import sys

DEBUG_MEM = True
import cv2
if not DEBUG_MEM:
    import depthai as dai
    import pyvirtualcam
import numpy as np
from multiprocessing import shared_memory
import struct
import time

TARGET_HSV_MIN = np.array([30, 120, 50], dtype=np.uint8)
TARGET_HSV_MAX = np.array([87, 255, 255], dtype=np.uint8)
MAX_CONTOUR_AREA = 140*140

# Global variables
shm = None
pipeline = None
width, height, fps = 256, 209, 120

def cleanup():
    """Cleanup shared memory"""
    global shm
    if shm is not None:
        try:
            # notified c++ process that transmit data is dine
            send_coor(0,0,0)
            shm.close()
            shm.unlink()
        except:
            pass

def signal_handler(signum, frame):
    cleanup()
    sys.exit(0)

def init_shared_memory():
    """Initialize shared memory"""
    global shm
    try:
        shm = shared_memory.SharedMemory(name="coor_mem", create=True, size=16)
        print("Created shared memory")
    except FileExistsError:
        shm = shared_memory.SharedMemory(name="coor_mem")
        print("Connected to existing shared memory")
    
    # Setup cleanup
    atexit.register(cleanup)
    signal.signal(signal.SIGINT, signal_handler)

def send_coor(x, y, flag):
    """Send coordinates to shared memory"""
    global shm
    if shm is not None:
        shm.buf[:12] = struct.pack("<iii", x, y, flag)
        if DEBUG_MEM:
            print(f"send {x}, {y}")

def setup_pipeline():
    """Setup camera pipeline"""
    global pipeline, width, height, fps
    
    pipeline = dai.Pipeline()
    camRgb = pipeline.createCamera()
    pipeline.setXLinkChunkSize(0)
    
    xoutVideo = pipeline.createXLinkOut()
    xoutVideo.setStreamName("video")
    
    # Properties
    camRgb.setPreviewSize(width, height)
    camRgb.setFps(fps)
    xoutVideo.input.setBlocking(False)
    xoutVideo.input.setQueueSize(1)
    
    # Linking
    camRgb.preview.link(xoutVideo.input)

def process_frame(frame):
    """Process frame to find ball position"""
    try:
        # Convert to HSV and threshold
        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        frame_threshold = cv2.inRange(frame_hsv, TARGET_HSV_MIN, TARGET_HSV_MAX)
        
        # Find contours
        contours, _ = cv2.findContours(
            frame_threshold,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )
        
        # Find best contour
        for contour in contours:
            moment = cv2.moments(contour)
            area = moment['m00']
            
            if 40 < area < MAX_CONTOUR_AREA:
                # Calculate centroid
                cx = moment['m10'] / (area + 1e-5)
                cy = moment['m01'] / (area + 1e-5)
                
                # Scale coordinates
                center_x = int(cx * 4)
                center_y = int(cy * 4)
                
                return center_x, center_y
        
        return 0, 0
        
    except Exception as e:
        print(f"[W] Frame processing error: {e}")
        return 0, 0

def run_camera():
    """Run camera mode"""
    global pipeline, width, height, fps
    
    if DEBUG_MEM:
        return
    
    setup_pipeline()
    
    try:
        with dai.Device(pipeline) as device:
            video = device.getOutputQueue(name="video", maxSize=1, blocking=False)
            
            with pyvirtualcam.Camera(width, height, fps, print_fps=False) as cam:
                print(f'Virtual cam started: {cam.device} ({cam.width}x{cam.height} @ {cam.fps}fps)')
                
                while True:
                    try:
                        videoIn = video.get()
                        frame = videoIn.getCvFrame()
                        
                        # Process frame
                        x, y = process_frame(frame)
                        send_coor(x, y, 1)
                        
                    except Exception as e:
                        print(f"[W] Frame error: {e}")
                        send_coor(0, 0, 1)
                        
    except Exception as e:
        print(f"[E] Camera setup error: {e}")
        send_coor(0, 0, 1)

def run_debug():
    """Run debug mode"""
    import  random
    print("Running in DEBUG mode")
    try:
        while True:
            x = random.randint(0,1000)
            y = random.randint(0,1000)
            send_coor(x, y, 1)
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nDebug interrupted")

def main():
    """Main function"""
    
    # Initialize
    init_shared_memory()
    
    try:
        if DEBUG_MEM:
            run_debug()
        else:
            run_camera()
    except KeyboardInterrupt:
        cleanup()

if __name__ == "__main__":
    main()