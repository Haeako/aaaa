# uses the preview stream for more flexible resolutions on oak
import cv2
import os
import depthai as dai
import pyvirtualcam
from pyvirtualcam import PixelFormat
import numpy as np
import time

FIFO_PATH = '/tmp/stepper_fifo'
try:
    if not os.path.exists(FIFO_PATH):
        os.mkfifo(FIFO_PATH, 0o666)
except OSError as e:
    if e.errno != errno.EEXIST:
        print(f"Failed to create FIFO: {e}")
fd = os.open(FIFO_PATH, os.O_WRONLY)

def write_to_fifo(message):
    try:
        # Open FIFO in non-blocking mode for writing
        os.write(fd, message.encode())
    except OSError as e:
        if e.errno == errno.ENXIO:
            # No reader available
            pass
        else:
            print(f"Error writing to FIFO: {e}")


pipeline = dai.Pipeline()

# Define source and output
camRgb = pipeline.createCamera()
# cam.initialControl.setManualFocus(130)
xoutVideo = pipeline.createXLinkOut()
xoutVideo.setStreamName("video")

# settings
width = 512
height = 418
fps = 60

# preview output using opencv imshow
preview = False

# print fps on virtual camera
print_fps = False

# Properties
camRgb.setPreviewSize(width, height)
# camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RG)
camRgb.setFps(fps)

xoutVideo.input.setBlocking(False)
xoutVideo.input.setQueueSize(1)

# Linking to preview stream
camRgb.preview.link(xoutVideo.input)

# Connect to device and start pipeline
with dai.Device(pipeline) as device:

    video = device.getOutputQueue(name="video", maxSize=1, blocking=False)
    
    with pyvirtualcam.Camera(width, height, fps, print_fps=True) as cam:
        print(f'Virtual cam started: {cam.device} ({cam.width}x{cam.height} @ {cam.fps}fps)')

        while True:
            previous_catch = time.time()
            videoIn = video.get()
            # Get BGR frame from NV12 encoded video frame to show with opencv
            frame = videoIn.getCvFrame()
            frame_hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV); 
            frame_hsv = cv2.inRange(frame,np.array([30,120,50]),np.array([87,255,255]))
            contours, hierarchy = cv2.findContours(frame_hsv, cv2.RETR_CCOMP,cv2.CHAIN_APPROX_SIMPLE)
            mu=[None]*len(contours)
            for i in range(len(contours)):
               mu[i]=cv2.moments(contours[i])
            mc=[None]*len(contours)
            index=0
            for i in range(len(contours)):
                # add 1e-5 to avoid division by zero
                if mu[i]['m00'] >50*50 and mu[i]['m00']< 240*240:
                    index=i
                    mc[i] = (mu[i]['m10'] / (mu[i]['m00'] + 1e-5), mu[i]['m01'] / (mu[i]['m00'] + 1e-5))
                    break
            xx0 = int(498/2)
            yy0 = int(404/2)
            xx1 = int(526/2)
            yy1 = int(430/2)
            frame = cv2.rectangle(frame, (xx0,yy0), (xx1,yy1), (0, 0, 255), 2)
            cv2.imshow("output", frame)
            cv2.waitKey(1)
            
            if index <= len(mc) and index!= 0:
                center = int(mc[index][0]),int(mc[index][1])
                #frame=cv2.circle(frame,(center),4,(255,0,0),-1)
                catch = time.time()
                write_to_fifo((' '.join(map(str,center))))
                
                print(catch - previous_catch)
                previous_catch = catch
            else:
                
                pos = ' '.join (map(str,(0,0)))
                write_to_fifo(pos)
                
