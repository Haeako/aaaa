# uses the preview stream for more flexible resolutions on oak
import cv2
import depthai as dai
import pyvirtualcam
from pyvirtualcam import PixelFormat
from sending import send_position, get_position
# import ultralytics
# from ultralytics import YOLO
import numpy as np
# import random as rng
# import matplotlib as plt
# Create pipeline

pipeline = dai.Pipeline()
import serial

# rng.seed(12345)
def send_data(comport, baudrate, data):
    ser = serial.Serial(comport, baudrate)
    ser.write(data.encode())  # Send data as bytes
    ser.close()

# Define source and output
camRgb = pipeline.createCamera()
# cam.initialControl.setManualFocus(130)
xoutVideo = pipeline.createXLinkOut()
xoutVideo.setStreamName("video")

# settings
width = 1024
height = 836
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
        # print(f'Virtual cam started: {cam.device} ({cam.width}x{cam.height} @ {cam.fps}fps)')

        while True:
            videoIn = video.get()
            # Get BGR frame from NV12 encoded video frame to show with opencv
            frame = videoIn.getCvFrame()
            frame_gray =cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            canny_output = cv2.Canny(frame_gray, 0.3, 0.6)   
            frame_hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV); 
            frame_hsv = cv2.inRange(frame,np.array([30,120,50]),np.array([87,255,255]))
            #cv2.imshow("end",frame_hsv)
            #cv2.waitKey(1)
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
            frame = cv2.rectangle(frame, (498,404), (526,430), (0, 0, 255), 2)
            # frame = cv2.rectangle(frame, (200,100), (812,800), (0, 0, 255), 2)
            # Draw contours
            # print (index)
            if index <= len(mc) and index!= 0:
                center = int(mc[index][0]),int(mc[index][1])
                send_position(' '.join(map(str,center)))
                print(' '.join(map(str,center)))

                frame=cv2.circle(frame,(center),4,(255,0,0),-1)

                cv2.imshow("end",frame)
                cv2.waitKey(1)
                #coordinate = get_position()
                #print(coordinate)
            else:
                send_position(' '.join (map(str,(0,0))))
                # print(' '.join (map(str,(0,0))))
                cv2.imshow("end",frame)
                cv2.waitKey(1)
                #oordinate = get_position()
                #print(coordinate)
                
