from imutils.video.pivideostream import PiVideoStream
from imutils.video import FPS
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import imutils
import serial
import time
import cv2
import sys

# Send_data
Uart_buf = [0x55,0xAA,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xAA]
# Setup Usart
port = serial.Serial("/dev/ttyAMA0", baudrate=115200, timeout=1.0)

camera = PiCamera()
camera.resolution = (320, 240)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(320, 240))
stream = camera.capture_continuous(rawCapture, format="bgr", use_video_port=True, burst=True)
camera.close()

vs = PiVideoStream().start()
time.sleep(2.0)
fps = FPS().start()

selection = None
drag_start = None
track_window = None
track_start = False
tracker = cv2.Tracker_create('KCF')


def onmouse(event, x, y, flags, param):
    global selection, drag_start, track_window, track_start
    if event == cv2.EVENT_LBUTTONDOWN:
        drag_start = (x, y)
        track_window = None
    if drag_start:
        xmin = min(x, drag_start[0])
        ymin = min(y, drag_start[1])
        xmax = max(x, drag_start[0])
        ymax = max(y, drag_start[1])
        selection = (xmin, ymin, xmax, ymax)
    if event == cv2.EVENT_LBUTTONUP:
        drag_start = None
        selection = None
        track_window = (xmin, ymin, xmax - xmin, ymax - ymin)
        if track_window and track_window[2] > 0 and track_window[3] > 0:
            track_start = True
            tracker.init(frame, track_window)


cv2.namedWindow('KCFTracker', cv2.WINDOW_NORMAL)
cv2.setMouseCallback('KCFTracker', onmouse)

while True:
    # Read a new frame
    frame = vs.read()
    frame = imutils.resize(frame, width=160)

    if selection:
        x0, y0, x1, y1 = selection
        cv2.rectangle(frame, (x0, y0), (x1, y1), (255, 0, 0), 2, 1)

    # Start timer
    timer = cv2.getTickCount()

    # Update tracker
    track_ok = None
    if track_start:
        track_ok, bbox = tracker.update(frame)

    # Calculate Frames per second (FPS)
    fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)

    # Draw bounding box
    if track_ok:
        # Tracking success
        p1 = (int(bbox[0]), int(bbox[1]))
        p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
        center_x = int(bbox[0]+bbox[2]/2)
        center_y = int(bbox[1]+bbox[3]/2)
        cv2.rectangle(frame, p1, p2, (255, 0, 0), 2, 1)
        cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
        Postion_x = int(center_x*100)
        Postion_y = int(center_y*100)
    elif not track_start:
        cv2.putText(frame, "No tracking target selected", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
        Postion_x = 8000
        Postion_y = 6000
    elif not track_ok:
        cv2.putText(frame, "Tracking failure detected", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
        Postion_x = 8000
        Postion_y = 6000
    
    Uart_buf = bytearray([0x55,0xAA,0x10,0x01,0x00,0x00,0x00,Postion_x>>8,Postion_x and 0x00ff,
                                Postion_y>>8,Postion_y and 0x00ff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xAA])
    port.write(Uart_buf)

    # Display tracker type on frame
    cv2.putText(frame, "KCF Tracker", (0, 0), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2);

    # Display FPS on frame
    cv2.putText(frame, "FPS : " + str(int(fps)), (0, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2);

    # Display result
    cv2.imshow("KCFTracker", frame)
    print(Postion_x,Postion_y)
    # Exit if ESC pressed
    k = cv2.waitKey(1) & 0xff
    if k == 27:
        break

fps.stop()
cv2.destroyAllWindows()
vs.stop()
