from imutils.video.pivideostream import PiVideoStream
from imutils.video import FPS
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import imutils
import serial
import time
import cv2 as cv
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

while True:
    frame = vs.read()
    frame = imutils.resize(frame, width=160)
    blurred = cv.GaussianBlur(frame, (11, 11), 0)
    # Convert BGR to HSV
    hsv = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)
    # define range of red color in HSV
    lower_green = np.array([60, 100, 100])
    upper_green = np.array([180, 255, 255])
    # Threshold the HSV image to get only blue colors
    mask = cv.inRange(hsv, lower_green, upper_green)
    mask = cv.erode(mask, None, iterations=2)
    mask = cv.dilate(mask, None, iterations=2)
    cv.imshow('mask', mask)
    cnts = cv.findContours(mask.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[-2]
    center = None
    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv.contourArea)
        ((x, y), radius) = cv.minEnclosingCircle(c)
        print(x, y)
        M = cv.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # only proceed if the radius meets a minimum size
        if radius > 5:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv.circle(frame, center, 5, (0, 0, 255), -1)
            Postion_x = int(x*100)
            Postion_y = int(y*100)
        else:
            Postion_x = 8000
            Postion_y = 6000
    else:
        Postion_x = 8000
        Postion_y = 6000

   

    Uart_buf = bytearray([0x55,0xAA,0x10,0x01,0x00,0x00,0x00,Postion_x>>8,Postion_x and 0x00ff,
                                Postion_y>>8,Postion_y and 0x00ff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xAA])
    port.write(Uart_buf)
    
    cv.imshow('frame', frame)
    print(Postion_x,Postion_y)
    key = cv.waitKey(1) & 0xFF
    if key == ord("q"):
	    break
    fps.update()

fps.stop()
print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))

cv.destroyAllWindows()
vs.stop()
