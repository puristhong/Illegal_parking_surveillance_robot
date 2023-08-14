from imutils.video import VideoStream
import cv2
import os
import pytesseract
import time
from pytesseract import Output
#from imutils.perspective import four_point_transform
#from imutils.contours import sort_contours
#import matplotlib.pyplot as plt
#import imutils
#import re
#import requests
#import numpy as np
from pytesseract import image_to_string
import RPi.GPIO as GPIO
#from gpiozero import Robot

GPIO.setmode(GPIO.BCM)                  # GPIO numbering
GPIO.setwarnings(False)                 # enable warning from GPIO
AN2 = 27                                # set pwm2 pin on MD10-Hat
AN1 = 18                                # set pwm1 pin on MD10-hat
DIG2 = 8                                # set dir2 pin on MD10-Hat
DIG1 = 7                                # set dir1 pin on MD10-Hat
GPIO.setup(AN2, GPIO.OUT)               # set pin as output
GPIO.setup(AN1, GPIO.OUT)               # set pin as output
GPIO.setup(DIG2, GPIO.OUT)              # set pin as output
GPIO.setup(DIG1, GPIO.OUT)              # set pin as output
time.sleep(1)                                # delay for 1 seconds
p1 = GPIO.PWM(AN1, 100)                 # set pwm for M1
p2 = GPIO.PWM(AN2, 100) 
#file_path = 'jpg'
cap = cv2.VideoCapture("/dev/video0")
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
#cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

#motor = Robot(left=(8, 7), right=(9, 10))
#robby = Robot(left=(16, 12), right=(20, 21))
GPIO.output(DIG1, GPIO.LOW)
GPIO.output(DIG2, GPIO.HIGH)
p1.start(0)
p2.start(0)
while True:
    # Capture frame-by-frame
    #motor.forward(speed=0.5)
    #robby.forward(speed=0.5)
    ret, cam = cap.read()
    cong = r'--oem 1 --psm 4 outputbase digits'
    abc = pytesseract.image_to_string(cam, lang='eng', config=cong)
    d = pytesseract.image_to_data(cam, output_type=Output.DICT)

    n_boxes = len(d['text'])
    for i in range(n_boxes):
        if int(d['conf'][i]) > 60:
            (text, x, y, w, h) = (d['text'][i], d['left'][i], d['top'][i], d['width'][i], d['height'][i])
            # don't show empty text
            if text and text.strip() != "":
                cam = cv2.rectangle(cam, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cam = cv2.putText(cam, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)

    if abc != None:
        #ret, dst2 = cap.read()
        suffix = ".jpg"
        filename = " ".join([abc, suffix])
        cv2.imwrite(filename, cam)
        print(filename)

    # Display the resulting frame
    cv2.imshow('frame', cam)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
 
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

