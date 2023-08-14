import RPi.GPIO as GPIO
from time import sleep
from imutils.video import VideoStream
from pyzbar import pyzbar
import argparse
import datetime
import imutils
import time
import cv2
import rospy
import math
import numpy
import os
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from gpiozero import Robot
from gpiozero import Servo
import time

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

ap = argparse.ArgumentParser()
ap.add_argument("-o", "--output", type=str, default="barcodes.csv",
        help="path to output CSV file containing barcodes")
args = vars(ap.parse_args())
print("[INFO] starting video stream...")

vs = cv2.VideoCapture("/dev/video2")       
vs.set(cv2.CAP_PROP_FRAME_WIDTH, 160)
vs.set(cv2.CAP_PROP_FRAME_HEIGHT, 120)

a = 0
b = 0
c = None
turn_flag = 0
avoid_flag = 0
road_flag = 0



def callback(msg):
    #time.sleep(1)
    global a
    global b
    global c
    global turn_flag
    global avoid_flag
    global road_flag
    os.system('clear')
    list_till_min=[]
    list_after_min=[]

    ret, frame = vs.read()
    barcodes = pyzbar.decode(frame)
    min_distance = min(msg.ranges)*100
    print("Distance of closest object is ",min_distance," cm")

    # Finding angle corresponding to closest object
    for i in range(0,len(msg.ranges)):
        list_till_min.append(msg.ranges[i]*100)
        if min_distance==msg.ranges[i]*100: 
            break
    index_till_min=i
    angle=i*0.017
    # converting angle in degrees
    print('Angle of closest object is :',(numpy.degrees(angle))," degrees")

    for barcode in barcodes:
        barcodeData = barcode.data.decode("utf-8")
        # QR코드의 각 지점 0 ~ 8
        if barcodeData == "0":
            print("0")
            if road_flag == 0:
                c = 0
                avoid_flag = 0 # avoid_flag = 0 >> dummy 값 >> 로봇의 구동을 멈춤
                d = 0
        elif barcodeData == "1":
            print("1")
            a = 0
            d = 1
        elif barcodeData == "2":
            print("2")
        elif barcodeData == "3":
            print("3")
        elif barcodeData == "4":
            print("4")
        elif barcodeData == "5":
            print("5")
        elif barcodeData == "6":
            print("6")
        elif barcodeData == "7":
            print("7")
        elif barcodeData == "8":
            print("8")
            # avoid_flag = 0 >> dummy 값 >> 로봇의 구동을 멈춤
            if road_flag == 0:
                a = 8
                c = 8
                avoid_flag = 0
       # 연석을 밟았을 때
        elif barcodeData == "9":
            print("9")
            # road_flag = 0 >> default 값
            if a < 4 and road_flag == 0: 
                b = 1
            if a > 4 and road_flag == 0:
                b = 2
        if barcodeData not in found:
            csv.write("{},{}\n".format(datetime.datetime.now(), barcodeData))
            csv.flush()
            found.add(barcodeData)
    # 현재 a와 b 값 출력
    print("a")
    print((a))
    print("turn_flag")
    print((turn_flag))
    print("b")
    print((b))

    cv2.imshow("Barcode Scanner", frame)
    key = cv2.waitKey(1) & 0xFF

    #구동 시작하면 일단 직진
    GPIO.output(DIG1, GPIO.LOW)
    GPIO.output(DIG2, GPIO.HIGH)
    p1.start(50)
    p2.start(50)

    if c == 0:  # 시작 지점에 있는 반환점을 밟았을 때
        print("Rotate")
        # QR코드 인식 탈출을 위해 후진
        GPIO.output(DIG1, GPIO.HIGH)
        GPIO.output(DIG2, GPIO.LOW)
        p1.start(0)
        p2.start(0)
        time.sleep(2.5)
        # 제자리 회전
        #GPIO.output(DIG1, GPIO.HIGH)
        #GPIO.output(DIG2, GPIO.HIGH)
        #p1.start(50)
        #p2.start(50)
        #time.sleep(5)
        # c에 dummy값으로 변환, a = 0 으로 변환
        c = 1
        a = 0
        road_flag = 1
    if c == 8:  # 끝 지점에 있는 반환점을 밟았을 때
        print("Rotate")
        # QR코드 인식 탈출을 위해 후진
        GPIO.output(DIG1, GPIO.HIGH)
        GPIO.output(DIG2, GPIO.LOW)
        p1.start(50)
        p2.start(50)
        time.sleep(2.0)
        # 제자리 회전
        GPIO.output(DIG1, GPIO.LOW)
        GPIO.output(DIG2, GPIO.LOW)
        p1.start(50)
        p2.start(50)
        time.sleep(4.4)
        # c에 dummy값으로 변환, a = 8 로 변환
        c = 7
        a = 8
        road_flag = 1 
    if b == 1:  # 시작 -> 끝 루트의 연석을 밟았을 때
        print("Road Stone")
        # QR코드 인식 탈출을 위해 후진
        if numpy.degrees(angle) > 180 and numpy.degrees(angle) < 200:
            GPIO.output(DIG1, GPIO.HIGH)
            GPIO.output(DIG2, GPIO.LOW)
            p1.start(0)
            p2.start(0)
            turn_flag = 1
        if numpy.degrees(angle) < 180 or numpy.degrees(angle) > 200:
            GPIO.output(DIG1, GPIO.HIGH)
            GPIO.output(DIG2, GPIO.LOW)
            p1.start(40)
            p2.start(40)
    if b == 1 and turn_flag == 1:
        if numpy.degrees(angle) > 680 or numpy.degrees(angle) < 10:
        # 좌측 방향으로 제자리 회전
            GPIO.output(DIG1, GPIO.LOW)
            GPIO.output(DIG2, GPIO.LOW)
            p1.start(0)
            p2.start(0)
            b = 0
            road_flag = 0
            turn_flag = 0
        if numpy.degrees(angle) < 680 and numpy.degrees(angle) > 10:
        # 좌측 방향으로 제자리 회전
            GPIO.output(DIG1, GPIO.LOW)
            GPIO.output(DIG2, GPIO.LOW)
            p1.start(40)
            p2.start(40)

    if b == 2: # 끝 -> 시작 루트의 연석을 밟았을 때
        print("Road Stone")
        # QR코드 인식 탈출을 위해 후진
        if numpy.degrees(angle) > 510 and numpy.degrees(angle) < 540:
            GPIO.output(DIG1, GPIO.HIGH)
            GPIO.output(DIG2, GPIO.LOW)
            p1.start(0)
            p2.start(0)
            turn_flag = 1
        if numpy.degrees(angle) < 510 or numpy.degrees(angle) > 540:
            GPIO.output(DIG1, GPIO.HIGH)
            GPIO.output(DIG2, GPIO.LOW)
            p1.start(40)
            p2.start(40)
    if b == 2 and turn_flag == 1:
        # 우측 방향으로 제자리 회전
        if numpy.degrees(angle) > 690 or numpy.degrees(angle) < 10:
            GPIO.output(DIG1, GPIO.LOW)
            GPIO.output(DIG2, GPIO.LOW)
            p1.start(0)
            p2.start(0)
            b = 0
            road_flag = 0
            turn_flag = 0
        if numpy.degrees(angle) < 690 and numpy.degrees(angle) > 10:
        # 좌측 방향으로 제자리 회전
            GPIO.output(DIG1, GPIO.HIGH)
            GPIO.output(DIG2, GPIO.HIGH)
            p1.start(40)
            p2.start(40)

    # 장애물과의 거리가 70cm 미만 일 때:
    if min_distance < 70:
        avoid_flag = 1
    if min_distance < 20:
        GPIO.output(DIG1, GPIO.LOW)
        GPIO.output(DIG2, GPIO.LOW)
        p1.start(0)
        p2.start(0)
    if avoid_flag == 1:
        if a == 0 and turn_flag == 0 and min_distance < 50:
            # 전방(320 ~ 380 degree)에 장애물이 있다면
            if numpy.degrees(angle) > 320 and numpy.degrees(angle) < 360:
                print("Front")
                # 일단 정지 후 turn_flag = 1로 변환
                GPIO.output(DIG1, GPIO.LOW)
                GPIO.output(DIG2, GPIO.HIGH)
                p1.start(0)
                p2.start(0)
                time.sleep(1)
                turn_flag = 1
            # 그 외의 상황일 경우 직진
            if numpy.degrees(angle) < 320 or numpy.degrees(angle) > 360:
                GPIO.output(DIG1, GPIO.LOW)
                GPIO.output(DIG2, GPIO.HIGH)
                p1.start(50)
                p2.start(50)
        if a == 0 and turn_flag == 1:
            # 좌측 방향으로 제자리 회전 후
            GPIO.output(DIG1, GPIO.LOW)
            GPIO.output(DIG2, GPIO.LOW)
            p1.start(40)
            p2.start(40)
            # 우측(155 ~ 195 degree)에 장애물이 보인다면 정지
            if numpy.degrees(angle) > 180 and numpy.degrees(angle) < 185:
                GPIO.output(DIG1, GPIO.HIGH)
                GPIO.output(DIG2, GPIO.HIGH)
                p1.start(0)
                p2.start(0)
                time.sleep(0.25)
                # a = 1 루트 변환 / turn_flag = 0 초기화
                a = 1
                turn_flag = 0
        if a == 1 and turn_flag == 0:
            # 우 후방(110 ~ 160 degree)에 장애물이 있다면
            if numpy.degrees(angle) > 75 and numpy.degrees(angle) < 115:
                print("Right Back")
                # turn_flag = 1 로 변환
                turn_flag = 1
            # 그 외의 상황일 경우 직진
            if numpy.degrees(angle) < 75 or numpy.degrees(angle) > 115:
                GPIO.output(DIG1, GPIO.LOW)
                GPIO.output(DIG2, GPIO.HIGH)
                p1.start(50)
                p2.start(50)
        if a == 1 and turn_flag == 1:
            # 우측 방향으로 제자리 회전 후
            GPIO.output(DIG1, GPIO.HIGH)
            GPIO.output(DIG2, GPIO.HIGH)
            p1.start(40)
            p2.start(40)
            # 우측(145 ~ 205 degree)에 장애물이 있다면
            if numpy.degrees(angle) > 235 and numpy.degrees(angle) < 260:
                # 정지 후 a = 2, turn_flag = 0 변환
                GPIO.output(DIG1, GPIO.HIGH)
                GPIO.output(DIG2, GPIO.HIGH)
                p1.start(0)
                p2.start(0)
                time.sleep(0.25)
                a = 2
                turn_flag = 0
        if a == 2 and turn_flag == 0:
            # 우 후방에 장애물이 있다면 turn_flag = 1로 변환
            # 차량이 일직선으로 가는 경우(경로 이탈 X)
            if numpy.degrees(angle) > 75 and numpy.degrees(angle) < 95:
                print("Right Back")
                turn_flag = 1
            # 그 외의 상황일 경우 직진
            if numpy.degrees(angle) < 75 or numpy.degrees(angle) > 95:
                GPIO.output(DIG1, GPIO.LOW)
                GPIO.output(DIG2, GPIO.HIGH)
                p1.start(50)
                p2.start(50)
        if a == 2 and turn_flag == 1:
            # 우측 방향으로 제자리 회전 후
            GPIO.output(DIG1, GPIO.HIGH)
            GPIO.output(DIG2, GPIO.HIGH)
            p1.start(40)
            p2.start(40)
            # 우측(145 ~ 205 degree)에 장애물이 있다면
            if numpy.degrees(angle) > 230 and numpy.degrees(angle) < 285:
                # 정지 후 a = 3, turn_flag = 0 변환
                time.sleep(0.25)
                a = 3
                turn_flag = 0

        #if a == 3 and turn_flag == 0:
            # 우 후방에 장애물이 있다면 turn_flag = 1로 변환
            #if numpy.degrees(angle) > 77.5 and numpy.degrees(angle) < 87.5:
                #print("Right Back")
                #turn_flag = 1
            # 그 외의 상황일 경우 직진
            #if numpy.degrees(angle) < 77.5 or numpy.degrees(angle) > 87.5:
                #robby.forward(speed=0.5)
                #motor.forward(speed=0.5)

        #if a == 3 and turn_flag == 1:
            # 우측 방향으로 제자리 회전 후
            #robby.forward(speed=1)
            #motor.backward(speed=1)
            # 우측(145 ~ 205 degree)에 장애물이 있다면
            #if numpy.degrees(angle) > 220 and numpy.degrees(angle) < 250:
                # 정지 후 a = 4, turn_flag = 0 변환
                #robby.forward(speed=0)
                #motor.forward(speed=0)
                #time.sleep(0.5)
                #a = 3.5
                #turn_flag = 0
                #robby.forward(speed=0.5)
                #motor.forward(speed=0.5)

        if a == 8 and turn_flag == 0 and min_distance < 140:
            # 전방(320 ~ 380 degree)에 장애물이 있다면
            if numpy.degrees(angle) > 320 and numpy.degrees(angle) < 380:
                turn_flag = 1
                print("Front")
                # 일단 정지
                GPIO.output(DIG1, GPIO.HIGH)
                GPIO.output(DIG2, GPIO.HIGH)
                p1.start(0)
                p2.start(0)
                time.sleep(0.5)
            # 그 외의 상황일 경우 직진
            if numpy.degrees(angle) < 320 or numpy.degrees(angle) > 380:
                GPIO.output(DIG1, GPIO.LOW)
                GPIO.output(DIG2, GPIO.HIGH)
                p1.start(50)
                p2.start(50)
        if a == 8 and turn_flag == 1:
            # 우측 방향으로 제자리 회전 후
            GPIO.output(DIG1, GPIO.HIGH)
            GPIO.output(DIG2, GPIO.HIGH)
            p1.start(40)
            p2.start(40)
            # 좌측(495 ~ 555 degree)에 장애물이 있다면
            if numpy.degrees(angle) > 500 and numpy.degrees(angle) < 525:
                # 정지 후 a = 7, turn_flag = 0 변환
                GPIO.output(DIG1, GPIO.HIGH)
                GPIO.output(DIG2, GPIO.HIGH)
                p1.start(0)
                p2.start(0)
                time.sleep(0.5)
                a = 7
                turn_flag = 0

        if a == 7 and turn_flag == 0:
            # 좌 후방(555 ~ 670 degree)에 장애물이 있다면
            if numpy.degrees(angle) > 560 and numpy.degrees(angle) < 600:
                print("Left Back")
                turn_flag = 1
            # 그 외의 상황일 경우 직진
            if numpy.degrees(angle) < 560 or numpy.degrees(angle) > 600:
                GPIO.output(DIG1, GPIO.LOW)
                GPIO.output(DIG2, GPIO.HIGH)
                p1.start(50)
                p2.start(50)
        if a == 7 and turn_flag == 1:
            # 좌측 방향으로 제자리 회전 후
            GPIO.output(DIG1, GPIO.LOW)
            GPIO.output(DIG2, GPIO.LOW)
            p1.start(40)
            p2.start(40)
            # 좌 전방(380 ~ 495 degree)에 장애물이 있다면
            if numpy.degrees(angle) > 420 and numpy.degrees(angle) < 435:
               # 정지 후 a = 6, turn_flag = 0 변환
                GPIO.output(DIG1, GPIO.HIGH)
                GPIO.output(DIG2, GPIO.HIGH)
                p1.start(0)
                p2.start(0)
                time.sleep(0.25)
                a = 6
                turn_flag = 0

        if a == 6 and turn_flag == 0:
             # 좌 후방(555 ~ 670 degree)에 장애물이 있다면
            if numpy.degrees(angle) > 620 and numpy.degrees(angle) < 670:
                print("Left Back")
                turn_flag = 1
            # 그 외의 상황일 경우 직진
            if numpy.degrees(angle) < 620 or numpy.degrees(angle) > 670:
                GPIO.output(DIG1, GPIO.LOW)
                GPIO.output(DIG2, GPIO.HIGH)
                p1.start(50)
                p2.start(50)
        if a == 6 and turn_flag == 1:
            # 좌측 방향으로 제자리 회전 후
            GPIO.output(DIG1, GPIO.LOW)
            GPIO.output(DIG2, GPIO.LOW)
            p1.start(40)
            p2.start(40)
            # 좌 전방(380 ~ 495 degree)에 장애물이 있다면
            if numpy.degrees(angle) > 380 and numpy.degrees(angle) < 495:
                # 정지 후 a = 5, turn_flag = 0 변환
                GPIO.output(DIG1, GPIO.HIGH)
                GPIO.output(DIG2, GPIO.HIGH)
                p1.start(0)
                p2.start(0)
                time.sleep(0.25)
                a = 5
                turn_flag = 0

        if road_flag == 1:
            road_flag == 0
        if c == 1 or c == 7:
            c = None
        #if cv2.waitKey(1) & 0xFF:
            #p1.start(0)
            #p2.start(0)
def rplidar_scan_trial():
        rospy.init_node('rplidar_listener')  #Initializing subscriber node
        rospy.Subscriber('/scan', LaserScan, callback) 
        #rospy.sleep(0.05)
        rospy.spin()



if __name__ == '__main__':
    rplidar_scan_trial()

