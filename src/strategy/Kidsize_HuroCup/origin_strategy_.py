#!/usr/bin/env python
#coding=utf-8
import rospy
import numpy as np
from Python_API import Sendmessage
import time
import cv2

imgdata = [[None for high in range(240)]for wight in range(320)]
aa = True
def imu_yaw():
    yaw = 1.5
    if yaw<0.0:
            yaw+=90.0
    elif yaw>0.0:
            yaw-=90.0
    return yaw

def imu_go(yaw):
    Y, R, L=camera()
    print(yaw)
    if yaw<=0.0:
        print(yaw)
        yaw+=90.0
        yaw_origin=yaw
    elif yaw>0.0:
        print(yaw)
        yaw-=90.0
        yaw_origin=yaw
    if L==1:
        while 1:
            yaw = imu_yaw()
            send.sendContinuousValue(100,0,0,7,0)
            if  yaw < yaw_origin+90.0:
                break
    elif R==1:
        while 1:
            yaw = imu_yaw()
            send.sendContinuousValue(100,0,0,-7,0)
            if  yaw < yaw_origin-90.0:
                break
    else:
        yaw = imu_yaw()
        send.sendContinuousValue(1000,0,0,0,0)
        while 1:
            Y, R, L = camera()
            if yaw-yaw_origin > 10.0:
                send.sendContinuousValue(800,0,0,-2,0)
            elif yaw-yaw_origin > -10.0:
                send.sendContinuousValue(800,0,0,2,0)
            elif R==1 or L==1:
                break
            else:
                send.sendContinuousValue(1000,0,0,0,0)

def camera():
    #cap = cv2.VideoCapture(7)
    STRAIGHT_casecade = cv2.CascadeClassifier("/home/iclab/Desktop/MAR/src/strategy/Parameter/cascade2Straight.xml")
    RIGHT_casecade = cv2.CascadeClassifier("/home/iclab/Desktop/MAR/src/strategy/Parameter/cascade2Right.xml")
    LEFT_casecade = cv2.CascadeClassifier("/home/iclab/Desktop/MAR/src/strategy/Parameter/cascade2Left.xml")
    s=0
    r=0
    l=0
    Yes=0
    right=0
    left=0
    #ret, frame = cap.read()
    frame = send.originimg
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.equalizeHist(gray)


    STRAIGHT= STRAIGHT_casecade.detectMultiScale(gray,1.1, 5,0,(8,8))
    RIGHT= RIGHT_casecade.detectMultiScale(gray,1.1, 5,0,(8,8))
    LEFT= LEFT_casecade.detectMultiScale(gray,1.1, 5,0,(8,8))
    if len(STRAIGHT)>0:
        for a in STRAIGHT:  # 單獨框出每一張人臉
            x, y, w, h = a
            cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
        s+=1
        r=0
        l=0

    elif len(RIGHT)>0:
        for a in RIGHT:  # 單獨框出每一張人臉
            x, y, w, h = a
            cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
        s=0
        r+=1
        l=0

    elif len(LEFT)>0:
        for a in LEFT:  # 單獨框出每一張人臉
            x, y, w, h = a
            cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
        s=0
        r=0
        l+=1

    if s>=10:
        s=0
        print("straight")
        send.drawImageFunction(1,1,x,x+w,y,y+h,255,0,0)
        Yes=1
    elif r>=10:
        r=0
        print("right")
        send.drawImageFunction(1,1,x,x+w,y,y+h,255,0,0)
        Yes=1
        right=1
    elif l>=10:
        l=0
        print("left")
        send.drawImageFunction(1,1,x,x+w,y,y+h,255,0,0)
        Yes=1
        left=1
    return Yes, right, left
def theta():
    s = 0
    s ,out= calculate()
    theta=0
    speed=0
    #walk straight
    if -0.1 < s < 0.1:
        theta = 0
        speed = 1400
    #turn right
    elif -0.2 < s < -0.1:
        theta = -1
        speed = 1200
    elif -0.4 < s < -0.2:
        theta = -2
        speed = 1000
    elif -0.6 < s < -0.4:
        theta = -3
        speed = 800
    elif -1 < s < -0.6:
        theta = -4
        speed = 600
    elif -100 < s < -1:
        theta = -5
        speed = 400
    elif s == -100:
        theta = -7
        speed = 100
    #turn left 
    elif 0.2 > s > 0.1:
        theta = 1
        speed = 1200
    elif 0.4 > s > 0.2:
        theta = 2
        speed = 1000
    elif 0.6 > s > 0.4:
        theta = 3
        speed = 800
    elif 1 > s > 0.6:
        theta = 4
        speed = 600
    elif 100> s > 1:
        theta = 5
        speed = 400
    elif s == 100:
        theta = 7
        speed = 100
    return theta, speed, out
def calculate():
    cnt1=0
    cnt2=0
    cnt3=0
    total_x1=0
    total_y1=0
    total_x2=0
    total_y2=0
    total_x3=0
    total_y3=0
    center_x1=0
    center_y1=0
    center_x2=0
    center_y2=0
    center_x3=0
    center_y3=0
    slope=0
    out=0
    a=0
    b=0
    for high in range(240):
        for wight in range(320):
            imgdata[wight][high]=send.Label_Model[high*320+wight]
            if 30 <high < 100:
                if imgdata[wight][high] != 0:
                    total_x1+=wight
                    total_y1+=high
                    cnt1+=1
            elif 100 < high < 170:
                if imgdata[wight][high] != 0:
                    total_x2+=wight
                    total_y2+=high
                    cnt2+=1
            elif high > 170:
                if imgdata[wight][high] != 0:
                    total_x3+=wight
                    total_y3+=high
                    cnt3+=1
    if cnt1!=0 and cnt1 > 100:#stop cnt = 0
        center_x1=total_x1/cnt1
        center_y1=total_y1/cnt1
    if cnt2!=0 and cnt2 > 100:#stop cnt = 0
        center_x2=total_x2/cnt2
        center_y2=total_y2/cnt2
    if cnt3!=0 and cnt3 > 100:#stop cnt = 0
        center_x3=total_x3/cnt3
        center_y3=total_y3/cnt3
    a=int(center_x1)
    b=int(center_y1)
    c=int(center_x2)
    d=int(center_y2)
    e=int(center_x3)
    f=int(center_y3)
    if center_y2-center_y3==0 or  center_y1-(center_y2+center_y3)/2==0: #no line
        print("None")
        slope = 0
    elif center_x1==0 and center_x2==0 and center_y1==0 and center_y2==0:
        out=1
        if center_x3 > 240:
                print("right")
                slope=-100
        elif center_x3 < 180:
                print("left")
                slope=100
    else:
        if center_x1==0 and center_y1==0:#first part don't have line
            slope = (center_x3-center_x2)/(center_y3-center_y2)
            send.drawImageFunction(2,0,c,e,d,f,0,0,0)
        elif center_x3==0 and center_y3==0:
            slope = (center_x2-center_x1)/(center_y2-center_y1)
            send.drawImageFunction(2,0,a,c,b,d,0,0,0)
        else:
            slope = (center_x3-(center_x1+center_x2)/2)/(center_y3-(center_y1+center_y2)/2)
            h=int((center_x1+center_x2)/2)
            i=int((center_y1+center_y2)/2)
            send.drawImageFunction(2,0,e,h,f,i,0,0,0)
    print(slope)
    return slope ,out
if __name__ == '__main__':
    try:
        send = Sendmessage()
        while not rospy.is_shutdown():
            send.sendHeadMotor(1,2048,50)
            send.sendHeadMotor(2,1400,50)
            if send.is_start == True and aa == True:
                send.sendBodyAuto(0,0,0,0,1,0)
                while 1:
                    # while 1:
                    #     T, S, O=theta()
                    #     Y, R, L = camera()
                    #     print(O)
                    #     print(Y)
                    #     send.sendContinuousValue(S,0,0,T,0)
                    #     if send.is_start == False:
                    #         break
                    #     elif Y==1 and O==1:
                    #         send.sendContinuousValue(0,0,0,0,0)
                    #         break
                    # yaw = send.imu_value_Yaw
                    # imu_go(send.imu_value_Yaw)
                    
                    print(send.imu_value_Yaw)
                    time.sleep(1)
                    send.sendSensorReset()
                    print(send.imu_value_Yaw)
                    time.sleep(1)
                    if send.is_start == False:
                        break
                aa = False
            if send.Web == False and aa == False:
                send.sendBodyAuto(0,0,0,0,1,0)
                aa = True
            


    except rospy.ROSInterruptException:
        pass
