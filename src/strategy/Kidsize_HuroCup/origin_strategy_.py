#!/usr/bin/env python
#coding=utf-8
import rospy
import numpy as np
from Python_API import Sendmessage
import time
import cv2
imgdata = [[None for high in range(240)]for wight in range(320)]
aa = True
def camera():
    #cap = cv2.VideoCapture(6)
    STRAIGHT_casecade = cv2.CascadeClassifier("/home/iclab/Desktop/MAR/src/strategy/Parameter/cascade2Straight.xml")
    RIGHT_casecade = cv2.CascadeClassifier("/home/iclab/Desktop/MAR/src/strategy/Parameter/cascade2Right.xml")
    LEFT_casecade = cv2.CascadeClassifier("/home/iclab/Desktop/MAR/src/strategy/Parameter/cascade2Left.xml")
    s=0
    r=0
    l=0
  # 從攝影機擷取一張影像
    #ret, frame = cap.read()
    frame = send.originimg
# 顯示圖片
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.equalizeHist(gray)


    STRAIGHT= STRAIGHT_casecade.detectMultiScale(gray,1.1, 2,0,(10,10))
    RIGHT= RIGHT_casecade.detectMultiScale(gray,1.1, 2,0,(10,10))
    LEFT= LEFT_casecade.detectMultiScale(gray,1.1, 2,0,(10,10))
    if len(STRAIGHT)>0:
        for a in STRAIGHT:  # 單獨框出每一張人臉
            x, y, w, h = a
            cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
            s+=1
            r=0
            l=0
            print("straight")
    elif len(RIGHT)>0:
        for a in RIGHT:  # 單獨框出每一張人臉
            x, y, w, h = a
            cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
            s=0
            r+=1
            l=0
            print("right")
    elif len(LEFT)>0:
        for a in LEFT:  # 單獨框出每一張人臉
            x, y, w, h = a
            cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
            s=0
            r=0
            l+=1
            print("left")
    if s==10:
        s=0
    elif r==10:
        r=0
    elif l==10:
        l=0
    #cv2.imshow('frame', frame)
    #cv2.waitKey(1)
def theta():
    s = 0
    s = calculate()
    theta=0
    #walk straight
    if -0.1 < s < 0.1:
        theta = 0
    #turn right
    elif -0.2 < s < -0.1:
        theta = -1
    elif -0.4 < s < -0.2:
        theta = -2
    elif -0.6 < s < -0.4:
        theta = -3
    elif -1 < s < -0.6:
        theta = -4
    elif -100 < s < -1:
        theta = -5
    elif s == -100:
        theta = -7
    #turn left 
    elif 0.2 > s > 0.1:
        theta = 1
    elif 0.4 > s > 0.2:
        theta = 2
    elif 0.6 > s > 0.4:
        theta = 3
    elif 1 > s > 0.6:
        theta = 4
    elif 100> s > 1:
        theta = 5
    elif s == 100:
        theta = 7
    return theta
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
    return slope
if __name__ == '__main__':
    try:
        send = Sendmessage()
        while not rospy.is_shutdown():
            if send.is_start == True and aa == True:
                send.sendBodyAuto(0,0,0,0,1,0)
                send.sendHeadMotor(2,1300,50)
                while 1:
                    #camera()
                    T=theta()
                    print(T)
                    send.sendContinuousValue(1000,0,0,T,0)
                    if send.is_start == False:
                        break
                aa = False
            if send.is_start == False and aa == False:
                send.sendBodyAuto(0,0,0,0,1,0)
                aa = True
            


    except rospy.ROSInterruptException:
        pass
