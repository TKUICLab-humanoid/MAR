#!/usr/bin/env python
#coding=utf-8
from traceback import print_tb
import rospy
import numpy as np
from Python_API import Sendmessage
import time
import cv2

imgdata = [[None for high in range(240)]for wight in range(320)]
aa = True
def imu_right(flag, cnt,origin_theta,origin_Y):#90度右轉
    if cnt==0:#第一次進入，Reset YAW值
        send.sendSensorReset()
        cnt=1
    flag=0
    yaw = send.imu_value_Yaw
    print('trun right')
    send.sendContinuousValue(900,origin_Y,0,-5+origin_theta,0)
    if  yaw < -90:#成功右轉90度
        print("end")
        send.sendSensorReset()
        flag=1
        cnt=0
    return flag, cnt
def imu_left(flag,cnt,origin_theta,origin_Y):#90度左轉
    if cnt==0:#第一次進入，Reset YAW值
        send.sendSensorReset()
        cnt=1
    flag=0
    yaw = send.imu_value_Yaw
    print('trun left')
    send.sendContinuousValue(900,origin_Y,0,5+origin_theta,0)
    if  yaw > 90:#成功左轉90度
        print("end")
        send.sendSensorReset()
        flag=1
        cnt=0
    return flag, cnt
def imu_go(S, origin_theta,cnt):#直走4
    if cnt==0:#第一次進入，Reset YAW值
        send.sendSensorReset()
        cnt=1
    T=0
    print("go go go!")
    yaw = send.imu_value_Yaw
    # print("go")
    if 5 > yaw > 3:
        S = 1300
        T = -5+origin_theta
    elif 7 > yaw > 5:
        S = 1100
        T = -4+origin_theta
    elif yaw > 7:
        S = 1000
        T = -5
    elif -5 < yaw < -3:
        S = 1300
        T = 2+origin_theta
    elif yaw < -5:
        S = 1100
        T = 3+origin_theta
    else:
        S += 100
        T = 0+origin_theta
    if S > 1500:
        S = 1500
    return S, T, cnt
def camera(Yes, right, left, s, r, l):#判斷箭頭
    #cap = cv2.VideoCapture(7)
    STRAIGHT_casecade = cv2.CascadeClassifier("/home/iclab/Desktop/MAR/src/strategy/Parameter/cascade2Straight.xml")
    RIGHT_casecade = cv2.CascadeClassifier("/home/iclab/Desktop/MAR/src/strategy/Parameter/cascade2Right.xml")
    LEFT_casecade = cv2.CascadeClassifier("/home/iclab/Desktop/MAR/src/strategy/Parameter/cascade2Left.xml")
    #ret, frame = cap.read()
    frame = send.originimg
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.equalizeHist(gray)
    STRAIGHT= STRAIGHT_casecade.detectMultiScale(gray,1.1, 5,0,(10,10))
    RIGHT= RIGHT_casecade.detectMultiScale(gray,1.1, 5,0,(10,10))
    LEFT= LEFT_casecade.detectMultiScale(gray,1.1, 5,0,(10,10))
    if len(STRAIGHT)>0:
        for a in STRAIGHT:  # 單獨框出每一張人臉
            x, y, w, h = a
            cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
        send.drawImageFunction(1,1,x,x+w,y,y+h,255,0,0)
        s+=1
        r=0
        l=0
        # print("straight")
    elif len(RIGHT)>0:
        for a in RIGHT:  # 單獨框出每一張人臉
            x, y, w, h = a
            cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
        send.drawImageFunction(1,1,x,x+w,y,y+h,255,0,0)
        s=0
        r+=1
        l=0
        # print("right")
    elif len(LEFT)>0:
        for a in LEFT:  # 單獨框出每一張人臉
            x, y, w, h = a
            cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
        send.drawImageFunction(1,1,x,x+w,y,y+h,255,0,0)
        s=0
        r=0
        l+=1
        # print("left")
    if s>=20:#成功連續判斷20次
        s=0
        print("go Straight")
        send.drawImageFunction(1,1,x,x+w,y,y+h,255,0,0)
        Yes=1
        right=0
        left=0
    elif r>=20:
        r=0
        print("go Right")
        send.drawImageFunction(1,1,x,x+w,y,y+h,255,0,0)
        Yes=1
        right+=1
        left=0
    elif l>=20:
        l=0
        print("go Left")
        send.drawImageFunction(1,1,x,x+w,y,y+h,255,0,0)
        Yes=1
        right=0
        left+=1
    return Yes, right, left, s, r, l
def theta(origin_theta):#判斷斜率
    s = 0
    s ,out, turn_right, turn_left= calculate()
    theta=0
    speed=0
    if turn_right==1:
        theta = -6
        speed = 1000
    elif turn_left==1:
        theta = 6
        speed = 1000
    else:
        #walk straight
        if -0.1 < s < 0.1:
            theta = 0+origin_theta
            speed = 1500
        #turn right
        elif -0.2 < s < -0.1:
            theta = -3+origin_theta
            speed = 1400
        elif -0.4 < s < -0.2:
            theta = -4+origin_theta
            speed = 1200
        elif -0.6 < s < -0.4:
            theta = -5+origin_theta
            speed = 1000
        elif -1 < s < -0.6:
            theta = -6+origin_theta
            speed = 800
        elif -100 < s < -1:
            theta = -7+origin_theta
            speed = 600
        elif s == -100:
            theta = -9+origin_theta
            speed = 200
        #turn left 
        elif 0.2 > s > 0.1:
            theta = 2+origin_theta
            speed = 1400
        elif 0.4 > s > 0.2:
            theta = 3+origin_theta
            speed = 1200
        elif 0.6 > s > 0.4:
            theta = 4+origin_theta
            speed = 1000
        elif 1 > s > 0.6:
            theta = 5+origin_theta
            speed = 800
        elif 100> s > 1:
            theta = 6+origin_theta
            speed = 600
        elif s == 100:
            theta = 8+origin_theta
            speed = 200
    return theta, speed, out
def calculate():#計算斜率
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
    turn_left=0
    turn_right=0
    for high in range(240):
        for wight in range(320):
            imgdata[wight][high]=send.Label_Model[high*320+wight]
            if 30 < high < 100:
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
    if cnt1 > 50:#去除雜訊點
        center_x1=total_x1/cnt1
        center_y1=total_y1/cnt1
    if cnt2 > 50:
        center_x2=total_x2/cnt2
        center_y2=total_y2/cnt2
    if cnt3 > 50:
        center_x3=total_x3/cnt3
        center_y3=total_y3/cnt3
    a=int(center_x1)
    b=int(center_y1)
    c=int(center_x2)
    d=int(center_y2)
    e=int(center_x3)
    f=int(center_y3)
    if center_y2-center_y3==0 or  center_y1-(center_y2+center_y3)/2==0: #找不到線
        print("None")
        slope = 0
    elif center_x1==0 and center_x2==0 and center_y1==0 and center_y2==0:#機器人偏移或是已經要進入第二階段
        if center_x3 > 240:
            slope=-100
        elif center_x3 < 180:
            slope=100
    else:#計算斜率
        if center_x3 < 80 and center_x2 < 80:
                turn_left=1
        elif center_x3 > 240 and center_x2 > 240:
                turn_right=1
        if center_x1==0 and center_y1==0:#first part don't have line
            slope = (center_x3-center_x2)/(center_y3-center_y2)
            send.drawImageFunction(2,0,c,e,d,f,0,0,0)
            out=1#進入第二階段的指標，線在機器人螢幕的正下方
        elif center_x3==0 and center_y3==0:
            slope = (center_x2-center_x1)/(center_y2-center_y1)
            send.drawImageFunction(2,0,a,c,b,d,0,0,0)
        else:
            if center_x1 < 80 and center_x2 < 80:
                turn_left=1
            elif center_x1 > 240 and center_x2 > 240:
                turn_right=1
            slope = (center_x3-(center_x1+center_x2)/2)/(center_y3-(center_y1+center_y2)/2)
            h=int((center_x1+center_x2)/2)
            i=int((center_y1+center_y2)/2)
            send.drawImageFunction(2,0,e,h,f,i,0,0,0)
    print(slope)
    return slope ,out ,turn_right, turn_left  
def correct_slope(origin_theta,next):
    s ,out, turn_right, turn_left= calculate()
    theta = 0
    speed = 0
    if -0.1 < s < 0.1:
        theta = 0+origin_theta
        next=1
    #turn right
    elif -0.2 < s < -0.1:
        theta = -3+origin_theta
    elif -0.4 < s < -0.2:
        theta = -4+origin_theta
    elif -0.6 < s < -0.4:
        theta = -5+origin_theta
    elif -1 < s < -0.6:
        theta = -6+origin_theta
    elif -101 < s < -1:
        theta = -8+origin_theta
        speed = -100
    #turn left 
    elif 0.2 > s > 0.1:
        theta = 3+origin_theta
    elif 0.4 > s > 0.2:
        theta = 4+origin_theta
    elif 0.6 > s > 0.4:
        theta = 5+origin_theta
    elif 1 > s > 0.6:
        theta = 6+origin_theta
    elif 101> s > 1:
        theta = 8+origin_theta
        speed = -100
    return next, theta, speed

if __name__ == '__main__':
    try:
        send = Sendmessage()
        while not rospy.is_shutdown():
            send.sendHeadMotor(2,1500,50)
            send.sendHeadMotor(1,2048,50)
            yes=0
            right=0
            left=0
            flag=0
            cnt=0
            cnt_straight=0
            speed=0
            next_stage=0
            s=0
            r=0 
            l=0
            Y=0
            R=0
            L=0
            N=0
            O=0
            origin_theta=-2
            origin_Y=-300
            if send.is_start == True and aa == True:
                send.sendBodyAuto(0,0,0,0,1,0)
                while 1:
                    if Y==1 and O==1:#判斷是否有箭頭與是否要進入第二階段
                        if next_stage==0:
                            N, T, S=correct_slope(origin_theta,next_stage)
                            send.sendContinuousValue(S,origin_Y,0,T,0)
                            next_stage=N
                            print(next_stage)
                        else:
                            Y, R, L, s, r, l=camera(yes, right, left, s, r, l)
                            yes=Y 
                            right=R
                            left=L
                            if R>=3:#多次成功判斷右轉
                                flag_right, cnt=imu_right(flag,cnt,origin_theta,origin_Y)
                                if flag_right==1:#完成90度右轉判斷旗標歸零
                                    right=0
                                    left=0
                                    flag=0
                                    cnt=0
                                    cnt_straight=0
                                    flag_right=0
                            elif L>=3:#多次成功判斷左轉
                                flag_left, cnt=imu_left(flag,cnt,origin_theta,origin_Y)
                                if flag_left==1:#完成90度左轉判斷旗標歸零
                                    right=0
                                    left=0
                                    flag=0
                                    cnt=0
                                    cnt_straight=0
                                    flag_left=0
                            else:#沒有任何判斷就直走
                                S, T, cnt_straight=imu_go(speed, origin_theta,cnt_straight)
                                send.sendContinuousValue(S,origin_Y,0,T,0)
                                speed = S
                    else:
                        T, S, O=theta(origin_theta)
                        Y, R, L, s, r, l = camera(yes, right, left, s, r, l)#判斷箭頭
                        yes=Y 
                        right=R
                        left=L
                        print(O)
                        print(Y)
                        send.sendContinuousValue(S,origin_Y,0,T,0)

                    if send.is_start == False:
                        break
                aa = False
            if send.is_start == False and aa == False:
                send.sendBodyAuto(0,0,0,0,1,0)
                aa = True
            


    except rospy.ROSInterruptException:
        pass
