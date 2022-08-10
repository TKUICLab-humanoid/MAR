#!/usr/bin/env python
#coding=utf-8
from traceback import print_tb
import rospy
import numpy as np
from Python_API import Sendmessage
import time
import cv2
import math
imgdata = [[None for high in range(240)]for wight in range(320)]
start = True
def initial():
    global i, cnt, speed, straight_temp, right_temp, left_temp, turn_left_flag, turn_now_flag, turn_right_flag, finish_turn_left_flag, finish_turn_right_flag, second_part_flag, next_stage_flag, go_to_second_part_flag, origin_theta, origin_Y
    i=0
    speed=0
    straight_temp=0 #成功判斷箭頭暫存
    right_temp=0 
    left_temp=0 
    turn_right_flag=0#成功判斷右轉
    turn_left_flag=0#成功判斷左轉
    finish_turn_left_flag=0#完成左轉
    finish_turn_right_flag=0#完成右轉
    turn_now_flag=0
#----------------------------------------------------------------------
    #第二階段旗標
    second_part_flag=0 #成功判斷銀幕內有箭頭
    go_to_second_part_flag=0#線段只有在銀幕下方
    next_stage_flag=0 #修正是否正對箭頭
#----------------------------------------------------------------------
    #步態初始化
    origin_theta=1
    origin_Y=0

def imu_right(flag,origin_theta,origin_Y):#90度右轉
    flag=0
    yaw = send.imu_value_Yaw
    print('trun right')
    send.sendContinuousValue(1200,origin_Y,0,-7+origin_theta,0)
    send.sendHeadMotor(2,1400,50)
    if  yaw < -83:#成功右轉90度
        print("end")
        send.sendSensorReset()
        flag=1
    return flag
def imu_left(flag,origin_theta,origin_Y):#90度左轉
    flag=0
    yaw = send.imu_value_Yaw
    print('trun left')
    send.sendContinuousValue(1200,origin_Y,0,7+origin_theta,0)
    send.sendHeadMotor(2,1400,50)
    if  yaw > 83:#成功左轉90度
        print("end")
        send.sendSensorReset()
        flag=1
    return flag
def imu_go(origin_theta,arrow_center_x):#直走
    theta=origin_theta+1
    print("go go go!")
    yaw = send.imu_value_Yaw
    speed = 2300
    if 0<arrow_center_x<=140:
        theta=5
        send.sendContinuousValue(speed,origin_Y,0,theta+origin_theta,0)
    elif arrow_center_x>=180:
        theta=-5
        send.sendContinuousValue(speed,origin_Y,0,theta+origin_theta,0)
    else:
        if  yaw > 3:
            theta = -3+origin_theta
            print('right')
        elif yaw < -3:
            theta = 2+origin_theta
            print('left')
    return speed, theta
def camera(straight_temp, right_temp, left_temp):#判斷箭頭
    #cap = cv2.VideoCapture(7)
    center_y=0
    center_x=0
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
        straight_temp+=1
        right_temp=0
        left_temp=0
        center_y=y+h/2 #計算箭頭的y軸位置
        center_x=x+w/2
        # print("straight")
    elif len(RIGHT)>0:
        for a in RIGHT:  # 單獨框出每一張人臉
            x, y, w, h = a
            cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
        send.drawImageFunction(1,1,x,x+w,y,y+h,255,0,0)
        straight_temp=0
        right_temp+=1
        left_temp=0
        center_y=y+h/2 #計算箭頭的y軸位置
        center_x=x+w/2
        # print("right")
    elif len(LEFT)>0:
        for a in LEFT:  # 單獨框出每一張人臉
            x, y, w, h = a
            cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
        send.drawImageFunction(1,1,x,x+w,y,y+h,255,0,0)
        straight_temp=0
        right_temp=0
        left_temp+=1
        center_y=y+h/2#計算箭頭的y軸位置
        center_x=x+w/2
        # print("left")
    
    return straight_temp, right_temp, left_temp, center_y, center_x

def correct_go_to_arrow(origin_theta,i):
    slope , go_to_second_part_flag, correct_walking_right, correct_walking_left, big_turn_right, big_turn_left= calculate()
    theta = 0
    speed = 0
    print('slope:', slope)
    if -0.05 < slope < 0.05:
        theta = 0+origin_theta
        i+=1
    #turn right
    elif -0.4 < slope < -0.05:
        theta = -4+origin_theta
    elif slope < -0.4:
        theta = -5+origin_theta
        speed = 0
    #turn left 
    elif 0.4 > slope > 0.05:
        theta = 5+origin_theta
    elif slope > 0.4:
        theta = 6+origin_theta
        speed = 0

    return theta, speed, i

def arrow_flag(straight_temp, right_temp, left_temp, second_part_flag, turn_right_flag, turn_left_flag):
    if straight_temp>=10:#成功連續判斷20次
        straight_temp=0
        print("go Straight")
        second_part_flag=1
        turn_right_flag=0
        turn_left_flag=0
    elif right_temp>=10:
        right_temp=0
        print("go Right")
        second_part_flag=1
        turn_right_flag+=1
        turn_left_flag=0
    elif left_temp>=5:
        left_temp=0
        print("go Left")
        second_part_flag=1
        turn_right_flag=0
        turn_left_flag+=1
    return second_part_flag, turn_right_flag, turn_left_flag
def theta_value(origin_theta):#判斷斜率
    slope , go_to_second_part_flag, correct_walking_right, correct_walking_left, big_turn_right, big_turn_left= calculate()
    theta=0
    speed=0
    if correct_walking_right==1:
        theta = -8+origin_theta
        speed = 1500
    elif correct_walking_left==1:
        theta = 7+origin_theta
        speed = 1500
    elif big_turn_right==1:
        theta = -9+origin_theta
        speed = 1200
    elif big_turn_left==1:
        theta = 8+origin_theta
        speed = 1200
    else:
        sp=[2500,2500,2400,2400,2300,2300,2200,2200,2100]
        th=[0,2,3,4,4,5,6,6,7]
        #walk straight
        if slope >= 0.9:
            theta = 7+origin_theta
            speed = 1500
        elif slope>=0:
            speed = int(sp[math.floor(slope/0.1)])
            theta = int(th[math.floor(slope/0.1)])+origin_theta
        elif  slope <= -0.9:
            theta = -7+origin_theta
            speed = 1500
        else:
            speed = int(sp[math.floor(-slope/0.1)])
            theta = -int(th[math.floor(-slope/0.1)])+origin_theta
    return theta, speed, go_to_second_part_flag
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
    go_to_second_part_flag=0
    correct_walking_left=0
    correct_walking_right=0
    big_turn_right=0
    big_turn_left=0
    for high in range(240):
        for wight in range(320):
            imgdata[wight][high]=send.Label_Model[high*320+wight]
            if 40 <= high < 110:
                if imgdata[wight][high] != 0:
                    total_x1+=wight
                    total_y1+=high
                    cnt1+=1
            elif 110 <= high < 180:
                if imgdata[wight][high] != 0:
                    total_x2+=wight
                    total_y2+=high
                    cnt2+=1
            elif high >= 180:
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
    center_x1=int(center_x1)
    center_y1=int(center_y1)
    center_x2=int(center_x2)
    center_y2=int(center_y2)
    center_x3=int(center_x3)
    center_y3=int(center_y3)
    if center_y2-center_y3==0 or  center_y1-(center_y2+center_y3)/2==0: #找不到線
        print("None")
        slope = 0
    elif center_x1==0 and center_x2==0 and center_y1==0 and center_y2==0:#機器人偏移或是已經要進入第二階段
        if center_x3 > 240:
            big_turn_right=1
        elif center_x3 < 80:
            big_turn_left=1
    else:#計算斜率
        if center_x3 < 100 and center_x2 < 100:
            correct_walking_left=1
        elif center_x3 > 220 and center_x2 > 220:
            correct_walking_right=1
        if center_x1==0 and center_y1==0:#first part don't have line
            slope = (center_x3-center_x2)/(center_y3-center_y2)
            send.drawImageFunction(2,0,center_x2,center_x3,center_y2,center_y3,0,0,0)
            go_to_second_part_flag=1#進入第二階段的指標，線在機器人螢幕的正下方
        elif center_x3==0 and center_y3==0:
            slope = (center_x2-center_x1)/(center_y2-center_y1)
            send.drawImageFunction(2,0,center_x1,center_x2,center_y1,center_y2,0,0,0)
        else:
            slope = (center_x3-(center_x1+center_x2)/2)/(center_y3-(center_y1+center_y2)/2)
            h=int((center_x1+center_x2)/2)
            i=int((center_y1+center_y2)/2)
            send.drawImageFunction(2,0,center_x3,h,center_y3,i,0,0,0)
    #print(slope)
    return slope , go_to_second_part_flag , correct_walking_right, correct_walking_left, big_turn_right, big_turn_left

if __name__ == '__main__':
    try:
        send = Sendmessage()
        r=rospy.Rate(5)
        while not rospy.is_shutdown():
            if send.is_start == True:
                if start == True:
                    initial()
                    send.sendHeadMotor(2,1600,50)
                    time.sleep(0.5)
                    send.sendHeadMotor(1,2048,50)
                    time.sleep(0.5)
                    send.sendSensorReset()
                    time.sleep(0.5)
                    send.sendBodyAuto(0,0,0,0,1,0)
                    start=False
                else:
                    if second_part_flag==1 and go_to_second_part_flag==1:#判斷是否有箭頭與是否要進入第二階段
                        straight_temp, right_temp, left_temp, arrow_center_y, arrow_center_x=camera(straight_temp, right_temp, left_temp)
                        second_part_flag, turn_right_flag, turn_left_flag=arrow_flag(straight_temp, right_temp, left_temp, second_part_flag, turn_right_flag, turn_left_flag)
                        print('Y:', arrow_center_y)
                        print('X:', arrow_center_x)
                        if next_stage_flag==0:
                            send.sendHeadMotor(2,1400,50)
                            theta, speed, i=correct_go_to_arrow(origin_theta,i)
                            if i>=5:
                                next_stage_flag=1
                                send.sendSensorReset()
                                i=0
                                send.sendHeadMotor(2,1600,50)
                                time.sleep(0.2)
                            print('next flag:', next_stage_flag)
                            send.sendContinuousValue(speed,origin_Y,0,theta,0)
                        else:
                            if arrow_center_y>=170:
                                speed=1000
                                i+=1
                                if i>=5:
                                    turn_now_flag=1
                                    i=0
                            #print(turn_now_flag)
                            if turn_right_flag>=2 and turn_now_flag==1:#多次成功判斷右轉與判斷箭頭在銀幕下方
                                finish_turn_right_flag=imu_right(finish_turn_right_flag,origin_theta,origin_Y)
                                if finish_turn_right_flag==1:#完成90度右轉判斷旗標歸零
                                    send.sendHeadMotor(2,1600,50)
                                    time.sleep(0.2)
                                    turn_right_flag=0
                                    finish_turn_right_flag=0
                                    turn_now_flag=0


                            elif turn_left_flag>=2 and turn_now_flag==1:#多次成功判斷左轉與判斷箭頭在銀幕下方
                                finish_turn_left_flag=imu_left(finish_turn_left_flag,origin_theta,origin_Y)
                                if finish_turn_left_flag==1:#完成90度左轉判斷旗標歸零
                                    send.sendHeadMotor(2,1600,50)
                                    time.sleep(0.2)
                                    turn_left_flag=0
                                    finish_turn_left_flag=0
                                    turn_now_flag=0
                            else:#沒有任何判斷就直走
                                speed, theta=imu_go(origin_theta,arrow_center_x)                    
                                send.sendContinuousValue(speed,origin_Y,0,theta,0)
                            
                                if turn_now_flag == 1:
                                    turn_now_flag=0
                                    
                    else:
                        theta, speed, go_to_second_part_flag=theta_value(origin_theta)
                        straight_temp, right_temp, left_temp, arrow_center_y, arrow_center_x=camera(straight_temp, right_temp, left_temp)#判斷是否有箭頭
                        second_part_flag, turn_right_flag, turn_left_flag=arrow_flag(straight_temp, right_temp, left_temp, second_part_flag, turn_right_flag, turn_left_flag)
                        print('line in camera bottom : ', go_to_second_part_flag)
                        print('arrow ok : ', second_part_flag)
                        if second_part_flag==1:
                            speed=1000
                        send.sendContinuousValue(speed,origin_Y,0,theta,0)
            if send.is_start == False:
                if start == False:
                    initial()
                    send.sendBodyAuto(0,0,0,0,1,0)
                    start=True
            r.sleep()
    except rospy.ROSInterruptException:
        pass
