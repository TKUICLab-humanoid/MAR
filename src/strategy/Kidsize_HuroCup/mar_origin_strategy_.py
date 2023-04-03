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
class MAR_API:
    def __init__(self):
        self.yaw_offset = 0
MAR = MAR_API()
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
    second_part_flag=1#成功判斷銀幕內有箭頭
    go_to_second_part_flag=1#線段只有在銀幕下方
    next_stage_flag=1 #修正是否正對箭頭
#----------------------------------------------------------------------
    #步態初始化
    origin_theta=0
    origin_Y=0
    MAR.yaw_offset = 0

def imu_right(flag,origin_theta,origin_Y):#90度右轉
    flag=0
    yaw = send.imu_value_Yaw-MAR.yaw_offset
    print('trun right')
    send.sendContinuousValue(2100,origin_Y,0,-5+origin_theta,0)
    send.sendHeadMotor(2,2800,50)
    if  yaw < -90:#成功右轉90度
        print("end")
        MAR.yaw_offset = send.imu_value_Yaw
        # send.sendSensorReset()
        flag=1
    return flag
def imu_left(flag,origin_theta,origin_Y):#90度左轉
    flag=0
    yaw = send.imu_value_Yaw-MAR.yaw_offset
    print('trun left')
    send.sendContinuousValue(2300,origin_Y,0,5+origin_theta,0)
    send.sendHeadMotor(2,2800,50)
    if  yaw > 80:#成功左轉90度
        print("end")
        MAR.yaw_offset = send.imu_value_Yaw
        # send.sendSensorReset()
        flag=1
    return flag
def imu_go(origin_theta,arrow_center_x):#直走
    theta=origin_theta
    print("go go go!")
    yaw = send.imu_value_Yaw-MAR.yaw_offset
    speed = 3000
    if 0<arrow_center_x<=140:
        theta=4
        send.sendContinuousValue(speed,origin_Y,0,theta+origin_theta,0)
    elif arrow_center_x>=180:
        theta=-4
        send.sendContinuousValue(speed,origin_Y,0,theta+origin_theta,0)
    else:
        if  yaw > 3:
            theta = -2+origin_theta
            print('right')
            # time.sleep(0.2)
        elif yaw < -3:
            theta = 2+origin_theta
            print('left')
            # time.sleep(0.2)
    return speed, theta
def camera(straight_temp, right_temp, left_temp):#判斷箭頭
    #cap = cv2.VideoCapture(7)
    center_y=0
    center_x=0
    STRAIGHT_casecade = cv2.CascadeClassifier("/home/iclab/Desktop/MAR/src/strategy/Parameter/cascade2Straight.xml")
    RIGHT_casecade = cv2.CascadeClassifier("/home/iclab/Desktop/MAR/src/strategy/Parameter/cascade2Right1.xml")
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
    speed = -500
    print('slope:', slope)
    if -0.05 < slope < 0.05:
        theta = 0+origin_theta
        i+=1
    #turn right
    elif -0.4 < slope < -0.05:
        theta = -3+origin_theta
    elif slope < -0.4:
        theta = -4+origin_theta
        speed = 0
    #turn left 
    elif 0.4 > slope > 0.05:
        theta = 3+origin_theta
    elif slope > 0.4:
        theta = 4+origin_theta
        speed = 0

    return theta, speed, i

def arrow_flag(straight_temp, right_temp, left_temp, second_part_flag, turn_right_flag, turn_left_flag):
    if straight_temp>=5:#成功連續判斷20次
        straight_temp=0
        print("go Straight")
        second_part_flag=1
        turn_right_flag=0
        turn_left_flag=0
    elif right_temp>=3:
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
    # print(straight_temp)
    # print(right_temp)
    # print(left_temp)
    return second_part_flag, turn_right_flag, turn_left_flag
def theta_value(origin_theta):#判斷斜率
    slope , go_to_second_part_flag, correct_walking_right, correct_walking_left, big_turn_right, big_turn_left= calculate()
    theta=0
    speed=0
    #if correct_walking_right==1:
     #   theta = -4+origin_theta
    #  speed = 3000
    #elif correct_walking_left==1:
     #   theta = 4+origin_theta
      #  speed = 3000
    if big_turn_right==1:
        theta = -4+origin_theta
        speed = 3000
    elif big_turn_left==1:
        theta = 4+origin_theta
        speed = 3000
    else:
        sp=[3500,3500,3300,3300,3200,3200,3100,3100,3000]
        th=[0,1,2,3,3,4,4,4,5]
        #walk straight
        if slope >= 0.9:
            theta = 5+origin_theta
            speed = 2800
        elif slope >= 0:
            speed = int(sp[math.floor(slope/0.1)])
            theta = int(th[math.floor(slope/0.1)])+origin_theta
        elif  slope <= -0.9:
            theta = -5+origin_theta
            speed = 2800
        else:
            speed = int(sp[math.floor(-slope/0.1)])
            theta = -int(th[math.floor(-slope/0.1)])+origin_theta
            if correct_walking_right==1:
                theta -= 3
                speed = 3000
            elif correct_walking_left==1:
                theta += 4
                speed = 3000
    return theta, speed, go_to_second_part_flag
def calculate():#計算斜率
    cnt1=0
    cnt2=0
    total_x1=0
    total_y1=0
    total_x2=0
    total_y2=0
    center_x1=0
    center_y1=0
    center_x2=0
    center_y2=0
    slope=0
    go_to_second_part_flag=0
    correct_walking_left=0
    correct_walking_right=0
    big_turn_right=0
    big_turn_left=0
    xmin = 320
    xmax = 0
    ymin = 240
    ymax = 0
   
    for i in range(8):
        for j in range(send.color_mask_subject_cnts[i]):
            if send.color_mask_subject_size[i][j] > 1000:
                if xmin > send.color_mask_subject_XMin[i][j]:
                    xmin = send.color_mask_subject_XMin[i][j]
                if xmax < send.color_mask_subject_XMax[i][j]:
                    xmax = send.color_mask_subject_XMax[i][j]
                if ymin > send.color_mask_subject_YMin[i][j]:
                    ymin = send.color_mask_subject_YMin[i][j]
                if ymax < send.color_mask_subject_YMax[i][j]:
                    ymax = send.color_mask_subject_YMax[i][j]
                if ymin < 50:
                    ymin = 50
    if xmax != 0 :
        for high in range(ymin, ymax):
            for wight in range(xmin, xmax):
                cen_y = (ymin+ymax)/2
                if high < cen_y:
                    if send.Label_Model[high*320+wight] != 0:
                        total_x1+=wight
                        total_y1+=high
                        cnt1+=1
                else: 
                    if send.Label_Model[high*320+wight] != 0:
                        total_x2+=wight
                        total_y2+=high
                        cnt2+=1
        center_x1=int(total_x1/cnt1)
        center_y1=int(total_y1/cnt1)
        center_x2=int(total_x2/cnt2)
        center_y2=int(total_y2/cnt2)
        slope = (center_x1-center_x2)/(center_y1-center_y2)
        send.drawImageFunction(5,1,xmin,xmax,ymin,ymax,0,0,0)
        send.drawImageFunction(2,0,center_x1,center_x2,center_y1,center_y2,0,0,0)
        print(slope)
    else :
        slope = 0
    if (center_y1+center_y2)/2 > 150:#機器人偏移或是已經要進入第二階段
        if center_x2 > 240:
            big_turn_right=1
        elif center_x2 < 80:
            big_turn_left=1
        else:
            go_to_second_part_flag=1
    else:
        if(center_x1+center_x2)/2 > 200:
            correct_walking_right=1
        elif (center_x1+center_x2)/2 < 120:
            correct_walking_left=1

    # else:#計算斜率
        
        # if center_x3 < 120 and center_x2 < 120:
        #     correct_walking_left=1
        # elif center_x3 > 200 and center_x2 > 200:
        #     correct_walking_right=1
        # if center_x1==0 and center_y1==0:#first part don't have line
        #     slope = (center_x3-center_x2)/(center_y3-center_y2)
        #     send.drawImageFunction(2,0,center_x2,center_x3,center_y2,center_y3,0,0,0)
        #     go_to_second_part_flag=1#進入第二階段的指標，線在機器人螢幕的正下方
        # elif center_x3==0 and center_y3==0:
        #     slope = (center_x2-center_x1)/(center_y2-center_y1)
        #     send.drawImageFunction(2,0,center_x1,center_x2,center_y1,center_y2,0,0,0)
        # else:
        #     slope = (center_x3-(center_x1+center_x2)/2)/(center_y3-(center_y1+center_y2)/2)
        #     h=int((center_x1+center_x2)/2)
        #     i=int((center_y1+center_y2)/2)
        #     send.drawImageFunction(2,0,center_x3,h,center_y3,i,0,0,0)
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
                    send.sendHeadMotor(2,2800,50)
                    time.sleep(0.5)
                    send.sendHeadMotor(1,2048,50)
                    time.sleep(0.5)
                    send.sendSensorReset()
                    time.sleep(0.5)
                    send.sendBodyAuto(0,0,0,0,1,0)
                    start=False
                else:
                    if second_part_flag==1 and go_to_second_part_flag==1:#判斷是否有箭頭與是否要進入第二階段
                        if turn_now_flag==0:
                            straight_temp, right_temp, left_temp, arrow_center_y, arrow_center_x=camera(straight_temp, right_temp, left_temp)
                            second_part_flag, turn_right_flag, turn_left_flag=arrow_flag(straight_temp, right_temp, left_temp, second_part_flag, turn_right_flag, turn_left_flag)
                        print('Y:', arrow_center_y)
                        print('X:', arrow_center_x)
                        if next_stage_flag==0:
                            send.sendHeadMotor(2,2800,50)
                            theta, speed, i=correct_go_to_arrow(origin_theta,i)
                            if i>=5:
                                next_stage_flag=1
                                turn_now_flag=1
                                # send.sendSensorReset()
                                MAR.yaw_offset = send.imu_value_Yaw
                                i=0
                                send.sendHeadMotor(2,2800,50)
                            print('next flag:', next_stage_flag)
                            send.sendContinuousValue(speed,origin_Y,0,theta,0)
                        else:
                            if arrow_center_y>=100:
                                speed=2000
                                i+=1
                                if i>=5:
                                    turn_now_flag=1
                                    i=0
                            #print(turn_now_flag)
                            if turn_right_flag>=1 and turn_now_flag==1:#多次成功判斷右轉與判斷箭頭在銀幕下方
                                finish_turn_right_flag=imu_right(finish_turn_right_flag,origin_theta,origin_Y)
                                if finish_turn_right_flag==1:#完成90度右轉判斷旗標歸零
                                    send.sendHeadMotor(2,2800,50)
                                    # time.sleep(0.2)
                                    turn_right_flag=0
                                    finish_turn_right_flag=0
                                    turn_now_flag=0


                            elif turn_left_flag>=1 and turn_now_flag==1:#多次成功判斷左轉與判斷箭頭在銀幕下方
                                finish_turn_left_flag=imu_left(finish_turn_left_flag,origin_theta,origin_Y)
                                if finish_turn_left_flag==1:#完成90度左轉判斷旗標歸零
                                    send.sendHeadMotor(2,2800,50)
                                    # time.sleep(0.2)
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
                        #print('line in camera bottom : ', go_to_second_part_flag)
                        #print('arrow ok : ', second_part_flag)
                        print('角度：', theta)
                        print('速度：', speed)
                        time.sleep(0.1)
                        send.sendContinuousValue(speed,origin_Y,0,theta,0)
            if send.is_start == False:
                if start == False:
                    initial()
                    send.sendBodyAuto(0,0,0,0,1,0)
                    start=True
            r.sleep()
    except rospy.ROSInterruptException:
        pass
