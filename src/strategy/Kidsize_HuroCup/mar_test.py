#!/usr/bin/env python
#coding=utf-8
from pickle import TRUE
from re import M
from traceback import print_tb
import rospy
import numpy as np
from Python_API import Sendmessage
import time
import cv2
import math

class MAR_API:
    def __init__(self):
        self.arrow_flag = False                                             #有看到箭頭 
        self.finish_turn_flag = True                                        #完成箭頭轉彎
        self.first_arrow_flag = True                                        #線道箭頭
        self.turn_now_flag = False                                          #可以轉彎
        self.head = 2600                                                    #頭部角度
        self.speed = 0                                                      #速度
        self.theta = 0                                                      #角度
        self.origin_theta = 0                                               #初始角度
        self.spped_range=np.array([3500,3400,3300,3300,3200,3200,3100,3100,3100])   #速度範圍
        self.theta_range=np.array([0,0,1,1,2,2,3,3,3])                              #角度範圍
        self.slope = 0                                                      #斜率
        self.center_1=np.array([0,0])
        self.center_2=np.array([0,0])
        self.center_3=np.array([0,0])
        self.walk_status = 'First'                                          #程式狀態
        self.right_arrow = 0                                                #右轉箭頭 turn_right_flag
        self.left_arrow = 0                                                 #左轉箭頭 turn_left_flag
        self.arrow_temp = np.array([0,0,0,0])                               #箭頭多次判斷[直走,右轉,左轉]
        self.arrow_center = np.array([0,0])                                 #箭頭中心[x,y]
        self.i = 0
        self.delay_cnt = 0
        self.arrow_status = 'None'                                          #箭頭種類
        self.line_status = 'go'
        self.line_area = np.array([0,0,320,320,0,0])                        #線的邊界[Xmax Ymax Xmin Ymin X Y]
        self.time1 = 0
        self.time2 = 0
        self.FPS = 0
    def initial(self):                                                      #initial
        self.arrow_flag = False
        self.turn_now_flag = False
        self.finish_turn_flag = False
        self.first_arrow_flag = True
        self.head = 2600
        self.speed = 0
        self.theta = 0
        self.slope = 0
        self.walk_status = 'First'
        self.center_1=np.array([0,0])
        self.center_2=np.array([0,0])
        self.center_3=np.array([0,0])
        self.arrow_temp = np.array([0,0,0,0])
        self.arrow_center = np.array([0,0])
        self.line_area = np.array([0,0,320,320,0,0])
        self.time1 = 0
        self.time2 = 0
        self.i = 0
        self.delay_cnt = 0
        self.arrow_status = 'None'
        self.line_status = 'go'
    def print_data(self):                                                   #print
        print('==============================================')
        print('Walk status : ', self.walk_status)
        print('Line status : ',self.line_status)
        print('Slope : ',self.slope)
        print('Theta : ',self.theta)
        print('Speed : ',self.speed)        
        print('==============================================')
        print('Arrow : ',self.arrow_status)
        print('Arrow center X : ',self.arrow_center[0])
        print('Arrow center Y : ',self.arrow_center[1])
        print('Time',1/self.FPS)

send = Sendmessage()
MAR = MAR_API()
imgdata = [[None for high in range(240)]for wight in range(320)]

def calculate_slop():       #計算斜率
    cnt1=0
    cnt2=0
    cnt3=0
    total_1=np.array([0,0])
    total_2=np.array([0,0])
    total_3=np.array([0,0])
    for high in range(240):
        for wight in range(320):
            imgdata[wight][high]=send.Label_Model[high*320+wight]
            if 1 <= high < 100:
                if imgdata[wight][high] != 0:
                    total_1[0]+=wight
                    total_1[1]+=high
                    cnt1+=1
            elif 100 <= high < 180:
                if imgdata[wight][high] != 0:
                    total_2[0]+=wight
                    total_2[1]+=high
                    cnt2+=1
            elif high >= 180:
                if imgdata[wight][high] != 0:
                    total_3[0]+=wight
                    total_3[1]+=high
                    cnt3+=1
    if cnt1 > 50:#去除雜訊點
        MAR.center_1[0]=int(total_1[0]/cnt1)
        MAR.center_1[1]=int(total_1[1]/cnt1)
    else:
        MAR.center_1=[0,0]
    if cnt2 > 50:
        MAR.center_2[0]=int(total_2[0]/cnt2)
        MAR.center_2[1]=int(total_2[1]/cnt2)
    else:
        MAR.center_2=[0,0]
    if cnt3 > 50:
        MAR.center_3[0]=int(total_3[0]/cnt3)
        MAR.center_3[1]=int(total_3[1]/cnt3)
    else:
        MAR.center_3=[0,0]
    if MAR.center_1[1]+MAR.center_2[1] == 0:
        MAR.slope = 0
    elif MAR.center_1[1] == 0:
        MAR.slope = (MAR.center_3[0]-MAR.center_2[0])/(MAR.center_3[1]-MAR.center_2[1])
        send.drawImageFunction(2,0,MAR.center_2[0],MAR.center_3[0],MAR.center_2[1],MAR.center_3[1],255,0,0)
    elif MAR.center_3[1] == 0:
        MAR.slope = (MAR.center_2[0]-MAR.center_1[0])/(MAR.center_2[1]-MAR.center_1[1])
        send.drawImageFunction(2,0,MAR.center_1[0],MAR.center_2[0],MAR.center_1[1],MAR.center_2[1],0,255,0)
    else:
        MAR.slope = (MAR.center_3[0]-(MAR.center_1[0]+MAR.center_2[0])/2)/(MAR.center_3[1]-(MAR.center_1[1]+MAR.center_2[1])/2)
        h=int((MAR.center_1[0]+MAR.center_2[0])/2)
        i=int((MAR.center_1[1]+MAR.center_2[1])/2)
        send.drawImageFunction(2,0,MAR.center_3[0],h,MAR.center_3[1],i,0,0,255)

def calculate_slop_new():
    MAR.line_area = np.array([0,0,320,320,0,0])
    cnt1 = 0
    cnt2 = 0
    MAR.center_1 = np.array([0,0])
    MAR.center_2 = np.array([0,0])
    total_1=np.array([0,0])
    total_2=np.array([0,0])
    for i in range(0,8):
        for j in range(send.color_mask_subject_cnts[i]):
            if send.color_mask_subject_size[i][j] > 1000:
                if send.color_mask_subject_XMax[i][j] > MAR.line_area[0]:
                    MAR.line_area[0] = send.color_mask_subject_XMax[i][j]
                if send.color_mask_subject_YMax[i][j] > MAR.line_area[1]:
                    MAR.line_area[1] = send.color_mask_subject_YMax[i][j]
                if send.color_mask_subject_XMin[i][j] < MAR.line_area[3]:
                    MAR.line_area[2] = send.color_mask_subject_XMin[i][j]
                if send.color_mask_subject_YMin[i][j] < MAR.line_area[3]:
                    MAR.line_area[3] = send.color_mask_subject_YMin[i][j]
    # for i in range(MAR.line_area[2],MAR.line_area[2]+20):
    #     for j in range(MAR.line_area[3],MAR.line_area[3]+20):
    #         if send.Label_Model[j*320+i] != 0:
    #             cnt1+=1
    # for i in range(MAR.line_area[0]-20,MAR.line_area[0]):
    #     for j in range(MAR.line_area[2]-20,MAR.line_area[2]):
    #         if send.Label_Model[j*320+i] != 0:
    #             cnt2+=1
    if MAR.line_area[0] == 0:
        MAR.slope = 0 
    else:
        for i in range(MAR.line_area[2],MAR.line_area[0]):
            for j in range(MAR.line_area[3],int((MAR.line_area[1]+MAR.line_area[3])/2)):
                if send.Label_Model[j*320+i] != 0:
                    total_1[0] += i
                    total_1[1] += j
                    cnt1 += 1
        for i in range(MAR.line_area[2],MAR.line_area[0]):
            for j in range(int((MAR.line_area[1]+MAR.line_area[3])/2),MAR.line_area[1]):
                if send.Label_Model[j*320+i] != 0:
                    total_2[0] += i
                    total_2[1] += j
                    cnt2 += 1
        if cnt1 > 0:
            MAR.center_1[0] = int(total_1[0]/cnt1)
            MAR.center_1[1] = int(total_1[1]/cnt1)
        if cnt2 > 0:
            MAR.center_2[0] = int(total_2[0]/cnt2)
            MAR.center_2[1] = int(total_2[1]/cnt2)
        MAR.slope = (MAR.center_1[0]-MAR.center_2[0])/(MAR.center_1[1]-MAR.center_2[1])
        send.drawImageFunction(2,0,MAR.center_1[0],MAR.center_2[0],MAR.center_1[1],MAR.center_2[1],255,255,255)
        send.drawImageFunction(5,1,MAR.line_area[2],MAR.line_area[0],MAR.line_area[3],MAR.line_area[1],0,0,0)

        # if cnt1 > 50 and cnt2 > 50:            
        #     MAR.slope = -int(MAR.line_area[0]-MAR.line_area[2])/int(MAR.line_area[1]-MAR.line_area[3]) 
        #     send.drawImageFunction(2,0,MAR.line_area[0],MAR.line_area[2],MAR.line_area[3],MAR.line_area[1],255,255,255)   
        #     print('aaaaaaaaaaa')        
        # else:
        #     MAR.slope = int(MAR.line_area[0]-MAR.line_area[2])/int(MAR.line_area[1]-MAR.line_area[3])
        #     send.drawImageFunction(2,0,MAR.line_area[2],MAR.line_area[0],MAR.line_area[3],MAR.line_area[1],255,255,255)
        #     print('bbbbbbbbbbbbbbb')
        MAR.line_area[4] = (MAR.line_area[0]+MAR.line_area[2])/2
        MAR.line_area[5] = (MAR.line_area[1]+MAR.line_area[3])/2        
        # send.drawImageFunction(5,1,MAR.line_area[2],MAR.line_area[0],MAR.line_area[3],MAR.line_area[1],0,0,0)

def line_control():         #修線
    if MAR.center_1[1]+MAR.center_2[1] == 0:
        if MAR.center_3[0] > 240:
            MAR.theta = -4
            MAR.speed = 3000
            MAR.line_status = '大右轉'
        elif MAR.center_3[0] < 80:
            MAR.theta = 4
            MAR.speed = 3000
            MAR.line_status = '大左轉'
    elif MAR.center_2[0] < 110 and MAR.center_3[0] < 110:
        MAR.theta = -5
        MAR.speed = 3000
        MAR.line_status = '線在左邊'
    elif MAR.center_2[0] > 210 and MAR.center_3[0] > 210:
        MAR.theta = 5
        MAR.speed = 3000
        MAR.line_status = '線在右邊'
    else:
        if MAR.slope >= 0.9:
            MAR.theta = 5+MAR.origin_theta
            MAR.speed = 3000
        elif MAR.slope>=0:
            MAR.speed = int(MAR.spped_range[math.floor(MAR.slope/0.1)])
            MAR.theta = int(MAR.theta_range[math.floor(MAR.slope/0.1)])
        elif  MAR.slope <= -0.9:
            MAR.theta = -5
            MAR.speed = 3000
        else:
            MAR.speed = int(MAR.spped_range[math.floor(-MAR.slope/0.1)])
            MAR.theta = -int(MAR.theta_range[math.floor(-MAR.slope/0.1)])
        MAR.line_status = 'go'

def line_control_new():         #修線
    if MAR.line_area[5] > 180:
        if MAR.center_2[0] > 240:
            MAR.theta = -4
            MAR.speed = 2800
            MAR.line_status = '大右轉'
        elif MAR.center_2[0] < 80:
            MAR.theta = 4
            MAR.speed = 2800
            MAR.line_status = '大左轉'
    if MAR.slope > -0.2 and MAR.slope < 0.2:
        if MAR.center_2[0] < 110:
            MAR.theta = 5
            MAR.speed = 2800
            MAR.line_status = '線在左邊'
        elif MAR.center_2[0] > 210:
            MAR.theta = -5
            MAR.speed = 2800
            MAR.line_status = '線在右邊'
    else:
        if MAR.slope >= 0.9:
            MAR.theta = 5+MAR.origin_theta
            MAR.speed = 2800
        elif MAR.slope>=0:
            MAR.speed = int(MAR.spped_range[math.floor(MAR.slope/0.1)])
            MAR.theta = int(MAR.theta_range[math.floor(MAR.slope/0.1)])
        elif  MAR.slope <= -0.9:
            MAR.theta = -5
            MAR.speed = 2800
        else:
            MAR.speed = int(MAR.spped_range[math.floor(-MAR.slope/0.1)])
            MAR.theta = -int(MAR.theta_range[math.floor(-MAR.slope/0.1)])
        MAR.line_status = 'go'

def find_arrow():           #尋找箭頭
    #cap = cv2.VideoCapture(7)
    MAR.arrow_center=np.array([0,0])
    STRAIGHT_casecade = cv2.CascadeClassifier("/home/iclab/Desktop/MAR/src/strategy/Parameter/cascade2Straight.xml")
    RIGHT_casecade = cv2.CascadeClassifier("/home/iclab/Desktop/MAR/src/strategy/Parameter/cascade2Right.xml")
    LEFT_casecade = cv2.CascadeClassifier("/home/iclab/Desktop/MAR/src/strategy/Parameter/cascade2Left.xml")
    #ret, frame = cap.read()
    frame = send.originimg
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.equalizeHist(gray)
    #分辨箭頭, detectMultiScale(img, 縮圖倍率, minNeighbors(只有其"鄰居"大於該值才被認為是正確結果), Flag(沒用到), minSize)
    STRAIGHT= STRAIGHT_casecade.detectMultiScale(gray,1.1, 5,0,(10,10))
    RIGHT= RIGHT_casecade.detectMultiScale(gray,1.1, 5,0,(10,10))
    LEFT= LEFT_casecade.detectMultiScale(gray,1.1, 5,0,(10,10))
    if len(STRAIGHT)>0:
        for a in STRAIGHT:  # 單獨框出每一張人臉
            x, y, w, h = a
            #框箭頭, cv2.rectangle(img, 左上頂點, 右下頂點, 框的顏色, 框的粗細)
            cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
        send.drawImageFunction(1,1,x,x+w,y,y+h,255,0,0)
        MAR.arrow_temp[0]+=1
        MAR.arrow_temp[1]=0
        MAR.arrow_temp[2]=0
        MAR.arrow_temp[3]=0
        MAR.arrow_center[1]=y+h/2 #計算箭頭的y軸位置
        MAR.arrow_center[0]=x+w/2
        if MAR.arrow_temp[0]>5:
            MAR.arrow_status = 'Stright'
            MAR.arrow_temp[0]=0
            MAR.arrow_flag = True
            MAR.right_arrow=0
            MAR.left_arrow=0
    elif len(RIGHT)>0:
        for a in RIGHT:  # 單獨框出每一張人臉
            x, y, w, h = a
            cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
        send.drawImageFunction(1,1,x,x+w,y,y+h,255,0,0)
        MAR.arrow_temp[0]=0
        MAR.arrow_temp[1]+=1
        MAR.arrow_temp[2]=0
        MAR.arrow_temp[3]=0
        MAR.arrow_center[1]=y+h/2 #計算箭頭的y軸位置
        MAR.arrow_center[0]=x+w/2
        if MAR.arrow_temp[1]>5:
            MAR.arrow_status = 'Right'
            MAR.arrow_temp[1]=0
            MAR.arrow_flag = True
            MAR.right_arrow+=1
            MAR.left_arrow=0
    elif len(LEFT)>0:
        for a in LEFT:  # 單獨框出每一張人臉
            x, y, w, h = a
            cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
        send.drawImageFunction(1,1,x,x+w,y,y+h,255,0,0)
        MAR.arrow_temp[0]=0
        MAR.arrow_temp[1]=0
        MAR.arrow_temp[2]+=1
        MAR.arrow_temp[3]=0
        MAR.arrow_center[1]=y+h/2#計算箭頭的y軸位置
        MAR.arrow_center[0]=x+w/2
        if MAR.arrow_temp[2]>=5:
            MAR.arrow_status = 'Left'
            MAR.arrow_temp[2]=0
            MAR.arrow_flag = True
            MAR.right_arrow=0
            MAR.left_arrow+=1
    else:
        MAR.arrow_temp[0]=0
        MAR.arrow_temp[1]=0
        MAR.arrow_temp[2]=0
        MAR.arrow_temp[3]+=1
        send.drawImageFunction(1,1,0,0,0,0,255,0,0)
        if MAR.arrow_temp[3]>20:
            MAR.arrow_status = 'None'
            MAR.arrow_temp[3]=0
            MAR.arrow_flag = False
            MAR.right_arrow=0
            MAR.left_arrow=0

def line_to_arrow():        #線到箭頭
    if -0.05 < MAR.slope < 0.05:
        MAR.theta=0
        MAR.speed=0
        MAR.i+=1
    elif -0.4 < MAR.slope < -0.05:
        MAR.theta = -1
    elif MAR.slope < -0.4:
        MAR.theta = -3
        MAR.speed = 0
    elif 0.4 > MAR.slope > 0.05:
        MAR.theta = 1
    elif MAR.slope > 0.4:
        MAR.theta = 3
        MAR.speed = 0
    if MAR.i>=5:
        send.sendSensorReset()
        MAR.walk_status = 'Arrow'
        MAR.i=0

def arrow_control():        #執行箭頭
    if MAR.arrow_center[1]>=170:
        MAR.speed=2000
        MAR.delay_cnt+=1
        if MAR.delay_cnt>=6:
            MAR.turn_now_flag=True
            MAR.delay_cnt=0
    if MAR.right_arrow>=1 and MAR.turn_now_flag == True:
        MAR.finish_turn_flag = False
        yaw = send.imu_value_Yaw
        MAR.speed = 2200
        MAR.theta = -5
        MAR.head = 2750
        if  yaw < -83:#成功右轉90度
            send.sendSensorReset()
            MAR.finish_turn_flag = True
            MAR.turn_now_flag = False
    elif MAR.left_arrow>=1 and MAR.turn_now_flag == True:
        MAR.finish_turn_flag = False
        yaw = send.imu_value_Yaw
        MAR.speed = 2200
        MAR.theta = 5
        MAR.head = 2750
        if  yaw > 78:#成功左轉90度
            send.sendSensorReset()
            MAR.finish_turn_flag = True
            MAR.turn_now_flag = False
    else:
        MAR.speed = 2000
        MAR.theta = 1
        yaw = send.imu_value_Yaw    
        if 0<MAR.arrow_center[0]<=140:
            MAR.theta=5
        elif MAR.arrow_center[0]>=180:
            MAR.theta=-5
        else:
            if  yaw > 3:
                MAR.theta = -3
            elif yaw < -3:
                MAR.theta = 2 
        MAR.turn_now_flag = False
  
if __name__ == '__main__':
    try:        
        r=rospy.Rate(30)
        while not rospy.is_shutdown():
            if send.is_start :   #箭頭
                MAR.time1 = time.time()
                if MAR.walk_status == 'First':
                    MAR.initial()
                    send.sendHeadMotor(2,MAR.head,50)
                    time.sleep(0.1) 
                    send.sendHeadMotor(1,2048,50)
                    send.sendSensorReset()
                    time.sleep(0.1)                    
                    send.sendBodyAuto(0,0,0,0,1,0)
                    time.sleep(0.1)
                    MAR.walk_status = 'Line'
                elif MAR.walk_status == 'Line':
                    calculate_slop_new()
                    line_control_new()
                    find_arrow()                    
                    if MAR.center_1[1] == 0 and MAR.arrow_flag:
                        MAR.walk_status = 'FirstArrow'
                elif MAR.walk_status == 'FirstArrow':
                    calculate_slop_new()
                    line_to_arrow()
                    find_arrow()
                elif MAR.walk_status == 'Arrow':
                    if MAR.turn_now_flag == False:
                        find_arrow()
                    arrow_control()   
                MAR.time2 = time.time()
                MAR.FPS = MAR.time2-MAR.time1
                MAR.print_data()             
                send.sendHeadMotor(2,MAR.head,50)
                send.sendContinuousValue(MAR.speed,0,0,MAR.theta+MAR.origin_theta,0)
                time.sleep(0.05)
                MAR.time1 = 0
                MAR.time2 = 0                

            if send.is_start == False:
                if MAR.walk_status != 'First':
                    MAR.initial()
                    send.sendBodyAuto(0,0,0,0,1,0)
                    send.sendHeadMotor(2,MAR.head,50)
                    time.sleep(0.1)
            r.sleep()
    except rospy.ROSInterruptException:
        pass