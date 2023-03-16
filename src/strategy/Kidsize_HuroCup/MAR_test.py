#!/usr/bin/env python
#coding=utf-8
from traceback import print_tb
import rospy
import numpy as np
from Python_API import Sendmessage
import time
import cv2
import math

class MAR_API:
    def __init__(self):
        self.start = False                                              #第一次進策略
        self.arrow_flag = False                                         #有看到箭頭 second_part_flag
        self.second_part = False
        self.finish_turn_flag = False                                   #完成箭頭轉彎 finish_turn_right_flag,finish_turn_left_flag
        self.first_arrow_flag = True                                   #對正箭頭 next_stage_flag
        self.turn_now_flag = False                                      #正在轉
        self.speed = 0                                                  #速度
        self.theta = 0                                                  #角度
        self.origin_theta = 0                                           #初始角度
        self.slope = 0                                                  #斜率
        self.center_point = np.zeros((3,5))                             #計算中心點[上、中、下段][總數,總x座標,總y座標,中心點x座標,中心點y座標]
        self.walk_line = 'online'                                       #走線狀態 'online'為直走 'line_at_right'線在右邊 'big_turn_right'大右轉 'arrow'進入箭頭(go_to_second_part_flag)
        self.right_arrow = 0                                            #右轉箭頭 turn_right_flag
        self.left_arrow = 0                                             #左轉箭頭 turn_left_flag
        self.arrow_temp = np.zeros(3)                                   #箭頭多次判斷[直走,右轉,左轉]
        self.arrow_center = np.zeros(2)                                 #箭頭中心[x,y]
        # self.imgdata =np.zeros((320*240))                               #影像
        self.i = 0
        self.center_x1 = 0
        self.center_y1 = 0
        self.center_x2 = 0
        self.center_y2 = 0
    def initial(self):                                                  #initial
        self.arrow_flag = False
        self.second_part = False
        self.finish_turn_flag = False
        self.first_arrow_flag = True
        self.speed = 0
        self.theta = 0
        self.slope = 0
        self.center_point = np.zeros((3,5))
        self.arrow_temp = np.zeros(3)
        self.arrow_center = np.zeros(2)
        self.center_x1 = 0
        self.center_y1 = 0
        self.center_x2 = 0
        self.center_y2 = 0
    def print_data(self):                                               #print全部寫在這裡
        print('line in camera bottom : ', MAR.walk_line)
        print('slope:',MAR.slope)
        print('theta',MAR.theta)
        print('speed',MAR.speed)
        print('arrow ok : ', MAR.arrow_flag)
        print(self.center_x1)
        print(self.center_x2)
        print('==============================================')
send = Sendmessage()
MAR = MAR_API()
imgdata = [[None for high in range(240)]for wight in range(320)]
#center point array
#       cnt     total_x     total_y     center_x     center_y 
# 上    cnt1    total_x1    total_y1
# 中    cnt2    total_x2    total_y2
# 下    cnt3    total_x3    total_y3


        


# def initial():
#     global i, cnt, MAR.speed, straight_temp, right_temp, left_temp, turn_left_flag, turn_now_flag, turn_right_flag, finish_turn_left_flag, finish_turn_right_flag, second_part_flag, next_stage_flag, go_to_second_part_flag, origin_theta
#     i=0
#     MAR.speed=0
#     straight_temp=0 #成功判斷箭頭暫存
#     right_temp=0 
#     left_temp=0 
#     turn_right_flag=0#成功判斷右轉
#     turn_left_flag=0#成功判斷左轉
#     finish_turn_left_flag=0#完成左轉
#     finish_turn_right_flag=0#完成右轉
#     turn_now_flag=0
# #----------------------------------------------------------------------
#     #第二階段旗標
#     second_part_flag=0 #成功判斷銀幕內有箭頭
#     go_to_second_part_flag=0#線段只有在銀幕center_y下方
#     next_stage_flag=0 #修正是否正對箭頭
# #----------------------------------------------------------------------
#     #步態初始化
#     origin_theta=1
#     origin_Y=0

def imu_right():#90度右轉
    MAR.finish_turn_flag = False
    yaw = send.imu_value_Yaw
    print('箭頭：右轉')
    send.sendContinuousValue(2200,0,0,-5+MAR.origin_theta,0)
    send.sendHeadMotor(2,1400,50)
    if  yaw < -83:#成功右轉90度
        print("箭頭右轉結束")
        send.sendSensorReset()
        MAR.finish_turn_flag = True

def imu_left():#90度左轉
    MAR.finish_turn_flag = False
    yaw = send.imu_value_Yaw
    print('箭頭：左轉')
    send.sendContinuousValue(2300,0,0,5+MAR.origin_theta,0)
    send.sendHeadMotor(2,1400,50)
    if  yaw > 78:#成功左轉90度
        print("箭頭左轉結束")
        send.sendSensorReset()
        MAR.finish_turn_flag = True

def imu_go():#直走
    MAR.theta = MAR.origin_theta+1
    print("直走")
    yaw = send.imu_value_Yaw
    MAR.speed = 2000
    if 0<MAR.arrow_center[0]<=140:
        MAR.theta=5
        send.sendContinuousValue(MAR.speed,0,0,MAR.theta+MAR.origin_theta,0)
    elif MAR.arrow_center[0]>=180:
        MAR.theta=-5
        send.sendContinuousValue(MAR.speed,0,0,MAR.theta+MAR.origin_theta,0)
    else:
        if  yaw > 3:
            MAR.theta = -3+MAR.origin_theta
            print('修正：右轉')
        elif yaw < -3:
            MAR.theta = 2+MAR.origin_theta
            print('修正：左轉')

def camera():#判斷箭頭
    #cap = cv2.VideoCapture(7)
    MAR.arrow_center[1]=0
    MAR.arrow_center[0]=0
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
        MAR.arrow_center[1]=y+h/2 #計算箭頭的y軸位置
        MAR.arrow_center[0]=x+w/2
        # print("straight")
    elif len(RIGHT)>0:
        for a in RIGHT:  # 單獨框出每一張人臉
            x, y, w, h = a
            cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
        send.drawImageFunction(1,1,x,x+w,y,y+h,255,0,0)
        MAR.arrow_temp[0]=0
        MAR.arrow_temp[1]+=1
        MAR.arrow_temp[2]=0
        MAR.arrow_center[1]=y+h/2 #計算箭頭的y軸位置
        MAR.arrow_center[0]=x+w/2
        # print("right")
    elif len(LEFT)>0:
        for a in LEFT:  # 單獨框出每一張人臉
            x, y, w, h = a
            cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
        send.drawImageFunction(1,1,x,x+w,y,y+h,255,0,0)
        MAR.arrow_temp[0]=0
        MAR.arrow_temp[1]=0
        MAR.arrow_temp[2]+=1
        MAR.arrow_center[1]=y+h/2#計算箭頭的y軸位置
        MAR.arrow_center[0]=x+w/2
        # print("left")
    
    

def correct_go_to_arrow():
    # MAR.slope , go_to_second_part_flag, correct_walking_right, correct_walking_left, big_turn_right, big_turn_left= calculate()
    print('slope:', MAR.slope)
    if -0.1 < MAR.slope < 0.1:
        MAR.theta = 0+MAR.origin_theta
        MAR.speed=0
        MAR.i+=1
    #turn right
    elif -0.4 < MAR.slope < -0.05:
        MAR.theta = -1+MAR.origin_theta
    elif MAR.slope < -0.4:
        MAR.theta = -3+MAR.origin_theta
        MAR.speed = -500
    #turn left 
    elif 0.4 > MAR.slope > 0.05:
        MAR.theta = 1+MAR.origin_theta
    elif MAR.slope > 0.4:
        MAR.theta = 3+MAR.origin_theta
        MAR.speed = -500
    if MAR.i>=5:
        MAR.first_arrow_flag = False
        send.sendSensorReset()
        MAR.i=0
        send.sendHeadMotor(2,1500,50)
        time.sleep(0.2)

def arrow_flag():
    if MAR.arrow_temp[0]>=5:#成功連續判斷20次
        MAR.arrow_temp[0]=0
        print("go Straight")
        MAR.arrow_flag = True
        MAR.right_arrow=0
        MAR.left_arrow=0
    elif MAR.arrow_temp[1]>=4:
        MAR.arrow_temp[1]=0
        print("go Right")
        MAR.arrow_flag = True
        MAR.right_arrow+=1
        MAR.left_arrow=0
    elif MAR.arrow_temp[2]>=5:
        MAR.arrow_temp[2]=0
        print("go Left")
        MAR.arrow_flag = True
        MAR.right_arrow=0
        MAR.left_arrow+=1

def theta_value():#判斷斜率
    if MAR.walk_line == 'big_turn_right':
        MAR.theta = -5+MAR.origin_theta
        MAR.speed = 3000
    elif MAR.walk_line == 'big_turn_left':
        MAR.theta = 5+MAR.origin_theta
        MAR.speed = 3000
    else:
        sp=[3500,3400,3300,3300,3200,3200,3100,3100,3100]
        th=[0,0,0,1,1,2,2,3,4]
        #walk straight
        if MAR.center_x2 < 140 and abs(MAR.slope) < 0.3:
            MAR.theta = 5+MAR.origin_theta
            MAR.speed = 3000
        elif MAR.center_x2 > 180 and abs(MAR.slope) < 0.3:
            MAR.theta = -5+MAR.origin_theta
            MAR.speed = 3000
        elif MAR.slope >= 0.9:
            MAR.theta = 5+MAR.origin_theta
            MAR.speed = 3000
        elif MAR.slope>=0:
            MAR.speed = int(sp[math.floor(MAR.slope/0.1)])
            MAR.theta = int(th[math.floor(MAR.slope/0.1)])+MAR.origin_theta
        elif  MAR.slope <= -0.9:
            MAR.theta = -5+MAR.origin_theta
            MAR.speed = 3000
        else:
            MAR.speed = int(sp[math.floor(-MAR.slope/0.1)])
            MAR.theta = -int(th[math.floor(-MAR.slope/0.1)])+MAR.origin_theta
        if MAR.walk_line == 'line_at_right':
            MAR.theta = -3+MAR.theta
            MAR.speed = 3000
        elif MAR.walk_line == 'line_at_left':
            MAR.theta = 3+MAR.theta
            MAR.speed = 3000

def region():
    targetxmin = 320
    targetxmax = 0
    targetymin = 240
    targetymax = 0
    MAR.center_x1 = 0
    MAR.center_y1 = 0
    MAR.center_x2 = 0
    MAR.center_y2 = 0
    for i in range (len(send.color_mask_subject_cnts)):
        for j in range (send.color_mask_subject_cnts[i]):
            if send.color_mask_subject_size[i][j] > 500:
                if send.color_mask_subject_XMin[i][j] < targetxmin:
                   targetxmin =  send.color_mask_subject_XMin[i][j]
                if send.color_mask_subject_XMax[i][j] > targetxmax:
                    targetxmax = send.color_mask_subject_XMax[i][j]
                if send.color_mask_subject_YMin[i][j] < targetymin:
                    targetymin = send.color_mask_subject_YMin[i][j]
                if send.color_mask_subject_YMax[i][j] > targetymax:
                    targetymax= send.color_mask_subject_YMax[i][j] 
    total_x1 = 0
    total_y1 = 0
    cnt1 = 0
    total_x2 = 0
    total_y2 = 0
    cnt2 = 0
    if targetxmin !=320:
        for i in range (targetxmin,targetxmax):
            for j in range (targetymin,targetymax):
                if j < (targetymax+targetymin)/2:
                    if send.Label_Model[320*j+i] != 0:
                        total_x1 += i
                        total_y1 += j
                        cnt1 += 1
                else:
                    if send.Label_Model[320*j+i] != 0:
                        total_x2 += i
                        total_y2 += j
                        cnt2 += 1
        send.drawImageFunction(5,1,targetxmax,targetxmin,targetymax,targetymin,0,0,0)
    if cnt1 > 0 and cnt2>0:
        MAR.center_x1 = int(total_x1 / cnt1)
        MAR.center_y1 = int(total_y1 / cnt1)
        MAR.center_x2 = int(total_x2 / cnt2)
        MAR.center_y2 = int(total_y2 / cnt2)
        MAR.slope = (MAR.center_x2 - MAR.center_x1) / (MAR.center_y2 - MAR.center_y1)
        send.drawImageFunction(2,0,MAR.center_x2,MAR.center_x1,MAR.center_y2,MAR.center_y1,0,0,0)
        MAR.walk_line = 'online'
    else:
        MAR.slope = 0

def modify():
    if (MAR.center_y1 + MAR.center_y2)/2 > 180:
        if MAR.center_x2 > 220:
            MAR.walk_line = 'big_turn_right'
        elif MAR.center_x2 < 80:
            MAR.walk_line = 'big_turn_left'
        else:
            MAR.walk_line='arrow'#進入第二階段的指標，線在機器人螢幕的正下方
    else:#計算斜率
        if (MAR.center_x2 + MAR.center_x1) / 2 < 140 :
            MAR.walk_line = 'line_at_left'
        elif (MAR.center_x2 + MAR.center_x1) / 2 > 180:
            MAR.walk_line = 'line_at_right'

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
    for high in range(240):
        for wight in range(320):
            imgdata[wight][high]=send.Label_Model[high*320+wight]
            if 1 <= high < 100:
                if imgdata[wight][high] != 0:
                    total_x1+=wight
                    total_y1+=high
                    cnt1+=1
            elif 100 <= high < 180:
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
        MAR.slope = 0
    elif center_x1==0 and center_x2==0 and center_y1==0 and center_y2==0:#機器人偏移或是已經要進入第二階段
        if center_x3 > 220:
            MAR.walk_line = 'big_turn_right'
            a2 = time.time()
            print(1/(a2-a1))
        if center_x3 < 110 and center_x2 < 110:
            MAR.walk_line = 'line_at_left'
        elif center_x3 > 210 and center_x2 > 210:
            MAR.walk_line = 'line_at_right'
        elif center_x1==0 and center_y1==0:#first part don't have line
            MAR.slope = (center_x3-center_x2)/(center_y3-center_y2)
            send.drawImageFunction(2,0,center_x2,center_x3,center_y2,center_y3,0,0,0)
            MAR.walk_line='arrow'#進入第二階段的指標，線在機器人螢幕的正下方
        elif center_x3==0 and center_y3==0:
            MAR.slope = (center_x2-center_x1)/(center_y2-center_y1)
            send.drawImageFunction(2,0,center_x1,center_x2,center_y1,center_y2,0,0,0)
            MAR.walk_line='online'
        else:
            MAR.slope = (center_x3-(center_x1+center_x2)/2)/(center_y3-(center_y1+center_y2)/2)
            h=int((center_x1+center_x2)/2)
            i=int((center_y1+center_y2)/2)
            send.drawImageFunction(2,0,center_x3,h,center_y3,i,0,0,0)
            MAR.walk_line='online'
    #print(slope)
 

if __name__ == '__main__':
    try:        
        r=rospy.Rate(30)
        while not rospy.is_shutdown():
            if send.is_start == True:   #箭頭
                if MAR.start == True:
                    MAR.initial()
                    send.sendHeadMotor(2,1500,50)
                    time.sleep(0.5)
                    send.sendHeadMotor(1,2048,50)
                    time.sleep(0.5)
                    send.sendSensorReset()
                    time.sleep(0.5)
                    send.sendBodyAuto(0,0,0,0,1,0)
                    MAR.start=False
                else:
                    if MAR.second_part == True:#判斷是否有箭頭與是否要進入第二階段
                        # straight_temp, right_temp, left_temp, arrow_center_y, arrow_center_x=camera(straight_temp, right_temp, left_temp)
                        if MAR.turn_now_flag == False:
                            camera()
                        # second_part_flag, turn_right_flag, turn_left_flag=arrow_flag(straight_temp, right_temp, left_temp, second_part_flag, turn_right_flag, turn_left_flag)
                            arrow_flag()
                        print('Yaw:',send.imu_value_Yaw)
                        print('Y:', MAR.arrow_center[1])
                        print('X:', MAR.arrow_center[0])
                        if MAR.first_arrow_flag == True:  #讓機器人正對第1個箭頭
                            send.sendHeadMotor(2,1500,50)
                            #calculate()
                            region()
                            correct_go_to_arrow()
                            print('next flag:', MAR.first_arrow_flag)
                            send.sendContinuousValue(MAR.speed,0,0,MAR.theta,0)
                            send.sendHeadMotor(2,1400,50)
                        else:
                            if MAR.arrow_center[1]>=170:
                                MAR.speed=2000
                                MAR.i+=1
                                if MAR.i>=6:
                                    MAR.turn_now_flag=True
                                    MAR.i=0
                            #print(turn_now_flag)
                            if MAR.right_arrow >= 1 and MAR.turn_now_flag == True:#多次成功判斷右轉與判斷箭頭在銀幕下方
                                imu_right()
                                if MAR.finish_turn_flag == True:#完成90度右轉判斷旗標歸零
                                    send.sendHeadMotor(2,1500,50)
                                    time.sleep(0.2)
                                    MAR.right_arrow = 0
                                    MAR.finish_turn_flag = False
                                    MAR.turn_now_flag = False


                            elif MAR.left_arrow >= 1 and MAR.turn_now_flag == True:#多次成功判斷左轉與判斷箭頭在銀幕下方
                                imu_left()
                                if MAR.finish_turn_flag == True:#完成90度左轉判斷旗標歸零
                                    send.sendHeadMotor(2,1500,50)
                                    time.sleep(0.2)
                                    MAR.left_arrow = 0
                                    MAR.finish_turn_flag = False
                                    MAR.turn_now_flag = False
                            else:#沒有任何判斷就直走
                                imu_go()                    
                                send.sendContinuousValue(MAR.speed,0,0,MAR.theta,0)
                            
                                if MAR.turn_now_flag == True:
                                    MAR.turn_now_flag = False
                                    
                    else:   #線
                        #calculate()
                        region()
                        modify()
                        theta_value()  #線的斜率
                        camera()#判斷是否有箭頭
                        arrow_flag()
                        # print('line in camera bottom : ', MAR.walk_line)
                        # print('arrow ok : ', MAR.arrow_flag)
                        MAR.print_data()
                        # time.sleep(0.05)
                        if MAR.arrow_flag == True and MAR.walk_line == 'arrow':
                            MAR.second_part = True
                        send.sendContinuousValue(MAR.speed,0,0,MAR.theta,0)
            if send.is_start == False:
                if MAR.start == False:
                    # initial()
                    send.sendBodyAuto(0,0,0,0,1,0)
                    MAR.start=True
            r.sleep()
    except rospy.ROSInterruptException:
        pass