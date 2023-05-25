#!/usr/bin/env python
#coding=utf-8
from traceback import print_tb
import rospy
import numpy as np
from Python_API import Sendmessage
import time
import cv2
import math
IMGDATA = [[None for high in range(240)]for wight in range(320)]     
ORIGIN_THETA = 0
ORIGIN_Y = 0
#第一段的速度變化
SPEED_CHANGE = [4000, 4000, 3800, 3800, 3700, 3700, 3600, 3600, 3600] 
#第一段的角度變化
THETA_CHANGE = [0, 1, 1, 2, 2, 3, 3, 3, 3]    
send = Sendmessage()
aaaa = rospy.init_node('talker', anonymous=True, log_level=rospy.INFO)

class Marathon:
    def __init__(self):     
        #imu完成左轉、右轉-----------------------------------------------------
        self.finish_left_flag = False       
        self.finish_right_flag = False               
        #face_the_arrow---------------------------------------------------
        self.positive_arrow_times = 0       #重複判斷對正箭頭
        #main------------------------------------------------------------------ 
        self.turn_now_flag = False          #找箭頭
        #----------------------------------------------------------------------
        self.initial()

    #初始化(撥指撥)
    def initial(self):
        #成功判斷多次右轉、左轉
        self.is_right_flag = False        
        self.is_left_flag = False 
        #步態初始化
        self.start = True
        # send.imu_value_Yaw_offset = 0
        send.imu_value_Yaw = 0
        self.speed = 0                      #初始速度
        self.theta = 0
        self.x_division_y = 0
        #imu------------------------------------------------------------------
        self.finish_left_flag = False       #完成左轉
        self.finish_right_flag = False      #完成右轉            
        #change_speed_theta----------------------------------------------------
        self.correct_walking_left = False   #線在機器人左邊
        self.correct_walking_right = False  #線在機器人右邊
        self.big_turn_right = False         #往右大轉
        self.big_turn_left = False          #往左大轉
        #----------------------------------------------------------------------
        #第二階段箭頭
        self.see_arrow_flag = False         #成功判斷銀幕內有箭頭 #已改
        self.line_bottom_flag = False       #線段只有在銀幕下方
        self.enter_arrow_flag = False        #進入第二段 #已改
        #arrow_train/judge_the_arrow---------------------------------------------             
        #成功判斷箭頭計數暫存
        self.straight_number = 0              
        self.right_number = 0 
        self.left_number = 0 
        #----------------------------------------------------------------------

    #修正yaw值(不超過180度)
    # def get_yaw(self):  
    #     if (send.imu_value_Yaw - send.imu_value_Yaw_offset) <= -200:
    #         send.imu_value_Yaw_offset -= 360
    #     elif (send.imu_value_Yaw - send.imu_value_Yaw_offset) >= 200:
    #         send.imu_value_Yaw_offset += 360
    #     send.imu_value_Yaw = send.imu_value_Yaw - send.imu_value_Yaw_offset
    #     rospy.logwarn(f"Yaw = {send.imu_value_Yaw}")

    #90度右轉
    def turn_right_ninety(self):    #已改
        rospy.loginfo(f'箭頭右轉')
        send.sendContinuousValue(2100, ORIGIN_Y, 0, -5 + ORIGIN_THETA, 0)
        send.sendHeadMotor(2, 2600, 50)
        time.sleep(0.01)
        # self.get_yaw()
        if  send.imu_value_Yaw < -85:         #成功右轉90度後停止
            rospy.loginfo(f"右轉結束")
            rospy.loginfo(f"====================")
            # send.imu_value_Yaw_offset = send.imu_value_Yaw 
            send.sendSensorReset(0, 0, 1)
            self.finish_right_flag = True

    #90度左轉
    def turn_left_ninety(self):     #已改
        # self.get_yaw()
        rospy.loginfo(f'箭頭左轉')
        send.sendContinuousValue(2100, ORIGIN_Y, 0, 5 + ORIGIN_THETA ,0)
        send.sendHeadMotor(2, 2600, 50)
        time.sleep(0.01)
        if  send.imu_value_Yaw > 80:          #成功左轉90度
            rospy.loginfo(f"左轉結束")
            rospy.loginfo(f"====================")
             #send.imu_value_Yaw_offset = send.imu_value_Yaw
            send.sendSensorReset(0, 0, 1)
            self.finish_left_flag = True

    #直走
    def go_straight(self):          #已改
        self.theta = ORIGIN_THETA
        rospy.loginfo(f"箭頭直走!")
        # self.get_yaw()
        self.speed = 3600
        if 0 < send.yolo_X <= 140: #大偏離箭頭修正
            self.theta = 4
            rospy.loginfo(f"箭頭向左修正")
            rospy.loginfo(f"====================")
            send.sendContinuousValue(self.speed, ORIGIN_Y, 0, self.theta + ORIGIN_THETA, 0)
        elif send.yolo_X >= 180:
            self.theta = -4
            rospy.loginfo(f"箭頭向右修正")
            rospy.loginfo(f"====================")
            send.sendContinuousValue(self.speed, ORIGIN_Y, 0, self.theta + ORIGIN_THETA, 0)   #副涵式結束後也有一個sendContinuousValue,是不是留一個就好？
        else:
            if  send.imu_value_Yaw > 8:       #第一段直線的偏離修正
                self.theta = -2 + ORIGIN_THETA
                rospy.loginfo(f"直線向右修正")
                rospy.loginfo(f"====================")
            elif send.imu_value_Yaw < -8:
                self.theta = 2 + ORIGIN_THETA
                rospy.loginfo(f"直線向左修正")
                rospy.loginfo(f"====================")

    #箭頭訓練
    def arrow_train(self):       
        send.drawImageFunction(1, 1, send.yolo_XMin, send.yolo_XMax, send.yolo_YMin, send.yolo_YMax, 255, 0, 0)
        rospy.loginfo(f"arrow_X = {send.yolo_X}")
        rospy.loginfo(f"arrow_Y = {send.yolo_Y}")
        rospy.loginfo(f"labei = {send.yolo_Label}")     #不知道?是判斷到的箭頭名稱咪?????QAQ
        if send.yolo_Label != "none":
            if send.yolo_Label == "straight":
                self.straight_number += 1
                self.right_number = 0
                self.left_number = 0
            elif send.yolo_Label == "left":
                self.straight_number = 0
                self.right_number = 0
                self.left_number += 1
            elif send.yolo_Label == "right":
                self.straight_number = 0
                self.right_number += 1
                self.left_number = 0

    #第二段箭頭對正
    def face_the_arrow(self):  #已改
        self.calculate_center_xy()
        self.theta = 0
        self.speed = -500
        rospy.loginfo(f"斜率 = {self.x_division_y}")
        if -0.05 <= self.x_division_y <= 0.05:       #!!!!!這邊這樣寫會有bug,當x_division_y=0.05就會跳過去#已改
            self.theta = 0 + ORIGIN_THETA
            self.positive_arrow_times += 1     #重複確認對正箭頭   #!!!!!flag 只會有true和false #已改
        #往右
        elif -0.4 <= self.x_division_y < -0.05:
            self.theta = -3 + ORIGIN_THETA
        elif self.x_division_y < -0.4:
            self.theta = -4 + ORIGIN_THETA
            self.speed = 0
        #往左
        elif 0.4 >= self.x_division_y > 0.05:
            self.theta = 3 + ORIGIN_THETA
        elif self.x_division_y > 0.4:
            self.theta = 4 + ORIGIN_THETA
            self.speed = 0

    #判斷出箭頭(一定會判斷出某個箭頭)
    def judge_the_arrow(self):   
            if self.straight_number >= 7:       #成功連續判斷5次
                self.straight_number = 0
                rospy.loginfo(f"直走箭頭判斷成功")
                self.see_arrow_flag = True
                #self.is_left_flag = False (我覺得不用?)
            elif self.right_number >= 7:
                self.right_number = 0
                rospy.loginfo(f"右轉箭頭判斷成功")
                self.see_arrow_flag = True
                self.is_right_flag = True
                #self.is_left_flag = False (我覺得不用?)
            elif self.left_number >= 5:
                self.left_number = 0
                rospy.loginfo(f"左轉箭頭判斷成功")
                self.see_arrow_flag = True
                self.is_left_flag = True

    #第一段的斜率對應機器人速度、角度
    def change_speed_theta(self):
        self.calculate_center_xy()
        #線在視窗最下面時(大轉的修正)
        if self.big_turn_right:
            self.theta = -4 + ORIGIN_THETA
            self.speed = 3200
        elif self.big_turn_left:
            self.theta = 4 + ORIGIN_THETA
            self.speed = 3200
        else:
            
            #直走
            if self.x_division_y >= 0.9:
                self.theta = 4 + ORIGIN_THETA
                self.speed = 3400
            elif self.x_division_y >= 0:
                self.speed = int(SPEED_CHANGE[math.floor(self.x_division_y / 0.1)])
                self.theta = int(THETA_CHANGE[math.floor(self.x_division_y / 0.1)]) + ORIGIN_THETA
            elif  self.x_division_y <= -0.9:
                self.theta = -4 + ORIGIN_THETA
                self.speed = 3400
            else:
                self.speed = int(SPEED_CHANGE[math.floor(-self.x_division_y / 0.1)])
                self.theta = -int(THETA_CHANGE[math.floor(-self.x_division_y / 0.1)]) + ORIGIN_THETA
            if self.correct_walking_right:
                self.theta -= 3
                self.speed = 3000
                self.correct_walking_right = False
            elif self.correct_walking_left:
                self.theta += 3
                self.speed = 3000
                self.correct_walking_left = False

    #計算xy中心位置
    def calculate_center_xy(self):    
        cnt1 = 0
        cnt2 = 0
        total_x1 = 0
        total_y1 = 0
        total_x2 = 0
        total_y2 = 0
        center_x1 = 0
        center_y1 = 0
        center_x2 = 0
        center_y2 = 0
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
                    if ymin < 75:
                        ymin = 75
        if xmax != 0 :
            for high in range(ymin, ymax):
                for wight in range(xmin, xmax):
                    cen_y = (ymin + ymax) / 2
                    if high < cen_y:
                        if send.Label_Model[high * 320 + wight] != 0:
                            total_x1 += wight
                            total_y1 += high
                            cnt1 += 1
                    else: 
                        if send.Label_Model[high*320 + wight] != 0:
                            total_x2 += wight
                            total_y2 += high
                            cnt2 += 1
            center_x1 = int(total_x1 / cnt1)
            center_y1 = int(total_y1 / cnt1)
            center_x2 = int(total_x2 / cnt2)   #已改
            center_y2 = int(total_y2 / cnt2)   #已改
            self.x_division_y = (center_x1 - center_x2) / (center_y1 - center_y2)      #已改
            send.drawImageFunction(5, 1, xmin, xmax, ymin, ymax, 0, 0, 0)
            send.drawImageFunction(2, 0, center_x1, center_x2, center_y1, center_y2, 0, 0, 0)
            rospy.logwarn(f"x = {(center_x1 + center_x2)/2}")
            rospy.logwarn(f"x%y = {self.x_division_y}")   #!!!!!print記得改掉 #已改
        else :
            self.x_division_y = 0
        #機器人大偏離或是已經要進入第二階段(線在視窗下方):
        if ymin > 150:
            if center_x2 > 240:
                self.big_turn_right = True
            elif center_x2 < 80:
                self.big_turn_left = True
            else:
                self.line_bottom_flag = True
        else:
            if(-0.4 < self.x_division_y < 0.4 and ((center_x1 + center_x2) / 2 > 180)):
                self.correct_walking_right = True
            elif (-0.4 < self.x_division_y < 0.4 and((center_x1 + center_x2) / 2 < 140)):
                self.correct_walking_left = True

    #開啟第二段
    def dial(self): 
        if send.DIOValue != 24:
            self.see_arrow_flag = True         #成功判斷銀幕內有箭頭 #已改
            self.line_bottom_flag = True       #線段只有在銀幕下方
            self.enter_arrow_flag = True        #進入第二段 #已改

    #主程式
    def main(self):
        self.dial()
        if send.is_start: 
            #rospy.loginfo(f"{start}")
            if self.start:
                #rospy.loginfo(f"{self.start}")
                self.initial()
                send.sendHeadMotor(2, 2600, 50)
                send.sendHeadMotor(1, 2048, 50)
                time.sleep(0.01)
                send.sendSensorReset(1, 1, 1)
                send.sendBodyAuto(0, 0, 0, 0, 1, 0)
                self.start = False  
            else:
                #判斷是否進入第二階段(看到箭頭、線在視窗下方)
                if self.see_arrow_flag and self.line_bottom_flag:
                    if not self.turn_now_flag:              #找箭頭
                        self.arrow_train()
                        self.judge_the_arrow()
                    if not self.enter_arrow_flag:            #進入第二段
                        send.sendHeadMotor(2, 2600, 50)       #已改
                        time.sleep(0.01)
                        self.face_the_arrow()
                        if self.positive_arrow_times >= 5:         #多次判斷有正對第一個箭頭
                            self.enter_arrow_flag = True
                            self.turn_now_flag = True
                            send.sendSensorReset(0, 0, 1)
                            # send.imu_value_Yaw_offset = send.imu_value_Yaw
                            self.positive_arrow_times = 0
                            send.sendHeadMotor(2, 2600, 50)
                            time.sleep(0.01)
                        rospy.loginfo(f"對正第一個箭頭 = {self.enter_arrow_flag}")
                        send.sendContinuousValue(self.speed, ORIGIN_Y, 0, self.theta, 0)
                    else:
                        if send.yolo_Y >= 200:
                            self.speed = 2000
                            self.positive_arrow_times += 1     
                            if self.positive_arrow_times >= 5:     #多次判斷有正對箭頭
                                self.turn_now_flag = True
                                self.positive_arrow_times = 0
                            rospy.loginfo(f"對正箭頭 = {self.turn_now_flag}")
                            
                        #多次成功判斷右轉
                        if self.is_right_flag and self.turn_now_flag:
                            self.turn_right_ninety()
                            if self.finish_right_flag:#完成90度右轉判斷旗標歸零
                                send.sendHeadMotor(2, 2600, 50)
                                time.sleep(0.01)
                                self.is_right_flag = False
                                self.finish_right_flag = False
                                self.turn_now_flag = False

                        #多次成功判斷左轉
                        elif self.is_left_flag and self.turn_now_flag:
                            self.turn_left_ninety()
                            if self.finish_left_flag:#完成90度左轉判斷旗標歸零
                                send.sendHeadMotor(2, 2600, 50)
                                time.sleep(0.01)
                                self.is_left_flag = False
                                self.finish_left_flag = False
                                self.turn_now_flag = False
                        else:#沒有任何判斷就直走
                            self.go_straight()                    
                            send.sendContinuousValue(self.speed, ORIGIN_Y, 0, self.theta, 0)
                            if self.turn_now_flag:
                                self.turn_now_flag = False               
                else:
                    #第一段
                    self.change_speed_theta()
                    #判斷是否有箭頭(下面那行)
                    self.arrow_train()
                    self.judge_the_arrow()
                    rospy.loginfo(f"角度：{self.theta}")
                    rospy.loginfo(f"速度：{self.speed}")
                    rospy.loginfo(f"====================")
                    send.sendContinuousValue(self.speed, ORIGIN_Y, 0, self.theta, 0)
        if not send.is_start:
            if not self.start:    #已改
                self.initial()
                time.sleep(0.01)
                send.sendBodyAuto(0, 0, 0, 0, 1, 0)
                self.start = True
            
if __name__ == '__main__':
    try:
        mar = Marathon()
        r = rospy.Rate(30)       #5Hz
        while not rospy.is_shutdown():
            mar.main() 
            r.sleep()
    except rospy.ROSInterruptException:
        pass
