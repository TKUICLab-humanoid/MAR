#!/usr/bin/env python
#coding=utf-8
import rospy
import numpy as np
from Python_API import Sendmessage
import time
# import cv2
import math

send = Sendmessage()
imgdata = [[None for high in range(240)]for wight in range(320)]
HEAD_Y_HIGH = 1500

class Mar:
    def __init__(self):
        self.initial()
    
    def initial(self): 
        self.start = False                                                 #initial
        self.arrow_flag = False
        self.arrow_part = False
        self.finish_turn_flag = False
        self.first_arrow_flag = True
        self.turn_now_flag = False
        self.speed_x = 0
        self.speed_y = 0
        self.origin_theta = 0 
        self.theta = 0
        self.slope = 0
        self.arrow_cnt_times = 0
        self.yaw_temp = 0
        self.yaw = 0
        self.arrow_right = 0                                            
        self.arrow_left = 0                                             
        self.arrow_straight_temp = 0
        self.arrow_right_temp = 0
        self.arrow_left_temp = 0
        self.arrow_x_center = 0
        self.arrow_y_center = 0
        self.arrow_yolo_Xmin = 0
        self.arrow_yolo_Xmax = 0
        self.arrow_yolo_Ymin = 0
        self.arrow_yolo_Ymax = 0
        self.arrow_yolo_X = 0
        self.arrow_yolo_Y = 0
        self.arrow_yolo_Label = 'none'
        self.line_status = 'online'
        self.object_center_x1 = 0
        self.object_center_y1 = 0
        self.object_center_x2 = 0
        self.object_center_y2 = 0
    
    def arrow_turn(self):
        self.finish_turn_flag = False
        self.yaw = send.imu_value_Yaw
        self.yaw_calculate()
        send.sendHeadMotor(2, HEAD_Y_HIGH - 100, 50)
        time.sleep(0.01)
        if self.arrow_right >= 1:
            rospy.loginfo(f'箭頭：右轉')
            send.sendContinuousValue(2500, 0, 0, -4 + self.origin_theta, 0)
            if  self.yaw - self.yaw_temp < -86:#成功右轉90度
                rospy.loginfo(f'箭頭右轉結束')
                self.yaw_temp = self.yaw
                rospy.logwarn(f'yaw = {self.yaw - self.yaw_temp}')
                self.finish_turn_flag = True
                self.arrow_right = 0 
        elif self.arrow_left >= 1:
            rospy.loginfo(f'箭頭：左轉')
            send.sendContinuousValue(2300, 0, 0, 5 + self.origin_theta, 0)
            if  self.yaw - self.yaw_temp > 86:#成功左轉90度
                rospy.loginfo(f'箭頭左轉結束')
                self.yaw_temp = self.yaw
                rospy.logwarn(f'yaw = {self.yaw - self.yaw_temp}')
                self.finish_turn_flag = True
                self.arrow_left = 0
        if self.finish_turn_flag:#完成90度左轉判斷旗標歸零
            send.sendHeadMotor(2, HEAD_Y_HIGH, 50)
            time.sleep(0.01)
            self.finish_turn_flag = False
            self.turn_now_flag = False
    
    # def imu_right(self):#90度右轉
    #     self.finish_turn_flag = False
    #     self.yaw = send.imu_value_Yaw
    #     rospy.loginfo(f'箭頭：右轉')
    #     send.sendContinuousValue(2500, 0, 0, -4 + self.origin_theta, 0)
    #     send.sendHeadMotor(2, HEAD_Y_HIGH - 100, 50)
    #     time.sleep(0.01)
    #     self.yaw_calculate()
    #     if  self.yaw - self.yaw_temp < -86:#成功右轉90度
    #         rospy.loginfo(f'箭頭右轉結束')
    #         self.yaw_temp = self.yaw
    #         rospy.logwarn(f'yaw = {self.yaw - self.yaw_temp}')
    #         self.finish_turn_flag = True   
    #     if self.finish_turn_flag:#完成90度右轉判斷旗標歸零
    #         send.sendHeadMotor(2, HEAD_Y_HIGH, 50)
    #         time.sleep(0.01)
    #         self.arrow_right = 0
    #         self.finish_turn_flag = False
    #         self.turn_now_flag = False

    # def imu_left(self):#90度左轉
    #     self.finish_turn_flag = False
    #     self.yaw = send.imu_value_Yaw
    #     self.yaw_calculate()
    #     rospy.loginfo(f'箭頭：左轉')
    #     send.sendContinuousValue(2300, 0, 0, 5 + self.origin_theta, 0)
    #     send.sendHeadMotor(2, HEAD_Y_HIGH - 100, 50)
    #     time.sleep(0.01)
    #     if  self.yaw - self.yaw_temp > 86:#成功左轉90度
    #         rospy.loginfo(f'箭頭左轉結束')
    #         self.yaw_temp = self.yaw
    #         rospy.logwarn(f'yaw = {self.yaw - self.yaw_temp}')
    #         self.finish_turn_flag = True

    def imu_go(self):#直走
        self.theta = self.origin_theta
        rospy.loginfo(f'直走')
        self.yaw = send.imu_value_Yaw
        self.speed_x = 2000
        if 0 < self.arrow_x_center <= 140:
            self.theta = 5
            send.sendContinuousValue(self.speed_x, 0, 0, self.theta + self.origin_theta, 0)
        elif self.arrow_x_center >= 180:
            self.theta = -5
            send.sendContinuousValue(self.speed_x, 0, 0, self.theta + self.origin_theta, 0)
        else:
            self.yaw_calculate()
            if  self.yaw - self.yaw_temp > 6:
                self.theta = -3 + self.origin_theta
                rospy.loginfo(f'修正：右轉')
            elif self.yaw - self.yaw_temp < 0:
                self.theta = 3 + self.origin_theta
                rospy.loginfo(f'修正：左轉')
        send.sendContinuousValue(self.speed_x, 0, 0, self.theta, 0)
        if self.turn_now_flag:
            self.turn_now_flag = False

    def yaw_calculate(self):
        if -240 > self.yaw - self.yaw_temp:
                self.yaw = self.yaw + 360
        elif self.yaw - self.yaw_temp > 240:
                self.yaw = self.yaw - 360

    # def arrow_detect(self):#判斷箭頭
    #     #cap = cv2.VideoCapture(7)
    #     self.arrow_y_center = 0
    #     self.arrow_x_center = 0
    #     STRAIGHT_casecade = cv2.CascadeClassifier("/home/iclab/Desktop/MAR/src/strategy/Parameter/cascade2Straight.xml")
    #     RIGHT_casecade = cv2.CascadeClassifier("/home/iclab/Desktop/MAR/src/strategy/Parameter/cascade2Right.xml")
    #     LEFT_casecade = cv2.CascadeClassifier("/home/iclab/Desktop/MAR/src/strategy/Parameter/cascade2Left.xml")
    #     #ret, frame = cap.read()
    #     frame = send.originimg
    #     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #     gray = cv2.equalizeHist(gray)
    #     #分辨箭頭, detectMultiScale(img, 縮圖倍率, minNeighbors(只有其"鄰居"大於該值才被認為是正確結果), Flag(沒用到), minSize)
    #     straight = STRAIGHT_casecade.detectMultiScale(gray, 1.1, 5, 0, (10, 10))
    #     right = RIGHT_casecade.detectMultiScale(gray, 1.1, 5, 0, (10, 10))
    #     left = LEFT_casecade.detectMultiScale(gray, 1.1, 5, 0, (10, 10))
    #     if len(straight) > 0:
    #         for a in straight:  # 單獨框出每一張人臉
    #             x, y, w, h = a
    #             #框箭頭, cv2.rectangle(img, 左上頂點, 右下頂點, 框的顏色, 框的粗細)
    #             cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
    #         send.drawImageFunction(1, 1, x, x + w, y, y + h, 255, 0, 0)
    #         self.arrow_straight_temp += 1
    #         self.arrow_right_temp = 0
    #         self.arrow_left_temp = 0
    #         self.arrow_y_center = y + h / 2 #計算箭頭的y軸位置
    #         self.arrow_x_center = x + w / 2
    #         # print("straight")
    #     elif len(right) > 0:
    #         for a in right:  # 單獨框出每一張人臉
    #             x, y, w, h = a
    #             cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
    #         send.drawImageFunction(1, 1, x, x + w, y, y + h, 255, 0, 0)
    #         self.arrow_straight_temp = 0
    #         self.arrow_right_temp += 1
    #         self.arrow_left_temp = 0
    #         self.arrow_y_center = y + h / 2 #計算箭頭的y軸位置
    #         self.arrow_x_center = x + w / 2
    #         # print("right")
    #     elif len(left) > 0:
    #         for a in left:  # 單獨框出每一張人臉
    #             x, y, w, h = a
    #             cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
    #         send.drawImageFunction(1, 1, x, x + w, y, y + h, 255, 0, 0)
    #         self.arrow_straight_temp = 0
    #         self.arrow_right_temp = 0
    #         self.arrow_left_temp += 1
    #         self.arrow_y_center = y + h / 2#計算箭頭的y軸位置
    #         self.arrow_x_center = x + w / 2
    #         # print("left")
    #     if self.arrow_straight_temp >= 5:#成功連續判斷20次
    #         self.arrow_straight_temp = 0
    #         rospy.logdebug(f'go Straight')
    #         self.arrow_flag = True
    #         self.arrow_right = 0
    #         self.arrow_left = 0
    #     elif self.arrow_right_temp >= 4:
    #         self.arrow_right_temp = 0
    #         rospy.logdebug(f'go Right')
    #         self.arrow_flag = True
    #         self.arrow_right += 1
    #         self.arrow_left = 0
    #     elif self.arrow_left_temp >= 5:
    #         self.arrow_left_temp = 0
    #         rospy.logdebug(f'go Left')
    #         self.arrow_flag = True
    #         self.arrow_right = 0
    #         self.arrow_left += 1
        
    def arrow_yolo(self):
        self.arrow_y_center = 0
        self.arrow_x_center = 0
        if send.yolo_Label != 'none':
            if send.yolo_Label == 'straight':
                self.arrow_straight_temp += 1
                self.arrow_right_temp = 0
                self.arrow_left_temp = 0 
            elif send.yolo_Label == 'left':
                self.arrow_straight_temp = 0
                self.arrow_right_temp = 0
                self.arrow_left_temp += 1
            elif send.yolo_Label == 'right':
                self.arrow_straight_temp = 0
                self.arrow_right_temp += 1
                self.arrow_left_temp = 0
            self.arrow_y_center = send.yolo_Y
            self.arrow_x_center = send.yolo_X
            if self.arrow_straight_temp >= 5:#成功連續判斷20次
                self.arrow_straight_temp = 0
                rospy.logdebug(f'go Straight')
                self.arrow_flag = True
                self.arrow_right = 0
                self.arrow_left = 0
            elif self.arrow_right_temp >= 4:
                self.arrow_right_temp = 0
                rospy.logdebug(f'go Right')
                self.arrow_flag = True
                self.arrow_right += 1
                self.arrow_left = 0
            elif self.arrow_left_temp >= 5:
                self.arrow_left_temp = 0
                rospy.logdebug(f'go Left')
                self.arrow_flag = True
                self.arrow_right = 0
                self.arrow_left += 1
        send.drawImageFunction(1, 1, send.yolo_XMin, send.yolo_XMax, send.yolo_YMin, send.yolo_YMax, 255, 0, 0)

    def correct_go_to_arrow(self):
        rospy.logwarn(f'slope = {self.slope}')
        if -0.1 < self.slope < 0.1:
            self.theta = 0 + self.origin_theta
            self.speed_x = 0
            self.arrow_cnt_times += 1
        #turn right
        elif -0.4 < self.slope < -0.05:
            self.theta = -2 + self.origin_theta
        elif self.slope < -0.4:
            self.theta = -4 + self.origin_theta
            self.speed_x = -500
            self.speed_y = 800
        #turn left 
        elif 0.4 > self.slope > 0.05:
            self.theta = 1 + self.origin_theta
        elif self.slope > 0.4:
            self.theta = 3 + self.origin_theta
            self.speed_x = -500
            self.speed_y = 500
        if self.arrow_cnt_times >= 5:
            self.first_arrow_flag = False
            self.yaw_temp = send.imu_value_Yaw
            self.arrow_cnt_times = 0
            send.sendHeadMotor(2, 1500, 50)
            time.sleep(0.01)

    # def arrow_check(self):
    #     if self.arrow_straight_temp >= 5:#成功連續判斷20次
    #         self.arrow_straight_temp = 0
    #         rospy.logdebug(f'go Straight')
    #         self.arrow_flag = True
    #         self.arrow_right = 0
    #         self.arrow_left = 0
    #     elif self.arrow_right_temp >= 4:
    #         self.arrow_right_temp = 0
    #         rospy.logdebug(f'go Right')
    #         self.arrow_flag = True
    #         self.arrow_right += 1
    #         self.arrow_left = 0
    #     elif self.arrow_left_temp >= 5:
    #         self.arrow_left_temp = 0
    #         rospy.logdebug(f'go Left')
    #         self.arrow_flag = True
    #         self.arrow_right = 0
    #         self.arrow_left += 1

    def theta_value(self):#判斷斜率
        if self.line_status == 'big_turn_right':
            self.theta = -5 + self.origin_theta
            self.speed_x = 3000
        elif self.line_status == 'big_turn_left':
            self.theta = 5 + self.origin_theta
            self.speed_x = 3000
        else:
            sp = [3500, 3400, 3300, 3300, 3200, 3200, 3100, 3100, 3100]
            th = [0, 0, 0, 1, 1, 2, 2, 3, 4]
            #walk straight
            if self.object_center_x2 < 140 and abs(self.slope) < 0.3:
                self.theta = 5 + self.origin_theta
                self.speed_x = 3000
            elif self.object_center_x2 > 180 and abs(self.slope) < 0.3:
                self.theta = -5 + self.origin_theta
                self.speed_x = 3000
            elif self.slope >= 0.9:
                self.theta = 5 + self.origin_theta
                self.speed_x = 3000
            elif self.slope >= 0:
                self.speed_x = int(sp[math.floor(self.slope / 0.1)])
                self.theta = int(th[math.floor(self.slope / 0.1)]) + self.origin_theta
            elif  self.slope <= -0.9:
                self.theta = -5 + self.origin_theta
                self.speed_x = 3000
            else:
                self.speed_x = int(sp[math.floor(-self.slope / 0.1)])
                self.theta = -int(th[math.floor(-self.slope / 0.1)]) + self.origin_theta
            if self.line_status == 'line_at_right':
                self.theta = -3 + self.theta
                self.speed_x = 3000
            elif self.line_status == 'line_at_left':
                self.theta = 3 + self.theta
                self.speed_x = 3000

    def region(self):
        targetxmin = 320
        targetxmax = 0
        targetymin = 240
        targetymax = 0
        self.object_center_y2 = 0
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
                        targetymax = send.color_mask_subject_YMax[i][j] 
        total_x1 = 0
        total_y1 = 0
        cnt1 = 0
        total_x2 = 0
        total_y2 = 0
        cnt2 = 0
        if targetxmin != 320:
            for i in range (targetxmin, targetxmax):
                for j in range (targetymin, targetymax):
                    if j < (targetymax + targetymin) / 2:
                        if send.Label_Model[320*j+i] != 0:
                            total_x1 += i
                            total_y1 += j
                            cnt1 += 1
                    else:
                        if send.Label_Model[320*j+i] != 0:
                            total_x2 += i
                            total_y2 += j
                            cnt2 += 1
            send.drawImageFunction(5, 1, targetxmax, targetxmin, targetymax, targetymin, 0, 0, 0)
        if cnt1 > 0 and cnt2 > 0:
            self.object_center_x1 = int(total_x1 / cnt1)
            self.object_center_y1 = int(total_y1 / cnt1)
            self.object_center_x2 = int(total_x2 / cnt2)
            self.object_center_y2 = int(total_y2 / cnt2)
            self.slope = (self.object_center_x2 - self.object_center_x1) / (self.object_center_y2 - self.object_center_y1)
            send.drawImageFunction(2, 0, self.object_center_x2, self.object_center_x1, self.object_center_y2, self.object_center_y1, 0, 0, 0)
            self.line_status = 'online'
        else:
            self.slope = 0

    def modify(self):
        if (self.object_center_y1 + self.object_center_y2) / 2 > 180:
            if self.object_center_x2 > 220:
                self.line_status = 'big_turn_right'
            elif self.object_center_x2 < 80:
                self.line_status = 'big_turn_left'
            else:
                self.line_status = 'arrow'#進入第二階段的指標，線在機器人螢幕的正下方
        else:#計算斜率
            if (self.object_center_x2 + self.object_center_x1) / 2 < 140 :
                self.line_status = 'line_at_left'
            elif (self.object_center_x2 + self.object_center_x1) / 2 > 180:
                self.line_status = 'line_at_right'

    def print_data(self):
        rospy.logdebug(f'line in image = {self.line_status}')
        rospy.logdebug(f'theta = {self.theta}')
        rospy.logdebug(f'speed = {self.speed_x}')
        rospy.logdebug(f'object_x1 = {self.object_center_x1}')
        rospy.logdebug(f'object_x2 = {self.object_center_x2}')
        rospy.logwarn(f'slope = {self.slope}')
        rospy.logwarn(f'arrow = {self.arrow_flag}')
        rospy.logwarn(f' ==============================================')
    
    def main(self):
        if send.is_start:   #箭頭
            if self.start:
                self.initial()
                send.sendHeadMotor(2, HEAD_Y_HIGH, 50)
                time.sleep(0.5)
                send.sendHeadMotor(1, 2048, 50)
                time.sleep(0.5)
                send.sendSensorReset()
                time.sleep(0.5)
                send.sendBodyAuto(0, 0, 0, 0, 1, 0)
                self.start = False
            else:
                if not self.arrow_part and send.DIOValue == 24:
                    self.region()
                    self.modify()
                    self.theta_value()  #線的斜率
                    # self.arrow_detect()#判斷是否有箭頭
                    self.arrow_yolo()
                    # self.arrow_check()
                    self.print_data()
                    if self.arrow_flag and self.line_status == 'arrow':
                        self.arrow_part = True
                    send.sendContinuousValue(self.speed_x, 0, 0, self.theta, 0)
                elif self.arrow_part or send.DIOValue != 24:#判斷是否有箭頭與是否要進入第二階段
                    if not self.turn_now_flag:
                        self.arrow_yolo()
                        # self.arrow_detect()
                        # self.arrow_check()
                    rospy.logdebug(f'X = {self.arrow_x_center}')
                    rospy.logdebug(f'Y = {self.arrow_y_center}')
                    rospy.loginfo(f'Yaw_send = {send.imu_value_Yaw}')
                    rospy.logwarn(f'Yaw = {self.yaw - self.yaw_temp}')
                    if self.first_arrow_flag:  #讓機器人正對第1個箭頭
                        send.sendHeadMotor(2, HEAD_Y_HIGH, 50)
                        self.region()
                        self.correct_go_to_arrow()
                        self.turn_now_flag = True
                        rospy.logdebug(f'next flag = {self.first_arrow_flag}')
                        send.sendContinuousValue(self.speed_x, self.speed_y, 0, self.theta, 0)
                        send.sendHeadMotor(2, HEAD_Y_HIGH - 100, 50)
                    else:
                        if self.arrow_y_center >= 170:
                            self.speed_x = 2000
                            self.arrow_cnt_times += 1
                            if self.arrow_cnt_times >= 4:
                                self.turn_now_flag = True
                                self.arrow_cnt_times = 0
                        #print(turn_now_flag)
                        if (self.arrow_right >= 1 or self.arrow_left >= 1) and self.turn_now_flag:
                            self.arrow_turn()
                        # if self.arrow_right >= 1 and self.turn_now_flag:#多次成功判斷右轉與判斷箭頭在銀幕下方
                        #     self.imu_right()
                        #     if self.finish_turn_flag:#完成90度右轉判斷旗標歸零
                        #         # send.sendHeadMotor(2, HEAD_Y_HIGH, 50)
                        #         time.sleep(0.2)
                        #         self.arrow_right = 0
                        #         self.finish_turn_flag = False
                        #         self.turn_now_flag = False
                        # elif self.arrow_left >= 1 and self.turn_now_flag:#多次成功判斷左轉與判斷箭頭在銀幕下方
                        #     self.imu_left()
                        #     if self.finish_turn_flag:#完成90度左轉判斷旗標歸零
                        #         # send.sendHeadMotor(2, HEAD_Y_HIGH, 50)
                        #         time.sleep(0.2)
                        #         self.arrow_left = 0
                        #         self.finish_turn_flag = False
                        #         self.turn_now_flag = False
                        else:#沒有任何判斷就直走
                            self.imu_go()                    
                            # send.sendContinuousValue(self.speed_x, 0, 0, self.theta, 0)
                            # if self.turn_now_flag:
                            #     self.turn_now_flag = False
        if not send.is_start:
            if not self.start:
                send.sendBodyAuto(0, 0, 0, 0, 1, 0)
                self.start = True

if __name__ == '__main__':
    try:
        mar = Mar()
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            mar.main()
            r.sleep()
    except rospy.ROSInterruptException:
        pass