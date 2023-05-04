#!/usr/bin/env python
#coding=utf-8
import rospy
import numpy as np
from Python_API import Sendmessage
import time
# import cv2
import math

ORIGIN_SPEED = 200
ORIGIN_THETA = 0
HEAD_Y_HIGH = 1500
SPEED_LIST = [3300, 3200, 3200, 3100, 3100, 3000, 3000, 2900, 2900, 2800]
THETA_LIST = [0, 1 , 1, 2, 2, 3, 3, 4, 4, 5]
aaaa = rospy.init_node('talker', anonymous=True, log_level=rospy.DEBUG)
send = Sendmessage()

class Mar:
    def __init__(self):
        self.initial()
    
    def initial(self): 
        self.status = 'First'                                                 #initial
        self.arrow_flag = False
        self.finish_turn_flag = False
        self.first_arrow_flag = True
        self.turn_now_flag = False
        self.speed_x = 0
        self.speed_y = 0
        self.theta = 0
        self.variation = 0
        self.arrow_cnt_times = 0
        self.yaw_temp = 0
        self.yaw = 0
        self.arrow_right = 0                                            
        self.arrow_left = 0                                             
        self.arrow_straight_temp = 0
        self.arrow_right_temp = 0
        self.arrow_left_temp = 0
        self.line_status = 'online'
        self.line_up_x = 0
        self.line_up_y = 0
        self.line_down_x = 0
        self.line_down_y = 0
        send.sendHeadMotor(2, HEAD_Y_HIGH, 50)
        send.sendHeadMotor(1, 2048, 50)
        send.sendSensorReset()

    def arrow_part(self):
        self.finish_turn_flag = False
        self.yaw_calculate()
        send.sendHeadMotor(2, HEAD_Y_HIGH - 100, 50)
        time.sleep(0.01)
        if self.arrow_right >= 1:
            rospy.logdebug(f'箭頭：右轉')
            self.speed_x = 2500
            self.theta = -5
            # send.sendContinuousValue(2500, 0, 0, -4 + ORIGIN_THETA, 0)
            if  self.yaw - self.yaw_temp < -86:#成功右轉90度
                rospy.logdebug(f'箭頭右轉結束')
                self.yaw_temp = self.yaw
                self.finish_turn_flag = True
                self.arrow_right = 0 
        elif self.arrow_left >= 1:
            rospy.logdebug(f'箭頭：左轉')
            self.speed_x = 2300
            self.theta = 5
            # send.sendContinuousValue(2300, 0, 0, 5 + ORIGIN_THETA, 0)
            if  self.yaw - self.yaw_temp > 83:#成功左轉90度
                rospy.logdebug(f'箭頭左轉結束')
                self.yaw_temp = self.yaw
                self.finish_turn_flag = True
                self.arrow_left = 0
        if self.finish_turn_flag:#完成90度左轉判斷旗標歸零
            send.sendHeadMotor(2, HEAD_Y_HIGH, 50)
            time.sleep(0.01)
            self.finish_turn_flag = False
            self.turn_now_flag = False
        # else:
            # self.theta = 0 + ORIGIN_THETA
            # rospy.logdebug(f'直走')
            # self.speed_x = 2000
            # if 0 < send.yolo_X <= 140:
            #     self.theta = 5
            #     # send.sendContinuousValue(self.speed_x, 0, 0, self.theta + ORIGIN_THETA, 0)
            # elif send.yolo_X >= 180:
            #     self.theta = -5
            #     # send.sendContinuousValue(self.speed_x, 0, 0, self.theta + ORIGIN_THETA, 0)
            # else:
            #     if  self.yaw - self.yaw_temp > 6:
            #         self.theta = -3
            #         rospy.logdebug(f'修正：右轉')
            #     elif self.yaw - self.yaw_temp < 0:
            #         self.theta = 3
            #         rospy.logdebug(f'修正：左轉')
            # # send.sendContinuousValue(self.speed_x, 0, 0, self.theta, 0)
            # if self.turn_now_flag:
            #     self.turn_now_flag = False
        
    def arrow_modify(self):#直走
        self.theta = 0 + ORIGIN_THETA
        rospy.logdebug(f'直走')
        self.speed_x = 2000
        self.speed_y = 300
        if 0 < send.yolo_X <= 130:
            self.theta = 5
            self.speed_x = 1700
            send.sendContinuousValue(self.speed_x, 0, 0, self.theta + ORIGIN_THETA, 0)
        elif send.yolo_X >= 190:
            self.theta = -5
            self.speed_x = 1700
            send.sendContinuousValue(self.speed_x, 0, 0, self.theta + ORIGIN_THETA, 0)
        else:
            self.yaw_calculate()
            if  self.yaw - self.yaw_temp > 6:
                self.theta = -3 + ORIGIN_THETA
                rospy.logdebug(f'修正：右轉')
            elif self.yaw - self.yaw_temp < 0:
                self.theta = 3 + ORIGIN_THETA
                rospy.logdebug(f'修正：左轉')
        send.sendContinuousValue(self.speed_x, self.speed_y, 0, self.theta, 0)
        rospy.logwarn(f'{self.yaw - self.yaw_temp}')
        if self.turn_now_flag:
            self.turn_now_flag = False

    def yaw_calculate(self):
        self.yaw = send.imu_value_Yaw
        if -240 > self.yaw - self.yaw_temp:
                self.yaw = self.yaw + 360
        elif self.yaw - self.yaw_temp > 240:
                self.yaw = self.yaw - 360
        rospy.loginfo(f'Yaw = {self.yaw - self.yaw_temp}')
        
    def arrow_yolo(self):
        if send.yolo_Label != 'none':
            if send.yolo_Label == 'stright':
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
            if self.arrow_straight_temp >= 5:#成功連續判斷20次
                self.arrow_straight_temp = 0
                rospy.logdebug(f'go Straight')
                self.arrow_flag = True
                self.arrow_right = 0
                self.arrow_left = 0
            elif self.arrow_right_temp >= 10:
                self.arrow_right_temp = 0
                rospy.logdebug(f'go Right')
                self.arrow_flag = True
                self.arrow_right += 1
                self.arrow_left = 0
            elif self.arrow_left_temp >= 10:
                self.arrow_left_temp = 0
                rospy.logdebug(f'go Left')
                self.arrow_flag = True
                self.arrow_right = 0
                self.arrow_left += 1
        send.drawImageFunction(1, 1, send.yolo_XMin, send.yolo_XMax, send.yolo_YMin, send.yolo_YMax, 255, 0, 0)
        rospy.loginfo(f'arrow = {self.arrow_flag}')
        rospy.logdebug(f'X = {send.yolo_X}')
        rospy.logdebug(f'Y = {send.yolo_Y}')
        rospy.loginfo(f'labei = {send.yolo_Label}')

    def line_to_arrow(self):
        rospy.logdebug(f'variation = {self.variation}')
        if -0.1 <= self.variation <= 0.1:
            self.theta = 0
            self.speed_x = 0
            self.speed_y = 0
            self.arrow_cnt_times += 1
        #turn right
        elif -0.4 <= self.variation < -0.1:
            self.speed_x = 0
            self.speed_y = 0
            self.theta = -2
        elif self.variation < -0.4:
            self.theta = -4
            self.speed_x = -500
            self.speed_y = 500
        #turn left 
        elif 0.4 >= self.variation > 0.1:
            self.speed_x = 0
            self.speed_y = 0
            self.theta = 1
        elif self.variation > 0.4:
            self.theta = 3
            self.speed_x = -500
            self.speed_y = 800
        if self.arrow_cnt_times >= 5:
            self.first_arrow_flag = False
            self.yaw_temp = send.imu_value_Yaw
            self.arrow_cnt_times = 0
            send.sendHeadMotor(2, HEAD_Y_HIGH, 50)
            time.sleep(0.01)
        rospy.loginfo(f'==========================================================')
    def theta_value(self):#判斷斜率
        if (self.line_up_y + self.line_down_y) / 2 > 180:
            if self.line_down_x > 220:
                self.theta = -5
                self.speed_x = 2800
            elif self.line_down_x < 80:
                self.theta = 5
                self.speed_x = 2800
            else:
                self.line_status = 'arrow'#進入第二階段的指標，線在機器人螢幕的正下方
                rospy.loginfo(f'line in image = {self.line_status}')
        else:                           #walk straight
            if self.line_down_x < 140 and abs(self.variation) < 0.3:
                self.theta = 5
                self.speed_x = 2800
            elif self.line_down_x > 180 and abs(self.variation) < 0.3:
                self.theta = -5
                self.speed_x = 2800
            # elif self.variation >= 0.9:
            #     self.theta = 5
            #     self.speed_x = 3000
            # elif self.variation >= 0:
            #     self.speed_x = int(SPEED_LIST[math.floor(self.variation / 0.1)])
            #     self.theta = int(THETA_LIST[math.floor(self.variation / 0.1)])
            # elif  self.variation <= -0.9:
            #     self.theta = -5
            #     self.speed_x = 3000
            # else:
            #     self.speed_x = int(SPEED_LIST[math.floor(-self.variation / 0.1)])
            #     self.theta = -int(THETA_LIST[math.floor(-self.variation / 0.1)])

            elif abs(self.variation) >= 0.9:
                self.theta = 5
                self.speed_x = 2800
            elif abs(self.variation) >= 0:
                self.speed_x = int(SPEED_LIST[math.floor(self.variation / 0.1)])
                self.theta = int(THETA_LIST[math.floor(self.variation / 0.1)])
            if self.variation < 0:
                self.theta = -self.theta
            if (self.line_down_x + self.line_up_x) / 2 < 140 :
                self.theta = 3 + self.theta
                self.speed_x = 2800
            elif (self.line_down_x + self.line_up_x) / 2 > 180:
                self.theta = -3 + self.theta
                self.speed_x = 2800
            self.line_status = 'online'
        self.speed_x = self.speed_x + ORIGIN_SPEED
        rospy.logdebug(f'speed = {self.speed_x}')
        rospy.logdebug(f'theta = {self.theta}')

    def calculate_line_model(self, x_min, x_max, y_min, y_max):
        total_x = 0
        total_y = 0
        cnt = 0
        for i in range (x_min, x_max):
            for j in range(y_min, y_max):
                if send.Label_Model[320 * j + i] != 0:
                    total_x += i
                    total_y += j
                    cnt += 1
        if cnt != 0:
            line_x = int(total_x / cnt)
            line_y = int(total_y / cnt)
            return line_x, line_y
        else:
            return 0, 0

    def region(self):
        target_xmin = 320
        target_xmax = 0
        target_ymin = 240
        target_ymax = 0
        self.line_down_y = 0
        for i in range (len(send.color_mask_subject_cnts)):
            for j in range (send.color_mask_subject_cnts[i]):
                if send.color_mask_subject_size[i][j] > 500:
                    if send.color_mask_subject_XMin[i][j] < target_xmin:
                        target_xmin =  send.color_mask_subject_XMin[i][j]
                    if send.color_mask_subject_XMax[i][j] > target_xmax:
                        target_xmax = send.color_mask_subject_XMax[i][j]
                    if send.color_mask_subject_YMin[i][j] < target_ymin:
                        target_ymin = send.color_mask_subject_YMin[i][j]
                    if send.color_mask_subject_YMax[i][j] > target_ymax:
                        target_ymax = send.color_mask_subject_YMax[i][j] 
        # total_x1 = 0
        # total_y1 = 0
        # cnt1 = 0
        # total_x2 = 0
        # total_y2 = 0
        # cnt2 = 0
        if target_xmin != 320:
            self.line_up_x, self.line_up_y = self.calculate_line_model(target_xmin, target_xmax, target_ymin, int((target_ymin + target_ymax) / 2))
            self.line_down_x, self.line_down_y = self.calculate_line_model(target_xmin, target_xmax, int((target_ymin + target_ymax) / 2), target_ymax)
            # for i in range (target_xmin, target_xmax):
            #     for j in range (target_ymin, target_ymax):
            #         if j < (target_ymax + target_ymin) / 2:
            #             if send.Label_Model[320 * j + i] != 0:
            #                 total_x1 += i
            #                 total_y1 += j
            #                 cnt1 += 1
            #         else:
            #             if send.Label_Model[320 * j + i] != 0:
            #                 total_x2 += i
            #                 total_y2 += j
            #                 cnt2 += 1
            send.drawImageFunction(5, 1, target_xmax, target_xmin, target_ymax, target_ymin, 0, 0, 0)
            rospy.loginfo(f'line_up_x, line_up_y = {self.line_up_x, self.line_up_y}')
            rospy.loginfo(f'line_down_x, line_down_y= {self.line_down_x, self.line_down_y}')
        
        # if cnt_up > 0 and cnt2 > 0:
        #     self.line_up_x = int(total_up_x / cnt_up)
        #     self.line_up_y = int(total_up_y / cnt_up)
        #     self.line_down_x = int(total_x2 / cnt2)
        #     self.line_down_y = int(total_y2 / cnt2)
            self.variation = (self.line_down_x - self.line_up_x) / (self.line_down_y - self.line_up_y)
            send.drawImageFunction(2, 0, self.line_down_x, self.line_up_x, self.line_down_y, self.line_up_y, 0, 0, 0)
            
        else:
            self.variation = 0

    def main(self):
        if send.is_start:   #箭頭
            if self.status == 'First':
                self.initial()
                time.sleep(0.1)
                send.sendBodyAuto(0, 0, 0, 0, 1, 0)
                self.status = 'line' if send.DIOValue == 24 else 'Arrow_Part'
            elif self.status == 'line':
                self.region()
                self.theta_value()  #線的斜率
                self.arrow_yolo()
                if self.arrow_flag and self.line_status == 'arrow':
                    self.status = 'Arrow_Part'
                # send.sendContinuousValue(self.speed_x, 0, 0, self.theta, 0)
            elif self.status == 'Arrow_Part':
                if not self.turn_now_flag:
                    self.arrow_yolo()
                if self.first_arrow_flag:  #讓機器人正對第1個箭頭
                    send.sendHeadMotor(2, HEAD_Y_HIGH, 50)
                    self.region()
                    self.line_to_arrow()
                    self.turn_now_flag = True
                    rospy.logdebug(f'next flag = {self.first_arrow_flag}')
                    # send.sendContinuousValue(self.speed_x, self.speed_y, 0, self.theta, 0)
                    send.sendHeadMotor(2, HEAD_Y_HIGH - 100, 50)
                else:
                    if send.yolo_Y >= 150:
                        self.speed_x = 2000
                        self.arrow_cnt_times += 1
                        if self.arrow_cnt_times >= 4:
                            self.turn_now_flag = True
                            self.arrow_cnt_times = 0
                    if (self.arrow_right >= 1 or self.arrow_left >= 1) and self.turn_now_flag:
                        self.arrow_part()
                    else:#沒有任何判斷就直走
                        self.arrow_modify()  
                    # self.arrow_part()    
            rospy.loginfo(f'status = {self.status}')
            send.sendContinuousValue(self.speed_x, self.speed_y, 0, self.theta + ORIGIN_THETA, 0)             
        if not send.is_start:
            if self.status != 'First':
                self.status = 'First'
                self.initial()
                time.sleep(0.1)
                send.sendBodyAuto(0, 0, 0, 0, 1, 0)

if __name__ == '__main__':
    try:
        mar = Mar()
        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            mar.main()
            r.sleep()
    except rospy.ROSInterruptException:
        pass