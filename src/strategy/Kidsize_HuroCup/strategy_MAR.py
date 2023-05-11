#!/usr/bin/env python
#coding=utf-8
from collections import Counter, deque
import rospy
import numpy as np
from Python_API import Sendmessage
import time
# import cv2
import math

ORIGIN_THETA = 0
ORIGIN_SPEED = 3000
send = Sendmessage()

class Coordinate:
    def __init__(self, x, y):
        self.x, self.y = x, y
    def __add__(self, other):
        return Coordinate((self.x + other.x), (self.y + other.y))
    def __sub__(self, other):
        return Coordinate((self.x - other.x), (self.y - other.y))
    def __floordiv__(self, other):
        return Coordinate((self.x // other), (self.y // other))

class Mar:
    def __init__(self):
        rospy.init_node('mar', anonymous=True, log_level=rospy.DEBUG)
        self.initial()
        self.speed_x = 0
        self.speed_y = 0
        self.theta = 0
        self.seek_line = Seek_line()
        self.arrow_center = Coordinate(0, 0)
        self.arrow_temp = deque(['None', 'None', 'None', 'None', 'None'], maxlen = 5)
    
    def initial(self):
        self.status = 'First'
        self.can_turn_flag = False                                         
        self.arrow_flag = False
        self.turn_now_flag = False
        self.arrow_cnt_times = 0
        self.yaw_temp = 0                                  
        self.line_status = 'online'
        send.sendHeadMotor(2, 1500, 50)
        send.sendHeadMotor(1, 2048, 50)
        send.sendSensorReset()

    def theta_value(self):#判斷斜率
        slope = self.seek_line.calculate_slope()
        middle_point = (self.seek_line.upper_center + self.seek_line.lower_center) // 2
        if middle_point.y > 180:
            if self.seek_line.lower_center.x > 220:
                self.theta = -5 + ORIGIN_THETA
                self.speed_x = ORIGIN_SPEED
            elif self.seek_line.lower_center.x < 80:
                self.theta = 5 + ORIGIN_THETA
                self.speed_x = ORIGIN_SPEED
            else:
                self.line_status = 'arrow'#進入第二階段的指標，線在機器人螢幕的正下方
                rospy.loginfo(f'line in image = {self.line_status}')
        else:           
            if abs(slope) >= 15:
                self.theta = ORIGIN_THETA
                self.speed_x = ORIGIN_SPEED + 500
            elif 7 <= abs(slope) < 15:
                self.theta = 0
                self.speed_x = ORIGIN_SPEED + 400
            elif 4 <= abs(slope) < 7:
                self.theta = 1 if slope > 0 else -1
                self.speed_x = ORIGIN_SPEED + 300
            elif 1.5 <= abs(slope) < 4:
                self.theta = 2 if slope > 0 else -2
                self.speed_x = ORIGIN_SPEED + 200
            else:
                self.theta = 4 if slope > 0 else -4
                self.speed_x = ORIGIN_SPEED + 100
            if self.seek_line.lower_center.x < 140 and abs(slope) > 2:
                self.theta = 5 + ORIGIN_THETA
                self.speed_x = ORIGIN_SPEED
            elif self.seek_line.lower_center.x > 180 and abs(slope) > 2:
                self.theta = -5 + ORIGIN_THETA
                self.speed_x = ORIGIN_SPEED
            self.line_status = 'online'
        rospy.logdebug(f'speed = {self.speed_x}')
        rospy.logdebug(f'theta = {self.theta}')

    def line_to_arrow(self):
        slope = self.seek_line.calculate_slope()
        if abs(slope) > 15:
            self.theta = 0 + ORIGIN_THETA
            self.speed_x = 0
            self.arrow_cnt_times += 1
        elif 7 < abs(slope) < 15:
            self.theta = 1 if slope > 0 else -2 
        elif abs(slope) < 7:
            self.theta = 3 if slope > 0 else -4
            self.speed_x = -500
            self.speed_y = -500 if slope > 0 else -800
        if self.arrow_cnt_times >= 5:
            self.yaw_temp = send.imu_value_Yaw
            self.arrow_cnt_times = 0
            self.status = 'Arrow_Part'

    def arrow_yolo(self):
        self.arrow_center.x, self.arrow_center.y = 0, 0
        self.arrow_temp.append(send.yolo_Label)
        arrow_cnt_temp = len(set(self.arrow_temp))
        if arrow_cnt_temp == 1 and self.arrow_temp[0] != 'None':
            if self.arrow_temp[0] == 'left' or self.arrow_temp[0] == 'right':
                self.can_turn_flag = True
            self.arrow_center.y = send.yolo_Y
            self.arrow_center.x = send.yolo_X
            send.drawImageFunction(1, 1, send.yolo_XMin, send.yolo_XMax, send.yolo_YMin, send.yolo_YMax, 255, 0, 0)
            return True

    def yaw_calculate(self):
        if -240 > self.yaw - self.yaw_temp:
                self.yaw = self.yaw + 360
        elif self.yaw - self.yaw_temp > 240:
                self.yaw = self.yaw - 360

    def arrow_turn(self):
        self.yaw = send.imu_value_Yaw
        self.yaw_calculate()
        if self.arrow_temp[0] == 'right':
            rospy.logdebug(f'箭頭：右轉')
            send.sendContinuousValue(2500, 0, 0, -4 + ORIGIN_THETA, 0)
        elif self.arrow_temp[0] == 'left':
            rospy.logdebug(f'箭頭：左轉')
            send.sendContinuousValue(2300, 0, 0, 5 + ORIGIN_THETA, 0)
        if  abs(self.yaw - self.yaw_temp) > 86:#成功轉90度
            rospy.logdebug(f'箭頭轉彎結束')
            self.yaw_temp = self.yaw
            self.turn_now_flag = False

    def imu_go(self):#直走
        self.theta = 0 + ORIGIN_THETA
        rospy.logdebug(f'直走')
        self.yaw = send.imu_value_Yaw
        self.speed_x = 2000
        if 0 < self.arrow_center.x <= 140:
            self.theta = 5
            send.sendContinuousValue(self.speed_x, 0, 0, self.theta + ORIGIN_THETA, 0)
        elif self.arrow_center.x >= 180:
            self.theta = -5
            send.sendContinuousValue(self.speed_x, 0, 0, self.theta + ORIGIN_THETA, 0)
        else:
            self.yaw_calculate()
            if  self.yaw - self.yaw_temp > 6:
                self.theta = -3 + ORIGIN_THETA
                rospy.logdebug(f'修正：右轉')
            elif self.yaw - self.yaw_temp < 0:
                self.theta = 3 + ORIGIN_THETA
                rospy.logdebug(f'修正：左轉')
        send.sendContinuousValue(self.speed_x, 0, 0, self.theta, 0)
    
    def main(self):
        if send.is_start:
            if self.status == 'First':
                self.initial()
                time.sleep(0.01)
                send.sendBodyAuto(0, 0, 0, 0, 1, 0)
                self.status = 'line'  if send.DIOValue == 24 else 'Arrow_Part'
            elif self.status == 'line' and send.DIOValue == 24:
                self.seek_line.update()
                self.theta_value()
                arrow = self.arrow_yolo()
                if arrow and self.line_status == 'arrow':
                    self.status = 'First_arrow'
                send.sendContinuousValue(self.speed_x, 0, 0, self.theta, 0)
            elif self.status == 'First_arrow':
                self.seek_line.update()
                self.line_to_arrow()
                send.sendContinuousValue(self.speed_x, self.speed_y, 0, self.theta, 0)
            elif self.status == 'Arrow_Part':
                if self.turn_now_flag:
                    self.arrow_turn()
                else:
                    self.arrow_yolo()
                    if self.can_turn_flag:
                        if self.arrow_center.y >= 170:
                            self.arrow_cnt_times += 1
                        if self.arrow_cnt_times >= 4:
                            self.turn_now_flag = True
                            self.arrow_cnt_times = 0
                    self.imu_go()
        else:
            if self.status != 'First':
                self.status = 'First'
                send.sendBodyAuto(0, 0, 0, 0, 1, 0)

class Seek_line:
    def __init__(self):
        self.upper_center = Coordinate(0, 0)
        self.lower_center = Coordinate(0, 0)

    def update(self):
        #影像輸出為一維陣列，8bits
        img_data = np.frombuffer(send.Label_Model, dtype = np.uint8)
        img_data = img_data.reshape(240, 320)
        y_coord, x_coord = np.where(img_data != 0)
        if len(x_coord) == 0:
            self.upper_center.x, self.upper_center.y = 0, 0
            self.lower_center.x, self.lower_center.y = 0, 0
            return
        middle_y = (np.max(y_coord) + np.min(y_coord)) / 2
        upper_filter = y_coord <= middle_y
        upper_x, upper_y = x_coord[upper_filter], y_coord[upper_filter]
        self.upper_center.x = np.mean(upper_x)
        self.upper_center.y = np.mean(upper_y)
        lower_filter = y_coord > middle_y
        lower_x, lower_y = x_coord[lower_filter], y_coord[lower_filter]
        self.lower_center.x = np.mean(lower_x)
        self.lower_center.y = np.mean(lower_y)
        send.drawImageFunction(2, 0, int(self.lower_center.x), int(self.upper_center.x), int(self.lower_center.y), int(self.upper_center.y), 0, 0, 0)

    def calculate_slope(self):
        delta = self.upper_center - self.lower_center
        if delta.x == 0:
            return float("inf")
        return delta.y / delta.x

if __name__ == '__main__':
    try:
        mar = Mar()
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            mar.main()
            r.sleep()
    except rospy.ROSInterruptException:
        pass