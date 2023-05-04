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
        self.status = 'First'
        self.seek_line = SeekLine()
    
    def initial(self):
        self.arrow = Arrow()
        self.turn_now_flag = False
        self.arrow_cnt_times = 0
        self.yaw_temp = 0
        self.yaw = 0
        send.sendHeadMotor(2, 1500, 50)
        send.sendHeadMotor(1, 2048, 50)
        send.sendSensorReset()
        time.sleep(0.01)

    def calc_speed_theta_by_slope(self):#判斷斜率
        slope = self.seek_line.calculate_slope()

        if abs(slope) >= 15:
            theta = ORIGIN_THETA
            speed_x = 3500
        elif 7 <= abs(slope) < 15:
            theta = 0
            speed_x = 3400
        elif 4 <= abs(slope) < 7:
            theta = 1 if slope > 0 else -1
            speed_x = 3300
        elif 1.5 <= abs(slope) < 4:
            theta = 2 if slope > 0 else -2
            speed_x = 3200
        else:
            theta = 5 if slope > 0 else -3
            speed_x = 3100
        if abs(theta) <= 2:
            if self.seek_line.center.x < 140:
                theta = 3
                speed_x = 3000
            elif self.seek_line.center.x > 180:
                theta = -3
                speed_x = 3000

        rospy.logdebug(f'speed = {speed_x}')
        rospy.logdebug(f'theta = {theta}')
        return speed_x, theta

    def line_to_arrow(self):
        slope = self.seek_line.calculate_slope()
        delta_theta = 0
        speed_x = 0
        speed_y = 0
        if abs(slope) > 15:
            delta_theta = 0
            self.arrow_cnt_times += 1
        elif 15 > abs(slope) > 7:
            delta_theta = -2 if slope < 0 else 1
        elif 7 > abs(slope):
            delta_theta = -4 if slope < 0 else 3
            speed_x = -500
            speed_y = 800 if slope < 0 else 500
        theta = delta_theta + ORIGIN_THETA

        if self.arrow_cnt_times >= 5:
            self.yaw_temp = send.imu_value_Yaw
            self.arrow_cnt_times = 0
            self.status = 'Follow_Arrow'

        return speed_x, speed_y, theta

    def yaw_calculate(self):
        if -240 > self.yaw - self.yaw_temp:
                self.yaw = self.yaw + 360
        elif self.yaw - self.yaw_temp > 240:
                self.yaw = self.yaw - 360

    def arrow_turn(self):
        self.yaw = send.imu_value_Yaw
        self.yaw_calculate()
        if self.arrow.type == 'right':
            rospy.logdebug(f'箭頭：右轉')
            send.sendContinuousValue(2500, 0, 0, -4 + ORIGIN_THETA, 0)
        else:
            rospy.logdebug(f'箭頭：左轉')
            send.sendContinuousValue(2300, 0, 0, 5 + ORIGIN_THETA, 0)


        if  abs(self.yaw - self.yaw_temp) > 86:#成功轉90度
            rospy.logdebug(f'箭頭轉彎結束')
            self.yaw_temp = self.yaw
            self.turn_now_flag = False


    def imu_go(self):#直走
        theta = 0 + ORIGIN_THETA
        rospy.logdebug(f'直走')
        self.yaw = send.imu_value_Yaw
        speed_x = 2000
        if 0 < self.arrow.center.x <= 140:
            theta = 5
        elif self.arrow.center.x >= 180:
            theta = -5
        else:
            self.yaw_calculate()
            if  self.yaw - self.yaw_temp > 6:
                theta = -3 + ORIGIN_THETA
                rospy.logdebug(f'修正：右轉')
            elif self.yaw - self.yaw_temp < 0:
                theta = 3 + ORIGIN_THETA
                rospy.logdebug(f'修正：左轉')
        send.sendContinuousValue(speed_x, 0, 0, theta, 0)

    
    def main(self):
        if send.is_start:
            if self.status == 'First':
                self.initial()
                send.sendBodyAuto(0, 0, 0, 0, 1, 0)
                self.status = 'Follow_Line'  if send.DIOValue == 24 else 'Follow_Arrow'
            elif self.status == 'Follow_Line' and send.DIOValue == 24:
                self.seek_line.update()
                speed_x, theta = self.calc_speed_theta_by_slope()
                detected = self.arrow.detect()
                if detected and self.seek_line.center.y > 180:
                    self.status = 'Align_First_Arrow'
                    self.turn_now_flag = False if self.arrow.type == 'stright' else True
                send.sendContinuousValue(speed_x, 0, 0, theta, 0)
            elif self.status == 'Align_First_Arrow':
                self.seek_line.update()
                speed_x, speed_y, theta = self.line_to_arrow()
                send.sendContinuousValue(speed_x, speed_y, 0, theta, 0)
            elif self.status == 'Follow_Arrow':
                if self.turn_now_flag:
                    self.arrow_turn()
                else:
                    detected = self.arrow.detect()
                    if detected:
                        if self.arrow.close_arrow:
                            self.arrow_cnt_times += 1
                        if self.arrow_cnt_times >= 4:
                            if self.arrow.type != 'stright':
                                self.turn_now_flag = True
                            self.arrow_cnt_times = 0

                    self.imu_go()
        else:
            if self.status != 'First':
                self.status = 'First'
                send.sendBodyAuto(0, 0, 0, 0, 1, 0)

class SeekLine:
    def __init__(self):
        self.upper_center = Coordinate(0, 0)
        self.lower_center = Coordinate(0, 0)

    def update(self):
        self.upper_center.x, self.upper_center.y = 0, 0
        self.lower_center.x, self.lower_center.y = 0, 0
        #影像輸出為一維陣列，8bits
        img_data = np.frombuffer(send.Label_Model, dtype = np.uint8)
        img_data = img_data.reshape(240, 320)
        img_data_copy = np.copy(img_data)
        img_data_copy[0, :] = 0
        y_coord, x_coord = np.where(img_data_copy != 0)
        if len(x_coord) != 0:
            middle_y = (np.max(y_coord) + np.min(y_coord)) / 2
            upper_filter, lower_filter = (y_coord <= middle_y), (y_coord > middle_y)
            self.upper_center.x = np.mean(x_coord[upper_filter])
            self.upper_center.y = np.mean(y_coord[upper_filter])
            self.lower_center.x = np.mean(x_coord[lower_filter])
            self.lower_center.y = np.mean(y_coord[lower_filter])
            rospy.logdebug(f'{self.upper_center.x}')
            rospy.logdebug(f'{self.upper_center.y}')
            rospy.logdebug(f'{self.lower_center.x}')
            rospy.logdebug(f'{self.lower_center.y}')
            send.drawImageFunction(2, 0, int(self.lower_center.x), int(self.upper_center.x), int(self.lower_center.y), int(self.upper_center.y), 0, 0, 0)
        self.center = (self.upper_center + self.lower_center) // 2

    def calculate_slope(self):
        delta = self.upper_center - self.lower_center
        return delta.y / delta.x if delta.x != 0 else float("inf")
    
class Arrow:
    def __init__(self):
        self.center =  Coordinate(0, 0)
        self.arrows = deque(['none', 'none', 'none', 'none', 'none'], maxlen = 5)
        self.type = 'none'
        self.close_arrow = False

    def detect(self):
        self.center.x, self.center.y = 0, 0
        self.arrows.append(send.yolo_Label)
        detected_species = len(set(self.arrows))
        self.type = self.arrows[0]
        if detected_species == 1 and self.arrows[0] != 'None':
            self.center.x, self.center.y = send.yolo_X, send.yolo_Y
            self.close_arrow = True if self.center.y >= 170 else False
            send.drawImageFunction(1, 1, send.yolo_XMin, send.yolo_XMax, send.yolo_YMin, send.yolo_YMax, 255, 0, 0)
            return True
        return False

if __name__ == '__main__':
    try:
        mar = Mar()
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            mar.main()
            r.sleep()
    except rospy.ROSInterruptException:
        pass