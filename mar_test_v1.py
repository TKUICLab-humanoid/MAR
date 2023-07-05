#!/usr/bin/env python
#coding=utf-8
from collections import Counter, deque
import rospy
import numpy as np
from Python_APIa import Sendmessage
import time
import math

ORIGIN_THETA = 0
ORIGIN_LINE_SPEED = 3000
ORIGIN_ARROW_SPEED = 3000

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
        send.use_new_color_mask = True
        self.initial()
        self.speed_x = 0
        self.speed_y = 0
        self.theta = 0
        self.pre_theta = 0
        self.seek_line = Seek_line()
        self.arrow_center = Coordinate(0, 0)
        self.arrow_temp = deque(['None', 'None', 'None', 'None', 'None'], maxlen = 5)
    
    def initial(self):
        self.status = 'First'
        self.can_turn_flag = False                                         
        self.arrow_flag = False
        self.turn_now_flag = False
        self.arrow_cnt_times = 0                        
        self.line_status = 'online'
        send.sendHeadMotor(2, 1400, 50)
        send.sendHeadMotor(1, 2048, 50)
        send.sendSensorReset(1, 1, 1)

    def theta_value(self):#判斷斜率
        slope = self.seek_line.calculate_slope()
        middle_point = (self.seek_line.upper_center + self.seek_line.lower_center) // 2
        rospy.loginfo(f'slope = {slope}')
        
        if middle_point.y > 180:
            if self.seek_line.lower_center.x > 220:
                self.theta = -5 + ORIGIN_THETA
                self.speed_x = ORIGIN_LINE_SPEED
            elif self.seek_line.lower_center.x < 80:
                self.theta = 5 + ORIGIN_THETA
                self.speed_x = ORIGIN_LINE_SPEED
            else:
                self.line_status = 'arrow'#進入第二階段的指標，線在機器人螢幕的正下方
                rospy.loginfo(f'line in image = {self.line_status}')
        else:           
            if abs(slope) >= 15:
                self.theta = ORIGIN_THETA
                self.speed_x = ORIGIN_LINE_SPEED + 500
            elif 7 <= abs(slope) < 15:
                self.theta = 0
                self.speed_x = ORIGIN_LINE_SPEED + 400
            elif 4 <= abs(slope) < 7:
                self.theta = 1 if slope > 0 else -1
                self.speed_x = ORIGIN_LINE_SPEED + 300
            elif 1.5 <= abs(slope) < 4:
                self.theta = 2 if slope > 0 else -2
                self.speed_x = ORIGIN_LINE_SPEED + 300
            elif 1 <= abs(slope) < 1.5:
                self.theta = 3 if slope > 0 else -3
                self.speed_x = ORIGIN_LINE_SPEED + 200
            else:
                self.theta = 4 if slope > 0 else -4
                self.speed_x = ORIGIN_LINE_SPEED + 100

            if middle_point.x == 0 and middle_point.y == 0:
                self.theta = ORIGIN_THETA
                self.speed_x = ORIGIN_LINE_SPEED
            elif self.seek_line.lower_center.x < 140 and abs(slope) > 5:
                self.theta = 5 + ORIGIN_THETA
                self.speed_x = ORIGIN_LINE_SPEED
            elif self.seek_line.lower_center.x > 180 and abs(slope) > 5:
                self.theta = -5 + ORIGIN_THETA
                self.speed_x = ORIGIN_LINE_SPEED

            self.line_status = 'online'
            
        # rospy.logdebug(f'speed = {self.speed_x}')
        # rospy.logdebug(f'theta = {self.theta}')

    def line_to_arrow(self):
        slope = self.seek_line.calculate_slope()

        if abs(slope) > 15:
            self.theta = ORIGIN_THETA
            self.speed_x = 500
            self.arrow_cnt_times += 1
        elif 7 < abs(slope) <= 15:
            self.theta = 1 + ORIGIN_THETA if slope > 0 else -1 + ORIGIN_THETA
        elif abs(slope) <= 7:
            self.theta = 3 + ORIGIN_THETA if slope > 0 else -3 + ORIGIN_THETA
            self.speed_x = 0
            self.speed_y = 0

        if self.arrow_cnt_times >= 5:
            send.sendSensorReset(0, 0, 1)
            self.arrow_cnt_times = 0
            # send.sendHeadMotor(2, 1400, 50)
            self.turn_now_flag = True if self.can_turn_flag else False
            self.speed_y = 0
            self.status = 'Arrow_Part'

    def arrow_yolo(self):
        self.arrow_center.x, self.arrow_center.y = 0, 0
        self.arrow_temp.append(send.yolo_Label)
        rospy.logdebug(f" arrow list: {self.arrow_temp}")
        arrow_cnt_temp = len(set(self.arrow_temp))

        if arrow_cnt_temp == 1 and self.arrow_temp[0] != 'None':
            if self.arrow_temp[0] == 'left' or self.arrow_temp[0] == 'right':
                rospy.logwarn("!!!!!!! set can_tuen_flag")
                self.can_turn_flag = True
            else:
                self.can_turn_flag = False

            self.arrow_center.y = send.yolo_Y
            self.arrow_center.x = send.yolo_X
            send.drawImageFunction(1, 1, send.yolo_XMin, send.yolo_XMax, send.yolo_YMin, send.yolo_YMax, 255, 0, 0)
            return True
        send.drawImageFunction(1, 1, 0, 0, 0, 0, 255, 0, 0)

    def arrow_turn(self):
        self.yaw = send.imu_value_Yaw

        if  abs(self.yaw) > 83:#成功轉90度
            rospy.loginfo(f'箭頭轉彎結束')
            send.sendSensorReset(0, 0, 1)
            self.turn_now_flag = False
            self.can_turn_flag = False
            self.speed_x = ORIGIN_ARROW_SPEED
            self.theta = ORIGIN_THETA
            return

        if self.arrow_temp[0] == 'right':
            rospy.logdebug(f'箭頭：右轉')
            self.speed_x = ORIGIN_ARROW_SPEED
            self.theta = -5 + ORIGIN_THETA
        elif self.arrow_temp[0] == 'left':
            rospy.logdebug(f'箭頭：左轉')
            self.speed_x = ORIGIN_ARROW_SPEED
            self.theta = 5 + ORIGIN_THETA

        # send.sendContinuousValue(self.speed_x, 0, 0, self.theta, 0)


    def imu_go(self):#直走
        self.theta = ORIGIN_THETA
        rospy.logdebug(f'箭頭：直走')
        self.yaw = send.imu_value_Yaw
        self.speed_x = ORIGIN_ARROW_SPEED

        if 0 < self.arrow_center.x <= 140:
            self.theta = 5 + ORIGIN_THETA
        elif self.arrow_center.x >= 180:
            self.theta = -5 + ORIGIN_THETA
        else:
            if  self.yaw > 8:
                self.theta = -3 + ORIGIN_THETA
                rospy.logdebug(f'箭頭修正：右轉')
            elif self.yaw < -8:
                self.theta = 3 + ORIGIN_THETA
                rospy.logdebug(f'箭頭修正：左轉')
        # send.sendContinuousValue(self.speed_x, 0, 0, self.theta, 0)
    
    def main(self):
        if send.is_start:
            if self.status == 'First':
                self.initial()
                time.sleep(0.01)
                send.sendBodyAuto(0, 0, 0, 0, 1, 0)
                self.status = 'line'  if send.DIOValue == 24 else 'Arrow_Part'
            elif self.status == 'line':
                if send.data_check:
                    self.seek_line.update()
                    self.theta_value()

                arrow = self.arrow_yolo()

                if arrow and self.line_status == 'arrow':
                    self.status = 'First_arrow'
                    rospy.logwarn(f'First_arrow')

            elif self.status == 'First_arrow':
                if send.data_check:
                    self.seek_line.update()
                    self.line_to_arrow()
                    
            elif self.status == 'Arrow_Part':
                if self.turn_now_flag:
                    self.arrow_turn()

                else:
                    self.arrow_yolo()

                    if self.can_turn_flag:
                        rospy.loginfo('can turn !!!')

                        if self.arrow_center.y >= 160:
                            self.arrow_cnt_times += 1

                        if self.arrow_cnt_times >= 4:
                            self.turn_now_flag = True 
                            self.arrow_cnt_times = 0
                    
                    if len(set(self.arrow_temp)) == 1 and self.arrow_temp[0] == 'stright':
                        self.arrow_cnt_times = 0

                    self.imu_go()

            if self.theta * self.pre_theta <= -4:
                self.theta = 0

            send.sendContinuousValue(self.speed_x, self.speed_y, 0, self.theta, 0)
            self.pre_theta = self.theta

        else:
            if self.status != 'First':
                send.sendBodyAuto(0, 0, 0, 0, 1, 0)
                send.first_in_color_mask = True
            
            self.status = 'First'
            self.initial()

class Seek_line:
    def __init__(self):
        self.upper_center = Coordinate(0, 0)
        self.lower_center = Coordinate(0, 0)

    def cvt_list2d2numpy(self, list2d):
        max_len = max([len(sub_lst) for sub_lst in list2d])
        np_array = np.vstack([np.pad(np.array(lst), (0, (max_len - len(lst)))) for lst in list2d])
        return np_array

    def update(self):
        img_size = self.cvt_list2d2numpy(send.color_mask_subject_size)
        img_xmin = self.cvt_list2d2numpy(send.color_mask_subject_XMin)
        img_xmax = self.cvt_list2d2numpy(send.color_mask_subject_XMax)
        img_ymin = self.cvt_list2d2numpy(send.color_mask_subject_YMin)
        img_ymax = self.cvt_list2d2numpy(send.color_mask_subject_YMax)

        filter_img_size = img_size > 380
        has_object = filter_img_size.any()
        send.data_check = False
        send.first_in_color_mask = False
        
        if not has_object:
            rospy.logdebug(f'no object')
            self.upper_center.x, self.upper_center.y = 0, 0
            self.lower_center.x, self.lower_center.y = 0, 0
            send.drawImageFunction(2, 0, 0, 0, 0, 0, 255, 0, 255)
            send.drawImageFunction(3, 0, 0, 0, 0, 0, 255, 0, 255)
            return
        
        img_xmin_new = int(img_xmin[filter_img_size].min())
        img_xmax_new = int(img_xmax[filter_img_size].max())
        img_ymin_new = int(img_ymin[filter_img_size].min())
        img_ymax_new = int(img_ymax[filter_img_size].max())
        if img_ymin_new < 50:
            img_ymin_new = 50

        send.drawImageFunction(2, 1, img_xmin_new, img_xmax_new, img_ymin_new, img_ymax_new, 255, 0, 255)
        #影像輸出為一維陣列，8bits
        img_data = np.frombuffer(send.Label_Model, dtype = np.uint8)
        img_data = img_data.reshape(240, 320)
        img_data = img_data[img_ymin_new : img_ymax_new, img_xmin_new : img_xmax_new]
        
        y_coord, x_coord = np.where(img_data != 0)

        if len(x_coord) == 0:
            self.upper_center.x, self.upper_center.y = 0, 0
            self.lower_center.x, self.lower_center.y = 0, 0
            return
        
        middle_y = (np.max(y_coord) + np.min(y_coord)) // 2
        upper_filter = y_coord <= middle_y
        upper_x, upper_y = x_coord[upper_filter], y_coord[upper_filter]
        self.upper_center.x = np.mean(upper_x) + img_xmin_new
        self.upper_center.y = np.mean(upper_y) + img_ymin_new
        
        lower_filter = y_coord > middle_y
        lower_x, lower_y = x_coord[lower_filter], y_coord[lower_filter]
        self.lower_center.x = np.mean(lower_x) + img_xmin_new
        self.lower_center.y = np.mean(lower_y) + img_ymin_new
        send.drawImageFunction(3, 0, int(self.upper_center.x), int(self.lower_center.x), int(self.upper_center.y), int(self.lower_center.y), 0, 0, 0)

    def calculate_slope(self):
        delta = self.upper_center - self.lower_center
        if delta.x == 0:
            return float("inf")
        return delta.y / delta.x

if __name__ == '__main__':
    try:
        mar = Mar()
        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            mar.main()
            r.sleep()
    except rospy.ROSInterruptException:
        pass