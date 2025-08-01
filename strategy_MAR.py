#!/usr/bin/env python3
#coding=utf-8
from collections import Counter, deque
import rospy
import numpy as np
import sys 
# sys.path.append('/home/iclab/Desktop/kid_hurocup/src/strategy')
from Python_API import Sendmessage
import time
# import cv2
import math

ORIGIN_THETA = 0
ORIGIN_SPEED = 4000
send = Sendmessage()

# 2025.8.1

class Coordinate: # 計算
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
        self.arrow_to_line_flag = False                           
        self.line_status = 'online'
        send.sendHeadMotor(2, 1400, 50)
        send.sendHeadMotor(1, 2048, 50)
        send.sendSensorReset(1, 1, 1)

    def theta_value(self): # 判斷斜率
        slope = self.seek_line.calculate_slope()
        middle_point = (self.seek_line.upper_center + self.seek_line.lower_center) // 2
        if middle_point.y > 215:
            if self.seek_line.lower_center.x > 220:
                self.theta = -6 + ORIGIN_THETA
                self.speed_x = ORIGIN_SPEED
            elif self.seek_line.lower_center.x < 80:
                self.theta = 6 + ORIGIN_THETA
                self.speed_x = ORIGIN_SPEED
            else:   
                self.line_status = 'arrow' # 進入第二階段的指標，線在機器人螢幕的正下方
                rospy.loginfo(f'line in image = {self.line_status}')
        else:
            if abs(slope) >= 14:
                self.theta = ORIGIN_THETA
                self.speed_x = ORIGIN_SPEED + 800
            elif 7 <= abs(slope) < 14:
                self.theta = 0
                self.speed_x = ORIGIN_SPEED + 600
            elif 4 <= abs(slope) < 7:
                self.theta = 2 if slope > 0 else -2
                self.speed_x = ORIGIN_SPEED + 300
            elif 1.5 <= abs(slope) < 4:
                self.theta = 4 if slope > 0 else -4
                self.speed_x = ORIGIN_SPEED - 0
            else:
                self.theta = 6 if slope > 0 else -6
                self.speed_x = ORIGIN_SPEED - 300
            if middle_point.x == 0 and middle_point.y == 0:
                self.theta = ORIGIN_THETA
                self.speed_x = ORIGIN_SPEED
            elif self.seek_line.lower_center.x < 130 and abs(slope) > 2:
                self.theta = 4 + ORIGIN_THETA
                self.speed_x = ORIGIN_SPEED
            elif self.seek_line.lower_center.x > 170 and abs(slope) > 2:
                self.theta = -4 + ORIGIN_THETA
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
            self.theta = 1 if slope > 0 else -1 
        elif abs(slope) < 7:
            self.theta = 3 if slope > 0 else -3
            self.speed_x = 300
            self.speed_y = 0 if slope > 0 else 0
        if self.arrow_cnt_times >= 7:
            send.sendSensorReset(0, 0, 1)
            self.arrow_cnt_times = 0
            self.speed_y = 0

            send.sendHeadMotor(2, 1650, 50)
            self.status = 'Arrow_Part'

    def arrow_yolo(self): # 檢測箭頭透過yolo_Label判斷出箭頭方向
        self.arrow_center.x, self.arrow_center.y = 0, 0
        self.arrow_temp.append(send.yolo_Label)
        rospy.loginfo(f" arrow list: {self.arrow_temp}")
        arrow_cnt_temp = len(set(self.arrow_temp)) # 用set去除重複，保證5項全部都一樣
        if arrow_cnt_temp == 1 and self.arrow_temp[0] != 'None':
            self.can_turn_flag = False
            if self.arrow_temp[0] == 'left' or self.arrow_temp[0] == 'right':
                rospy.logwarn("!!!!!!! set can_tuen_flag")
                self.can_turn_flag = True
            self.arrow_center.y = send.yolo_Y
            self.arrow_center.x = send.yolo_X
            send.drawImageFunction(1, 1, send.yolo_XMin, send.yolo_XMax, send.yolo_YMin, send.yolo_YMax, 255, 0, 0)
            return True
        return False


    def arrow_turn(self): # 遇到箭頭轉彎
        self.yaw = send.imu_value_Yaw
        print("yaw = ", send.imu_value_Yaw)
        # self.yaw_calculate()
        if self.arrow_temp[0] == 'right':
            rospy.logdebug(f'箭頭：右轉')
            send.sendContinuousValue(2000, 0, 0, -6 + ORIGIN_THETA, 0)
        elif self.arrow_temp[0] == 'left':
            rospy.logdebug(f'箭頭：左轉')
            send.sendContinuousValue(2000, 0, 0, 6 + ORIGIN_THETA, 0)
        if  abs(self.yaw) > 85: # 成功轉90度
            send.sendSensorReset(0, 0, 1)    # imu Reset
            rospy.logdebug(f'箭頭轉彎結束')
            # self.yaw_temp = self.yaw
            self.turn_now_flag = False # 轉彎完將旗標關閉
            self.can_turn_flag = False

    def imu_go(self):# 直走   yaw 可以知道機器人面向 
        self.theta = 0 + ORIGIN_THETA
        rospy.logdebug(f'直走')
        self.yaw = send.imu_value_Yaw
        rospy.logdebug({self.yaw})
        slope = self.seek_line.calculate_slope()
        middle_point = (self.seek_line.upper_center + self.seek_line.lower_center) // 2

        self.speed_x = 3000
        
        if middle_point.y > 100 and middle_point.y < 140:
            if abs(slope) > 5:
                self.arrow_to_line_flag += 1
                rospy.loginfo(f" arrow_to_line_flag : {self.arrow_to_line_flag}")
        if 0 < self.arrow_center.x <= 140: # 在畫面左邊，向左修正
            self.theta = 5
            send.sendContinuousValue(self.speed_x, 0, 0, self.theta + ORIGIN_THETA, 0)
        elif self.arrow_center.x >= 180: # 在畫面右邊，向右修正
            self.theta = -5
            send.sendContinuousValue(self.speed_x, 0, 0, self.theta + ORIGIN_THETA, 0)
        else: # 還沒接近箭頭，以 imu 做修正
            if  self.yaw  > 3: # 機器人偏向偏向左邊，向右修正
                self.theta = -3 + ORIGIN_THETA
                rospy.logdebug(f'修正：右轉')
            elif self.yaw  < -6: # 機器人偏向右邊，向左修正
                self.theta = 3 + ORIGIN_THETA
                rospy.logdebug(f'修正：左轉')
        send.sendContinuousValue(self.speed_x, 0, 0, self.theta, 0)
    

    ###############  main  ###############
    def main(self):
        if send.is_start: # 確認機器人啟動
            # while True:
            #     print(send.imu_value_Yaw)
            print(self.status)
            if self.status == 'First': #初始化
                self.initial()
                time.sleep(1)
                send.sendBodyAuto(0, 0, 0, 0, 1, 0) # mode = 1為continue步態
                self.status = 'line'  if send.DIOValue == 48 else 'Arrow_Part'
                if self.status == 'Arrow_Part':
                    send.sendHeadMotor(2, 1500, 50) # 馬達編號1為水平，2為垂直調整   位置為馬達目標刻度，2048為正朝前方

            elif self.status == 'line':            
                if send.data_check == True: #畫面所有顏色都更新完畢
                    self.seek_line.update()
                    self.theta_value()
                arrow = self.arrow_yolo()

                if arrow and self.line_status == 'arrow':
                    self.status = 'First_arrow'
                    rospy.logwarn(f'status = {self.status}')
                send.sendContinuousValue(self.speed_x, 0, 0, self.theta, 0)

            elif self.status == 'First_arrow':
                if send.data_check == True:
                    self.seek_line.update()
                    self.line_to_arrow()
                send.sendContinuousValue(self.speed_x , self.speed_y , 0, self.theta, 0)

            elif self.status == 'Arrow_Part':
                if self.arrow_to_line_flag > 10 :
                    self.status = 'line'
                    self.line_status = 'online'
                    self.arrow_to_line_flag = 0

                elif self.turn_now_flag:
                    self.arrow_turn()
                
                else:
                    self.arrow_yolo()
                    if self.can_turn_flag:
                        rospy.loginfo('can turn !!!')
                        if self.arrow_center.y >= 185:
                            self.arrow_cnt_times += 1
                        if self.arrow_cnt_times >= 10:
                            self.turn_now_flag = True
                            self.arrow_cnt_times = 0
                    self.seek_line.update()
                    self.imu_go()

        else:
            if self.status != 'First':
                # self.initial()
                self.status = 'First'
                send.sendBodyAuto(0, 0, 0, 0, 1, 0) #關閉



class Seek_line:
    def __init__(self):
        self.upper_center = Coordinate(0, 0)
        self.lower_center = Coordinate(0, 0)
    #功能:對我們的陣列做補0
    def cvt_list2d2numpy(self, list2d):   #將一個2D的Python列表（list2d）轉換為NumPy的2D數組
        max_len = max([len(sub_lst) for sub_lst in list2d])
        np_array = np.vstack([np.pad(np.array(lst), (0, (max_len - len(lst)))) for lst in list2d])
        return np_array

    def update(self):
        img_size = self.cvt_list2d2numpy(send.color_mask_subject_size)
        img_xmin = self.cvt_list2d2numpy(send.color_mask_subject_XMin)
        img_xmax = self.cvt_list2d2numpy(send.color_mask_subject_XMax)
        img_ymin = self.cvt_list2d2numpy(send.color_mask_subject_YMin)
        img_ymax = self.cvt_list2d2numpy(send.color_mask_subject_YMax)
        # img_size = np.array(send.color_mask_subject_size)
        # img_xmin = np.array(send.color_mask_subject_XMin)
        # img_xmax = np.array(send.color_mask_subject_XMax)
        # img_ymin = np.array(send.color_mask_subject_YMin)
        # img_ymax = np.array(send.color_mask_subject_YMax)
        filter_img_size = img_size > 380   #濾雜訊
        has_object = filter_img_size.any() #filter_img_size 輸出是布林值
        send.data_check = False
        if not has_object:   #確認沒有看到物體
            rospy.logdebug(f'no object')
            self.upper_center.x, self.upper_center.y = 0, 0
            self.lower_center.x, self.lower_center.y = 0, 0
            return
        
        #print(img_ymin[filter_img_size]) 
        img_xmin_new = int(img_xmin[filter_img_size].min()) 
        img_xmax_new = int(img_xmax[filter_img_size].max()) 
        img_ymin_new = int(img_ymin[filter_img_size].min())
        img_ymax_new = int(img_ymax[filter_img_size].max())
        send.drawImageFunction(7, 1, img_xmin_new, img_xmax_new, img_ymin_new, img_ymax_new, 255, 0, 255)
        #影像輸出為一維陣列，8bits(0~255)
        img_data = np.frombuffer(send.Label_Model, dtype = np.uint8)
        img_data = img_data.reshape(240, 320)
        img_data = img_data[img_ymin_new : img_ymax_new, img_xmin_new : img_xmax_new] #
        
        y_coord, x_coord = np.where(img_data != 0) #抓取不等於 0 的

        if len(x_coord) == 0:
            self.upper_center.x, self.upper_center.y = 0, 0
            self.lower_center.x, self.lower_center.y = 0, 0
            return
        
        middle_y = (np.max(y_coord) + np.min(y_coord)) // 2 #中心點
        upper_filter = y_coord <= middle_y
        upper_x, upper_y = x_coord[upper_filter], y_coord[upper_filter]
        self.upper_center.x = np.mean(upper_x) + img_xmin_new #因為抓色模區間會重新定義原點 + img_xmin_new 可以知道其真實位置
        self.upper_center.y = np.mean(upper_y) + img_ymin_new
        # upper_xmin = upper_x.min() + img_xmin_new
        # upper_xmax = upper_x.max() + img_xmin_new
        # upper_ymin = upper_y.min() + img_ymin_new
        # upper_ymax = upper_y.max() + img_ymin_new
        # send.drawImageFunction(5, 1, upper_xmin, upper_xmax, upper_ymin, upper_ymax, 255, 0, 0)
        lower_filter = y_coord > middle_y
        lower_x, lower_y = x_coord[lower_filter], y_coord[lower_filter]
        self.lower_center.x = np.mean(lower_x) + img_xmin_new
        self.lower_center.y = np.mean(lower_y) + img_ymin_new
        # lower_xmin = lower_x.min() + img_xmin_new
        # lower_xmax = lower_x.max() + img_xmin_new
        # lower_ymin = lower_y.min() + img_ymin_new
        # lower_ymax = lower_y.max() + img_ymin_new
        # send.drawImageFunction(6, 1, lower_xmin, lower_xmax, lower_ymin, lower_ymax, 0, 255, 0)
        send.drawImageFunction(2, 0, int(self.upper_center.x), int(self.lower_center.x), int(self.upper_center.y), int(self.lower_center.y), 0, 0, 0)
        #send.drawImageFunction(編號, 型態, xmin, xmax, ymin, ymax, r, g, b)
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
    except rospy.ROSInterruptException: #捕獲 Ctrl+C 的中斷信號
        pass