#!/usr/bin/env python
#coding=utf-8
import rospy
import numpy as np
from rospy import Publisher
from tku_msgs.msg import Interface,HeadPackage,SandHandSpeed,DrawImage,SingleMotorData,\
SensorSet,ObjectList,LabelModelObjectList,RobotPos,SetGoalPoint,SoccerDataList,SensorPackage,YoloData, parameter,Parameter_message
from std_msgs.msg import Int16,Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import time

class Sendmessage:     
    def __init__(self):
        self.walkingGait_pub = rospy.Publisher('SendBodyAuto_Topic', Interface, queue_size=100)
        self.head_motor_pub = rospy.Publisher("/package/HeadMotor",HeadPackage, queue_size=10)
        self.sector_pub = rospy.Publisher("/package/Sector",Int16, queue_size=100)
        self.hand_speed_pub = rospy.Publisher("/package/motorspeed",SandHandSpeed, queue_size=100)
        self.draw_image_pub = rospy.Publisher("/strategy/drawimage",DrawImage, queue_size=100)
        self.continuous_value_pub = rospy.Publisher("/ChangeContinuousValue_Topic",Interface, queue_size=100)
        self.single_motor_data_pub = rospy.Publisher("/package/SingleMotorData",SingleMotorData, queue_size=100)
        self.sensor_pub = rospy.Publisher("sensorset",SensorSet, queue_size=100)
        self.paradata_pub = rospy.Publisher("/package/parameterdata",Parameter_message, queue_size=100)
        self.parameter_pub = rospy.Publisher("/web/parameter_Topic",parameter, queue_size=100)
        self.continuous_back = rospy.Publisher("/walkinggait/Continuousback",Bool, queue_size=100)
        
        self.Web = False
        self.Label_Model = [0 for i in range(320*240)]
        # self.Label_Model = np.zeros([320*240])
        self.bridge = CvBridge()
        self.color_mask_subject_cnts = [0 for i in range(8)]
        self.color_mask_subject_X = [0 for i in range(8)]
        self.color_mask_subject_Y = [0 for i in range(8)]
        self.color_mask_subject_XMin = [0 for i in range(8)]
        self.color_mask_subject_XMax = [0 for i in range(8)]
        self.color_mask_subject_YMax = [0 for i in range(8)]
        self.color_mask_subject_YMin = [0 for i in range(8)]
        self.color_mask_subject_Width = [0 for i in range(8)]
        self.color_mask_subject_Height = [0 for i in range(8)]
        self.color_mask_subject_size = [0 for i in range(8)]
        self.imu_value_Roll = 0
        self.imu_value_Yaw = 0
        self.imu_value_Pitch = 0
        self.DIOValue = 0x00
        self.is_start = False
        self.data_check = False
        self.time = 0
        self.yolo_IoU = 0
        self.yolo_Label = ''
        self.yolo_XMin = 0
        self.yolo_YMin = 0
        self.yolo_XMax = 0
        self.yolo_YMax = 0
        self.yolo_Y = (self.yolo_YMin+self.yolo_YMax)/2
        self.yolo_X = (self.yolo_XMin+self.yolo_XMax)/2
        object_list_sub = rospy.Subscriber("/Object/List",ObjectList, self.getObject)
        label_model_sub = rospy.Subscriber("/LabelModel/List",LabelModelObjectList, self.getLabelModel)
        #compress_image_sub = rospy.Subscriber("compress_image",Image, self.catchImage)
        #image_raw_sub = rospy.Subscriber("colormodel_image",Image, self.RawImage)
        #origin_image_sub = rospy.Subscriber("orign_image",Image, self.OriginImage)
        start_sub = rospy.Subscriber("/web/start",Bool, self.startFunction)
        DIO_ack_sub = rospy.Subscriber("/package/FPGAack",Int16, self.DIOackFunction)
        sensor_sub = rospy.Subscriber("/package/sensorpackage",SensorPackage, self.sensorPackageFunction)


        yolo_sub = rospy.Subscriber('highest/msg',YoloData, self.YoloFunction)

    def sendBodyAuto(self,x,y,z,theta,mode,sensor):	#步態啟動
        walkdata = Interface()
        walkdata.x = x
        walkdata.y = y
        walkdata.z = z
        walkdata.theta = theta
        walkdata.walking_mode = mode
        walkdata.sensor_mode = sensor
        self.walkingGait_pub.publish(walkdata)

    def sendHeadMotor(self,ID,Position,Speed):	#頭部馬達
        HeadData = HeadPackage()
        HeadData.ID = ID
        HeadData.Position = Position
        HeadData.Speed = Speed
        self.head_motor_pub.publish(HeadData)

    def sendBodySector(self,Sector):	#磁區
        SectorData = Int16()
        SectorData.data = int(Sector)
        self.sector_pub.publish(SectorData)

    def sendHandSpeed(self,Sector,Speed):   #馬達速度放進磁區for投籃
        HandSpeedData = SandHandSpeed()
        HandSpeedData.sector = Sector
        HandSpeedData.speed = Speed
        self.hand_speed_pub.publish(HandSpeedData)

    def drawImageFunction(self,cnt,mode,xmin,xmax,ymin,ymax,r,g,b):
        ImageData = DrawImage()
        ImageData.cnt = cnt
        ImageData.XMax = xmax
        ImageData.XMin = xmin
        ImageData.YMax = ymax
        ImageData.YMin = ymin
        ImageData.rValue = r
        ImageData.gValue = g
        ImageData.bValue = b
        ImageData.Mode = mode
        self.draw_image_pub.publish(ImageData)

    def sendContinuousValue(self,x,y,z,theta,sensor):
        walkdata = Interface()
        walkdata.x = x
        walkdata.y = y
        walkdata.z = z
        walkdata.theta = theta
        walkdata.sensor_mode = sensor
        self.continuous_value_pub.publish(walkdata)

    def sendSingleMotor(self,ID,Position,Speed):
        MotorData = SingleMotorData()
        MotorData.ID = ID
        MotorData.Position = Position
        MotorData.Speed = Speed
        self.single_motor_data_pub.publish(MotorData)

    def sendWalkParameter(self, function, *,\
                                walk_mode = 1,\
                                com_y_shift = 0,\
                                y_swing = 6.5,\
                                period_t = 450,\
                                t_dsp = 0,\
                                base_default_z = 3,\
                                right_z_shift = 0,\
                                base_lift_z = 4,\
                                com_height = 62,\
                                stand_height = 47.3,\
                                back_flag = False):
        
        #function       指定功能 save為儲存參數 send為送參數給FPGA ※send可以直接做動作,但如果不save次做動作會回歸原本參數
        #walk_mode      指定步態 1為走路 2為上板 3為下板 
        #com_y_shift    控制走路起步的質心位置,上下板為控制第2步的質心位置
        #y_swing        質心晃動程度
        #period_t       步態週期
        #t_dsp          雙支撐時間(比例)
        #base_default_z 抬腳高
        #right_z_shift  控制上下板第2步的抬腳高補償
        #base_lift_z    板子高度
        #com_height     質心高度
        #stand_height   機器人初始站姿高度(由踝關節(15、21)到髖關節(11、17)馬達的距離)
        #back_flag      後退旗標
        walkparameter = parameter()
        parasend2FPGA = Parameter_message()

        if function == 'save':
            if walk_mode == 1:
                self.continuous_back.publish(back_flag)
    
            walkparameter.mode = walk_mode
            walkparameter.X_Swing_Range = com_y_shift
            walkparameter.Y_Swing_Range = y_swing if y_swing >= 6.5 else 6.5
            walkparameter.Z_Swing_Range = com_height
            walkparameter.Period_T = period_t if period_t % 30 == 0 else 450
            walkparameter.Period_T2 = 720
            walkparameter.Sample_Time = 20
            walkparameter.OSC_LockRange = t_dsp if t_dsp >= 0 else 0
            walkparameter.BASE_Default_Z = base_default_z if base_default_z >= 0 else 0
            walkparameter.X_Swing_COM = right_z_shift
            walkparameter.Y_Swing_Shift = stand_height
            walkparameter.BASE_LIFT_Z = base_lift_z
            walkparameter.rightfoot_shift_z = right_z_shift
            walkparameter.com_y_swing = com_y_shift
            walkparameter.now_stand_height = stand_height
            walkparameter.now_com_height = com_height
            walkparameter.Stand_Balance = False

            self.parameter_pub.publish(walkparameter)
        elif function == 'send':
            parasend2FPGA.Walking_Mode = walk_mode
            parasend2FPGA.X_Swing_Range = com_y_shift
            parasend2FPGA.Y_Swing_Range = y_swing if y_swing >= 6.5 else 6.5
            parasend2FPGA.Z_Swing_Range = com_height
            parasend2FPGA.Period_T = period_t if period_t % 30 == 0 else 450
            parasend2FPGA.Period_T2 = 720
            parasend2FPGA.Sample_Time = 20
            parasend2FPGA.OSC_LockRange = t_dsp if t_dsp >= 0 else 0
            parasend2FPGA.BASE_Default_Z = base_default_z if base_default_z >= 0 else 0
            parasend2FPGA.X_Swing_COM = right_z_shift
            parasend2FPGA.Y_Swing_Shift = stand_height
            parasend2FPGA.BASE_LIFT_Z = base_lift_z
            parasend2FPGA.rightfoot_shift_z = right_z_shift
            parasend2FPGA.com_y_swing = com_y_shift
            parasend2FPGA.now_stand_height = stand_height
            parasend2FPGA.now_com_height = com_height
            parasend2FPGA.Stand_Balance = False

            self.paradata_pub.publish(parasend2FPGA)
        else:
            rospy.logerr(f'sendWalkParameter Error !!!')

    def sendPIDSet(self,P,I,D,MotorID):
    #設定馬達PID參數(原廠設定 P:800 I:0 D:0)
        msg = SensorSet()
        msg.motor_P = P
        msg.motor_I = I
        msg.motor_D = D
        msg.motorID = MotorID
        for i in range(3):
            if  i == 0:
                msg.Pflag
            elif i == 1:
                msg.Iflag
            else:
                msg.Dflag
            self.pid_sub.publish(msg)


    def sendSensorSet(self,P,I,D,modeset):
        msg = SensorSet()
        msg.sensor_P = P * 1000
        msg.sensor_I = I * 1000
        msg.sensor_D = D * 1000
        msg.sensor_modeset = modeset
        self.sensor_pub.publish(msg)

    def sendSensorReset(self, reset_roll, reset_pitch, reset_yaw):
        msg = SensorSet()
        msg.sensor_P = reset_roll
        msg.sensor_I = reset_pitch
        msg.sensor_D = reset_yaw
        msg.sensor_modeset = 0x02
        self.sensor_pub.publish(msg)

    def strategy(self):
        send = Sendmessage()
        while not rospy.is_shutdown():
            if send.Web == True:
                # send.sendSensorReset()
                # cv2.imshow("aaaaaa",send.rawimg)
                # cv2.waitKey(3)
                # print(send.Label_Model[33333])
                send.drawImageFunction(5,1,send.yolo_XMin,send.yolo_XMax,send.yolo_YMin,send.yolo_YMax,0,0,255)
                print(send.yolo_Label)
            
    #def catchImage(self,msg):
    #    self.cvimg = self.bridge.imgmsg_to_cv2(msg,"bgr8")
    #def RawImage(self,msg):
    #    self.rawimg = self.bridge.imgmsg_to_cv2(msg,"bgr8")
    #def OriginImage(self,msg):
    #    self.originimg = self.bridge.imgmsg_to_cv2(msg,"bgr8")
    def startFunction(self,msg):
        self.Web = msg.data
    def getLabelModel(self,msg):
        self.Label_Model = msg.LabelModel
    def getObject(self,msg):
        if self.data_check == False:
            # for i in range (8):
            #     self.color_mask_subject_cnts[i] = msg.Objectlist[i].cnt
            #     self.color_mask_subject_X[i] = [msg.Objectlist[i].Colorarray[j].X for j in range(msg.Objectlist[i].cnt)]
            #     self.color_mask_subject_Y[i] = [msg.Objectlist[i].Colorarray[j].Y for j in range(msg.Objectlist[i].cnt)]
            #     self.color_mask_subject_XMin[i] = [msg.Objectlist[i].Colorarray[j].XMin for j in range(msg.Objectlist[i].cnt)]
            #     self.color_mask_subject_YMin[i] = [msg.Objectlist[i].Colorarray[j].YMin for j in range(msg.Objectlist[i].cnt)]
            #     self.color_mask_subject_XMax[i] = [msg.Objectlist[i].Colorarray[j].XMax for j in range(msg.Objectlist[i].cnt)]
            #     self.color_mask_subject_YMax[i] = [msg.Objectlist[i].Colorarray[j].YMax for j in range(msg.Objectlist[i].cnt)]
            #     self.color_mask_subject_Width[i] = [msg.Objectlist[i].Colorarray[j].Width for j in range(msg.Objectlist[i].cnt)]
            #     self.color_mask_subject_Height[i] = [msg.Objectlist[i].Colorarray[j].Height for j in range(msg.Objectlist[i].cnt)]
            #     self.color_mask_subject_size[i] = [msg.Objectlist[i].Colorarray[j].size for j in range(msg.Objectlist[i].cnt)]
            self.color_mask_subject_cnts = [0 for i in range(8)]
            self.color_mask_subject_X = [[0]*320 for i in range(8)]
            self.color_mask_subject_Y = [[0]*320 for i in range(8)]
            self.color_mask_subject_XMin = [[0]*320 for i in range(8)]
            self.color_mask_subject_XMax = [[0]*320 for i in range(8)]
            self.color_mask_subject_YMax = [[0]*320 for i in range(8)]
            self.color_mask_subject_YMin = [[0]*320 for i in range(8)]
            self.color_mask_subject_Width = [[0]*320 for i in range(8)]
            self.color_mask_subject_Height = [[0]*320 for i in range(8)]
            self.color_mask_subject_size = [[0]*320 for i in range(8)]
            for i in range (8):
                self.color_mask_subject_cnts[i] = msg.Objectlist[i].cnt
                for j in range (self.color_mask_subject_cnts[i]):

                    self.color_mask_subject_X[i][j] = msg.Objectlist[i].Colorarray[j].X
                    self.color_mask_subject_Y[i][j] = msg.Objectlist[i].Colorarray[j].Y
                    self.color_mask_subject_XMin[i][j] = msg.Objectlist[i].Colorarray[j].XMin
                    self.color_mask_subject_YMin[i][j] = msg.Objectlist[i].Colorarray[j].YMin
                    self.color_mask_subject_XMax[i][j] = msg.Objectlist[i].Colorarray[j].XMax
                    self.color_mask_subject_YMax[i][j] = msg.Objectlist[i].Colorarray[j].YMax
                    self.color_mask_subject_Width[i][j] = msg.Objectlist[i].Colorarray[j].Width
                    self.color_mask_subject_Height[i][j] = msg.Objectlist[i].Colorarray[j].Height
                    self.color_mask_subject_size[i][j] = msg.Objectlist[i].Colorarray[j].size
            self.data_check = True
    
    def sensorPackageFunction(self,msg):        
        self.imu_value_Roll  = msg.IMUData[0]
        self.imu_value_Pitch = msg.IMUData[1]
        self.imu_value_Yaw   = msg.IMUData[2]
    def DIOackFunction(self,msg):
        if msg.data & 0x10:
            self.is_start = True
        else:
            self.is_start = False
        self.DIOValue = msg.data
    
    def YoloFunction(self,msg):
        self.yolo_Conf = msg.Conf
        self.yolo_Label = msg.Label
        self.yolo_XMin =int(msg.XMin)
        self.yolo_YMin = int(msg.YMin)
        self.yolo_XMax = int(msg.XMax)
        self.yolo_YMax = int(msg.YMax)
        self.yolo_Y = (self.yolo_YMin+self.yolo_YMax)/2
        self.yolo_X = (self.yolo_XMin+self.yolo_XMax)/2

        
    
if __name__ == '__main__':
    # r = rospy.Rate(100)
    try:
        
        aa = Sendmessage() 
        aa.strategy()
        # r.sleep()
    except rospy.ROSInterruptException:
        pass