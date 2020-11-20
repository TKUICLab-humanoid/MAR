#include <ros/ros.h>
#include <ros/package.h>
#include <vector>
#include <stdio.h>
#include <std_msgs/String.h>
#include "tku_libs/strategy_info.h"
#include "tku_libs/TKU_tool.h"
#include "strategy/MarathonInfo.h"
#include "tku_libs/RosCommunication.h"

//origin marathon.h
#ifndef MARATHON_H
#define MARATHON_H
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>


//Classify
#define pix_top 150 //30 by old strategy  //300
#define pix_mid 200 //150 by old strategy
#define pix_down 200
#define screen 30
#define screen_y 40
/////////////////////////////
#define A_pix_top 5

//Body
#define turn_angle 12 //轉彎極限值
#define debug_line_theta 12 //找線時身體轉的角度
#define debug_speed 80 //100  //速度修正常數
#define line_right 180//190
#define line_left 140//130
#define debug_theta 9 //離線補救角度
#define base_slope 0.36//0.32  //速度斜率基準
//#define side_line_walk_cnt 150

using namespace std;
#define PI 3.14159265358979323846

ros::Publisher WalkingGait_Publish;
ros::Publisher HeadMotor_Publish;
ros::Publisher Sector_Publish;
ros::Publisher HandSpeed_Publish;
ros::Publisher DrawImage_Publish;
ros::Publisher ContinuousValue_Publish;
ros::Publisher SingleMotorData_Publish;

ros::Subscriber Start_subscribe;
ros::Subscriber DIOack_subscribe;
ros::Subscriber LabelModel_subscribe;
ros::Subscriber ObjectList_subscribe;

//bool isStart = false;
bool FindArrowFlag;

class KidsizeStrategy
{
public:

	
	KidsizeStrategy(ros::NodeHandle &nh)
	{
		strategy_info = StrategyInfoInstance::getInstance();
		tool = ToolInstance::getInstance();
		ros_com = RosCommunicationInstance::getInstance();
	};
	~KidsizeStrategy(){};
	void strategymain();

	RosCommunicationInstance *ros_com;
	ToolInstance *tool;
	StrategyInfoInstance *strategy_info;

		//origin marathon.h
	float IMU_Value_Yaw();
	bool StrategyClassify();
	bool StrategyHead();
	bool StrategyInitial();
    bool load_parameter();
    bool load_dirtxt();
    bool StrategyBody();
    bool readini();
	void initparameterpath();
    void linear_regression(void);
	void IMU_Transform();
    int arrow;
	bool CatchArrowFlag;
	//bool FindArrowFlag;
	bool DirFlag;
	bool LineFlag;
	bool if_arrow;
	int  debug_line_flag = 1;

    int turn_theta;
    enum direction{no,straight,right,left};
    int send_x_value;
	int BodyDebug;
	int HeadDebug;
	bool first;
	string parameter_path="N";
	
int roll = 0;
int head_down_count = 0;

int is_walking = 1;
int old_x = 0,old_theta = 0;
};

//origin marathon.h
#endif
