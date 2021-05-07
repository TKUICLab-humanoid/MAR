#pragma once

#ifndef MarathonInfoH
#define MarathonInfoH
//===========================================================================
///#include "..\..\..\Coordinate\TCoordinate.h"
///#include "..\..\..\VisionBase\TVision.h"
///#include <string>
///#include <vector>
//===========================================================================
///using namespace std;

class MarathonInfo
{
public:

    MarathonInfo();
    float angle;

    //IMU
    float IMU_begin;
    float IMU_now;
    float IMU_right;
    float IMU_left;
    float IMU_right_max;
    float IMU_right_min;
    float IMU_left_max;
    float IMU_left_min;
    float IMU_debug_left_min;
    float IMU_debug_left_max;
    float IMU_debug_right_min;
    float IMU_debug_right_max;
    float IMU_debug_straight_max;
    float IMU_debug_straight_min;
    int   IMU_range;
    float IMU_Yaw;
	bool  IMU_Once;

    int BrightLineColor;
    int DarkLineColor;
    int LineColor_backupY;
    int LineColor_backupO;

    ////////////////////////////////////////

    int arrow_black;
    int arrow_white;

    ////////////////////////////////////

    float slope;
    float pre_slope;
    int walk_straight;
    int after_turn;
    int mid_pix_count;
    int down_pix_count;
    int mid_cetX;
    int mid_cetY;
    ////////////////////

    int A_top_cetX;
    int A_top_cetY;

    ///////////////////
    int down_cetX;
    int down_cetY;
    int mid_locX;
    int mid_locY;
    //////////////////////////////

    int A_top_locX;
    int A_top_locY;

//////////////////////////////////////
    int down_locX;
    int down_locY;
    int mid_prelocX;	//pre-locationX in mid area <= use when lost line
    int mid_prelocY;
    int down_prelocX;
    int down_prelocY;
    int top_pix_count;
    int delay;
    ////////////////////////////
    int top_pix_A_count;
    int top_A_locX;
    int top_A_locY;
////////////////////////////////////

    int line_pix;
    int head_x;
    int lineX;
    int avg_line;
    int line_lock;
    int line_dir;
    int down_lock;
    int turn;
    int head_y;
    int head_x_range;
    int head_y_range;

    //new
    int initial_theta;

    int tern_mode_theta;//一次轉幾度
    int max_speed;
    int min_speed;
    int ChangeUpXValue;
    int ChangeDownXValue;
    int ChangeTurnXValue;//轉彎減速
    int leftturn_debug;

    //HEAD
    int HeadChange;
    int HeadHigh;

    int dirmap[21];
    int Dcount;

    bool head_line_flag;
    int head_sw ;
    int turn_left_clock;
    int turn_right_clock;
    //////////////////////////
    int data[320][240];
    int imagedata[320][240];
    double x_average;
    double y_average;
    double s_xx;
    double s_xy;

    double x_loc;
    double y_loc;
    int reverse_area;
    int white_count;//計算白膜數量(原n)
    int black_count;//計算黑膜數量(原nb)
    double arrow_slope;//箭頭斜率
    int arrow_model_cnt;
    int graph_next_x;
    int graph_next_y;
    int graph_x;
    int graph_y;
    int MaxWhiteSize;

    int arrow;
    int reverse;
    int xmax;
    int xmin;
    int ymax;
    int ymin;

    int real;

    int arrow_turn;
    int Time_block;
    int Left_Time_block;

    int black_module_x [320];
    int black_module_y [320];
    int black_module_xmin [320];
    int black_module_ymin [320];
    int black_module_xmax [320];
    int black_module_ymax [320];
    int black_module_size [320];
    int black_module_width [320];
    int black_module_Heigh [320];
    int black_module_cnt;
    int DIO1;
    int DIO2;
    int DIO3;
    int DIO4;
    int DIO0;
    /////////////////////////////////////////////////////////////////////////
    int arrowdata;
    int arrow_x;
    int arrow_y;
    int arrow_width;
    int arrow_height;
    int arrow_mid;
    int arrow_y_mid;
	int slow_down = 0; //++++++++++

};
extern MarathonInfo* marathoninfo;
#endif
