#include "strategy/MarathonInfo.h"
//===============================================
///#include "..\..\..\ModelBase\ModelParameter.h"
///#include "..\..\..\ImageBase\ImageUnit.h"
//===============================================
MarathonInfo* marathoninfo = new MarathonInfo();
MarathonInfo::MarathonInfo(void)
{

BrightLineColor = 0x20;//red_color
DarkLineColor = 0x08;//green_color
LineColor_backupO = 0x01; 	//Orange
LineColor_backupY = 0x02;	//Yellow

//////////////////////////////////////
arrow_black = 0x10;
arrow_white = 0x40;

	//IMU
    IMU_now   = 0;
	IMU_right = 0;
	IMU_left  = 0;
    IMU_right_max = 0;
    IMU_right_min = 0;
    IMU_left_max  = 0;
    IMU_left_min  = 0;


	slope = 0;
    angle = 0;
	mid_locX = 0;
	mid_locY = 0;
	down_locX = 0;
	down_locY = 0;
	mid_pix_count = 0;
	down_pix_count = 0;
	top_pix_count = 0;
    head_x = 2051;
    head_y = 1603;	//5.23//380
	lineX = 0;
	Dcount = 0;
    avg_line = 0;
	//////////////////////////////////
	
	top_pix_A_count = 0;
	top_A_locX = 0;
	top_A_locY = 0;

	//////////////////////////////////

    head_x_range = 403;
    head_y_range = 3;

	for (int i=0;i<=20;i++)
	{
		dirmap[i] = 0;
	}
	Dcount = 0;

    head_line_flag = false;
    head_sw =0;
	turn_left_clock = 150;
	turn_right_clock = 150;

	for (int i=0;i<320;i++)
	{
		for (int j=0;j<240;j++)
		{
			data[i][j]=0;
		}
	}
	for (int i=0;i<320;i++)
	{
		for (int j=0;j<240;j++)
		{
			imagedata[i][j]=0;
		}
	}

    x_average=0;
	y_average=0;
	s_xx=1;

	s_xy=0;
	x_loc=0;
	y_loc=0;
	white_count=1;
    arrow_slope=0;

	graph_next_x=0;
	graph_next_y=0;
	graph_x=0;
	graph_y=0;

	arrow=0;
    reverse=0;
    reverse_area=0;
	arrow_turn=0;

    DIO0=0;
    DIO1=0;
    DIO2=0;
    DIO3=0;
    DIO4=0;




}
