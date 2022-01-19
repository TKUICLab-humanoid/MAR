#include "strategy/strategy_main.h"
#include "strategy/detector.h"

void catch_the_arrow(const strategy::detector &msg)
{
    marathoninfo->arrowdata = msg.arrow;
    marathoninfo->arrow_x   = msg.x;
    marathoninfo->arrow_y   = msg.y;
    marathoninfo->arrow_width = msg.width;
    marathoninfo->arrow_height = msg.height;
    marathoninfo->arrow_mid = ((marathoninfo->arrow_x) + (marathoninfo->arrow_x + marathoninfo->arrow_width))/2;
    marathoninfo->arrow_y_mid = ((marathoninfo->arrow_y) + (marathoninfo->arrow_y + marathoninfo->arrow_height))/2;

    if (marathoninfo->arrowdata != 0)
    {
        FindArrowFlag = true;
    }
    else
    {
        FindArrowFlag = false;
    }
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "kidsize");
	ros::NodeHandle nh;
    ros::Subscriber arrow_SUB = nh.subscribe("arrowdata",1, catch_the_arrow);
	KidsizeStrategy KidsizeStrategy(nh);

	ros::Rate loop_rate(30);
    KidsizeStrategy.initparameterpath();

	while (nh.ok())
	{
		KidsizeStrategy.strategymain();
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

float KidsizeStrategy::IMU_Value_Yaw()
{
	return strategy_info->getIMUValue().Yaw;
} 
void KidsizeStrategy::IMU_Transform()                               
{
    marathoninfo->IMU_now = IMU_Value_Yaw();                                                
    marathoninfo->IMU_right = marathoninfo->IMU_now - 90;                                   
    marathoninfo->IMU_left  = marathoninfo->IMU_now + 90;                                   
    marathoninfo->IMU_right_max = -90 + marathoninfo->IMU_range;       
    marathoninfo->IMU_right_min = -90 - marathoninfo->IMU_range;       
    marathoninfo->IMU_left_max  =  90  + marathoninfo->IMU_range;
    marathoninfo->IMU_left_min  =  90  - marathoninfo->IMU_range;
}

void KidsizeStrategy::strategymain()
{
		enum direction2{no,straight,right,left};
        strategy_info->get_label_model_flag = true;
        strategy_info->get_image_flag = true;
       	load_parameter();
        ROS_INFO("%d",marathoninfo->arrowdata);

        for(int high=0;high<240;high++)
        {
            for(int width=0;width<320;width++)
            {
                marathoninfo->imagedata[width][high]=strategy_info->label_model[high*320+width];
            }
        }

        if(marathoninfo->arrow==0)//show arrow
        {
            ROS_INFO("no arrow\n");
        }
        else if(marathoninfo->arrow==straight)
        {
            ROS_INFO("straight\n");
        }
        else if(marathoninfo->arrow==right)
        {
            ROS_INFO(" turn right\n");
        }
        else if(marathoninfo->arrow==left)
        {
            ROS_INFO("turn left\n");
        }
        /*if(marathoninfo->real==1)//draw on the web
        {
            ros_com->drawImageFunction(1, DrawMode::DrawObject, marathoninfo->xmin, marathoninfo->xmax, marathoninfo->ymin, marathoninfo->ymax, 125, 38, 205);
            ros_com->drawImageFunction(2, DrawMode::DrawObject, marathoninfo->x_average-5, marathoninfo->x_average+5, 240-(marathoninfo->y_average-5), 240-(marathoninfo->y_average+5), 125, 38, 205);
        }
        else
        {
            ros_com->drawImageFunction(1, DrawMode::DrawObject, marathoninfo->xmin, marathoninfo->xmax, marathoninfo->ymin, marathoninfo->ymax, 255, 0, 0);
            ros_com->drawImageFunction(2, DrawMode::DrawObject, marathoninfo->x_average-5, marathoninfo->x_average+5, 240-(marathoninfo->y_average-5), 240-(marathoninfo->y_average+5),255, 0, 0);
        }*/
	if(strategy_info->getStrategyStart())
	{
		StrategyClassify();
        strategy_info->get_image_flag = true;
        ros_com->drawImageFunction(1, DrawMode::DrawLine, 0, 320, marathoninfo->arrow_y_mid, marathoninfo->arrow_y_mid, 152, 245, 255);
        ros_com->drawImageFunction(2, DrawMode::DrawLine, marathoninfo->arrow_mid, marathoninfo->arrow_mid, 0, 240, 152, 245, 255);
        //ros_com->drawImageFunction(3, DrawMode::DrawObject, 80, 240, 60, 180, 125, 38, 205);
        ros_com->drawImageFunction(4, DrawMode::DrawObject, strategy_info->color_mask_subject[0][0].XMin, strategy_info->color_mask_subject[0][0].XMax, strategy_info->color_mask_subject[0][0].YMin, strategy_info->color_mask_subject[0][0].YMax, 255, 165, 0);
        ros_com->drawImageFunction(5, DrawMode::DrawObject, marathoninfo->arrow_x, (marathoninfo->arrow_x + marathoninfo->arrow_width), marathoninfo->arrow_y, (marathoninfo->arrow_y + marathoninfo->arrow_height), 0, 255, 0);

        ROS_INFO("Dirflag=%d\n",DirFlag);
        ROS_INFO("LineFlag=%d\n",LineFlag);
        ROS_INFO("Dcount=%d\n",marathoninfo->Dcount);
        ROS_INFO("line_dir=%d\n",marathoninfo->line_dir);
        
        StrategyHead();
        ros_com->sendHeadMotor(HeadMotorID::HorizontalID,marathoninfo->head_x,603);
        tool->Delay(30);
        ros_com->sendHeadMotor(HeadMotorID::VerticalID,marathoninfo->head_y,607);
        tool->Delay(30);


        ROS_INFO("marathoninfo->head_x%d\n",marathoninfo->head_x);
        ROS_INFO("marathoninfo->head_y%d\n",marathoninfo->head_y);

        ROS_INFO("head_sw%d\n",marathoninfo->head_sw);
        //ROS_INFO("HeadDebug=%d\n",HeadDebug);

        StrategyBody();
        ROS_INFO("slope=%f\n",marathoninfo->slope);
        ROS_INFO("angle=%f\n",marathoninfo->angle);
        ROS_INFO("turn_theta=%d\n",turn_theta);
        ROS_INFO("send_x_value=%d\n",send_x_value);
        ROS_INFO("Dcount=%d\n",marathoninfo->Dcount);
        ROS_INFO("dirmap=%d\n",marathoninfo->dirmap[marathoninfo->Dcount]);
        ROS_INFO("\n");
        if(is_walking)
        {
            ros_com->sendBodyAuto(send_x_value,0,0,turn_theta,WalkingMode::ContinuousStep,SensorMode::None);
            tool->Delay(30);
            is_walking=0;
        }

            ros_com->sendContinuousValue(send_x_value,0,0,turn_theta,SensorMode::None);
            tool->Delay(30);

            old_x = send_x_value;
            old_theta = turn_theta;

	}
	else
	{
		marathoninfo->Dcount = (marathoninfo->DIO0)*1 + (marathoninfo->DIO1)*2 + (marathoninfo->DIO2)*4;
        ros_com->sendHeadMotor(HeadMotorID::HorizontalID,2051,603);
        tool->Delay(30);
        ros_com->sendHeadMotor(HeadMotorID::VerticalID,1247,603);
        tool->Delay(30);
        if(!is_walking)
        {
            turn_theta = 0;
            ros_com->sendBodyAuto(send_x_value,0,0,turn_theta,WalkingMode::ContinuousStep,SensorMode::None);
            tool->Delay(30);
            is_walking=1;
        }

        StrategyInitial();
        marathoninfo->DIO0=strategy_info->DIOValue.Switch.D0;
        marathoninfo->DIO1=strategy_info->DIOValue.Switch.D1;
        marathoninfo->DIO2=strategy_info->DIOValue.Switch.D2;
        marathoninfo->DIO3=strategy_info->DIOValue.Switch.D3;
        ROS_INFO("head_y=%d\n",marathoninfo->head_y);
        ROS_INFO("Dcount=%d\n",marathoninfo->Dcount);
        ROS_INFO("arrow=%d\n",marathoninfo->arrow);
        ROS_INFO("dirmap=%d\n",marathoninfo->dirmap[marathoninfo->Dcount]);
  	    ROS_INFO("arrowdata = %d\n", marathoninfo->arrowdata);
	}

}

bool KidsizeStrategy::StrategyClassify()
{
    if (DirFlag == false)	//尋找方向旗標，斷軌為true，有找到線為false
    {
        CatchArrowFlag = true;
        marathoninfo->arrow=0;
        marathoninfo->arrow_turn=0;
        marathoninfo->down_pix_count = 0;
        marathoninfo->down_locX = 0;
        marathoninfo->down_locY = 0;
        marathoninfo->mid_pix_count = 0;
        marathoninfo->mid_locX = 0;
        marathoninfo->mid_locY = 0;
        marathoninfo->top_pix_count = 0;
    
        for (int h = 60 ; h < 180 ; h++)  //中段
        {
            for (int w = 0 ; w < 320 ; w++)
            {
                if (marathoninfo->imagedata[w][h] == marathoninfo->BrightLineColor )//red_color	//抓取亮模 red
                {
                    marathoninfo->mid_locX = marathoninfo->mid_locX + w;	//加總水平的所有點(亮暗模的計數相同)
                    marathoninfo->mid_locY = marathoninfo->mid_locY + h;	//加總高的所有點(亮暗模的計數相同)
                    marathoninfo->mid_pix_count++;	//計數(亮暗模的計數相同)
                }
                else if(marathoninfo->imagedata[w][h] == marathoninfo->DarkLineColor)//green_color	//抓取暗模
                {

                        marathoninfo->mid_locX = marathoninfo->mid_locX + w;
                        marathoninfo->mid_locY = marathoninfo->mid_locY + h;
                        marathoninfo->mid_pix_count++;

                }else if(marathoninfo->imagedata[w][h] == marathoninfo->LineColor_backupO)	//Orange	//抓取
                {

                        marathoninfo->mid_locX = marathoninfo->mid_locX + w;
                        marathoninfo->mid_locY = marathoninfo->mid_locY + h;
                        marathoninfo->mid_pix_count++;

                }else if(marathoninfo->imagedata[w][h] == marathoninfo->LineColor_backupY)//Yellow  //抓取暗模
                {

                        marathoninfo->mid_locX = marathoninfo->mid_locX + w;
                        marathoninfo->mid_locY = marathoninfo->mid_locY + h;
                        marathoninfo->mid_pix_count++;

                }
            }
        }
        if ( marathoninfo->mid_pix_count >pix_mid)	//計數大於一定pix才當作有線
        {
            marathoninfo->mid_cetX = marathoninfo->mid_locX/marathoninfo->mid_pix_count;	//計算中心點(加總/計數)
            marathoninfo->mid_cetY = marathoninfo->mid_locY/marathoninfo->mid_pix_count;	//計算中心點(加總/計數)
            marathoninfo->mid_prelocX = marathoninfo->mid_cetX;
            marathoninfo->mid_prelocY = marathoninfo->mid_cetY;	//儲存中心點，破模的時候用來算斜率的點(破模時以前一張畫面為參考)
            DirFlag = false;

        }
        else
        {
            for (int h = 0 ; h < 60 ; h++)  //上段
            {
                for (int w = 0 ; w < 320 ; w++)
                {
                    if (marathoninfo->imagedata[w][h] == marathoninfo->BrightLineColor)
                    {
                        marathoninfo->top_pix_count++;
                    }
                    else if (marathoninfo->imagedata[w][h] == marathoninfo->DarkLineColor)
                    {
                            marathoninfo->top_pix_count++;
                    }
                    else if (marathoninfo->imagedata[w][h] == marathoninfo->LineColor_backupO)
                    {
                            marathoninfo->top_pix_count++;
                    }
                    else if (marathoninfo->imagedata[w][h] == marathoninfo->LineColor_backupY)
                    {   
                            marathoninfo->top_pix_count++;
                    }
                }

            }
            if(marathoninfo->top_pix_count <= pix_top)  //判斷是否破模(若上段有線、中段沒線 = 中段破模)
            {	
                DirFlag = true;
                IMU_Transform();
                marathoninfo->IMU_Yaw = marathoninfo->IMU_now;
                marathoninfo->IMU_now = marathoninfo->IMU_now - marathoninfo->IMU_Yaw;     

                if(marathoninfo->IMU_now < -180)
                {
                    marathoninfo->IMU_now += 360;
                }
                else if(marathoninfo->IMU_now > 180)
                {
                    marathoninfo->IMU_now -=360;
                }
            }
            else
            {
                DirFlag = false;
            }
        }

        if(DirFlag == false)	//有線
        {

            for (int h = 180 ; h < 240 ; h++)    //下段  //240
            {
                for (int w = 0 ; w < 320 ; w++)
                {
                    if (marathoninfo->imagedata[w][h] == marathoninfo->BrightLineColor)
                    {
                        marathoninfo->down_locX = marathoninfo->down_locX + w;
                        marathoninfo->down_locY = marathoninfo->down_locY + h;
                        marathoninfo->down_pix_count++;
                    }
                    else  if(marathoninfo->imagedata[w][h] == marathoninfo->DarkLineColor )
                    {
                            marathoninfo->down_locX = marathoninfo->down_locX + w;
                            marathoninfo->down_locY = marathoninfo->down_locY + h;
                            marathoninfo->down_pix_count++;
                    }
                    else  if(marathoninfo->imagedata[w][h] == marathoninfo->LineColor_backupO )
                    {
                            marathoninfo->down_locX = marathoninfo->down_locX + w;
                            marathoninfo->down_locY = marathoninfo->down_locY + h;
                            marathoninfo->down_pix_count++;
                    }
                    else  if(marathoninfo->imagedata[w][h] == marathoninfo->LineColor_backupY )
                    {
                            marathoninfo->down_locX = marathoninfo->down_locX + w;
                            marathoninfo->down_locY = marathoninfo->down_locY + h;
                            marathoninfo->down_pix_count++;
                    }
                }
            }
            if ( marathoninfo->down_pix_count > pix_down)   ////下段有線
            {
                marathoninfo->down_cetX = marathoninfo->down_locX / marathoninfo->down_pix_count;  //計算像素平均
                marathoninfo->down_cetY = marathoninfo->down_locY / marathoninfo->down_pix_count;
                marathoninfo->down_prelocX = marathoninfo->down_cetX;
                marathoninfo->down_prelocY = marathoninfo->down_cetY;


                //

            }
            if(marathoninfo->mid_pix_count > pix_mid && marathoninfo->down_pix_count > pix_down)	//中、下段同時有線
            {

            }
            else
            {
                if (marathoninfo->down_pix_count <= pix_down)	//破模時 以前一張畫面替代
                {
                    marathoninfo->down_cetX = marathoninfo->down_prelocX;
                    marathoninfo->down_cetY = marathoninfo->down_prelocY;
                }
                if (marathoninfo->mid_pix_count <= pix_mid)
                {
                    marathoninfo->mid_cetX = marathoninfo->mid_prelocX;
                    marathoninfo->mid_cetY = marathoninfo->mid_prelocY;
                }
            }
            marathoninfo->slope = (float)(marathoninfo->mid_cetX - marathoninfo->down_cetX) / (float)(marathoninfo->mid_cetY - marathoninfo->down_cetY);
            ROS_INFO("slope=%f\n",marathoninfo->slope);
            //printf("slope=%f\n",marathoninfo->slope); //測試用
            //計算斜率(中、下段)	注意:此為X/Y，與數學認知不同
        }
    }
    else ///Dirflag == true
    {

        if (marathoninfo->arrow_mid < 160 - screen)//箭頭在左邊
        {
            marathoninfo->line_lock = left;
        }
        else if (marathoninfo->arrow_mid > 160 + screen)//箭頭在右邊
        {
            marathoninfo->line_lock = right;
        }
        else
        {
            marathoninfo->line_lock = straight;
        }
        
        if (marathoninfo->arrow_y_mid < 120 - screen_y)//箭頭在畫面上面
        {
            marathoninfo->head_y -= 5;
            if (marathoninfo->head_y <= 1400)
            {marathoninfo->head_y = 1400;}
        }
        else if (marathoninfo->arrow_y_mid > 120 + screen_y)//箭頭在畫面下面
        {
            marathoninfo->head_y += 5;
            if (marathoninfo->head_y <= 1400)
            {marathoninfo->head_y = 1400;}
        }
        else//箭頭在畫面中間
        {
            marathoninfo->head_y = marathoninfo->head_y;
        }
        if (FindArrowFlag == false)
        {

        }
        else
        {
            if(marathoninfo->arrow_turn== 0)
            {
                 printf("\n\n\nget the arrow\n\n\n");
                 marathoninfo->IMU_Yaw = IMU_Value_Yaw();
                 debug_line_flag = 1;
                 tool->Delay(30);
                 readini();
            }
            FindArrowFlag = false;
        }
    }
    return true;
}
bool KidsizeStrategy::StrategyHead(void)
{
    // 	System::Windows::Forms::MessageBox::Show("Head!", "Error", System::Windows::Forms::MessageBoxButtons::OK, System::Windows::Forms::MessageBoxIcon::Error);

    if (DirFlag == false)
    {
        marathoninfo->head_x = 2047;
        marathoninfo->head_y -= 33;
        if(marathoninfo->head_y <1400)
        {
            marathoninfo->head_y = 1400;   //335 //old 1303
        }
        ROS_INFO("start\n");

        //HeadDebug=0;
    }
    else
    {
        if (CatchArrowFlag == false) 
        {
            marathoninfo->IMU_now = IMU_Value_Yaw();
            marathoninfo->IMU_now = marathoninfo->IMU_now - marathoninfo->IMU_Yaw;  
            
            if(marathoninfo->IMU_now < -180)
            {
                marathoninfo->IMU_now += 360;
            }
            else if(marathoninfo->IMU_now > 180)
            {
                marathoninfo->IMU_now -=360;
            }
          
            if (marathoninfo->arrow == right && marathoninfo->IMU_now >= marathoninfo->IMU_right_min && marathoninfo->IMU_now <= marathoninfo->IMU_right_max)
            {
                roll++;
                tool->Delay(30);
                ROS_INFO("\nroll = %d\n",roll);
                marathoninfo->head_y = 1700;
                //HeadDebug=1;
            }
            else if(marathoninfo->arrow == left && marathoninfo->IMU_now >= marathoninfo->IMU_left_min && marathoninfo->IMU_now <= marathoninfo->IMU_left_max)
            {
                roll++;
                tool->Delay(30);
                ROS_INFO("\nroll = %d\n",roll);
                marathoninfo->head_y = 1700;
                //HeadDebug=1;
            }
            else if(marathoninfo->arrow == straight)
            {
                roll++;
                tool->Delay(30);
                ROS_INFO("\nroll = %d\n",roll);
                marathoninfo->head_y = 1700;
                if(roll > marathoninfo->walk_straight)
                {                    
                    marathoninfo->turn = straight;
                }
                //HeadDebug=1;
            }
            else
            {
               marathoninfo->turn = marathoninfo->arrow;
            }
        }
    }
    return true;
}

bool KidsizeStrategy::StrategyBody(void)
{
    marathoninfo->slope = (float)(marathoninfo->mid_cetX - marathoninfo->down_cetX) / (float)(marathoninfo->mid_cetY - marathoninfo->down_cetY);
    marathoninfo->angle = (atan((float)(marathoninfo->slope))*180/3.14);	//計算結果?
	//改 看angle 數值
    turn_theta = marathoninfo->angle / 5; //angle / x, the larger x is, the better chance the robot goes into 90 degree line
    if (turn_theta > turn_angle + marathoninfo->initial_theta)
    {
        turn_theta = turn_angle - 1;
    }else if (turn_theta < -turn_angle + marathoninfo->initial_theta){
        turn_theta = -turn_angle + 1;
    }

    if (DirFlag == true)
    {
	//******************************************/
	if((send_x_value > marathoninfo->min_speed) && (marathoninfo->slow_down == 1))
        { 
            send_x_value = send_x_value - marathoninfo->ChangeUpXValue;
        }
	//******************************************/
        if (CatchArrowFlag == false)
        {
            marathoninfo->IMU_now = IMU_Value_Yaw();
            
            switch(marathoninfo->arrow)
            {
		
            case straight:							//前進
                printf("--------------------------Body straight----------------------------\n");
                if (debug_line_flag == 1)
                {
                    marathoninfo->IMU_now = marathoninfo->IMU_now - marathoninfo->IMU_Yaw;
                    if(marathoninfo->IMU_now < -180)
                    {
                        marathoninfo->IMU_now += 360;
                    }
                    else if(marathoninfo->IMU_now > 180)
                    {
                        marathoninfo->IMU_now -=360;
                    }
                    if (marathoninfo->IMU_now > marathoninfo->IMU_debug_straight_max)
                    {
                        turn_theta = -debug_line_theta + marathoninfo->initial_theta;
                    }
                    else if (marathoninfo->IMU_now < marathoninfo->IMU_debug_straight_min)
                    {
                        turn_theta = debug_line_theta + marathoninfo->initial_theta;
                    }
                    else
                    {
                        if(send_x_value < marathoninfo->max_speed)//1400     
                        { 
                            send_x_value = send_x_value + marathoninfo->ChangeUpXValue;
                        }
                        turn_theta = marathoninfo->initial_theta;//NEW CHANGE
                        marathoninfo->arrow_turn= 0;
                    }

                    if (roll > marathoninfo->after_turn)
                        {
                            roll = 0;
                            debug_line_flag = 0;
                        }
                }
                else
                {
                    if (marathoninfo->line_lock == right)
                    {
                        turn_theta = -debug_line_theta + marathoninfo->initial_theta;
                    }
                    else if (marathoninfo->line_lock == left)
                    {
                        turn_theta = debug_line_theta + marathoninfo->initial_theta;
                    }
                    else if (marathoninfo->line_lock == straight)
                    {
                        if(send_x_value < marathoninfo->max_speed)//1400
                        { 
                            send_x_value = send_x_value + marathoninfo->ChangeUpXValue;
                        }
                        turn_theta = marathoninfo->initial_theta;//NEW CHANGE
                        marathoninfo->arrow_turn= 0;
                    }
                }               
                break;

            case left:											//左轉
                printf("--------------------------Body left----------------------------\n");
                if (send_x_value > marathoninfo->max_speed - 200) //1200
                    {
                       send_x_value = send_x_value - marathoninfo->ChangeDownXValue;
                    }
                turn_theta=marathoninfo->tern_mode_theta+marathoninfo->initial_theta;
                marathoninfo->arrow_turn= left;
               
                marathoninfo->IMU_now = marathoninfo->IMU_now - marathoninfo->IMU_Yaw;
                if(marathoninfo->IMU_now < -180)
                {
                    marathoninfo->IMU_now += 360;
                }
                else if(marathoninfo->IMU_now > 180)
                {
                   marathoninfo->IMU_now -=360;
                }
                if(marathoninfo->IMU_now >= marathoninfo->IMU_left_min && marathoninfo->IMU_now <= marathoninfo->IMU_left_max)
                {
                    if (debug_line_flag == 1)
                    {
                        if (marathoninfo->IMU_now >= marathoninfo->IMU_debug_left_max )
                        {
                            turn_theta = -debug_line_theta + marathoninfo->initial_theta;
                        }
                        else if (marathoninfo->IMU_now <= marathoninfo->IMU_debug_left_min)
                        {
                            turn_theta = debug_line_theta + marathoninfo->initial_theta;
                        }
                        else 
                        {
                            turn_theta = marathoninfo->initial_theta; //NEW CHANGE
                            if(send_x_value < marathoninfo->max_speed - 100)
                            {
                               send_x_value = send_x_value + marathoninfo->ChangeUpXValue;
                            }
                            marathoninfo->arrow_turn= 0;
                        }

                        if (roll > marathoninfo->after_turn)
                        {
                            roll = 0;
                            debug_line_flag = 0;
                        }
                    }
                    else
                    {
                        if (marathoninfo->line_lock == right)
                        {
                            turn_theta = -debug_line_theta + marathoninfo->initial_theta;
                        }
                        else if (marathoninfo->line_lock == left)
                        {
                            turn_theta = debug_line_theta + marathoninfo->initial_theta;
                        }
                        else 
                        {
                            turn_theta = marathoninfo->initial_theta; //NEW CHANGE
                            if(send_x_value < marathoninfo->max_speed - 100)
                            {
                               send_x_value = send_x_value + marathoninfo->ChangeUpXValue;
                            }
                            marathoninfo->arrow_turn= 0;
                        }
                    }      
                }

                break;

            case right:		
                printf("--------------------------Body right----------------------------\n");
                if (send_x_value > marathoninfo->max_speed - 200)
                {
                    send_x_value = send_x_value - marathoninfo->ChangeDownXValue;
                }

                turn_theta=(-(marathoninfo->tern_mode_theta))+marathoninfo->initial_theta;

                marathoninfo->arrow_turn= right;

                marathoninfo->IMU_now = marathoninfo->IMU_now - marathoninfo->IMU_Yaw;
                    
                if(marathoninfo->IMU_now < -180)
                {
                    marathoninfo->IMU_now += 360;
                }
                else if(marathoninfo->IMU_now > 180)
                {
                    marathoninfo->IMU_now -=360;
                }
                if(marathoninfo->IMU_now >= marathoninfo->IMU_right_min && marathoninfo->IMU_now <= marathoninfo->IMU_right_max )
                {
                    if (debug_line_flag == 1)
                    {
                        if (marathoninfo->IMU_now >= marathoninfo->IMU_debug_right_max  )
                        {
                            turn_theta = -debug_line_theta + marathoninfo->initial_theta;
                        }
                        else if (marathoninfo->IMU_now <= marathoninfo->IMU_debug_right_min)
                        {
                            turn_theta = debug_line_theta + marathoninfo->initial_theta;
                        }
                        else 
                        {
                            turn_theta = marathoninfo->initial_theta; //NEW CHANGE
                            if(send_x_value < marathoninfo->max_speed - 100)
                            {
                               send_x_value = send_x_value + marathoninfo->ChangeUpXValue;
                            }
                            marathoninfo->arrow_turn= 0;
                        }

                        if (roll > marathoninfo->after_turn)
                        {
                            roll = 0;
                            debug_line_flag = 0;
                        }
                    }
                    else
                    {
                        if (marathoninfo->line_lock == right)
                        {
                            turn_theta = -debug_line_theta + marathoninfo->initial_theta;
                        }
                        else if (marathoninfo->line_lock == left)
                        {
                            turn_theta = debug_line_theta + marathoninfo->initial_theta;
                        }
                        else 
                        {
                            turn_theta = marathoninfo->initial_theta; //NEW CHANGE

                            if(send_x_value < marathoninfo->max_speed - 100)//1300
                            { 
                                send_x_value = send_x_value + marathoninfo->ChangeUpXValue;
                            }
                            marathoninfo->arrow_turn= 0;
                        }
                    }     
                }
                
                break;
            }
        }
    }
    else //DirFlag == false
    {
        marathoninfo->head_line_flag = false;
        marathoninfo->head_sw = 0;
        roll = 0;
	
        if (marathoninfo->slope >base_slope)    //右轉
        {
            if (send_x_value > marathoninfo->max_speed - 100 - abs(turn_theta)*debug_speed)
            {
                send_x_value = send_x_value - marathoninfo->ChangeTurnXValue;
            }
            if (send_x_value < marathoninfo->max_speed - 100 - abs(turn_theta)*debug_speed)
            {
                send_x_value = send_x_value + marathoninfo->ChangeTurnXValue;
            }

        }
        else if (marathoninfo->slope < -base_slope )    //左轉
        {
            if (send_x_value > marathoninfo->max_speed - 100 - abs(turn_theta)*debug_speed)
            {
                send_x_value = send_x_value - marathoninfo->ChangeTurnXValue;
            }
            if (send_x_value < marathoninfo->max_speed - 100 - abs(turn_theta)*debug_speed)
            {
                send_x_value = send_x_value + marathoninfo->ChangeTurnXValue;
            }

        }
        else if (marathoninfo->slope > -base_slope && marathoninfo->slope < base_slope) //直走
        {
            if (send_x_value < marathoninfo->max_speed - 100 - abs(turn_theta)*debug_speed)
            {
                send_x_value = send_x_value + marathoninfo->ChangeUpXValue;
            }
            if (send_x_value > marathoninfo->max_speed - 100 - abs(turn_theta)*debug_speed)
            {
                send_x_value = send_x_value - marathoninfo->ChangeUpXValue;
            }

            if (marathoninfo-> down_cetX > 150 && marathoninfo-> down_cetX < 170 )
            {
                if(marathoninfo-> mid_cetX < 150)
                {
                    if(send_x_value < marathoninfo->max_speed - 100){ //1300
                        send_x_value = send_x_value + marathoninfo->ChangeUpXValue;
                    }
                }
                else if(marathoninfo-> mid_cetX > 170)
                {
                    if(send_x_value < marathoninfo->max_speed - 100){ //1300
                        send_x_value = send_x_value + marathoninfo->ChangeUpXValue;
                    }
                }
            }
        }

        if (marathoninfo->mid_cetX > line_right && marathoninfo->down_cetX > line_right) //走路左偏, 向右修正
            {

                if (send_x_value > marathoninfo->max_speed - 200)
                {
                     send_x_value = send_x_value - marathoninfo->ChangeDownXValue;
                }

                turn_theta = -debug_theta;

                if(marathoninfo->slope > 0.15)
                { //原本位在線邊 走回線上 //NEW CHANGE(roll deleted)
                    if(send_x_value < marathoninfo->max_speed - 200)//1200
                    {
                        send_x_value = send_x_value + marathoninfo->ChangeUpXValue;
                    }
                    tool->Delay(30);

                    printf("walk into right line\n");

                    if(marathoninfo-> down_cetX < 40){

                        if (send_x_value > marathoninfo->max_speed - 200)
                        {
                            send_x_value = send_x_value - marathoninfo->ChangeDownXValue;
                        }

                        turn_theta = debug_theta;
                        if(marathoninfo->mid_cetX > 120 && marathoninfo->mid_cetX < 200){
                            if(send_x_value < marathoninfo->max_speed - 100){ //1300
                                send_x_value = send_x_value + marathoninfo->ChangeUpXValue;
                            }
                        }
                    }
                }
            }
            if (marathoninfo->mid_cetX < line_left && marathoninfo->down_cetX < line_left) //走路右偏, 向左修正
            {
                if (send_x_value > marathoninfo->max_speed - 200)
                {
                    send_x_value = send_x_value - marathoninfo->ChangeDownXValue;
                }

                turn_theta = debug_theta;
                if(marathoninfo->slope < -0.15)//原本位在線邊 走回線上 //NEW CHANGE(roll deleted)
                {
                    if(send_x_value < marathoninfo->max_speed - 200)//1200
                    {
                        send_x_value = send_x_value + marathoninfo->ChangeUpXValue;
                    }
                    tool->Delay(30);
                    printf("walk into left line\n");

                    if(marathoninfo->down_cetX > 280)
                    {
                        if (send_x_value > marathoninfo->max_speed - 200)
                            {
                                send_x_value = send_x_value - marathoninfo->ChangeDownXValue;
                            }

                        turn_theta = -debug_theta;
                        if(marathoninfo->mid_cetX > 120 && marathoninfo->mid_cetX < 200){
                            if(send_x_value < marathoninfo->max_speed - 100){ //1300
                                send_x_value = send_x_value + marathoninfo->ChangeUpXValue;
                            }
                        }
                    }
                }
            }

         if (send_x_value > marathoninfo->max_speed)
        {
            send_x_value = marathoninfo->max_speed;
        }
        else if(send_x_value < marathoninfo->min_speed)
        {
            send_x_value = marathoninfo->min_speed;
        }

        turn_theta = turn_theta + marathoninfo->initial_theta;

        marathoninfo->delay = 0;
        marathoninfo->mid_cetX = marathoninfo->mid_prelocX;
        marathoninfo->mid_cetY = marathoninfo->mid_prelocY;
        marathoninfo->down_cetX = marathoninfo->down_prelocX;
        marathoninfo->down_cetY = marathoninfo->down_prelocY;
    }
    return true;
}

bool KidsizeStrategy::StrategyInitial(void)
{
    send_x_value = 0;
    first = true;
    DirFlag = false;
    marathoninfo->head_line_flag = false;
    marathoninfo->head_sw = 0; //斷軌時計數用
    marathoninfo->turn=0;
    marathoninfo->arrow = 0;
    marathoninfo->arrow_turn = 0;
	marathoninfo->line_lock = 0; //判斷走線方向
    marathoninfo->line_dir = no ; //判斷斷軌方向
    //LineFlag = false;
    return true;
}

void KidsizeStrategy::initparameterpath()
{
	while(parameter_path == "N")
	{
		parameter_path = tool->getPackagePath("strategy");
	}
	printf("parameter_path is %s\n", parameter_path.c_str());
}

bool KidsizeStrategy::load_parameter(void)
{
    fstream fin;
    string sTmp;
    ROS_INFO("arrow is Left\n\n");
    char line[100];
    char path[200];
    strcpy(path, parameter_path.c_str());
    strcat(path, "/quickchange.ini");
    fin.open(path, ios::in);
    try
    {
        marathoninfo->Time_block = tool->readvalue(fin, "Time_block", 1);
        marathoninfo->max_speed  = tool->readvalue(fin, "max_speed", 1);
        marathoninfo->min_speed  = tool->readvalue(fin, "min_speed", 1);

        marathoninfo->ChangeUpXValue   = tool->readvalue(fin, "ChangeUpXValue", 1);
        marathoninfo->ChangeDownXValue = tool->readvalue(fin, "ChangeDownXValue", 1);
        marathoninfo->ChangeTurnXValue = tool->readvalue(fin, "ChangeTurnXValue", 1);

        marathoninfo->tern_mode_theta = tool->readvalue(fin, "tern_mode_theta", 1);

        marathoninfo->HeadChange = tool->readvalue(fin,"HeadChange",1);
        marathoninfo->HeadHigh = tool->readvalue(fin,"HeadHigh",1);

        marathoninfo->initial_theta = tool->readvalue(fin,"initial_theta",1);

        marathoninfo->leftturn_debug= tool->readvalue(fin,"leftturn_debug",1);
        marathoninfo->after_turn = tool->readvalue(fin,"after_turn",1);
        marathoninfo->walk_straight = tool->readvalue(fin,"walk_straight",1);
        marathoninfo->IMU_range = tool->readvalue(fin,"IMU_range",1);
        
        marathoninfo->IMU_debug_left_max = tool->readvalue(fin,"IMU_debug_left_max",1);
        marathoninfo->IMU_debug_left_min = tool->readvalue(fin,"IMU_debug_left_min",1);
        marathoninfo->IMU_debug_right_max = tool->readvalue(fin,"IMU_debug_right_max",1);
        marathoninfo->IMU_debug_right_min = tool->readvalue(fin,"IMU_debug_right_min",1);
        marathoninfo->IMU_debug_straight_max = tool->readvalue(fin,"IMU_debug_straight_max",1);
        marathoninfo->IMU_debug_straight_min = tool->readvalue(fin,"IMU_debug_straight_min",1);
        //marathoninfo->Dcount = tool->readvalue(fin,"Dcount",1);
    }
    catch(exception e)
    {

    }
    return true;
}
/*
bool KidsizeStrategy::load_dirtxt(void)
{
    fstream fin;
    string sTmp;
    char line[100];
    char path[200];
    strcpy(path, parameter_path.c_str());
    strcat(path, "/dirmap.ini");
    fin.open(path, ios::in);
    try
    {
        marathoninfo->dirmap[0] = tool->readvalue(fin, "dir1", 1);
        marathoninfo->dirmap[1] = tool->readvalue(fin, "dir2", 1);
        marathoninfo->dirmap[2] = tool->readvalue(fin, "dir3", 1);
        marathoninfo->dirmap[3] = tool->readvalue(fin, "dir4", 1);
        marathoninfo->dirmap[4] = tool->readvalue(fin, "dir5", 1);
        marathoninfo->dirmap[5] = tool->readvalue(fin, "dir6", 1);
        marathoninfo->dirmap[6] = tool->readvalue(fin, "dir7", 1);
        marathoninfo->dirmap[7] = tool->readvalue(fin, "dir8", 1);
        marathoninfo->dirmap[8] = tool->readvalue(fin, "dir9", 1);
        marathoninfo->dirmap[9] = tool->readvalue(fin, "dir10", 1);
        marathoninfo->dirmap[10] = tool->readvalue(fin, "dir11", 1);
        marathoninfo->dirmap[11] = tool->readvalue(fin, "dir12", 1);

        fin.close();

    }
    catch(exception e)
    {
    }
    return true;
}*/

void KidsizeStrategy::linear_regression(void)
{
    marathoninfo->top_A_locX = 0;
    marathoninfo->top_A_locY = 0;
    marathoninfo->top_pix_A_count = 0;

    for (int h = 0 ; h < 240 ; h++)  //all range
    {
        for (int w = 0 ; w < 320 ; w++)
        {
            if (marathoninfo->imagedata[w][h] == marathoninfo->arrow_black)
            {
                marathoninfo->top_A_locX = marathoninfo->top_A_locX + w;	//加總水平的所有點(亮暗模的計數相同)
                marathoninfo->top_A_locY = marathoninfo->top_A_locY + h;	//加總高的所有點(亮暗模的計數相同)
                marathoninfo->top_pix_A_count++;
            }
            else if (marathoninfo->imagedata[w][h] == marathoninfo->arrow_white)
            {
                marathoninfo->top_A_locX = marathoninfo->top_A_locX + w;	//加總水平的所有點(亮暗模的計數相同)
                marathoninfo->top_A_locY = marathoninfo->top_A_locY + h;	//加總高的所有點(亮暗模的計數相同)
                marathoninfo->top_pix_count++;
            }
        }
    }
    if ( marathoninfo->top_pix_count > A_pix_top)
    {
        marathoninfo->mid_cetX = marathoninfo->top_A_locX/marathoninfo->top_pix_count;	//計算中心點(加總/計數)
        marathoninfo->mid_cetY = marathoninfo->top_A_locY/marathoninfo->top_pix_count;	//計算中心點(加總/計數)
    }
    else
    {
        printf("no arrow's pix\n");
    }
    /*
    else
    {
        for (int h = 60 ; h < 180 ; h++)  //mid
        {
            for (int w = 0 ; w < 320 ; w++)
            {
                if (marathoninfo->imagedata[w][h] == marathoninfo->arrow_black)
                {
                    marathoninfo->top_A_locX = marathoninfo->top_A_locX + w;	//加總水平的所有點(亮暗模的計數相同)
                    marathoninfo->top_A_locY = marathoninfo->top_A_locY + h;	//加總高的所有點(亮暗模的計數相同)
                    marathoninfo->top_pix_A_count++;
                }
                else if (marathoninfo->imagedata[w][h] == marathoninfo->arrow_white)
                {
                    marathoninfo->top_A_locX = marathoninfo->top_A_locX + w;	//加總水平的所有點(亮暗模的計數相同)
                    marathoninfo->top_A_locY = marathoninfo->top_A_locY + h;	//加總高的所有點(亮暗模的計數相同)
                    marathoninfo->top_pix_count++;
                }
            }
        }
        if ( marathoninfo->top_pix_count > A_pix_top)
        {
            marathoninfo->mid_cetX = marathoninfo->top_A_locX/marathoninfo->top_pix_count;	//計算中心點(加總/計數)
            marathoninfo->mid_cetY = marathoninfo->top_A_locY/marathoninfo->top_pix_count;	//計算中心點(加總/計數)
        }
    }
    */
}

bool KidsizeStrategy::readini()
{
    if(marathoninfo->arrowdata != 0)
    {
        if(first == true)
        {
            marathoninfo->arrow = marathoninfo->arrowdata;
            first = false;
            CatchArrowFlag = false;
        }
        else
        {
        marathoninfo->arrow = marathoninfo->arrowdata;
        printf("\n into readini \n");
        CatchArrowFlag = false;
        }
    }
     else
    {
        marathoninfo->arrow = 0;
        CatchArrowFlag = true; 
	tool->Delay(2000);
    }
}

