#include<ros/ros.h>
#include<iostream>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/opencv.hpp>
#include"strategy/detector.h"
#include "strategy/strategy_main.h"

using namespace std;
using namespace cv;

/** 				 **/

Mat buffer;//buffer = src
CascadeClassifier cascader2Straight;
CascadeClassifier cascader2Right;
CascadeClassifier cascader2Left;
String straightfile = "/home/iclab/Desktop/MAR/src/strategy/Parameter/cascade2Straight.xml";
String rightfile = "/home/iclab/Desktop/MAR/src/strategy/Parameter/cascade2Right.xml";
String leftfile = "/home/iclab/Desktop/MAR/src/strategy/Parameter/cascade2Left.xml";

ros::Subscriber Imagesource_subscriber;									//for robot
bool checkImageSource = false;

void GetImagesourceFunction(const sensor_msgs::ImageConstPtr& msg)
{
	//printf("fnt in\n");
    cv_bridge::CvImagePtr cv_ptr;
    checkImageSource=true;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      buffer = cv_ptr->image;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }																//for robot
}

int main(int argc,char** argv)
{
	int counter_s, counter_r, counter_l;
	size_t s_t, r_t, l_t;
	s_t = 0;
	r_t = 0;
	l_t = 0;
	counter_s = 0;
	counter_r = 0;
	counter_l = 0;
	ros::init(argc, argv, "detector");
	ros::NodeHandle nh;
	ros::Publisher detector = nh.advertise<strategy::detector>("arrowdata",1000);
	ros::Rate loop_rate(30);
	strategy::detector msg;
	image_transport::ImageTransport it(nh);

	if(!cascader2Straight.load(straightfile)){
		ROS_INFO("Could not load Straight...\n\n");
	}
	if(!cascader2Right.load(rightfile)){
		ROS_INFO("Could not load Right...\n\n");
	}
	if(!cascader2Left.load(leftfile)){
		ROS_INFO("Could not load Left...\n\n");
	}
																							//for pc
	Imagesource_subscriber = nh.subscribe("/usb_cam/image_raw", 10, GetImagesourceFunction);																											//for	pc

	while (nh.ok())
	{
		ros::spinOnce(); //subscriber take data																			//for robot

		if (buffer.empty())
		{
			ROS_INFO("could not load image \n\n");
		}
		else
		{
			Mat gray;
			vector<Rect> Straight;
			vector<Rect> Right;
			vector<Rect> Left;
			cvtColor(buffer, gray, COLOR_BGR2GRAY);
			equalizeHist(gray, gray);

			cascader2Straight.detectMultiScale(gray, Straight, 1.1, 2, 0, Size(10, 10));
			cascader2Right.detectMultiScale(gray, Right, 1.1, 2, 0, Size(10, 10));
			cascader2Left.detectMultiScale(gray, Left, 1.1, 2, 0, Size(10, 10));

			//ROS_INFO("%d\n",Straight.size());
			if(Straight.size())
			{
				rectangle(buffer, Straight[s_t], Scalar(255, 0, 0), 2, 8, 0);
				s_t++;
				counter_s += 1;
			}
			else
			{
				counter_s = 0;
			}

			if(Right.size())
			{
				rectangle(buffer, Right[r_t], Scalar(255, 0, 0), 2, 8, 0);
				r_t++;
				counter_r += 1;
			}
			else
			{
				counter_r = 0;
			}

			if(Left.size())
			{
				rectangle(buffer, Left[l_t], Scalar(255, 0, 0), 2, 8, 0);
				l_t++;
				counter_l += 1;
			}
			else
			{
				counter_l = 0;
			}

			if(counter_s >= 2)
			{	
				slow_down = 1;		//************************

				msg.x 			= Straight[0].x;
				ROS_INFO("SSSSSSSSSSSSSSSSSSSSSSSSSSSSS : msg.x = %d\n",Straight[0].x);

				msg.y 			= Straight[0].y;
				ROS_INFO("SSSSSSSSSSSSSSSSSSSSSSSSSSSSS : msg.y = %d\n",Straight[0].y);

				msg.height 		= Straight[0].height;
				ROS_INFO("SSSSSSSSSSSSSSSSSSSSSSSSSSSSS : msg.height = %d\n",Straight[0].height);

				msg.width 		= Straight[0].width;
				ROS_INFO("SSSSSSSSSSSSSSSSSSSSSSSSSSSSS : msg.width = %d\n",Straight[0].width);
				if(counter_s >= 2)
				{
					msg.arrow = 1;
					ROS_INFO("arrow is Straight\n\n");
				    slow_down = 0;		//************************
					counter_s=0;
				}
			}
			else if(counter_r >= 4)
			{
				slow_down = 1;		//************************
				msg.x 			= Right[0].x;
				ROS_INFO("RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR : msg.x = %d\n",Right[0].x);

				msg.y 			= Right[0].y;
				ROS_INFO("RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR : msg.y = %d\n",Right[0].y);

				msg.height 		= Right[0].height;
				ROS_INFO("RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR : msg.height = %d\n",Right[0].height);

				msg.width 		= Right[0].width;
				ROS_INFO("RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR : msg.width = %d\n",Right[0].width);
				if(counter_r >= 7)
				{
					msg.arrow = 2;
					ROS_INFO("arrow is Right\n\n");
				    slow_down = 0;		//************************
					counter_r = 0;
				}
			}
			else if(counter_l >= 4)
			{
				slow_down = 1;		//************************
				msg.x 			= Left[0].x;
				ROS_INFO("LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL : msg.x = %d\n",Left[0].x);

				msg.y 			= Left[0].y;
				ROS_INFO("LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL : msg.y = %d\n",Left[0].y);

				msg.height 		= Left[0].height;
				ROS_INFO("LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL : msg.height = %d\n",Left[0].height);

				msg.width 		= Left[0].width;
				ROS_INFO("LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL : msg.width = %d\n",Left[0].width);
				if(counter_l >= 7)
				{
					msg.arrow = 3;
					ROS_INFO("arrow is Left\n\n");
				    slow_down = 0;		//************************
					counter_l = 0;
				}
			}
			else
			{
				msg.arrow = 0;

				msg.x 			= 160;

				msg.y 			= 120;

				msg.height 		= 0;

				msg.width 		= 0;
			}

			detector.publish(msg);
			//imshow("detector", buffer);
			cv::waitKey(30);
		}
	}
	waitKey(0);
	loop_rate.sleep();
	return 0;
}
