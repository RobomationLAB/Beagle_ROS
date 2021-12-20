
#include "ros/ros.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <stdio.h>


#include "std_msgs/String.h"

class AIcamera
{
private:
 bool pubimage = true;
	
public:
	bool connect(void);

	ros::Subscriber AIcameraCMD;	
    image_transport::Publisher pub;
	
	

	void CMD_Callback(const std_msgs::String::ConstPtr &msg);	


};

void AIcamera::CMD_Callback(const std_msgs::String::ConstPtr &msg)
{
	std::string ss_start, ss_stop;
	ss_start = "start";
	ss_stop = "stop";

	if (msg->data.find(ss_start) == 0)
	{
		std::cout << "START pubimage data" << std::endl;
		pubimage = true;
	}
	if (msg->data.find(ss_stop) == 0)
	{
		std::cout << "STOP pubimage data" << std::endl;
		pubimage = false;
	}
}

bool AIcamera::connect(void)
{
	cv::Mat img;	

	cv::VideoCapture cap_img;
	cap_img.open("http://192.168.66.1:9527/videostream.cgi?loginuse=admin&loginpas=admin");	
	if (!cap_img.isOpened())
	{
		std::cout << "could not open Cam R" << std::endl;
		cap_img.release();
	}
	else
	{
		std::cout << "open Cam: OK" << std::endl;
	}
	cap_img >> img;
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
	ros::Rate rate(30);
	
	while(ros::ok())
	{
		cap_img >> img;
		msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
		pub.publish(msg);

		cv::imshow("cam", img);
		cv::waitKey(1);
		rate.sleep();
	}

	
	return true;
}
