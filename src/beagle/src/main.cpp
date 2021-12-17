#include "serial.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "beagle_car");
    ros::NodeHandle nh("~");
    
    Beagle beagle;
    nh.param<int>("baudrate", beagle.baudrate, 460800);
    nh.param<std::string>("port", beagle.port, "/dev/ttyUSB0");
    nh.param<std::string>("frame_id", beagle.frame_id, "laser");
    nh.param<std::string>("scan_topic", beagle.scan_topic, "/scan");
    nh.param<double>("angle_min", beagle.angle_min , 0.0);
	nh.param<double>("angle_max", beagle.angle_max , 360.0);
	nh.param<double>("angle_increment", beagle.angle_increment , 1.0);    
    nh.param<double>("range_min", beagle.range_min, 0.005);
	nh.param<double>("range_max", beagle.range_max, 3.0);


    
    beagle.connect();
    beagle.BeagleCMD = nh.subscribe("/Cmd4Beagle", 10, &Beagle::CMD_Callback, &beagle);
    beagle.sub_vel = nh.subscribe("/cmd_vel", 1, &Beagle::vel_callback, &beagle);
    beagle.pub_odom = nh.advertise<nav_msgs::Odometry>("/odom", 1);
    beagle.scan_pub = nh.advertise<sensor_msgs::LaserScan>(beagle.scan_topic.c_str(), 1);

    ros::spin();

    return 0;
}