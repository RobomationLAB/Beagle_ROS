
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/poll.h>
#include <termios.h>
#include <fcntl.h>
#include <thread>
#include <mutex>


#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include "nav_msgs/Odometry.h"
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>

class Beagle
{
private:
	std::mutex fd_mtx;	
	int fd;
	std::thread receive_thread, send_thread;
	unsigned char index = 0x00;
	double lidarscan[360];
	double lidarIntensity[360];
	void send_task(void);
	void receive_task(void);

private:
	bool serialOpenFlag = false;
	bool MotorMode_toogle = false;
	bool MotorContinuous_toogle = false;
	int EncoderClearFlag = 0;
	int iter = 0;

	// NUS Data packet
	int Nus_messageID;
	int Nus_dataLength;
	int Nus_RC_index;
	int Nus_RC_timeStamep;
	int Nus_RC_encoderLeft;
	int Nus_RC_encoderRight;
	int preNus_RC_encoderLeft;
	int preNus_RC_encoderRight;

	int Nus_RC_prceedDistance;
	int Nus_RC_temperature;
	int Nus_RC_rssi;
	float Nus_RC_batVolt;
	int Nus_RC_batteryState;
	int Nus_RC_motorState;
	int Nus_RC_soundState;
	int Nus_RC_servoSpeed1;
	int Nus_RC_servoAngle1;
	int Nus_RC_servoSpeed2;
	int Nus_RC_servoAngle2;
	int Nus_RC_servoSpeed3;
	int Nus_RC_servoAngle3;
	int MAX_MTU_SIZE = 244;
	int DataLength = (MAX_MTU_SIZE + 7);
	int nus_DataLength = (MAX_MTU_SIZE + 2);	
	char nus_rxBuffer4car[244];
	char nus_rxBuffer4imu[244];
	char nus_rxBuffer4lidar[244];

	int callLegacyMode = 0;
	int callNusMode = 0;
	/** IMU **/
	int Nus_IMU_index;
	int Nus_IMU_timeStamep;
	int Nus_IMU_GyroOdr;
	int Nus_IMU_GyroRange;
	int Nus_IMU_AccOdr;
	int Nus_IMU_AccRange;
	int Nus_IMU_MagOdr;
	int Nus_IMU_MagRepXY;
	int Nus_IMU_MagRepZ;
	int Nus_IMU_DataID;
	int Nus_IMU_DataIndexGyro;
	int Nus_IMU_DataIndexAcc;
	int Nus_IMU_DataIndexMag;
	int Nus_IMU_GyroX;
	int Nus_IMU_GyroY;
	int Nus_IMU_GyroZ;
	int Nus_IMU_AccX;
	int Nus_IMU_AccY;
	int Nus_IMU_AccZ;
	int Nus_Imu_AccTemp;
	float Nus_IMU_MagX;
	float Nus_IMU_MagY;
	float Nus_IMU_MagZ;
	int Nus_IMU_NewTimeStamp;
	int Nus_IMU_DataIndexTime;
	int imuPayloadPos = 0;

	/*Lidar*/
	int Nus_Lidar_Header;
	int Nus_Lidar_MsgID;
	int Nus_Lidar_Idx;
	int Nus_Lidar_SystemState;
	int Nus_Lidar_Sync;
	int Nus_Lidar_AngelResolution;
	int Nus_Lidar_DataLength;

	// NUS Data packet
	int Tx_messageID = 0;
	int Tx_dataLength = 0;
	int Tx_RC_index = 0;
	int Tx_RC_motorState = 0;
	int Tx_RC_PpsLeft = 0;
	int Tx_RC_PpsRight = 0;
	int Tx_RC_DistanceID;
	int Tx_RC_DistanceRange;
	int Tx_RC_SoundLow = 0;
	int Tx_RC_SoundMid = 0;
	int Tx_RC_SoundHigh = 0;
	int Tx_RC_musicalNote = 0;
	int Tx_RC_playSound = 0;
	int Tx_RC_servoSpeed1 = 0;
	int Tx_RC_servoAngle1 = 0;
	int Tx_RC_servoSpeed2 = 0;
	int Tx_RC_servoAngle2 = 0;
	int Tx_RC_servoSpeed3 = 0;
	int Tx_RC_servoAngle3 = 0;

	int oldEncoderClearFlag = 0;

	int Temp_Tx_RC_PpsLeft  = 0;
	int Temp_Tx_RC_PpsRight = 0;
	float robot_x = 0.0;
	float robot_y = 0.0;
	float robot_th = 0.0;


	void NusTxCarInfo(void);

public:
	bool connect(void);
	bool start_Lagacy(void);
	bool start_Nus(void);
	bool start_lidar(void);
	bool stop_lidar(void);
	bool stop(void);
	void speckle_filter(void);
	void NusParsing(void);
	void resetEncoder(void);
	

	ros::Subscriber BeagleCMD, sub_vel;
	ros::Publisher scan_pub, pub_odom;

	tf::TransformBroadcaster odom_broadcaster;

	void CMD_Callback(const std_msgs::String::ConstPtr &msg);
	void vel_callback(const geometry_msgs::Twist::ConstPtr& msg);	
	void scan_publish(ros::Time start_scan_time, double scan_duration, double *lidarscan);
	
	int LidarTxIndex = 0;
	int baudrate;
	std::string port;
	std::string frame_id, scan_topic;
	sensor_msgs::LaserScan scan_msg;
	double angle_max, angle_min;
	double range_max, range_min;
	double angle_increment;
	ros::Time start_scan_time, end_scan_time;
	double scan_duration;

	float radius = 0.0325;
	float WHEEL_BASE_SIZE = 0.097;	
	float onecycle_pps = 2000.0;
	float VEL2Pps =  onecycle_pps / (2.0 * radius * 3.141592);

	float PULSE2DIST = (2.0 * radius * 3.141592) / onecycle_pps;
};
void Beagle::resetEncoder(void)
{
	Nus_RC_encoderLeft = 0;
	Nus_RC_encoderRight= 0;
	preNus_RC_encoderLeft= 0;
	preNus_RC_encoderRight= 0;
	robot_x = 0.0;
	robot_y = 0.0;
	robot_th = 0.0;
	EncoderClearFlag = EncoderClearFlag + 1;
}
void Beagle::vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
	float set_l_vel, set_r_vel;
	set_l_vel = (float)msg->linear.x;
	set_r_vel = (float)msg->angular.z * 3.141592 / 180.0f;

	if(set_l_vel == 0 && set_r_vel == 0)
		MotorMode_toogle = true;
	else
		MotorMode_toogle = false;
	
	float radian_r = set_r_vel;
    float VEL_DIFF = WHEEL_BASE_SIZE*radian_r*0.5;

    Tx_RC_PpsLeft = (set_l_vel - VEL_DIFF)*VEL2Pps;
    Tx_RC_PpsRight = (set_l_vel + VEL_DIFF)*VEL2Pps;
}

void Beagle::CMD_Callback(const std_msgs::String::ConstPtr &msg)
{
	std::string cmdstart_Nus, cmdstart_Lagacy, cmdstart_lidar, cmdstop_lidar, cmdEncoder_reset;
	cmdstart_Nus = "start_Nus";
	cmdstart_Lagacy = "start_Lagacy";
	cmdstart_lidar = "start_lidar";
	cmdstop_lidar = "stop_lidar";
	cmdEncoder_reset = "Encoder_reset";
	

	if (msg->data.find(cmdstart_Nus) == 0)
		start_Nus();
	if (msg->data.find(cmdstart_Lagacy) == 0)
		start_Lagacy();
	if (msg->data.find(cmdstart_lidar) == 0)
		start_lidar();
	if (msg->data.find(cmdstop_lidar) == 0)
		stop_lidar();
	if (msg->data.find(cmdEncoder_reset) == 0)
	{
		resetEncoder();
	}
}

bool Beagle::start_Nus(void)
{   
	ROS_INFO("[Beagle] Nus mode Call");
	std::string motoring = "";
	char hex[20] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
     
	unsigned char packet[10];
	packet[0] = 0xEE;
	packet[1] = 0x43;
	packet[2] = 0x41;
	packet[3] = 0x4C;
	packet[4] = 0x4C;
	packet[5] = 0x5F;
	packet[6] = 0x4E;
	packet[7] = 0x55;
	packet[8] = 0x53;
	packet[9] = 0x00;
	for(int i = 0; i < 20; i++)
    {
     int j = (packet[i] >> 4) & 0x0F;
     int k = packet[i] &0x0F;
      
     motoring += hex[j];
     motoring += hex[k];
    }
	motoring += "\r";
	int i = motoring.length();
	char conststr[41];
	strcpy(conststr, motoring.c_str());


	int val = write(fd, conststr, i);
	val = write(fd, conststr, i);
	val = write(fd, conststr, i);
	return true;
}
bool Beagle::start_Lagacy(void)
{   
	ROS_INFO("[Beagle] Lagacy mode Call");
    int CK_A = 0x00;
    int CK_B = 0x00;
	unsigned char TxBuffer[15];
    
    TxBuffer[0] = 0x52;
    TxBuffer[1] = 0x49;
    TxBuffer[2] = 0x0A;
    
    TxBuffer[3] = 0xEE;
    TxBuffer[4] = 0x08;
    TxBuffer[5] = 0x43;
    TxBuffer[6] = 0x41;
    TxBuffer[7] = 0x4C;
    TxBuffer[8] = 0x4C;
    TxBuffer[9] = 0x5F;
    TxBuffer[10] = 0x4C;
    TxBuffer[11] = 0x45;
    TxBuffer[12] = 0x47;
   // TxBuffer[13] = (byte)0xEE; //CK_A
   // TxBuffer[14] = (byte)0xEC; //CK_B

    
    for(int  i= 0; i<13; i++)
    {
      CK_A = CK_A + TxBuffer[i];
      CK_B = CK_A + CK_B;  
    }
    
    TxBuffer[13] = (unsigned char)CK_A;
    TxBuffer[14] = (unsigned char)CK_B;
	int val = write(fd, TxBuffer, sizeof(TxBuffer));
	val = write(fd, TxBuffer, sizeof(TxBuffer));
	val = write(fd, TxBuffer, sizeof(TxBuffer));
	return true;
}

bool Beagle::start_lidar(void)
{
			
	unsigned char TxBuffer[11];

	int CK_A = 0x00;
	int CK_B = 0x00;
	
    TxBuffer[0] = 0x52;
    TxBuffer[1] = 0x49;
    TxBuffer[2] = 6;//robotiics car payload
    
    TxBuffer[3] = 0x30;
    TxBuffer[4] = 0x04;
    TxBuffer[5] = (unsigned char) 0xFD;
    TxBuffer[6] = (unsigned char) 0xE0;
    TxBuffer[7] = LidarTxIndex++;
    TxBuffer[8] = (unsigned char) 0x0F; //Start

	for (int i = 0; i < 9; i++)
	{
		CK_A = CK_A + TxBuffer[i];
		CK_B = CK_A + CK_B;
	}

	TxBuffer[9] = (unsigned char)CK_A;
	TxBuffer[10] = (unsigned char)CK_B;

	int val = write(fd, TxBuffer, sizeof(TxBuffer));
	val = write(fd, TxBuffer, sizeof(TxBuffer));
	val = write(fd, TxBuffer, sizeof(TxBuffer));
	return true;
}
bool Beagle::stop_lidar(void)
{
		
	unsigned char TxBuffer[11];

	int CK_A = 0x00;
	int CK_B = 0x00;
	
    TxBuffer[0] = 0x52;
    TxBuffer[1] = 0x49;
    TxBuffer[2] = 6;//robotiics car payload
    
    TxBuffer[3] = 0x30;
    TxBuffer[4] = 0x04;
    TxBuffer[5] = (unsigned char) 0xFD;
    TxBuffer[6] = (unsigned char) 0xE0;
    TxBuffer[7] = LidarTxIndex++;
    TxBuffer[8] = (unsigned char) 0xF0; //Stop

	for (int i = 0; i < 9; i++)
	{
		CK_A = CK_A + TxBuffer[i];
		CK_B = CK_A + CK_B;
	}

	TxBuffer[9] = (unsigned char)CK_A;
	TxBuffer[10] = (unsigned char)CK_B;

	int val = write(fd, TxBuffer, sizeof(TxBuffer));
	val = write(fd, TxBuffer, sizeof(TxBuffer));
	val = write(fd, TxBuffer, sizeof(TxBuffer));
	return true;
}

bool Beagle::connect(void)
{
	fd = 0;
	struct termios newtio;

	while(ros::ok())
	{
		fd = open(port.c_str(), O_RDWR | O_NOCTTY );
		if(fd<0) 
		{ 
			ROS_ERROR("[Beagle] connecion error %d. retry", fd);
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}
		else break;
	}
	
	
    memset( &newtio, 0, sizeof(newtio) );

    newtio.c_cflag = B460800;
    ROS_INFO("[Beagle] connetion established. baudrate: B460800");
  	newtio.c_cflag |= CS8;      // data length 8bit 
    newtio.c_cflag |= CLOCAL;	// Use iternel commutication port
    newtio.c_cflag |= CREAD;	// enable read & write
    newtio.c_iflag = 0;			// no parity bit
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;
    tcflush (fd, TCIFLUSH );
    tcsetattr(fd, TCSANOW, &newtio );

	serialOpenFlag = true;

	// init Serial field
	send_thread = std::thread(&Beagle::send_task, this);
	receive_thread = std::thread(&Beagle::receive_task, this);	
	return true;
}
void Beagle::NusTxCarInfo(void)
{	
	unsigned char TxBuffer[27];

	int CK_A = 0x00;
	int CK_B = 0x00;

	TxBuffer[0] = 0x52;
	TxBuffer[1] = 0x49;
	TxBuffer[2] = 22;	 // robotiics car payload
	Tx_messageID = 0x10; // Robotics car info
	TxBuffer[3] = (unsigned char)Tx_messageID;
	Tx_dataLength = 20;
	TxBuffer[4] = (unsigned char)Tx_dataLength;
	TxBuffer[5] = (unsigned char)Tx_RC_index++;

	Tx_RC_motorState = 0x00;
	if (MotorMode_toogle == false)
	{
		Tx_RC_motorState += 0x04; // KEEP
	}
	else
	{ /*SLEEP*/
	}

	if (EncoderClearFlag != oldEncoderClearFlag)
	{
		oldEncoderClearFlag = EncoderClearFlag;
		Tx_RC_motorState += EncoderClearFlag * 64;
		robot_x = 0.0;
		robot_y = 0.0;
		robot_th = 0.0;
	}
	else
	{
	}

	TxBuffer[6] = (unsigned char)Tx_RC_motorState;
	TxBuffer[7] = (unsigned char)(Tx_RC_PpsLeft & 0x00FF);
	TxBuffer[8] = (unsigned char)((Tx_RC_PpsLeft & 0xFF00) >> 8);
	TxBuffer[9] = (unsigned char)(Tx_RC_PpsRight & 0x00FF);
	TxBuffer[10] = (unsigned char)((Tx_RC_PpsRight & 0xFF00) >> 8);
	TxBuffer[11] = (unsigned char)Tx_RC_DistanceID;
	TxBuffer[12] = (unsigned char)(Tx_RC_DistanceRange & 0x00FF);
	TxBuffer[13] = (unsigned char)((Tx_RC_DistanceRange & 0xFF00) >> 8);

	TxBuffer[14] = (unsigned char)Tx_RC_SoundLow;
	TxBuffer[15] = (unsigned char)Tx_RC_SoundMid;
	TxBuffer[16] = (unsigned char)Tx_RC_SoundHigh;
	TxBuffer[17] = (unsigned char)Tx_RC_musicalNote;
	TxBuffer[18] = (unsigned char)Tx_RC_playSound;

	TxBuffer[19] = (unsigned char)Tx_RC_servoSpeed1;
	TxBuffer[20] = (unsigned char)Tx_RC_servoAngle1;
	TxBuffer[21] = (unsigned char)Tx_RC_servoSpeed2;
	TxBuffer[22] = (unsigned char)Tx_RC_servoAngle2;
	TxBuffer[23] = (unsigned char)Tx_RC_servoSpeed3;
	TxBuffer[24] = (unsigned char)Tx_RC_servoAngle3;

	for (int i = 0; i < 25; i++)
	{
		CK_A = CK_A + TxBuffer[i];
		CK_B = CK_A + CK_B;
	}

	TxBuffer[25] = (unsigned char)CK_A;
	TxBuffer[26] = (unsigned char)CK_B;

	if (serialOpenFlag == 1)
	{
		int val = write(fd, TxBuffer, sizeof(TxBuffer));
	}
}

void Beagle::send_task(void)
{
	ros::Rate rate(30);
	for(int i=0; i < 5; i++)
	{
		resetEncoder();
		NusTxCarInfo();
	}

	while (ros::ok())
	{
		ros::spinOnce();
		NusTxCarInfo();
		rate.sleep();
	}
}
void Beagle::receive_task(void)
{
	ros::Rate rate(100);
	for(int i=0; i < 5; i++)
	{
		NusParsing();		
	}

	while (ros::ok())
	{
		ros::spinOnce();
		NusParsing();
		rate.sleep();
	}
}
void Beagle::speckle_filter(void)
{
	float p0, p1 ,p2;	

	for (int i = 0; i < 360; i++)
	{
		if (i == 0)
		{
			p0 = lidarscan[359];
			p1 = lidarscan[0];
			p2 = lidarscan[1];
		}else if (i == 359)
		{
			p0 = lidarscan[358];
			p1 = lidarscan[359];
			p2 = lidarscan[0];
		}
		else{
			p0 = lidarscan[i-1];
			p1 = lidarscan[i];
			p2 = lidarscan[i+1];
		}

		float dist1 = sqrt((p0-p1)*(p0-p1));
		float dist2 = sqrt((p2-p1)*(p2-p1));
		if( dist1 < 0.1 || dist2 < 0.1)
		{
			lidarscan[i] = lidarscan[i];
		}
		else
		{
			lidarscan[i] = std::numeric_limits<float>::infinity();
		}		
	}
}
void Beagle::scan_publish(ros::Time start_scan_time, double scan_duration, double* lidarscan)
{
	scan_msg.header.frame_id = frame_id;
	scan_msg.header.seq = 1;
	scan_msg.header.stamp = start_scan_time;
	
	scan_msg.scan_time = scan_duration;
	scan_msg.time_increment = scan_duration / (double) (360 -1);

	scan_msg.angle_max = angle_max * M_PI / 180.0;
	scan_msg.angle_min = angle_min * M_PI / 180.0;
	scan_msg.angle_increment = M_PI / 180.0; // 1 degree
	
	scan_msg.range_max = range_max;
	scan_msg.range_min = range_min;
	
	scan_msg.ranges.resize(360);
	scan_msg.intensities.resize(360);
	for(int i = 0; i < 360; i++){
		
		if(lidarscan[359-i] < range_min){
			scan_msg.ranges[i] = -std::numeric_limits<float>::infinity();
		}else if(lidarscan[359-i] > range_max){
			scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
		}else{
			scan_msg.ranges[i] = lidarscan[359-i];
		}

		if(i<90)
			scan_msg.intensities[i] = 90;
		else if(i<180)
			scan_msg.intensities[i] = 180;
		else if(i<270)
			scan_msg.intensities[i] = 270;
		else if(i<360)
			scan_msg.intensities[i] = 360;
	}	

	scan_pub.publish(scan_msg);
}

void Beagle::NusParsing(void)
{
	float* accXData = new float[100];
	float* accYData = new float[100];
	float* accZData = new float[100];

	float* gyroXData = new float[100];
	float* gyroYData = new float[100];
	float* gyroZData = new float[100];

	float* magXData = new float[100];
	float* magYData = new float[100];
	float* magZData = new float[100];

	int stateTemp;	
	int indexTemp;
	int CK_A = 0x00;
	int CK_B = 0x00;

	unsigned char Header1[1];
	unsigned char Header2[1];
	unsigned char datalength[1];
	unsigned char Nus_ID[1];
	unsigned char nus_rxBuffer[244];
	int data_length = 0;
	
	int read_size = read(fd, Header1, 1);
	if (read_size == 1 && Header1[0] == 0x52)
	{		
		read_size = read(fd, Header2, 1);		
		if(Header2[0] == 0x4F)
		{	
			read_size = read(fd, datalength, 1);
			data_length = datalength[0];
			read_size = read(fd, Nus_ID, 1);
			if(Nus_ID[0] == 0x10)
			{
				//printf("Getting Data nus_rxBuffer4car------------(%d)\n", data_length);
				nus_rxBuffer4car[0] = Header1[0];
				nus_rxBuffer4car[1] = Header2[0];
				nus_rxBuffer4car[2] = datalength[0];
				nus_rxBuffer4car[3] = Nus_ID[0];
				read_size = read(fd, &nus_rxBuffer4car[4], data_length -1 + 2);
			}
			if (Nus_ID[0] == 0x20)
			{
				//printf("Getting Data nus_rxBuffer4imu------------(%d)\n", data_length);
				nus_rxBuffer4imu[0] = Header1[0];
				nus_rxBuffer4imu[1] = Header2[0];
				nus_rxBuffer4imu[2] = datalength[0];
				nus_rxBuffer4imu[3] = Nus_ID[0];
				read_size = read(fd, &nus_rxBuffer4imu[4], data_length -1 + 2);
			}
			if (Nus_ID[0] == 0x30)
			{
				//printf("Getting Data nus_rxBuffer4lidar------------(%d)\n", data_length);
				nus_rxBuffer4lidar[0] = Header1[0];
				nus_rxBuffer4lidar[1] = Header2[0];
				nus_rxBuffer4lidar[2] = datalength[0];
				nus_rxBuffer4lidar[3] = Nus_ID[0];				
				read_size = read(fd, &nus_rxBuffer4lidar[4], data_length -1 + 2);
			}			
		}
		else{			
			return;
		}
	}
	else{
		//printf("Fail  header Data Discription------------(0x%x)\n", Header1[0]);
		return;
	}
	for (int i = 0; i < data_length + 5; i++)
	{
		if(Nus_ID[0] == 0x10)
			nus_rxBuffer[i] = nus_rxBuffer4car[i];
		if (Nus_ID[0] == 0x20)
			nus_rxBuffer[i] = nus_rxBuffer4imu[i];
		if (Nus_ID[0] == 0x30)
			nus_rxBuffer[i] = nus_rxBuffer4lidar[i];
	}
	data_length = data_length + 5;
	CK_A = 0; // init chksum;
	CK_B = 0;

	// Cal chksum
	for (int i = 0; i < (data_length - 2); i++)
	{
		CK_A = CK_A + nus_rxBuffer[i];
		CK_B = CK_B + CK_A;
	}

	CK_A = CK_A & 0x00FF;
	CK_B = CK_B & 0x00FF;
	if (CK_A == nus_rxBuffer[(data_length - 2)] && CK_B == nus_rxBuffer[(data_length - 1)]) // Checksum OK
	{
		//printf("check sum ok \n");
		for (int i = 0; i < data_length; i++)
		{
			nus_rxBuffer[i] = nus_rxBuffer[i + 3];
		}
	}
	Nus_messageID = (int)nus_rxBuffer[0];
	Nus_dataLength = (int)nus_rxBuffer[1];
	switch (Nus_messageID)
	{
	case (int)0x10:
	{
		//printf("CAR \n");
		Nus_RC_index = nus_rxBuffer[2];
		Nus_RC_timeStamep = nus_rxBuffer[3] + (int)nus_rxBuffer[4] * 256;

		Nus_RC_encoderLeft = (int)nus_rxBuffer[5] + (int)nus_rxBuffer[6] * 256 + (int)nus_rxBuffer[7] * 65536 + (int)nus_rxBuffer[8] * 16777216;
		Nus_RC_encoderRight = (int)nus_rxBuffer[9] + (int)nus_rxBuffer[10] * 256 + (int)nus_rxBuffer[11] * 65536 + (int)nus_rxBuffer[12] * 16777216;


		int L_d_pose = Nus_RC_encoderLeft - preNus_RC_encoderLeft;
		int R_d_pose = Nus_RC_encoderRight - preNus_RC_encoderRight;

		double L_d_distance = (double)L_d_pose * PULSE2DIST;
		double R_d_distance = (double)R_d_pose * PULSE2DIST;

		double distance = (L_d_distance + R_d_distance) / 2.0;
		double radian = (R_d_distance - L_d_distance) / WHEEL_BASE_SIZE;
		double angle = radian / 3.141592 * 180.0;

		double x = cos(radian) * distance;
		double y = -sin(radian) * distance;


		robot_th = robot_th + angle;
		radian = robot_th * 3.141592 / 180.0;
		robot_x = robot_x + x*cos(radian) - y*sin(radian);
		robot_y = robot_y + x*sin(radian) + y*cos(radian);
		// printf("robot_x:  %3.3lf \n", robot_x);
		// printf("robot_y:  %3.3lf \n", robot_y);
		// printf("robot_th:  %3.3lf \n", robot_th);
		nav_msgs::Odometry odom;
		
		odom.header.stamp = ros::Time::now(); // robot_data.get_odom_time; //ros::Time::now();
		odom.header.frame_id = "odom";
		odom.child_frame_id = "base_link";

		odom.pose.pose.position.x = robot_x;
		odom.pose.pose.position.y = robot_y;
		odom.pose.pose.position.z = 0.0;

		double radianpub = robot_th / 180.0 * 3.141592;
		odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(radianpub);

		pub_odom.publish(odom);
		tf::TransformBroadcaster odom_broadcaster;		
		geometry_msgs::TransformStamped odom_tf;
		odom_tf.header = odom.header;
		odom_tf.child_frame_id = odom.child_frame_id;
		odom_tf.transform.translation.x = odom.pose.pose.position.x;
		odom_tf.transform.translation.y = odom.pose.pose.position.y;
		odom_tf.transform.translation.z = odom.pose.pose.position.z;
		odom_tf.transform.rotation = odom.pose.pose.orientation;
		odom_broadcaster.sendTransform(odom_tf);

		preNus_RC_encoderLeft = Nus_RC_encoderLeft;
		preNus_RC_encoderRight = Nus_RC_encoderRight;

		Nus_RC_temperature = nus_rxBuffer[13];
		Nus_RC_rssi = nus_rxBuffer[14];
		stateTemp = nus_rxBuffer[15];
		Nus_RC_batVolt = ((float)(stateTemp & 0x000000FF) + 200) / 100;

		Nus_RC_batteryState = nus_rxBuffer[16];
		Nus_RC_motorState = nus_rxBuffer[17];
		Nus_RC_soundState = nus_rxBuffer[18];

		Nus_RC_servoSpeed1 = nus_rxBuffer[19];
		Nus_RC_servoAngle1 = nus_rxBuffer[20];
		Nus_RC_servoSpeed2 = nus_rxBuffer[21];
		Nus_RC_servoAngle2 = nus_rxBuffer[22];
		Nus_RC_servoSpeed3 = nus_rxBuffer[23];
		Nus_RC_servoAngle3 = nus_rxBuffer[24];

		break;
	}
	case (int)0x20:
	{
		//printf("IMU \n");
		Nus_IMU_index = nus_rxBuffer[2];
		Nus_IMU_timeStamep = nus_rxBuffer[3] + (int)nus_rxBuffer[4] * 256;
		Nus_IMU_GyroOdr = nus_rxBuffer[5];
		Nus_IMU_GyroRange = nus_rxBuffer[6];
		Nus_IMU_AccOdr = nus_rxBuffer[7];
		Nus_IMU_AccRange = nus_rxBuffer[8];
		Nus_IMU_MagOdr = nus_rxBuffer[9];
		Nus_IMU_MagRepXY = nus_rxBuffer[10];
		Nus_IMU_MagRepZ = nus_rxBuffer[11];

		imuPayloadPos = 12;
		do
		{
			indexTemp = nus_rxBuffer[imuPayloadPos++];
			switch (indexTemp & 0xF0)
			{
			case (int)0x10: // gyro

				Nus_IMU_DataIndexGyro = indexTemp & 0x0F;
				Nus_IMU_GyroX = (short)(nus_rxBuffer[imuPayloadPos++] + (int)nus_rxBuffer[imuPayloadPos++] * 256);
				Nus_IMU_GyroY = (short)(nus_rxBuffer[imuPayloadPos++] + (int)nus_rxBuffer[imuPayloadPos++] * 256);
				Nus_IMU_GyroZ = (short)(nus_rxBuffer[imuPayloadPos++] + (int)nus_rxBuffer[imuPayloadPos++] * 256);

				for (int i = 0; i < 99; i++)
				{
					gyroXData[i] = gyroXData[i + 1];
					gyroYData[i] = gyroYData[i + 1];
					gyroZData[i] = gyroZData[i + 1];
				}
				gyroXData[99] = Nus_IMU_GyroX;
				gyroYData[99] = Nus_IMU_GyroY;
				gyroZData[99] = Nus_IMU_GyroZ;
				break;

			case (int)0x20: // acc
				Nus_IMU_DataIndexAcc = indexTemp & 0x0F;
				Nus_IMU_AccX = (short)(nus_rxBuffer[imuPayloadPos++] + (int)nus_rxBuffer[imuPayloadPos++] * 256);
				Nus_IMU_AccY = (short)(nus_rxBuffer[imuPayloadPos++] + (int)nus_rxBuffer[imuPayloadPos++] * 256);
				Nus_IMU_AccZ = (short)(nus_rxBuffer[imuPayloadPos++] + (int)nus_rxBuffer[imuPayloadPos++] * 256);
				Nus_Imu_AccTemp = nus_rxBuffer[imuPayloadPos++];

				for (int i = 0; i < 99; i++)
				{
					accXData[i] = accXData[i + 1];
					accYData[i] = accYData[i + 1];
					accZData[i] = accZData[i + 1];
				}
				accXData[99] = Nus_IMU_AccX;
				accYData[99] = Nus_IMU_AccY;
				accZData[99] = Nus_IMU_AccZ;
				break;

			case (int)0x30: // mag
				Nus_IMU_DataIndexMag = indexTemp & 0x0F;
				for (int i = 0; i < 99; i++)
				{
					magXData[i] = magXData[i + 1];
					magYData[i] = magYData[i + 1];
					magZData[i] = magZData[i + 1];
				}
				magXData[99] = Nus_IMU_MagX;
				magYData[99] = Nus_IMU_MagY;
				magZData[99] = Nus_IMU_MagZ;

				break;

			case (int)0xF0:
				Nus_IMU_DataIndexTime = indexTemp & 0x0F;
				Nus_IMU_NewTimeStamp = (short)(nus_rxBuffer[imuPayloadPos++] + (int)nus_rxBuffer[imuPayloadPos++] * 256);
			default:
				break;
			}

		} while (Nus_dataLength > imuPayloadPos);
		break;
	}
	case (int)0x30:
	{
		
		//printf("LIDAR \n");
		Nus_Lidar_Header = nus_rxBuffer[2];
		Nus_Lidar_MsgID = (int)nus_rxBuffer[3];
		if( (Nus_Lidar_Header == 0xFE || Nus_Lidar_Header == 0xFD))
		{
			int MAX_DISTANCE = 450;
			int distanceIdx;
			int lidarSystemState;
			int sync;
			int angle;
			int distanceDataLength;			
			int outputRateHz = 0;

			int systemState;

			distanceIdx = nus_rxBuffer[4];
			lidarSystemState = nus_rxBuffer[5];
			systemState = lidarSystemState;
			sync = nus_rxBuffer[6];
			angle = nus_rxBuffer[7];
			outputRateHz = nus_rxBuffer[8];
			distanceDataLength = nus_rxBuffer[9] + (nus_rxBuffer[10] * 256);

			start_scan_time = ros::Time::now();
			fd_mtx.lock();			
			if (Nus_Lidar_MsgID == 0x30)
			{	
				
				
			}
			if (Nus_Lidar_MsgID == 0x10)
			{
				for (int i = 0; i < distanceDataLength * 2; i = i + 2)
				{					
					int distance = nus_rxBuffer[i+11] + nus_rxBuffer[i+12] * 256;
					//printf("[%d] %d ", i/2, (int)(distance*0.001));
					lidarscan[iter++] = distance*0.001;
				}
				// printf("\n");
			}
			fd_mtx.unlock();
			if(nus_rxBuffer[6]==4){
				
				tf::TransformBroadcaster laser_broadcaster;		
				geometry_msgs::TransformStamped laser_tf;
				laser_tf.header.stamp = ros::Time::now(); // robot_data.get_odom_time; //ros::Time::now();
				laser_tf.header.frame_id = "base_link";
				laser_tf.child_frame_id = "laser";
				double laserth = 0.0;
				double radianpub = laserth / 180.0 * 3.141592;
				

				laser_tf.transform.translation.x = 0.0;
				laser_tf.transform.translation.y = 0.0;
				laser_tf.transform.translation.z = 0.0;
				laser_tf.transform.rotation = tf::createQuaternionMsgFromYaw(radianpub);
				laser_broadcaster.sendTransform(laser_tf);
				
				speckle_filter();
				end_scan_time = ros::Time::now();
				scan_duration = (end_scan_time - start_scan_time).toSec();
				scan_publish(start_scan_time, scan_duration, lidarscan);
				iter = 0;
			}
		}

		break;
	}

	default:
	{
		break;
	}
	}
}


