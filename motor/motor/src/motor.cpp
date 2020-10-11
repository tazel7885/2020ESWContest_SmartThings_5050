/***********************************************************

Author : HY-mec
Date : 2020.10.10

***********************************************************/



//header file
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include "/home/tazel/catkin_ws/src/motor/DynamixelSDK/ros/include/dynamixel_sdk/dynamixel_sdk.h"
#include <stdint.h>


//motor viriable
#define torque_enable   64
#define goal_velocity   116
#define present_position  132
#define protocol_version  2.0
#define dxl_id_1       1
#define dxl_id_2       2
#define baudrate       57600
#define devicename     "/dev/ttyUSB0"

#define enable 1
#define disable 0

//viriable
int k = 1;
int32_t position_1 = 0;
int32_t position_2 = 0;

//Callback function
void callback(const std_msgs::Int32::ConstPtr &msg)
{
	dis = *msg;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "motor");
	ros::NodeHandle nh;

	uint8_t dxl_error = 0;
	int dxl_result = COMM_TX_FAIL;

	dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(devicename);

	dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(protocol_version);
	
	portHandler->openPort();

	portHandler->setBaudRate(baudrate);

	dxl_result = packetHandler->write1ByteTxRx(portHandler, dxl_id_1, torque_enable, enable, &dxl_error);

	dxl_result = packetHandler->write1ByteTxRx(portHandler, dxl_id_2, torque_enable, enable, &dxl_error);
	

  	ros::Subscriber sub_dis = nh.subscribe("/dispensor_1", 10, callback);

  	ros::Rate loop_rate(10);
	
	
	//ROS_INFO("%d", dxl_error);
	while(ros::ok())
	{
		dxl_result = packetHandler->write4ByteTxRx(portHandler, dxl_id_1 ,goal_velocity, 2400, &dxl_error);
		dxl_result = packetHandler->write4ByteTxRx(portHandler, dxl_id_2 ,goal_velocity, 2400, &dxl_error);
	
		if (dis.data == 1)
		{
			if (k == 1)
			{
				dxl_wb.goalPosition(dxl_id_1, (int32_t)1500);
				dxl_wb.goalPosition(dxl_id_2, (int32_t)1500);
				if (position_1 == 1500 and position_2 == 1500)
				{
					k = 2;
				}
			}
			else if (k ==2)
			{
				dxl_wb.goalPosition(dxl_id_1, (int32_t)2500);
				dxl_wb.goalPosition(dxl_id_2, (int32_t)2500);
				if (position_1 == 2500 and position_2 == 2500)
				{
					k = 3;
				}	
			}
			else
			{
				dxl_wb.goalPosition(dxl_id_1, (int32_t)2048);
				dxl_wb.goalPosition(dxl_id_2, (int32_t)2048);
			}
		}
		else
		{
			dxl_wb.goalPosition(dxl_id_1, (int32_t)2048);
			dxl_wb.goalPosition(dxl_id_2, (int32_t)2048);
			k = 1;
		}
		loop_rate.sleep();
		ros::spinOnce();
	}
	return 0;
}	











