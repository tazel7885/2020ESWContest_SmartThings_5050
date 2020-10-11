/***********************************************************

Author : HY-mec
Date : 2020.10.10

***********************************************************/


//header file

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int32MultiArray.h>

//viriable
std_msgs::Int32 map_1;
std_msgs::Int32 map_2;
std_msgs::Int32 map_3;
std_msgs::Int32 map_4;
std_msgs::Int32 map_5;

std_msgs::Int32 goal;
std_msgs::Int32MultiArray array;

std_msgs::Int32 go;
int size = 0;
int order[100] = {0,};
int robot[3] = {1,1,1};
int able = 0;
int work = 0;


//Callback function

void Callback_goal_1(const std_msgs::Int32::ConstPtr &msg)
{
	goal = *msg;
	if( goal.data == 1)
	{
		robot[0] = 1;
	}
	else
	{
		robot[0] = 0;
	}
}

void Callback_goal_2(const std_msgs::Int32::ConstPtr &msg)
{
	goal = *msg;
	if( goal.data == 1)
	{
		robot[1] = 1;
	}
	else
	{
		robot[1] = 0;
	}
}
void Callback_goal_3(const std_msgs::Int32::ConstPtr &msg)
{
	goal = *msg;
	if( goal.data == 1)
	{
		robot[2] = 1;
	}
	else
	{
		robot[2] = 0;
	}
}

void Callback_order(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
	array = *msg;
	size = array.data[0];
	for( int i = 0; i < size; i++)
	{
		order[i] = array.data[i+1];
	}
}

void Callback_go(const std_msgs::Int32::ConstPtr &msg)
{
	go = *msg;
}


//update function
void find(void)
{
	for( int i = 0; i < 5; i++)
	{
		if( robot[i] == 1)
		{
			able = i+1;
			break;
		}
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "order");
	ros::NodeHandle nh;

	ros::Publisher pub_map_1 = nh.advertise<std_msgs::Int32>("/map_1",1);
	ros::Publisher pub_map_2 = nh.advertise<std_msgs::Int32>("/map_2",1);
	ros::Publisher pub_map_3 = nh.advertise<std_msgs::Int32>("/map_3",1);

	ros::Subscriber sub_goal_1 = nh.subscribe("/goal_1", 1, Callback_goal_1);
	ros::Subscriber sub_goal_2 = nh.subscribe("/goal_2", 1, Callback_goal_2);
	ros::Subscriber sub_goal_3 = nh.subscribe("/goal_3", 1, Callback_goal_3);

	ros::Subscriber sub_go = nh.subscribe("/go_2",1,Callback_go);
	ros::Subscriber sub_order = nh.subscribe("/order", 1, Callback_order);
	
	ros::Rate loop_rate(10);

	while(ros::ok())
	{
		if( go.data == 1)
		{		
			if( work != size)
			{		
				find();
				if( able == 1)
				{
					ROS_INFO("1");
					map_1.data = order[work];
					robot[0] = 0;
					pub_map_1.publish(map_1);
					work++;
				}

				else if( able == 2)
				{
					ROS_INFO("2");
					map_2.data = order[work];
					pub_map_2.publish(map_2);
					robot[1] = 0;
					work++;
				}
				else if( able == 3)
				{
					ROS_INFO("3");
					map_3.data = order[work];
					pub_map_3.publish(map_3);
					robot[2] = 0;
					work++;
				}
			}
		}
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}




















