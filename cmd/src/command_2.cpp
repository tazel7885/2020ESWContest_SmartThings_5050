/***********************************************************

Author : HY-mec
Date : 2020.10.10

***********************************************************/


//header file
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <math.h>

//viriable
geometry_msgs::Twist cmd;
nav_msgs::Odometry position;
nav_msgs::Odometry odom;
sensor_msgs::Imu imu;
std_msgs::Float32 yaw;
std_msgs::Int32 map;
std_msgs::Int32 go;
std_msgs::Int32 goal;
std_msgs::Int32 dis;
std_msgs::Int32 object;
int point = -1;
int check = 0;
double map1[2][4];
double map2[2][4];
double map3[2][4];
double limit[3];
double offset[2] = {0.09, 0.1};

//Callback function
void Callback_imu(const sensor_msgs::Imu::ConstPtr &msg)
{
	imu = *msg;
}

void Callback_odom(const nav_msgs::Odometry::ConstPtr &msg)
{
	odom = *msg;
	odom.pose.pose.position.x = odom.pose.pose.position.x;
	odom.pose.pose.position.y = odom.pose.pose.position.y;
}

void Callback_yaw(const std_msgs::Float32::ConstPtr &msg)
{
	yaw = *msg;
}

void Callback_map(const std_msgs::Int32::ConstPtr &msg)
{
	map = *msg;
}

void Callback_go(const std_msgs::Int32::ConstPtr &msg)
{
	go = *msg;
}

void Callback_object(const std_msgs::Int32::ConstPtr &msg)
{
	object = *msg;
}

void Callback_dis(const std_msgs::Int32::ConstPtr &msg)
{
	dis = *msg;
}

//update function
void map_update(int map, int point)
{		
	if( map == 1)
	{
		if( limit[0] == point)
		{
			goal.data = 1;
		}
		else if( point == 2 and object.data == 0)
		{
			dis.data = 1;
			check = 1;
		}		
		else
		{	
			dis.data = 0;
			check = 0;
			point++;
			position.pose.pose.position.x = map1[point][0] + offset[0];
			position.pose.pose.position.y = map1[point][1] + offset[1];
			goal.data = 0;
		}	
	}
	else if( map == 2)
	{
		if( limit[1] == point)
		{
			goal.data = 1;
		}
		else if( point == 2 and object.data == 0)
		{
			dis.data = 1;
			check = 2;
		}
		else
		{	
			dis.data = 0;
			check = 0;
			point++;	
			position.pose.pose.position.x = map2[point][0] + offset[0];
			position.pose.pose.position.y = map2[point][1] + offset[1];
			goal.data = 0;
		}
	}
	else if ( map == 3)
	{
		if( limit[2] == point)
		{
			goal.data = 1;
		}
		else if( point == 2 and object.data == 0)
		{
			dis.data = 1;
			check = 3;
		}
		else
		{	
			dis.data = 0;
			check = 0;
			point++;
			position.pose.pose.position.x = map3[point][0] + offset[0];
			position.pose.pose.position.y = map3[point][1] + offset[1];
			goal.data = 0;
		}
	}
	else
	{
		goal.data = 1;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "hello");

	ros::NodeHandle nh;
	nh.getParam("map1/point1_x", map1[0][0]);
	nh.getParam("map1/point1_y", map1[0][1]);
	nh.getParam("map1/point2_x", map1[1][0]);
	nh.getParam("map1/point2_y", map1[1][1]);
	nh.getParam("map1/point3_x", map1[2][0]);
	nh.getParam("map1/point3_y", map1[2][1]);
	nh.getParam("map1/point4_x", map1[3][0]);
	nh.getParam("map1/point4_y", map1[3][1]);
	
	nh.getParam("map2/point1_x", map2[0][0]);
	nh.getParam("map2/point1_y", map2[0][1]);
	nh.getParam("map2/point2_x", map2[1][0]);
	nh.getParam("map2/point2_y", map2[1][1]);
	nh.getParam("map2/point3_x", map2[2][0]);
	nh.getParam("map2/point3_y", map2[2][1]);
	nh.getParam("map2/point4_x", map1[3][0]);
	nh.getParam("map2/point4_y", map1[3][1]);

	nh.getParam("map3/point1_x", map3[0][0]);
	nh.getParam("map3/point1_y", map3[0][1]);
	nh.getParam("map3/point2_x", map3[1][0]);
	nh.getParam("map3/point2_y", map3[1][1]);
	nh.getParam("map3/point3_x", map3[2][0]);
	nh.getParam("map3/point3_y", map3[2][1]);
	nh.getParam("map3/point4_x", map1[3][0]);
	nh.getParam("map3/point4_y", map1[3][1]);

	nh.getParam("map1/cols", limit[0]);
	nh.getParam("map2/cols", limit[1]);
	nh.getParam("map3/cols", limit[2]);

	ros::Publisher pub_cmd = nh.advertise<geometry_msgs::Twist>("/b/cmd_vel_2", 1);
	ros::Publisher pub_goal = nh.advertise<std_msgs::Int32>("/goal_2", 1);
	ros::Publisher pub_dis_1 = nh.advertise<std_msgs::Int32>("/dispenser_1",1);
	ros::Publisher pub_dis_2 = nh.advertise<std_msgs::Int32>("/dispenser_2",1);
	ros::Publisher pub_dis_3 = nh.advertise<std_msgs::Int32>("/dispenser_3",1);

	ros::Subscriber sub_odom = nh.subscribe("/b/odom_2", 1, Callback_odom);
	ros::Subscriber sub_imu = nh.subscribe("/b/imu_2", 1, Callback_imu);
	ros::Subscriber sub_yaw = nh.subscribe("/b/yaw_2", 1, Callback_yaw);
	ros::Subscriber sub_map = nh.subscribe("/map_2", 1, Callback_map);
	ros::Subscriber sub_go = nh.subscribe("/go", 1, Callback_go);
	ros::Subscriber sub_ob = nh.subscribe("/b/object_2", 1,Callback_object);
	ros::Subscriber sub_dis = nh.subscibe("/a/dis_1", 1, Callback_dis);

	ros::Rate loop_rate(10);
	
	while(ros::ok())
	{
		if( go.data == 0)
		{
			cmd.linear.x = 0;
			cmd.angular.z = 0;
			point = -1;
		}
		else
		{
			if( dis.data < 30)
			{
				cmd.linear.x = 0;
				cmd.angular.z = 0;
			}
			else
			{
				if( odom.pose.pose.position.x - position.pose.pose.position.x < -0.01)
				{
					if( yaw.data > 5)
					{
						cmd.linear.x = 0;
						cmd.angular.z = 1;
				
				
					}
					else if( yaw.data < -5)
					{
						cmd.linear.x = 0;
						cmd.angular.z = -1;
				
					}
					else
					{
						cmd.linear.x = 0.5;
						cmd.angular.z = 0;
				
					}
				}	
				else if( odom.pose.pose.position.x - position.pose.pose.position.x > 0.01)
				{
					if ( yaw.data > -175)
					{
						cmd.linear.x = 0;
						cmd.angular.z = 1;
				
					}
					else if( yaw.data < 175)
					{
						cmd.linear.x = 0;
						cmd.angular.z = -1;
				
					}
					else
					{
						cmd.linear.x = 0.5;
						cmd.angular.z = 0;
				
					}
				}
				else if( odom.pose.pose.position.y - position.pose.pose.position.y < -0.01)
				{
					if( yaw.data > 95)
					{
						cmd.linear.x = 0;
						cmd.angular.z = 1;
				
					}
					else if( yaw.data < 85.0)
					{
						cmd.linear.x = 0;
						cmd.angular.z = -1;
				
					}
					else
					{
						cmd.linear.x = 0.5;
						cmd.angular.z = 0;
				
					}
				}
				else if( odom.pose.pose.position.y - position.pose.pose.position.y > 0.01)
				{
					if( yaw.data < -95)
					{
						cmd.linear.x = 0;
						cmd.angular.z = -1;
				
					}
					else if( yaw.data > -85)
					{
						cmd.linear.x = 0;
						cmd.angular.z = 1;
					}
					else
					{
						cmd.linear.x = 0.5;
						cmd.angular.z = 0;
				
					}
				}
				else
				{
					cmd.linear.x = 0;
					cmd.angular.z = 0;
					map_update(map.data, point);			
				}
			}
		}
		if (check == 1)
		{
			pub_dis_1.publish(dis);
		}
		else if(check == 2)
		{
			pub_dis_2.publish(dis);
		}
		else if(check ==3)
		{
			pub_dis_3.publish(dis);
		}
		pub_goal.publish(goal);
		pub_cmd.publish(cmd);
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}

