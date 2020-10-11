#include "swarm_name.h"
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <IMU.h>
#include <geometry_msgs/Twist.h>
#include <DynamixelWorkbench.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


void sensor_inti(void);
void imu_init(void);
void updateMotorEncoder(void);
void updateMotorVel(void);
void updateOdom(double t);
void updateImu(void);
void updateTF(void);
void getIMU(void);
void getYaw(void);
void updateObject(void);
void updateDistance(void);
void encoder_init(void);
void Set_GoalVelocity(const geometry_msgs::Twist& cmd_vel_msg);
void publishOdom(double t);
void publishIMU(void);
void publishDistacne(void);
void publishObject(void);
void publishYaw(void);
void set_motor(void);
void publishEncoder(void);
DynamixelWorkbench dxl_wb;
ros::NodeHandle nh;

cIMU imu;

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel_1", Set_GoalVelocity);


std_msgs::Float32 yaw_msg;
ros::Publisher yaw_pub("yaw_1", &yaw_msg);

std_msgs::Int32 encoderRR;
std_msgs::Int32 encoderLL;
ros::Publisher L_pub("encoderL_1", &encoderLL);
ros::Publisher R_pub("encoderR_1", &encoderRR);

nav_msgs::Odometry odom_msg;
geometry_msgs::Quaternion odom_quat;
ros::Publisher odom_pub("odom_1", &odom_msg);

sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu_1", &imu_msg);

std_msgs::Float32 dis_msg;
ros::Publisher dis_pub("dis_1", &dis_msg);

std_msgs::Int32 object_msg;
ros::Publisher object_pub("object_1", &object_msg);

geometry_msgs::TransformStamped odom_tf;
tf::TransformBroadcaster tf_broadcaster;
