#include "swarm_header.h"

void setup()
{
  result = dxl_wb.init(DEVICE_NAME, 115200, &log_err);

  result = dxl_wb.ping(dxl_id_1, &model_number, &log_err);
  
  result = dxl_wb.ping(dxl_id_2, &model_number, &log_err);

  result = dxl_wb.wheelMode(dxl_id_1, 0, &log_err);
  
  result = dxl_wb.wheelMode(dxl_id_2, 0, &log_err);
  nh.initNode();
  nh.getHardware()->setBaud(115200);

  nh.subscribe(cmd_vel_sub);

  nh.advertise(odom_pub);
  nh.advertise(imu_pub);
  nh.advertise(dis_pub);
  nh.advertise(object_pub);
  nh.advertise(yaw_pub);
  nh.advertise(R_pub);
  nh.advertise(L_pub);
  motor_init();
  sensor_init();

  encoder_init();
}

void loop()
{
  uint32_t t = millis();

  if ((t - Time[0]) >= motor_control_freqeuncy)
  {
    publishIMU();
    publishOdom((t - Time[0])/1000);
    publishYaw();
    set_motor();
    Time[0] = t;
  }
  if ((t - Time[1]) >= sensor_control_freqeuncy)
  { 
    publishDistance();
    publishObject();
    
    Time[1] = t;
  }
  updateIMU();
  
  nh.spinOnce();
}

void Set_GoalVelocity(const geometry_msgs::Twist& cmd_vel_msg)
{
  goal_velocity_x = cmd_vel_msg.linear.x;
  goal_velocity_z = cmd_vel_msg.angular.z;

  //속도 -> 각 바퀴별 속도
  vel_R = (goal_velocity_x - goal_velocity_z * wheel_base / 2) * 100;
  vel_L = (goal_velocity_x + goal_velocity_z * wheel_base / 2) * 100;
}

void publishOdom(double t)
{
  updateMotorEncoder();
  updateMotorVel(t);
  updateOdom(t);
  updateTF();
  odom_pub.publish(&odom_msg);
}

void publishIMU(void)
{
  getIMU();
  imu_pub.publish(&imu_msg);
}

void publishDistance(void)
{
  updateDistance();
  dis_pub.publish(&dis_msg);
}

void publishObject(void)
{
  updateObject();
  object_pub.publish(&object_msg);
}

void publishYaw(void)
{
  getYaw();
  yaw_pub.publish(&yaw_msg);
}
void encoder_init(void)
{
  result = dxl_wb.itemRead(dxl_id_1, "Present_Position", &encoderR, &log_err);
  result = dxl_wb.itemRead(dxl_id_2, "Present_Position", &encoderL, &log_err);
  encoderR = -encoderR;
  prev_encoderR = encoderR;
  prev_encoderL = encoderL;
}

void sensor_init(void)
{
  imu_init();

  pinMode(button1_pin, OUTPUT);
  pinMode(button2_pin, INPUT);

  pinMode(distance_pin, INPUT);
}

void imu_init(void)
{
   imu.begin();
}

void set_motor(void)
{
  dxl_wb.goalVelocity(dxl_id_1, -(int32_t)vel_R);
  dxl_wb.goalVelocity(dxl_id_2, (int32_t)vel_L);
}
void updateMotorVel(double t)
{
  velR = (encoderR - prev_encoderR) * wheel_radius / t / 4096;
  velL = (encoderL - prev_encoderL) * wheel_radius / t / 4096;

  prev_encoderR = encoderR;
  prev_encoderL = encoderL;
}

void updateMotorEncoder(void)
{
  result = dxl_wb.itemRead(dxl_id_1, "Present_Position", &encoderR, &log_err);
  result = dxl_wb.itemRead(dxl_id_2, "Present_Position", &encoderL, &log_err);
  encoderR = -encoderR;
  publishEncoder();
}


void updateOdom(double t)
{
  double linear_vel = (velL + velR) / 2;
  double angular_vel = (velR - velL) / wheel_base;

  double distance_delta = linear_vel * t;
  double angular_delta = angular_vel * t;

  x_dist = x_dist + distance_delta * cos(angular + angular_delta / 2);
  y_dist = y_dist + distance_delta * sin(angular + angular_delta / 2);

  angular += angular_delta;

  odom_msg.header.stamp = nh.now();
  odom_msg.header.frame_id = "odom";
  odom_msg.pose.pose.position.x = x_dist;
  odom_msg.pose.pose.position.y = y_dist;
  odom_msg.pose.pose.position.z = 0;
  odom_msg.pose.pose.orientation = tf::createQuaternionFromYaw(angular);

  odom_msg.child_frame_id = "twist";
  odom_msg.twist.twist.linear.x = linear_vel;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.angular.z = angular_vel;
}

void updateTF(void)
{
}

void updateIMU(void)
{
  imu.update();
}

void getIMU(void)
{
  imu_msg.angular_velocity.x = imu.gyroData[0] * gyro_factor;
  imu_msg.angular_velocity.y = imu.gyroData[1] * gyro_factor;
  imu_msg.angular_velocity.z = imu.gyroData[2] * gyro_factor;

  imu_msg.linear_acceleration.x = imu.accData[0] * accel_factor;
  imu_msg.linear_acceleration.y= imu.accData[1] * accel_factor;
  imu_msg.linear_acceleration.z = imu.accData[2] * accel_factor;

  imu_msg.orientation.w = imu.quat[0];
  imu_msg.orientation.x = imu.quat[1];
  imu_msg.orientation.y = imu.quat[2];
  imu_msg.orientation.z = imu.quat[3];
}

void getYaw(void)
{
  yaw_msg.data = imu.rpy[2];
}

void updateDistance(void)
{
  int a = analogRead(distance_pin);
  int v = map(a, 0, 1023, 0, 5000);
  dis_msg.data = a;
}

void updateObject(void)
{ 
  digitalWrite(button2_pin, HIGH);
  int a = digitalRead(button1_pin);
  
  if (a == 1)
  {
    object_msg.data  = 1;
  }
}

void publishEncoder(void)
{ 
  encoderRR.data = encoderR;
  encoderLL.data = encoderL;
  R_pub.publish(&encoderRR);
  L_pub.publish(&encoderLL);
}
