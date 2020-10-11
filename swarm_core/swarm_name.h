#define wheel_radius 0.20724
#define wheel_base 0.1282

#define dxl_id_1   1
#define dxl_id_2   2

#if defined(__OPENCM904__)
  #define DEVICE_NAME "3" //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
#elif defined(__OPENCR__)
  #define DEVICE_NAME ""
#endif  

bool result = false;

#define motor_control_freqeuncy 100
#define sensor_control_freqeuncy 50

#define accel_factor 0.000598550415
#define gyro_factor 0.0010642

#define button1_pin  2
#define button2_pin  3
#define distance_pin  A0

const char *log_err;
  
int32_t encoderR = 0;
int32_t encoderL = 0;
double prev_encoderR = 0;
double prev_encoderL = 0;
double goal_velocity_x = 0;
double goal_velocity_z = 0;
uint16_t model_number = 0;
  
double velR = 0;
double velL = 0;
double vel_R = 0;
double vel_L = 0;

double x_dist = 0;
double y_dist = 0;
float angular = 0;

double Time[2] = {0, };

double yaw_e = 0;
double yaw_e_p = 0;
