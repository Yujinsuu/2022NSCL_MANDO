#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <MsTimer2.h>
#include <SPI.h>

// Front Motor Drive
#define MOTOR1_PWM 2
#define MOTOR1_IN1 3
#define MOTOR1_IN2 4
//뒷모터
#define MOTOR2_PWM 5
#define MOTOR2_IN1 6
#define MOTOR2_IN2 7
/////////////////////////////  Steering PID 제어 ////////////////////////////////
#define Steering_Sensor A15  // Analog input pin that the potentiometer is attached to
#define NEURAL_ANGLE -2
#define LEFT_STEER_ANGLE -25
#define RIGHT_STEER_ANGLE 25
#define MOTOR3_PWM 8
#define MOTOR3_IN1 9
#define MOTOR3_IN2 10

#define ENC1_ADD 22
#define ENC2_ADD 23

void cmd_vel_callback(const geometry_msgs::Twist& msg);

ros::NodeHandle nh;
geometry_msgs::Twist cmd_vel; //모터명령 수신을 위한 변수(수신)
std_msgs::Int32 encoder_data1; //모터 엔코더1값 전달을 위한 변수(송신)
std_msgs::Int32 encoder_data2; //모터 엔코더2값 전달을 위한 변수(송신)
std_msgs::Int32 Target_value; //Steering 목표값
std_msgs::Int32 Present_value; //Steering 현재값


ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", cmd_vel_callback);
ros::Publisher cmd_pub("cmd_vel2", &cmd_vel);
ros::Publisher encoder_pub1("encoder1", &encoder_data1);
ros::Publisher encoder_pub2("encoder2", &encoder_data2);
ros::Publisher target_pub("target_val", &Target_value);
ros::Publisher present_pub("present_val", &Present_value);


float Kp = 1.7;
float Ki = 1.5;
float Kd = 7.0; //PID 상수 설정, 실험에 따라 정해야 함 중요!
double Setpoint, Input, Output; //PID 제어 변수
double realError, error_old;
double accError, errorGap;
int pwm_output;

int sensorValue = 0;        // value read from the pot
int Steer_Angle_Measure = 0;        // value output to the PWM (analog out)
int Steering_Angle = NEURAL_ANGLE;


void setup() {
  pinMode(13, OUTPUT);
  // Front Motor Drive Pin Setup
  pinMode(MOTOR1_PWM, OUTPUT);
  pinMode(MOTOR1_IN1, OUTPUT);  // L298 motor control direction
  pinMode(MOTOR1_IN2, OUTPUT);

  // Rear Motor Drive Pin Setup
  pinMode(MOTOR2_PWM, OUTPUT);
  pinMode(MOTOR2_IN1, OUTPUT);  // L298 motor control direction
  pinMode(MOTOR2_IN2, OUTPUT);

  initEncoders();          // initialize encoder
  clearEncoderCount(1); 
  clearEncoderCount(2); 
  
  //Steer
  pinMode(MOTOR3_PWM, OUTPUT);
  pinMode(MOTOR3_IN1, OUTPUT);  // L298 motor control direction
  pinMode(MOTOR3_IN2, OUTPUT);  // L298 motor control PWM

  realError = accError = errorGap = error_old = 0.0;
  pwm_output = 0;
  
  nh.initNode();
  nh.subscribe(cmd_sub); // subscribing cmd_vel1
  nh.advertise(cmd_pub); // publilshing cmd_vel2
  nh.advertise(encoder_pub1);
  nh.advertise(encoder_pub2);
  nh.advertise(target_pub);
  nh.advertise(present_pub);
}


int f_speed = 0, r_speed = 0, brake = 0;
int velocity = 0, input_velocity = 0, vel_gap = 0;
int steer_angle = 0, input_steer = 0, steer_gap = 0;
int brake_time = 0;

void cmd_vel_callback(const geometry_msgs::Twist& msg) {
  input_velocity = (int)msg.linear.x;
  
  input_steer = (int)msg.angular.z;

  brake = (int)msg.linear.z;
}


void control_callback()
{
  static boolean output = HIGH;
  
  digitalWrite(13, output);
  output = !output;

  vel_gap = input_velocity - velocity;
  steer_gap = input_steer - steer_angle;
  
  if(brake == 0) // 일반 주행
  {
    brake_time=0;
    if(fabs(vel_gap) > 0 && fabs(vel_gap) < 5)//속도제어
    {
      velocity += vel_gap;
    }
    else if(vel_gap >=  5) velocity += 5;
    else if(vel_gap <= -5) velocity -= 5;
  }
  else if (brake == 2) // 후진 브레이크
  {
    if (brake_time >= 50) velocity=0;
    else
    {
      velocity = 40;
      brake_time += 1;
    }
  }
  else if (brake == 3) // 저속 브레이크
  {
    if (brake_time >= 50) velocity=0;
    else
    {
      velocity = -40;
      brake_time += 1;
    }
  }
  else // 일반 브레이크
  {
    if (brake_time >= 100) velocity=0;
    else if (brake_time >= 30)
    {
      velocity = -20;
      brake_time += 1;
    }
    else 
    {
      velocity = -80;
      brake_time += 1;
    }
  }
    
  if(velocity >= 255) velocity = 255; //pwm 최고값제한
  else if(velocity <=-255) velocity = -255; //pwm 최고값제한
  
  f_speed = r_speed = velocity;
  motor_control(f_speed, r_speed);
  if(brake == 0)
  {
    if(fabs(steer_gap)>0 && fabs(steer_gap)<25)//조향제어
    {
     steer_angle += steer_gap;
    }
    else if(steer_gap > 20)
    {
     steer_angle += 20;
    }
    else if(steer_gap < -20)
    {
     steer_angle -= 20;
    }
  }
  else steer_angle = 0;
  
  // read the analog in value:
  sensorValue = analogRead(Steering_Sensor);
  // map it to the range of the analog out:
  Steer_Angle_Measure = map(sensorValue, 50, 1050, LEFT_STEER_ANGLE, RIGHT_STEER_ANGLE);
  Steering_Angle = NEURAL_ANGLE + steer_angle;
  steering_control();  
}


signed long encoder1count = 0;
signed long encoder2count = 0;

void initEncoders() {
  
  // Set slave selects as outputs
  pinMode(ENC1_ADD, OUTPUT);
  pinMode(ENC2_ADD, OUTPUT); 
  
  // Raise select pins
  // Communication begins when you drop the individual select signsl
  digitalWrite(ENC1_ADD,HIGH);
  digitalWrite(ENC2_ADD,HIGH);
 
  SPI.begin();
  
  // Initialize encoder 1
  //    Clock division factor: 0
  //    Negative index input
  //    free-running count mode
  //    x4 quatrature count mode (four counts per quadrature cycle)
  // NOTE: For more information on commands, see datasheet
  digitalWrite(ENC1_ADD,LOW);        // Begin SPI conversation
  SPI.transfer(0x88);                       // Write to MDR0
  SPI.transfer(0x03);                       // Configure to 4 byte mode
  digitalWrite(ENC1_ADD,HIGH);       // Terminate SPI conversation 

  // Initialize encoder 2
  //    Clock division factor: 0
  //    Negative index input
  //    free-running count mode
  //    x4 quatrature count mode (four counts per quadrature cycle)
  // NOTE: For more information on commands, see datasheet
  digitalWrite(ENC2_ADD,LOW);        // Begin SPI conversation
  SPI.transfer(0x88);                       // Write to MDR0
  SPI.transfer(0x03);                       // Configure to 4 byte mode
  digitalWrite(ENC2_ADD,HIGH);       // Terminate SPI conversation 
}

long readEncoder(int encoder_no) 
{  
  // Initialize temporary variables for SPI read
  unsigned int count_1, count_2, count_3, count_4;
  long count_value;   
  
  digitalWrite(ENC1_ADD + encoder_no-1,LOW);      // Begin SPI conversation
   // digitalWrite(ENC4_ADD,LOW);      // Begin SPI conversation
  SPI.transfer(0x60);                     // Request count
  count_1 = SPI.transfer(0x00);           // Read highest order byte
  count_2 = SPI.transfer(0x00);           
  count_3 = SPI.transfer(0x00);           
  count_4 = SPI.transfer(0x00);           // Read lowest order byte
  digitalWrite(ENC1_ADD+encoder_no-1,HIGH);     // Terminate SPI conversation 
  //digitalWrite(ENC4_ADD,HIGH);      // Begin SPI conversation
// Calculate encoder count
  count_value= ((long)count_1<<24) + ((long)count_2<<16) + ((long)count_3<<8 ) + (long)count_4;
  
  return count_value;
}

void clearEncoderCount(int encoder_no) {    
  // Set encoder1's data register to 0
  digitalWrite(ENC1_ADD+encoder_no-1,LOW);      // Begin SPI conversation  
  // Write to DTR
  SPI.transfer(0x98);    
  // Load data
  SPI.transfer(0x00);  // Highest order byte
  SPI.transfer(0x00);           
  SPI.transfer(0x00);           
  SPI.transfer(0x00);  // lowest order byte
  digitalWrite(ENC1_ADD+encoder_no-1,HIGH);     // Terminate SPI conversation 
  
  delayMicroseconds(100);  // provides some breathing room between SPI conversations
  
  // Set encoder1's current data register to center
  digitalWrite(ENC1_ADD+encoder_no-1,LOW);      // Begin SPI conversation  
  SPI.transfer(0xE0);    
  digitalWrite(ENC1_ADD+encoder_no-1,HIGH);     // Terminate SPI conversation 
}


void motor_control(int motor1_pwm,int motor2_pwm)
{
   if (motor1_pwm > 0) // forward
  {
    digitalWrite(MOTOR1_IN1, HIGH);
    digitalWrite(MOTOR1_IN2, LOW);
    analogWrite(MOTOR1_PWM, motor1_pwm);

  }
  else if (motor1_pwm < 0) // backward
  {

    digitalWrite(MOTOR1_IN1, LOW);
    digitalWrite(MOTOR1_IN2, HIGH);
    analogWrite(MOTOR1_PWM, -motor1_pwm);
  }
  else
  {
    digitalWrite(MOTOR1_IN1, LOW);
    digitalWrite(MOTOR1_IN2, LOW);
    digitalWrite(MOTOR1_PWM, 0);
  }
  
  if (motor2_pwm > 0) // forward
  {
    digitalWrite(MOTOR2_IN1, HIGH);
    digitalWrite(MOTOR2_IN2, LOW);
    analogWrite(MOTOR2_PWM, motor2_pwm);

  }
  else if (motor2_pwm < 0) // backward
  {

    digitalWrite(MOTOR2_IN1, LOW);
    digitalWrite(MOTOR2_IN2, HIGH);
    analogWrite(MOTOR2_PWM, -motor2_pwm);
  }
  else
  {
    digitalWrite(MOTOR2_IN1, LOW);
    digitalWrite(MOTOR2_IN2, LOW);
    digitalWrite(MOTOR2_PWM, 0);
  }
}


void steer_motor_control(int motor_pwm)
{
  if (motor_pwm > 0) // forward
  {
    digitalWrite(MOTOR3_IN1, LOW);
    digitalWrite(MOTOR3_IN2, HIGH);
    analogWrite(MOTOR3_PWM, motor_pwm);
  }
  else if (motor_pwm < 0) // backward
  {
    digitalWrite(MOTOR3_IN1, HIGH);
    digitalWrite(MOTOR3_IN2, LOW);
    analogWrite(MOTOR3_PWM, -motor_pwm);
  }
  else // stop
  {
    digitalWrite(MOTOR3_IN1, LOW);
    digitalWrite(MOTOR3_IN2, LOW);
    analogWrite(MOTOR3_PWM, 0);
  }
}


void steering_control()
{
  if (Steering_Angle <= LEFT_STEER_ANGLE + NEURAL_ANGLE)  Steering_Angle = LEFT_STEER_ANGLE + NEURAL_ANGLE;
  if (Steering_Angle >= RIGHT_STEER_ANGLE + NEURAL_ANGLE)  Steering_Angle = RIGHT_STEER_ANGLE + NEURAL_ANGLE;
  PID_Control();
}


void PID_Control()
{
  realError = input_steer - Steer_Angle_Measure;  // steering error 계산
  //realError = Steering_Angle - Steer_Angle_Measure;
  accError += realError;
  errorGap = realError - error_old;
  accError = (accError >=  80) ?  80 : accError;
  accError = (accError <= -80) ? -80 : accError;

  pwm_output = Kp * realError + Kd * errorGap + Ki * accError;
  pwm_output = (pwm_output >=  255) ?  255 : pwm_output;
  pwm_output = (pwm_output <= -255) ? -255 : pwm_output;

  if (fabs(realError) <= 1)
  {
    steer_motor_control(0);
    accError = 0;
  }
  else steer_motor_control(pwm_output);
  error_old = realError;  
}


void loop() {
  
  encoder1count = readEncoder(1);
  encoder2count = readEncoder(2);

  encoder_data1.data = brake_time;
  encoder_data2.data = encoder2count;
  
  encoder_pub1.publish(&encoder_data1);
  encoder_pub2.publish(&encoder_data2);
  
  Target_value.data = int(input_steer); //목표값
  //Target_value.data = int(Steering_Angle); //목표값
  Present_value.data = int(Steer_Angle_Measure); //현재값

  target_pub.publish(&Target_value);
  present_pub.publish(&Present_value);
  
  cmd_vel.linear.x = velocity;
  cmd_vel.angular.z = steer_angle;
  cmd_pub.publish(&cmd_vel);

  control_callback();
  delay(10);
  
  nh.spinOnce();
}
