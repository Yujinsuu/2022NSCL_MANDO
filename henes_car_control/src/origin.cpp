#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <sstream>
#include <time.h>
#include "ackermann_msgs/AckermannDriveStamped.h"

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)

// unit m
#define Sonar_Detect_MAX_Range 3.0
#define Sonar_Obstacle_Range 0.5
#define Lidar_Obstacle_Range 2.0

// Steering Control
#define Neutral_Angle_Offset 0
#define Right_MAX 30
#define Left_MAX -30


geometry_msgs::Twist teleop_cmd_vel_data;
std_msgs::Int16 auto_mode;

int auto_drive = 0; // 0 : controller mode, 1 : auto-drive mode, 2 : something detected, 3 : stop mode, 4 : parking mode
int lidar_scan = 0;
int sonar_scan = 0;
int camera_scan = 0;
float sonar[3]={0,0,0};

int tf_flag = 0;
int steering_angle = 0;
int steering_angle_cmd = 0;
int motor_speed = 0;
int motor_speed_cmd = 0;
int brake = 0;


void CarControlCallback(const geometry_msgs::Twist& msg)
{
    //ROS_INFO("I heard: [%s]", msg->data.c_str());

    steering_angle = (int)(msg.angular.z);

    if(steering_angle <= Left_MAX) steering_angle = Left_MAX;
    if(steering_angle >= Right_MAX) steering_angle = Right_MAX;

    motor_speed = (int)msg.linear.x;
    if(motor_speed >= 255) motor_speed = 255;
    if(motor_speed <= -255) motor_speed = -255;

    if((int)msg.linear.y == 1) auto_drive=0; //press 'c'
    if((int)msg.linear.z == 1) auto_drive=1; //press 'e'
}

void AutoControlCallback(const ackermann_msgs::AckermannDriveStamped& cmd)
{
    if(auto_drive == 1)
    {
        steering_angle = int(cmd.drive.steering_angle);


    	if(steering_angle <= Left_MAX) steering_angle = Left_MAX;
	    if(steering_angle >= Right_MAX) steering_angle = Right_MAX;

    	motor_speed = int(cmd.drive.speed);
    	if(motor_speed >= 255) motor_speed = 255;
    	if(motor_speed <= -255) motor_speed = -255;
    }
}

int now = 0, stop_time = 0, parking_time = 0;
int stop_speed = 0, stop_steer = 0;
int stop_flag = 0;
int parking_speed = 0, parking_steer = 0;
int parking_flag = 0;

void STtimer()
{
    now = (unsigned)time(NULL);
    int time_gap = now - stop_time;
    if(time_gap < 5)
    {
        stop_speed = 20;
        stop_steer = 0;
    }
    else auto_drive = 1;
}

void PKtimer()
{
    now = (unsigned)time(NULL);
    int time_gap = now - parking_time;
    if(time_gap < 5)
    {
        parking_speed = 0;
        parking_steer = 0;
    }
    else auto_drive = 1;
}

void PathCallback(const std_msgs::Int32& path)
{
    if(auto_drive != 0)
    {
        if(path.data == 1) tf_flag = 1; // 직진 신호등 구간
        else if(path.data == 2) tf_flag = 2; // 좌회전 신호등 구간
        else if((stop_flag == 0) && (path.data == 3) && (auto_drive != 3)) // 정차 구간
        {
            auto_drive = 3;
            stop_time = (unsigned)time(NULL);
            stop_flag = 1;
        }
        else if((parking_flag == 0) && (path.data == 4) && (auto_drive != 4)) // 주차 구간
        {
            auto_drive = 4;
            parking_time = (unsigned)time(NULL);
            parking_flag = 1;
        }
        else tf_flag = 0; 
    }
}

void TFlightCallback(const std_msgs::Int32MultiArray& light) // -1 : None, 0 : green, 1 : left, 2 : red, 3 : straightleft, 4 : yellow
{
    if(auto_drive != 0)
    {
        if((tf_flag == 1) or (tf_flag == 2))
        {
            if (((light.data[0] == 0) || (light.data[0] == 3)) && (tf_flag == 1)) camera_scan = 0;
            else if (((light.data[0] == 1) || (light.data[0] == 3)) && (tf_flag == 2)) camera_scan = 0;
            else camera_scan = 1;
        }
    }
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    if(auto_drive != 0)
    {
        int count = (int)( 360. / RAD2DEG(scan->angle_increment));
        int sum = 0;
        //ROS_INFO("I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
        //ROS_INFO("%f %f", scan->scan_time, scan->time_increment);
        //ROS_INFO("angle_range, %f, %f %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max), RAD2DEG(scan->angle_increment));

    /*    for(int i=0;i<count;i++)
        {
            float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
            ROS_INFO(": [%f, %f]", degree, scan->ranges[i]);
        }
    */
    
        for(int i=0;i<count;i++)
        {
            float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);

            if(((degree >= 90-10)&&(degree <= 90+10)))
            {
                ROS_INFO(": [%f, %f]", degree, scan->ranges[i]);
                if(scan->ranges[i] <= Lidar_Obstacle_Range)
                {
                    sum++;
                    ROS_INFO("sum = %d", sum);
                }
            }
        }
        //ROS_INFO("sum = %d", sum);
        if(sum >= 10)
        {
            lidar_scan = 1;
            ROS_INFO("Lidar Obstacle Detected !!");
            sum=0;
        }
        else lidar_scan = 0;
    }
}

void sonar1Callback(const std_msgs::Float32& sonar_data)
{
    sonar[0] = (float)(sonar_data.data);
}

void sonar2Callback(const std_msgs::Float32& sonar_data)
{
    sonar[1] = (float)(sonar_data.data);
}

void sonar3Callback(const std_msgs::Float32& sonar_data)
{
    sonar[2] = (float)(sonar_data.data);
}


int main(int argc, char **argv)
{
    char buf[2];
    ros::init(argc, argv, "Car_Control");

    ros::NodeHandle n;

    std_msgs::String msg;

    ros::Subscriber controller_sub = n.subscribe("/cmd_vel", 10, &CarControlCallback);
    ros::Subscriber ackermann_sub = n.subscribe("/ackermann_cmd", 10, &AutoControlCallback);

    ros::Subscriber tf_path_sub = n.subscribe("/path_mode", 10, &PathCallback);
    ros::Subscriber tf_ligth_sub = n.subscribe("/traffic_light", 10, &TFlightCallback);

    ros::Subscriber lidar_sub = n.subscribe("/scan", 1000, &scanCallback);

    ros::Subscriber sonar_sub1 = n.subscribe("/sonar1", 10, &sonar1Callback);
    ros::Subscriber sonar_sub2 = n.subscribe("/sonar2", 10, &sonar1Callback);
    ros::Subscriber sonar_sub3 = n.subscribe("/sonar3", 10, &sonar1Callback);


    ros::Publisher teleop_cmd_vel_pub = n.advertise<geometry_msgs::Twist>("teleop_cmd_vel", 10);
    ros::Publisher auto_pub = n.advertise<std_msgs::Int16>("auto_mode", 1);
    ros::Rate loop_rate(10);

    /**
     * A count of how many messages we have sent. This is used to create
     * a unique string for each message.
     */

    int count = 0;
    auto_mode.data = 0;
    stop_time = (unsigned)time(NULL);

    while (ros::ok())
    {	
        if(auto_drive != 0)
        {
            // printf("sonar %6.3lf \n", sonar);
            ///// sonar obstacle detection /////
            if(((sonar[0] > 0) && (sonar[0] <= Sonar_Obstacle_Range)) || ((sonar[1] > 0) && (sonar[1] <= Sonar_Obstacle_Range)) || ((sonar[2] > 0) && (sonar[2] <= Sonar_Obstacle_Range)))
            {
	            sonar_scan = 1;
		        //ROS_INFO("Sonar Obstacle detection : %3.2lf %3.2lf %3.2lf", sonar[0], sonar[1], sonar[2]);
            }
            else sonar_scan = 0;
        }

        if((auto_drive == 1) || (auto_drive == 2))
        {
            if((lidar_scan == 0) && (sonar_scan == 0) && (camera_scan == 0)) auto_drive = 1;
            else auto_drive = 2;
        }

        if((auto_drive != 3) && (auto_drive != 4))
        {
            if(auto_drive == 2)
            {
                brake = 1;
                motor_speed_cmd = 0;
                steering_angle_cmd = 0;
            }
            else
            {
                brake = 0;
                motor_speed_cmd = motor_speed;
                steering_angle_cmd = steering_angle;
            }
        }
        
        else if(auto_drive == 3)
        {
            STtimer();
            brake = 0;
            motor_speed_cmd = stop_speed;
            steering_angle_cmd = stop_steer;
        }

        else if(auto_drive == 4)
        {
            PKtimer();
            brake = 0;
            motor_speed_cmd = parking_speed;
            steering_angle_cmd = parking_steer;
        }

        auto_mode.data = auto_drive;

        teleop_cmd_vel_data.linear.z = brake;
        teleop_cmd_vel_data.angular.z = steering_angle_cmd;
        teleop_cmd_vel_data.linear.x = motor_speed_cmd;
        teleop_cmd_vel_pub.publish(teleop_cmd_vel_data);
        auto_pub.publish(auto_mode);
		
        printf("Mode : %d | Speed : %3d | Steering : %2d\n", auto_drive, motor_speed_cmd, steering_angle_cmd);

        //ROS_INFO("Sonar : %5.1lf %5.1lf %5.1lf", sonar[0], sonar[1], sonar[2]);
				
        loop_rate.sleep();
        ros::spinOnce();
        ++count;
    }
    
    return 0;
    
}
