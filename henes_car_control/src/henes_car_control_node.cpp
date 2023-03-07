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
#define Lidar_Obstacle_Range 3.0

// Steering Control
#define Neutral_Angle_Offset 0
#define Right_MAX 30
#define Left_MAX -30


geometry_msgs::Twist cmd_vel_data;
std_msgs::Int16 yolo_scan;

int auto_drive = 0; // 0 : controller mode, 1 : auto-drive mode, 2 : something detected, 3 : stop mode
int lidar_scan = 0;

int steering_angle = 0;
int steering_angle_cmd = 0;
int motor_speed = 0;
int motor_speed_cmd = 0;
int brake = 0; // 0 : no brake, 1 : at fast, 2 : at slope, 3 : at low, 4 : to park

int now = 0, start_time = 0;
int stop_speed = 0, stop_steer = 0;
int timer_flag[4] = {0,0,0,0};
int timer_time[4] = {3,3,3,5};
int path_flag = 0;

void CarControlCallback(const geometry_msgs::Twist& msg)
{
    //ROS_INFO("I heard: [%s]", msg->data.c_str());

    steering_angle = (int)(msg.angular.z);

    if(steering_angle <= Left_MAX) steering_angle = Left_MAX;
    if(steering_angle >= Right_MAX) steering_angle = Right_MAX;

    motor_speed = (int)msg.linear.x;
    if(motor_speed >= 255) motor_speed = 255;
    if(motor_speed <= -255) motor_speed = -255;

    if((int)msg.linear.y == 1) //press 'c'
    {
        brake = 1;
        auto_drive = 0;
    }
    else brake = 0;
    if((int)msg.linear.z == 1) auto_drive = 1; //press 'e'
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


void STtimer()
{
    now = (unsigned)time(NULL);
    int time_gap = now - start_time;
    if(time_gap <= timer_time[path_flag-4]) 
    {
        brake = path_flag - 3;
    }
    else 
    {
        brake = 0;
        auto_drive = 1;
    }
}

void PathCallback(const std_msgs::Int32MultiArray& path)
{
    if(auto_drive != 0)
    {
        path_flag = path.data[3];
        
        if (path_flag == 2) // 신호등 출발
        {
            auto_drive = 1;
            brake = 0;
        }
        else if (path_flag == 3) // 신호등 정지
        {
            auto_drive = 2;
            brake = 1;
        }

        if ((path_flag >= 4) && (auto_drive != 3)) // 주정차 구역
        {
            if (timer_flag[path_flag-4] == 0)
            {
                auto_drive = 3;
                start_time = (unsigned)time(NULL);
                timer_flag[path_flag-4] = 1;
                stop_speed = 0;
                stop_steer = 0;
            }
        }

        if (path_flag==-1) 
        {
            brake=1;
            auto_drive = 0;
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

            if(((degree >= 90-30)&&(degree <= 90+30)))
            {
                //ROS_INFO(": [%f, %f]", degree, scan->ranges[i]);
                if(scan->ranges[i] <= Lidar_Obstacle_Range)
                {
                    sum++;
                    //ROS_INFO("sum = %d", sum);
                }
            }
        }
        //ROS_INFO("sum = %d", sum);
        if(sum >= 2 && path_flag == 1)
        {
            lidar_scan = 1;
            brake = 1;
            ROS_INFO("Lidar Obstacle Detected !!");
            sum=0;
        }
        else lidar_scan = 0;
    }
}

int main(int argc, char **argv)
{
    char buf[2];
    ros::init(argc, argv, "Car_Control");

    ros::NodeHandle n;

    std_msgs::String msg;

    ros::Subscriber controller_sub = n.subscribe("/teleop_cmd_vel", 10, &CarControlCallback);
    ros::Subscriber ackermann_sub = n.subscribe("/ackermann_cmd", 10, &AutoControlCallback);

    ros::Subscriber path_mode_sub = n.subscribe("/waypoint", 10, &PathCallback);

    ros::Subscriber lidar_sub = n.subscribe("/scan", 10, &scanCallback);

    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    ros::Rate loop_rate(10);

    /**
     * A count of how many messages we have sent. This is used to create
     * a unique string for each message.
     */


    while (ros::ok())
    {	
        if(((auto_drive == 1) || (auto_drive == 2)) && (path_flag == 1))
        {
            if(lidar_scan == 0) auto_drive = 1;
            else auto_drive = 2;
        }

        if(auto_drive == 2)
        {
            motor_speed_cmd = 0;
            steering_angle_cmd = 0;
        }
        
        else if(auto_drive == 3)
        {
            STtimer();
            motor_speed_cmd = stop_speed;
            steering_angle_cmd = stop_steer;
        }

        else if(auto_drive == 1)
        {
            brake = 0;
            motor_speed_cmd = motor_speed;
            steering_angle_cmd = steering_angle;
        }

        else
        {
            motor_speed_cmd = motor_speed;
            steering_angle_cmd = steering_angle;            
        }


        cmd_vel_data.linear.z = brake;
        cmd_vel_data.angular.z = steering_angle_cmd;
        cmd_vel_data.linear.x = motor_speed_cmd;
        cmd_vel_pub.publish(cmd_vel_data);
		
        printf("Mode : %d | Speed : %3d | Steering : %2d\n", auto_drive, motor_speed_cmd, steering_angle_cmd);

        loop_rate.sleep();
        ros::spinOnce();
    }
    
    return 0;
    
}
