// 前进旋转测试代码
#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

using namespace std;


int main(int argc, char** argv)
{
    ros::init(argc,argv,"path_track_node");
    ros::NodeHandle nh;
    ros::Publisher pub_cmd = nh.advertise<geometry_msgs::Twist>("smooth_cmd_vel", 1);
    ros::Rate loop_rate(1);
    geometry_msgs::Twist cmd_vel;
    while(ros::ok())
    { 
        int choice = -1;
        cout << "1. 前进" << endl;
        cout << "2. 后退" << endl;
        cout << "3. 左旋" << endl;
        cout << "4. 右旋" << endl;
        cin >> choice;
        switch(choice){
            case 1:
                
                cmd_vel.linear.x = 3;
                cmd_vel.angular.z = 0;
                pub_cmd.publish(cmd_vel);
                break;
            case 2:
                cmd_vel.linear.x = -3;
                cmd_vel.angular.z = 0;
                pub_cmd.publish(cmd_vel);
                break;
            case 3:
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 3;
                pub_cmd.publish(cmd_vel);
                break;
            case 4:
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = -3;
                pub_cmd.publish(cmd_vel);
                break;
            default:
                break;
        }
        loop_rate.sleep();   
    }
    return 0;
}

/////////////////////////////////////////////////////////////////////////

