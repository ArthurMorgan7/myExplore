#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h> 
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <novatel_gps_msgs/NovatelUtmPosition.h>

using namespace std;

int main(int argc, char** argv)
{
    double dt = 0.01;
    ros::init(argc,argv,"path_track_node");
    ros::NodeHandle nh;
    ros::Publisher pub_cmd = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.1;
    cmd_vel.angular.z = 0.1;
    pub_cmd.publish(cmd_vel);
    // 让运动持续一段时间，即程序休眠一段时间
    ros::Duration(dt).sleep();

    return 0;
}