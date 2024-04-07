// 使用参数服务器获取停止标志
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

#define PI 3.1415926535

// 可供修改的值
bool start_flag = false;
bool stop_flag = false;
bool done_flag = false;
double tolerance_distance = 0.1; // 10cm
string filepath = "/home/arthur/视频/采摘路径/car_path.pcd";    // TODO
double dt = 0.7;    // 一次运动命令的执行时间
double max_angular_speed = 0.4;   // 最大转速
double max_speed = 0.4;           // 最大直线速度


struct rtk_Data{
    float heading = 0.0f;
    double northing = 0;
    double easting = 0;
    double height = 0;
}rtk_data;

void pathstartflagCallback(const std_msgs::Bool::ConstPtr &msg);
void rtkCallback(const novatel_gps_msgs::NovatelUtmPosition::ConstPtr &msg);
double cal_distance(float &x, float &y);
double cal_target_orientation(float &x, float &y);
double cal_control_speed(double &distance);
double cal_control_rotate(double &diff_angle);

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    ros::init(argc,argv,"path_track_node");
    ros::NodeHandle nh;
    ros::Subscriber sub_start_flag = nh.subscribe("/path_start_flag", 1, pathstartflagCallback);
    ros::Subscriber sub_current_pose = nh.subscribe("/RTK", 1, rtkCallback); 
    ros::Publisher pub_done_flag = nh.advertise<std_msgs::Bool>("path_done_flag",1);
    ros::Publisher pub_cmd = nh.advertise<geometry_msgs::Twist>("smooth_cmd_vel", 1);
    
    ros::Rate loop_rate(1);
    while(ros::ok())
    { 
        cout << "----------------------------------------------------" << endl;
        ros::spinOnce();
        if(start_flag == true)
        {
            pcl::io::loadPCDFile<pcl::PointXYZ>(filepath, *cloud);
            float base_x, base_y;
            for(size_t i = 0; i < cloud->points.size(); i++)
            {
                /* --------------------------------- PCD文件读取 -------------------------------- */
                if(i == 0){
                    base_x = cloud->points[0].x;
                    base_y = cloud->points[0].y;
                    cout << setiosflags(ios::fixed) << setprecision(7) << "基准点：" << "x:" << base_x << " y: " << base_y << endl;
                    system("pause");
                    cout << "--------------------------------" << endl;
                    continue;
                }
                float point_x = cloud->points[i].x + base_x;
                float point_y = cloud->points[i].y + base_y;
                cout << setiosflags(ios::fixed) << setprecision(7) << "第" << i << "个目标点：" << "x:" << point_x << " y: " << point_y << endl;
                double diff_angle = 0;
                double target_orientation = 0;
                double distance = 0;    // 直线距离
                bool turn_done = false;
                /* ---------------------------------------------------------------------------- */
                
                // 先预读一次，更新最新的自身的位置，并计算横向距离，如果下一个点足够近，就不用动
                ros::spinOnce();
                distance = cal_distance(point_x, point_y);
                while(distance > tolerance_distance)
                {
                    // 是否停下采摘水果
                    nh.getParam("sign_track", stop_flag);
                    if(stop_flag == 1){
                        cout << "------停下摘水果------" << endl;
                        geometry_msgs::Twist cmd_vel;
                        cmd_vel.linear.x = 0;
                        cmd_vel.angular.z = 0;
                        pub_cmd.publish(cmd_vel);
                        ros::Duration(1).sleep();
                    }

                    // 再次更新自己的位置
                    ros::spinOnce();
                    distance = cal_distance(point_x, point_y);  // 目标点距离
                    
                    target_orientation = cal_target_orientation(point_x, point_y); 
            
                    /* ---------------------------------- 跟踪算法：先旋转，再移动 ---------------------------------- */           
                    double control_speed, control_rotate;

                    // 偏差角 diff_angle。正-左边， 负-右边
                    diff_angle = target_orientation - rtk_data.heading; 
                    // cout << "目标点方向(转换前): " << diff_angle << endl; 
                    if(diff_angle > 180){
                        diff_angle =  diff_angle - 360;
                    }
                    else if(diff_angle < -180){
                        diff_angle =  diff_angle + 360;
                    }
                    cout << "距离目标点: " << distance << "米" << endl; 
                    //cout << "目标点方位(绝对): " <<  target_orientation << endl;
                    cout << "目标点方向(相对): " <<  diff_angle << "角度" << endl;

                    if (abs(diff_angle) < 5) {
                        turn_done = true;
                    }
                    else{
                        turn_done = false;
                    }

                    // 旋转
                    if(!turn_done){
                        
                        control_speed = 0;
                        control_rotate = cal_control_rotate(diff_angle); 
                    }

                    // 移动
                    if(turn_done){
                        
                        control_rotate = 0;
                        control_speed = cal_control_speed(distance);
                    }

                    geometry_msgs::Twist cmd_vel;
                    cmd_vel.linear.x = control_speed;
                    cmd_vel.angular.z = control_rotate;
                    pub_cmd.publish(cmd_vel);
                }
            }
            cout << "所有路径点跟踪完毕！" << endl;
            // 发布结束信号
            std_msgs::Bool done_flag;
            done_flag.data = true;
            pub_done_flag.publish(done_flag);

            // 开始信号重新置零，等待下次读取
            start_flag = false;   
        }
        else
        {
            cout << "Start_flag为False, 未开始跟踪!" << endl;
        } 
        loop_rate.sleep();   
    }
    return 0;
}

/////////////////////////////////////////////////////////////////////////
void pathstartflagCallback(const std_msgs::Bool::ConstPtr &msg){
    start_flag = msg->data;
}

void rtkCallback(const novatel_gps_msgs::NovatelUtmPosition::ConstPtr &msg){
    rtk_data.northing = msg->northing;
    rtk_data.easting = msg->easting;
    rtk_data.height = msg->height;
    rtk_data.heading = msg->diff_age;
}

double cal_distance(float &x, float &y){
    return sqrt((x - rtk_data.easting) * (x - rtk_data.easting) + (y - rtk_data.northing)*(y - rtk_data.northing));
}

// 计算目标点相对于机器人的绝对方位
double cal_target_orientation(float &x, float &y)
{
    double target_orientation = 0;
    if(y == rtk_data.northing){
        if(x>rtk_data.easting){
            return -90.0;
        }
        else{
            return 90.0;
        }
    }
    double tmp_orientation = atan((x - rtk_data.easting)/(y-rtk_data.northing))/PI*180.0;

    if( x>rtk_data.easting && y>rtk_data.northing){
        target_orientation = -tmp_orientation;
    }
    else if(x>rtk_data.easting && y<rtk_data.northing){
        target_orientation = -180.0 - tmp_orientation;
    }
    else if(x<rtk_data.easting && y>rtk_data.northing){
        target_orientation = -tmp_orientation;
    }
    else if(x<rtk_data.easting && y<rtk_data.northing){
        target_orientation = -tmp_orientation + 180.0;
    }
    return target_orientation;
}

// 计算旋转速度.正-左拐，负-右拐。
double cal_control_rotate(double &diff_angle){
    double control_rotate = diff_angle/180*PI / dt; // 转换为弧度，供控制
    if(control_rotate > max_angular_speed){
        control_rotate = max_angular_speed;
    }
    else if(control_rotate < -max_angular_speed){
        control_rotate = -max_angular_speed;
    }
    cout << "正在控制方向,角速度:" << control_rotate << endl;
    cout << "--------------------------------" << endl;
    return control_rotate;
}
// 计算前进角度
double cal_control_speed(double &distance){
    double control_speed;
    control_speed = distance / dt;
    if(control_speed > max_speed){
        control_speed = max_speed;
    }
    cout << "方向合适，正在控制速度,线速度:" << control_speed << endl;
    cout << "--------------------------------" << endl;
    return control_speed;
}