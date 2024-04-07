// 测试对PCD所有点的跟踪

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

bool start_flag = true;
bool stop_flag = false;

double dt = 1;    // 一次运动命令的执行时间
double vir_x = 0;   // TODO
double vir_y = 0;
double vir_h = 0;
double max_angular_speed = 1;
double max_speed = 0.5;
struct rtk_Data{
    float heading;
    double northing;
    double easting;
    double height;
}rtk_data;

void pathstartflagCallback(const std_msgs::Bool::ConstPtr &msg);
void pathstopflagCallback(const std_msgs::Bool::ConstPtr &msg);
void rtkCallback(const novatel_gps_msgs::NovatelUtmPosition::ConstPtr &msg);

double cal_distance(float &x, float &y);
double cal_target_orientation(float &x, float &y);
double cal_control_speed(double &distance);
double cal_control_rotate(double &diff_angle);

int main(int argc, char** argv)
{
    string filepath = "/home/arthur/视频/采摘路径/car_path.pcd";
    double tolerance_distance = 0.1; // 10cm
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    ros::init(argc,argv,"path_track_node");
    ros::NodeHandle nh;


    if(start_flag == true && stop_flag == false){
        // 1. 读路径点
        pcl::io::loadPCDFile<pcl::PointXYZ>(filepath, *cloud);
        float base_x, base_y;

        // 2. 对每个路径点进行跟踪
        for(size_t i = 0; i < cloud->points.size(); i++)
        {
            /* --------------------------------- PCD文件读取 -------------------------------- */
            if(i == 0){
                base_x = cloud->points[0].x;
                base_y = cloud->points[0].y;
                cout << "基准点：" << "x:" << base_x << " y: " << base_y << endl;
                continue;
            }
            float point_x = cloud->points[i].x + base_x;
            float point_y = cloud->points[i].y + base_y;
            cout << "----------------------------------------------------" << endl;
            cout << "第" << i << "个目标点：" << "x:" << point_x << " y: " << point_y << endl;

            double diff_angle = 0;
            double target_orientation = 0;
            double distance = 0;    // 直线距离
            bool turn_done = false;

            // 先预读一次，更新最新的自身的位置，并计算横向距离，如果下一个点足够近，就不用动
            distance = cal_distance(point_x, point_y);
            cout << "距离1: " << distance << endl; 
            while(distance > tolerance_distance)
            {
                // 再次更新自己的位置
                distance = cal_distance(point_x, point_y);  // 目标点距离
                cout << "距离2: " << distance << endl; 
                target_orientation = cal_target_orientation(point_x, point_y);  // 目标点绝对方位（正北为0） [-180,180] 
                cout << "绝对方位: " << target_orientation << endl; 
                /* ---------------------------------- 跟踪算法：先旋转，再移动 ---------------------------------- */           
                double control_speed, control_rotate;

                // 偏差角 diff_angle。正-左边， 负-右边
                diff_angle = target_orientation - vir_h; 
                cout << "目标点方向(转换前): " << diff_angle << endl; 
                if(diff_angle > 180){
                    diff_angle =  diff_angle - 360;
                }
                else if(diff_angle < -180){
                    diff_angle =  diff_angle + 360;
                }
                cout << "目标点距离: " <<  distance << endl;
                cout << "目标点方向(转换后): " <<  diff_angle << endl;

                if (abs(diff_angle) < 5) {
                    turn_done = true;
                }
                else{
                    turn_done = false;
                }
                
                // 旋转
                if(!turn_done){
                    cout << "正在控制方向！" << endl;
                    control_speed = 0;
                    control_rotate = cal_control_rotate(diff_angle); 
                    
                }

                // 移动
                if(turn_done){
                    cout << "方向合适，正在控制速度！" << endl;
                    control_rotate = 0;
                    control_speed = cal_control_speed(distance);
                }

            //     /* ------------------------------------ - ----------------------------------- */
            }
        }

    }    
    return 0;
}

/////////////////////////////////////////////////////////////////////////
double cal_distance(float &x, float &y){
    return sqrt((x - vir_x) * (x - vir_x) + (y - vir_y)*(y - vir_y));
}
// 计算目标点相对于机器人的绝对方位
double cal_target_orientation(float &x, float &y)
{
    double target_orientation = 0;
    if(y == vir_y){
        if(x>vir_x){
            return -90.0;
        }
        else{
            return 90.0;
        }
    }
    double tmp_orientation = atan((x - vir_x)/(y-vir_y))/PI*180.0;

    if( x>vir_x && y>vir_y){
        target_orientation = -tmp_orientation;
    }
    else if(x>vir_x && y<vir_y){
        target_orientation = -180.0 - tmp_orientation;
    }
    else if(x<vir_x && y>vir_y){
        target_orientation = -tmp_orientation;
    }
    else if(x<vir_x && y<vir_y){
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
    vir_h += control_rotate *dt /PI*180;
    if(vir_h>180){
        vir_h = vir_h -360;
    }
    cout << "[vir_x]:" << vir_x << endl;
    cout << "[vir_y]:" << vir_y << endl;
    cout << "[vir_h]:" << vir_h << endl;
    cout << "角速度:" << control_rotate << endl;
    return control_rotate;
}
// 计算前进角度
double cal_control_speed(double &distance){
    double control_speed;
    control_speed = distance / dt;
    if(control_speed > max_speed){
        control_speed = max_speed;
    }
    vir_x += control_speed*dt *sin(-vir_h/180*PI);   
    vir_y += control_speed*dt *cos(-vir_h/180*PI);
    cout << "[vir_x]:" << vir_x << endl;
    cout << "[vir_y]:" << vir_y << endl;
    cout << "[vir_h]:" << vir_h << endl;
    cout << "线速度:" << control_speed << endl;
    return control_speed;
}