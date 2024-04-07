#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h> 

using namespace std;

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    string filepath = "/home/arthur/视频/采摘路径/car_path.pcd";
    pcl::io::loadPCDFile<pcl::PointXYZ>(filepath, *cloud);
    float base_x, base_y;
    // 2. 对每个路径点进行跟踪
    for(size_t i = 0; i < cloud->points.size(); i++)
    {
        if(i == 0){
            base_x = cloud->points[0].x;
            base_y = cloud->points[0].y;
            continue;
        }
        float point_x = cloud->points[i].x + base_x;
        float point_y = cloud->points[i].y + base_y;
        cout << "x:" << point_x << "\t" << "y:" << point_y << endl;
        cout << endl;
    }
    return 0;
}