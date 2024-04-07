#include <ros/ros.h>
#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
using namespace std;
/* ----------------------------------- 全局变量 ---------------------------------- */
const double PI = 3.1415926;
int vehicleZ = 0;
double minRelZ = -1.5;
double maxRelZ = 0.2;
double disRatioZ = 0.2;
double noDecayDis = 4.0;        
double clearingDis = 8.0;   
double minDyObsDis = 0.3;
bool clearDyObs = false;
int vehicleHeight = 1;
double absDyObsRelZThre = 0.2;
double minDyObsVFOV = -16.0;
double maxDyObsVFOV = 16.0;
double minDyObsAngle = 0;
double minDyObsRelZ = -0.5;
int minBlockPointNum = 10;
bool useSorting = true;         // 
double quantileZ = 0.25;        // 
bool considerDrop = false;      // 
bool limitGroundLift = false;   // 
double maxGroundLift = 0.15;

// 处理用体素地形相关
float terrainVoxelSize = 1.0;   // 地形体素大小
const int terrainVoxelLong = 20;    // 地形体素长度
const int terrainVoxelWidth = 10;   // 地形体素宽度
const int terrainVoxelNum = terrainVoxelLong * terrainVoxelWidth;  // 地形体素数量 200

// 规划用地形体素参数(1/5)
float planarVoxelSize = 0.2;   
const int planarVoxelLong = 100;    
const int planarVoxelWidth = 50;  
const int planarVoxelNum = 5000; 


// 点云相关
bool newlaserCloud = false;
double laserCloudTime = 0;  // 点云的时间戳
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>()); // 传感器初始的点云
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCrop(new pcl::PointCloud<pcl::PointXYZI>()); // 经过初步筛选的点云
pcl::PointCloud<pcl::PointXYZI>::Ptr voxCloud(new pcl::PointCloud<pcl::PointXYZI>()); // 体素筛选后的点云
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloud[terrainVoxelNum];    // 点云的数组
int   terrainVoxelUpdateNum[terrainVoxelNum]  = {0};  // 记录每一个体素网格中的点云数量

// 点云滤波相关
int voxelPointUpdateThre = 100; // 一个网格点云数量的阈值
float scanVoxelSize = 0.05;
pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;  // 创建滤波器
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudDwz(new pcl::PointCloud<pcl::PointXYZI>()); // 滤波后的点云

// 提取地面点云
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud(new pcl::PointCloud<pcl::PointXYZI>());
float planarVoxelElev[planarVoxelNum]  = {0};         // 保存了id附近点云高程的最小值
int   planarVoxelEdge[planarVoxelNum]  = {0};         
int   planarVoxelDyObs[planarVoxelNum] = {0};         // 障碍物信息存储数组
vector<float> planarPointElev[planarVoxelNum];        // 存储了地面体素网格附近一个平面网格的所有点云的高程信息
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudElev(new pcl::PointCloud<pcl::PointXYZI>());

/* ----------------------------------- 函数 ----------------------------------- */
void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &msg_Cloud) {
    laserCloudTime = msg_Cloud->header.stamp.toSec();
    
    // laserCloud是一个指向点云对象的指针，该指针指向的内存空间是不动的，每次来的点云都存到这部分空间，所以需要对上一帧点云清除
    laserCloud->clear();    
    laserCloudCrop->clear();    // 

    // 把ROS格式的点云转换为PCL格式，两个变量都是指针，所以需要加*
    // 先转换格式并暂存到本地代码比较方便。
    pcl::fromROSMsg(*msg_Cloud, *laserCloud);

    // 对点云副本laserCloud进行筛选，然后存到
    pcl::PointXYZI point;
    int laserCloudSize = laserCloud->points.size();
    for (int i = 0; i < laserCloudSize; i++) 
    {
        point = laserCloud->points[i];

        // 计算平面距离，并进行高度和距离筛选
        float dis = sqrt((point.x * point.x) + (point.y*point.y));
        if (point.z - vehicleZ > minRelZ - disRatioZ * dis 
            && point.z - vehicleZ < maxRelZ + disRatioZ * dis 
            && dis < terrainVoxelSize * (terrainVoxelWidth + 1))  
        {
            point.intensity = laserCloudTime;
            laserCloudCrop->push_back(point);
        }
    }

    newlaserCloud = true;
}

/* ----------------------------------- 主程序 ---------------------------------- */
int main(int argc, char **argv) {
    
    ros::init(argc, argv, "terrainAnalysis");   // 节点名称为terrainAnalysis
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");   // 节点下的话题和变量，都有/terrainAnalysis的前缀

    // 订阅
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/lslidar_point_cloud", 5, laserCloudHandler);

    // 发布
    ros::Publisher pubTmp = nh.advertise<sensor_msgs::PointCloud2>("/tmp", 2);
    ros::Publisher pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/terrain_map", 2);

    // 初始化下采样z
    downSizeFilter.setLeafSize(scanVoxelSize, scanVoxelSize, scanVoxelSize);    

    ros::Rate rate(100);
    bool status = ros::ok();
    while (status) 
    {
        ros::spinOnce();

        if (newlaserCloud) // 经回调函数处理后的点云为 laserCloudCrop
        {
            newlaserCloud = false;

            // sensor_msgs::PointCloud2 tmpCloud;
            // pcl::toROSMsg(*laserCloudCrop, tmpCloud);
            // tmpCloud.header.stamp = ros::Time().fromSec(laserCloudTime);
            // tmpCloud.header.frame_id = "laser_link";
            // pubTmp.publish(tmpCloud);

            /* ----------------------------------- 初始化 ---------------------------------- */
            for (int i = 0; i < terrainVoxelNum; i++) {
                terrainVoxelCloud[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
            }// 
            // voxCloud->clear();
            /* ----------------------------- 将激光雷达点云转换为体素网格 ----------------------------- */
            pcl::PointXYZI point;
            int laserCloudCropSize = laserCloudCrop->points.size();
            int minX=0, maxX=0;
            int minY=0, maxY=0;
            for (int i = 0; i < laserCloudCropSize; i++) 
            {
                point = laserCloudCrop->points[i];

                // 找点的坐标范围, x~[-7,9], y~[0,10]
                // if(point.x<minX){minX = point.x;}
                // if(point.x>maxX){maxX = point.x;}
                // if(point.y<minY){minY = point.y;}
                // if(point.y>maxY){maxY = point.y;}
                // cout << "int()" << int((point.x + terrainVoxelSize / 2) / terrainVoxelSize) << endl;
                int indX = int(point.x / terrainVoxelSize) + terrainVoxelLong/2;
                int indY = int(point.y / terrainVoxelSize);

                // 在规定的体素地图范围内
                if (indX >= 0 && indX < terrainVoxelLong && indY >= 0 &&indY < terrainVoxelWidth) 
                {
                    // TODO indX和indY多集中在十几
                    // cout <<"indX: " << indX << ", " << "indY: " << indY << endl;
                    
                    // voxCloud->push_back(point);
                    // 一维数组存放二维的数据
                    terrainVoxelCloud[terrainVoxelWidth * indX + indY]->push_back(point);
                    terrainVoxelUpdateNum[terrainVoxelWidth * indX + indY]++;// 有一个点，计数+1
                }
                // TODO 这里也需要一个输出可视化
            }
            
            /* ----------------------------------- 筛除点 ---------------------------------- */
            //遍历每一个体素网格，对网格中的不符合条件的点进行筛除
            for (int ind = 0; ind < terrainVoxelNum; ind++) 
            {
                // 如果落在某一个网格的点足够多
                if (terrainVoxelUpdateNum[ind] >= voxelPointUpdateThre) 
                {
                    // 取出该网格的点，并进行滤波，得到 laserCloudDwz
                    pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr = terrainVoxelCloud[ind];

                    // terrainVoxelCloudPtr是一个指针，对其修改就是对 terrainVoxelCloud修改
                    laserCloudDwz->clear();
                    downSizeFilter.setInputCloud(terrainVoxelCloudPtr);
                    downSizeFilter.filter(*laserCloudDwz);

                    // 清空体素网格内的点云，准备接受新的点云
                    terrainVoxelCloudPtr->clear();

                    // 遍历滤波后的每一个点
                    int laserCloudDwzSize = laserCloudDwz->points.size();
                    for (int i = 0; i < laserCloudDwzSize; i++) 
                    {
                        point = laserCloudDwz->points[i];
                        float dis = sqrt(point.x*point.x + point.y*point.y);
                        if ( point.z - vehicleZ > minRelZ - disRatioZ * dis  && point.z - vehicleZ < maxRelZ + disRatioZ * dis 
                            || dis < noDecayDis && dis < clearingDis)
                        {
                            terrainVoxelCloudPtr->push_back(point);
                        }
                    }
                    // TODO 这里需要一个输出可视化
                    // sensor_msgs::PointCloud2 tmpCloud;
                    // pcl::toROSMsg(*terrainVoxelCloudPtr, tmpCloud);
                    // tmpCloud.header.stamp = ros::Time().fromSec(laserCloudTime);
                    // tmpCloud.header.frame_id = "laser_link";
                    // pubTmp.publish(tmpCloud);
                }
            }

            /* ------------------------------ 把各体素点云合并到一个点云 ----------------------------- */
            terrainCloud->clear();
            for (int indX = 0; indX < terrainVoxelLong; indX++) 
            {
                for (int indY = 0; indY < terrainVoxelWidth; indY++) 
                {
                    *terrainCloud += *terrainVoxelCloud[terrainVoxelWidth * indX + indY];
                }
            }
            // TODO 可视化
            // sensor_msgs::PointCloud2 tmpCloud;
            // pcl::toROSMsg(*terrainCloud, tmpCloud);
            // tmpCloud.header.stamp = ros::Time().fromSec(laserCloudTime);
            // tmpCloud.header.frame_id = "laser_link";
            // pubTmp.publish(tmpCloud);
            /* ------------------------------------ TODO ----------------------------------- */
            for (int i = 0; i < planarVoxelNum; i++) 
            {
                planarVoxelElev[i] = 0;
                planarVoxelEdge[i] = 0;
                planarVoxelDyObs[i] = 0;
                planarPointElev[i].clear();
            }

            // 对每个点附近的高程信息进行保存到 planarPointElev
            int terrainCloudSize = terrainCloud->points.size();
            for (int i = 0; i < terrainCloudSize; i++) 
            {
                point = terrainCloud->points[i];

                int indX = int(point.x/ planarVoxelSize) + planarVoxelLong/2;
                int indY = int(point.y/ planarVoxelSize) ;

                // 高度在允许的范围内
                if (point.z - vehicleZ > minRelZ && point.z - vehicleZ < maxRelZ) 
                {
                    planarPointElev[planarVoxelWidth * indX + indY ].push_back(point.z);
                }
                
                                // 清楚动态障碍物选项
                if (clearDyObs) 
                {
                    if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 && indY < planarVoxelWidth) 
                    {
                        float pointX = point.x;
                        float pointY = point.y;
                        float pointZ = point.z;

                        float dis = sqrt(pointX * pointX + pointY * pointY);
                        if (dis > minDyObsDis) 
                        {
                            float angle = atan2(pointZ - minDyObsRelZ, dis) * 180.0 / PI;
                            if (angle > minDyObsVFOV && angle < maxDyObsVFOV || fabs(pointZ) < absDyObsRelZThre) 
                            {
                                planarVoxelDyObs[planarVoxelLong * indX + indY]++;
                            }
                        }
                        else 
                        {
                            planarVoxelDyObs[planarVoxelWidth * indX + indY] += 1;
                        }
                    }
                }
            }
            
            if (clearDyObs) 
            {
                for (int i = 0; i < laserCloudCropSize; i++) 
                {
                    point = laserCloudCrop->points[i];
                    int indX = int(point.x / planarVoxelSize) +planarVoxelWidth/2;
                    int indY = int(point.y /planarVoxelSize);

                    if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 &&indY < planarVoxelWidth) 
                    {
                        float pointX1 = point.x;
                        float pointY1 = point.y;
                        float pointZ1 = point.z;

                        float dis1 = sqrt(pointX1 * pointX1 + pointY1 * pointY1);
                        float angle1 = atan2(pointZ1 - minDyObsRelZ, dis1) * 180.0 / PI;
                        if (angle1 > minDyObsAngle) 
                        {
                            planarVoxelDyObs[planarVoxelWidth * indX + indY] = 0;
                        }
                    }
                }
            }
           
            if (useSorting) 
            {
                for (int i = 0; i < planarVoxelNum; i++) 
                {
                    int planarPointElevSize = planarPointElev[i].size();
                    if (planarPointElevSize > 0) 
                    {
                        sort(planarPointElev[i].begin(), planarPointElev[i].end());

                        int quantileID = int(quantileZ * planarPointElevSize);
                        if (quantileID < 0)
                            quantileID = 0;
                        else if (quantileID >= planarPointElevSize)
                            quantileID = planarPointElevSize - 1;

                        if (planarPointElev[i][quantileID] >planarPointElev[i][0] + maxGroundLift &&limitGroundLift) 
                        {
                            planarVoxelElev[i] = planarPointElev[i][0] + maxGroundLift;
                        } 
                        else 
                        {
                            planarVoxelElev[i] = planarPointElev[i][quantileID];
                        }
                    }
                }
            } 

            
            terrainCloudElev->clear();
            int terrainCloudElevSize = 0;
            for (int i = 0; i < terrainCloudSize; i++) 
            {
                point = terrainCloud->points[i];    // 体素地图一部分的点云
                // 相对于车辆的高度是否再范围内，对于符合高度的点，计算其投影后所属的平面体素
                if (point.z  > minRelZ && point.z - vehicleZ < maxRelZ) 
                {
                    // 计算所属平面体素的索引
                    int indX = int(point.x/ planarVoxelSize) + planarVoxelWidth/2;
                    int indY = int(point.y/ planarVoxelSize) ;

                    if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 &&indY < planarVoxelLong) 
                    {
                        float disZ = point.z - planarVoxelElev[planarVoxelLong * indX + indY];

                        int planarPointElevSize = planarPointElev[planarVoxelLong * indX + indY].size();
                        if (disZ >= 0 && disZ < vehicleHeight && planarPointElevSize >= minBlockPointNum) 
                        {
                            // 地面点
                            terrainCloudElev->push_back(point);
                            terrainCloudElev->points[terrainCloudElevSize].intensity = disZ;

                            terrainCloudElevSize++;
                        }
                    }
                }


                // 遍历所有体素网格
                // for (int i = 0; i < planarVoxelNum; i++) 
                // {
                //     // 体素的某个属性大于阈值，则被判定为障碍
                //     if (planarVoxelEdge[i] > noDataBlockSkipNum) 
                //     {
                //         // 手动为被判定为障碍的体素内部填充点云
                //         int indX = int(i / planarVoxelLong);
                //         int indY = i % planarVoxelLong;
                //         // 在中心附近来回偏移，添加点云
                //         point.x =planarVoxelSize * (indX - planarVoxelWidth/2);
                //         point.y =planarVoxelSize * indY ;
                //         point.z = vehicleZ;
                //         point.intensity = vehicleHeight;

                //         point.x -= planarVoxelSize / 4.0;
                //         point.y -= planarVoxelSize / 4.0;
                //         terrainCloudElev->push_back(point);

                //         point.x += planarVoxelSize / 2.0;
                //         terrainCloudElev->push_back(point);

                //         point.y += planarVoxelSize / 2.0;
                //         terrainCloudElev->push_back(point);

                //         point.x -= planarVoxelSize / 2.0;
                //         terrainCloudElev->push_back(point);
                //     }
                // }
            }
            // 发布点云 terrainCloudElev
            sensor_msgs::PointCloud2 tmpCloud;
            pcl::toROSMsg(*terrainCloudElev, tmpCloud);
            tmpCloud.header.stamp = ros::Time().fromSec(laserCloudTime);
            tmpCloud.header.frame_id = "laser_link";
            pubTmp.publish(tmpCloud);
            // sensor_msgs::PointCloud2 terrainCloud2;
            // pcl::toROSMsg(*terrainCloudElev, terrainCloud2);
            // terrainCloud2.header.stamp = ros::Time().fromSec(laserCloudTime);
            // terrainCloud2.header.frame_id = "map";
            // pubLaserCloud.publish(terrainCloud2);
        }

        status = ros::ok();
        rate.sleep();
    }

    return 0;
}
