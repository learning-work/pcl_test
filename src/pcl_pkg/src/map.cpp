#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Eigen>
#include <math.h>
#include <vector>

#include<base/SYS/MyROS.hpp>
#include<base/PTC/MyTopic.hpp>
#include<base/PTC/MyServer.hpp>
#include<base/SYS/Some_define.hpp>




using namespace std;
using namespace Eigen;

ros::Publisher map_pub;
pcl::PointCloud<pcl::PointXYZ> cloudMap;
sensor_msgs::PointCloud2 globalMap_pcd;

double drone_size,cylinder_r,cube_r, resolution;
double z_size;


//高密度圆柱体生成函数
void GenerateCylinderHighDensity(const Eigen::Vector3d &center, double radius, double height, double res)
{
    pcl::PointXYZ pt;

    // 底面/顶面：半径步长更细，角度步长更细
    double r_step = res / 2.0;                 // 半径间隔小一点
    double theta_step = (radius > 1e-9) ? res / (2.0 * radius) : res;  // 角度间隔小一点，防止除零

    // 底面
    for(double r = 0; r <= radius; r += r_step){
        for(double theta = 0; theta < 2*M_PI; theta += theta_step){
            pt.x = center(0) + r * cos(theta);
            pt.y = center(1) + r * sin(theta);
            pt.z = center(2);
            cloudMap.points.push_back(pt);
        }
    }

    // 顶面
    for(double r = 0; r <= radius; r += r_step){
        for(double theta = 0; theta < 2*M_PI; theta += theta_step){
            pt.x = center(0) + r * cos(theta);
            pt.y = center(1) + r * sin(theta);
            pt.z = center(2) + height;
            cloudMap.points.push_back(pt);
        }
    }

    // 侧面：Z方向间隔减小，theta步长减小
    double z_step = res / 2.0; // 更密
    for(double z = 0; z <= height; z += z_step){
        for(double theta = 0; theta < 2*M_PI; theta += theta_step){
            pt.x = center(0) + radius * cos(theta);
            pt.y = center(1) + radius * sin(theta);
            pt.z = center(2) + z;
            cloudMap.points.push_back(pt);
        }
    }
}

/// 生成一个圆柱体（底面+顶面+侧面）
void GenerateCylinder(const Eigen::Vector3d &center, double radius, double height, double res)
{
    pcl::PointXYZ pt;
    double theta_step = (radius > 1e-9) ? res / radius : res; // 角度分辨率

    // 底面
    for(double theta = 0; theta < 2*M_PI; theta += theta_step){
        for(double r = 0; r <= radius; r += res){
            pt.x = center(0) + r * cos(theta);
            pt.y = center(1) + r * sin(theta);
            pt.z = center(2);
            cloudMap.points.push_back(pt);
        }
    }

    // 顶面
    for(double theta = 0; theta < 2*M_PI; theta += theta_step){
        for(double r = 0; r <= radius; r += res){
            pt.x = center(0) + r * cos(theta);
            pt.y = center(1) + r * sin(theta);
            pt.z = center(2) + height;
            cloudMap.points.push_back(pt);
        }
    }

    // 侧面
    for(double z = 0; z <= height; z += res){
        for(double theta = 0; theta < 2*M_PI; theta += theta_step){
            pt.x = center(0) + radius * cos(theta);
            pt.y = center(1) + radius * sin(theta);
            pt.z = center(2) + z;
            cloudMap.points.push_back(pt);
        }
    }
}

/// 生成一个正方体（放在底面，底面中心在原点）
/// cube_r 表示正方体半边长（即边长 = 2*cube_r）
/// 底面中心在 (0,0,0)，因此 z 从 0 到 2*cube_r
void GenerateCube(double half_r, double res)
{
    if (half_r <= 0 || res <= 0) return;
    pcl::PointXYZ pt;

    double x_min = -half_r, x_max = half_r;
    double y_min = -half_r, y_max = half_r;
    double z_min = 0.0,    z_max = 2.0 * half_r;

    // 底面 (z = z_min)
    for(double x = x_min; x <= x_max + 1e-9; x += res){
        for(double y = y_min; y <= y_max + 1e-9; y += res){
            pt.x = x; pt.y = y; pt.z = z_min; cloudMap.points.push_back(pt);
        }
    }

    // 顶面 (z = z_max)
    for(double x = x_min; x <= x_max + 1e-9; x += res){
        for(double y = y_min; y <= y_max + 1e-9; y += res){
            pt.x = x; pt.y = y; pt.z = z_max; cloudMap.points.push_back(pt);
        }
    }

    // 四个侧面
    // 面 x = x_min 与 x = x_max
    for(double z = z_min; z <= z_max + 1e-9; z += res){
        for(double y = y_min; y <= y_max + 1e-9; y += res){
            pt.x = x_min; pt.y = y; pt.z = z; cloudMap.points.push_back(pt);
            pt.x = x_max; pt.y = y; pt.z = z; cloudMap.points.push_back(pt);
        }
    }

    // 面 y = y_min 与 y = y_max
    for(double z = z_min; z <= z_max + 1e-9; z += res){
        for(double x = x_min; x <= x_max + 1e-9; x += res){
            pt.x = x; pt.y = y_min; pt.z = z; cloudMap.points.push_back(pt);
            pt.x = x; pt.y = y_max; pt.z = z; cloudMap.points.push_back(pt);
        }
    }
}

/// 生成整个地图（3根圆柱体 + 正方体）
void GenerateMap(std::vector<Eigen::Vector3d> xy)
{
    // 清除旧点（防止多次调用时重复追加）
    cloudMap.points.clear();

    // 固定三个圆柱体中心坐标（在地图范围内）
    std::vector<Eigen::Vector3d> centers = xy;

    for(auto &c : centers){
        GenerateCylinder(c, cylinder_r, z_size, resolution);
    //   GenerateCylinderHighDensity(c, cylinder_r, z_size, resolution);
    }

    // 在原点放一个正方体，底面中心在 (0,0,0)，大小由 cube_r 控制
    GenerateCube(cube_r, resolution);

    cloudMap.width = static_cast<uint32_t>(cloudMap.points.size());
    cloudMap.height = 1;
    cloudMap.is_dense = true;

    pcl::toROSMsg(cloudMap, globalMap_pcd);
    globalMap_pcd.header.frame_id = "world";
    
}

/// 发布地图
void pubMap()
{
    
    globalMap_pcd.header.stamp = ros::Time::now();
    map_pub.publish(globalMap_pcd);
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map");
    ros::NodeHandle nh("~");
    map_pub = nh.advertise<sensor_msgs::PointCloud2>("/global_map", 10);
    ros::Rate rate(1.0); // 发布频率
    
    resolution = 0.03;
    ros::param::param("cylinder_r",cylinder_r,0.5);
    ros::param::param("z_size",z_size,1.0);
    ros::param::param("cube_r",cube_r,0.5);

    //圆柱体参数


    //正方体参数
    // cube_r = 0.5;

    //三个圆柱体参数
    std::vector<Eigen::Vector3d> xy = {{1.0,2.0,0.0},{-1.0,-2.0,0.0},{3.0,0.0,0.0}};
    // cylinder_r = 0.5;
    // z_size = 1.0;

    GenerateMap(xy);

    while(ros::ok())
    {
        pubMap();
        rate.sleep();

    }
    return 0;
}
