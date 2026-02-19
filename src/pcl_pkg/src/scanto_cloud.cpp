#include<base/SYS/MyROS.hpp>
#include<base/PTC/MyTopic.hpp>
#include<base/PTC/MyServer.hpp>
#include<ros/ros.h>


#include<sensor_msgs/LaserScan.h>
#include<sensor_msgs/PointCloud2.h>
#include<laser_geometry/laser_geometry.h>
#include<tf2_ros/transform_listener.h>







// void scancallback(const sensor_msgs::LaserScan::ConstPtr& msg) {

//     sensor_msgs::PointCloud2 cloud;

//     projector.transformLaserScanToPointCloud("map",*msg,cloud,tfBuffer);
    
//     cloud_pub.send(cloud);
    
// }
int main(int argc, char** argv) {

    MYROS myros(argc, argv, "scanto_cloud");

    MYTOPIC cloud_pub("/cloud_raw",10);
    MYTOPIC scan_sub("/laser/scan",10);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    laser_geometry::LaserProjection projector;


    auto scancallback = [&](const sensor_msgs::LaserScan::ConstPtr& msg) {

        sensor_msgs::PointCloud2 cloud;

        projector.transformLaserScanToPointCloud("base_link",*msg,cloud,tfBuffer);
    
        cloud_pub.Send(cloud);
    
    };


    scan_sub.Get<sensor_msgs::LaserScan>(scancallback);


    ros::spin();

}