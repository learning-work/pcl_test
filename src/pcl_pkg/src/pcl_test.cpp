#include<base/SYS/MyROS.hpp>
#include<base/PTC/MyTopic.hpp>
#include<base/PTC/MyServer.hpp>
#include<ros/ros.h>

#include<sensor_msgs/PointCloud2.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/search/kdtree.h>
#include<pcl/segmentation/extract_clusters.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>





// void cloudcallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {

//     pcl::fromROSMsg(*msg, *cloud_msg);

//     ROS_INFO("Recevied %lu",cloud_msg->points.size());

// }

sensor_msgs::PointCloud2 output;

void cloudcallback(const sensor_msgs::PointCloud2::ConstPtr& msg) 
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_msg(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*msg,*cloud_msg);

    //PassThrough Z 轴 （对二维雷达的不需要）

    //欧式聚类
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_msg);

    std::vector<pcl::PointIndices> cloud_Indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.2);
    ec.setMinClusterSize(1);
    ec.setMaxClusterSize(360);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_msg);
    ec.extract(cloud_Indices);


    //给每一个簇上色
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
    int cloud_id = 0;
    for(const auto& indices: cloud_Indices)
    {
        //不同的簇颜色不一样
        uint8_t r = rand()% 256;
        uint8_t g = rand()% 256;
        uint8_t b = rand()% 256;

        for(int idx: indices.indices)
        {
            pcl::PointXYZRGB p;
            p.x = cloud_msg->points[idx].x;
            p.y = cloud_msg->points[idx].y;
            p.z = cloud_msg->points[idx].z;
            p.r = r;
            p.g = g;
            p.b = b;
            cloud_colored->points.push_back(p);
        }
        
        cloud_id++;
          
    }

        //发布
        
    pcl::toROSMsg(*cloud_colored,output);
    output.header = msg->header;
}

int main(int argc, char** argv) {

    MYROS myros(argc,argv,"pcl_test");
    
    MYTOPIC point_sub("/cloud_raw",10);

    MYTOPIC cloud_pub("/cloud_colored",10);


    //订阅点云数据
    // auto cloudcallback = [&](const sensor_msgs::PointCloud2::ConstPtr& msg) {

    //     pcl::fromROSMsg(*msg, *cloud_msg);

    //     //ROS_INFO("Recevied %lu",cloud_msg->points.size());

    //     ROS_INFO("%f",cloud_msg->points[1])


    // };

    point_sub.Get<sensor_msgs::PointCloud2>(cloudcallback);
    while(ros::ok()) 
    {
        cloud_pub.Send(output);
        ros::spinOnce();
        myros.Delay(50);
          
    }
    

    return 0;

}
