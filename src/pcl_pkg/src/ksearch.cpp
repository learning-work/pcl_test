#include<base/SYS/MyROS.hpp>
#include<base/PTC/MyTopic.hpp>
#include<base/PTC/MyServer.hpp>
#include<ros/ros.h>

#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/visualization/cloud_viewer.h>
#include<pcl/octree/octree_search.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl_conversions/pcl_conversions.h>
typedef pcl::PointXYZ PointT;

using namespace std;
pcl::PointCloud<PointT>::Ptr cloud_msg(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr searchcloud(new pcl::PointCloud<PointT>);
void cloudcallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    
    pcl::fromROSMsg(*msg, *cloud_msg);

    //
    float resolution = 0.2f;
    pcl::octree::OctreePointCloudSearch<PointT> octree(resolution);
    octree.setInputCloud(cloud_msg);
    octree.addPointsFromInputCloud();//通过点云构建octree

    
    PointT searchPoint;
    searchPoint = cloud_msg->points[0];

    int k = 10;
    vector<int> indices;
    vector<float> distances;

    int size = octree.nearestKSearch(searchPoint, k, indices,distances);

    searchcloud->clear();
    for(auto idx: indices)
    {
        //cout<<" "<<cloud_msg->points[idx].x<<" "<<cloud_msg->points[idx].y<<std::endl;
        searchcloud->push_back(cloud_msg->points[idx]);
    }
    //可视化点云数据

    

}

int main(int argc, char** argv)
{
    MYROS myros(argc, argv, "Ksearch_node");
    MYTOPIC cloud_sub("cloud_raw",10);
    
    cloud_sub.Get<sensor_msgs::PointCloud2>(cloudcallback);
    ros::spinOnce();

    pcl::visualization::PointCloudColorHandlerCustom<PointT> originColorHandler(cloud_msg,255,255,255);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> searchColorHandler(searchcloud,0,255,0);

    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor(0.1176,0.1176,0.2353);
    viewer.addPointCloud<PointT>(cloud_msg, originColorHandler, "cloud");
    viewer.addPointCloud<PointT>(searchcloud, searchColorHandler, "search_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "search_cloud");
    while(!viewer.wasStopped() && ros::ok())
    {

        ros::spinOnce();

        viewer.updatePointCloud(cloud_msg, "cloud");
        viewer.updatePointCloud(searchcloud, "search_cloud");

        viewer.spinOnce(10);
    }


    return 0;
}