#include<base/SYS/MyROS.hpp>
#include<base/PTC/MyTopic.hpp>
#include<base/PTC/MyServer.hpp>
#include<ros/ros.h>


#include<sensor_msgs/PointCloud2.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/search/kdtree.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>


using namespace std;
typedef pcl::PointXYZ PointT;


void callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<PointT>::Ptr cloud_msg(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*msg, *cloud_msg);

    //pcl::PointCloud<PointT>::Ptr input_cloud(new pcl::PointCloud<PointT>);

    //定义KdTree对象 

    pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
    kdtree->setInputCloud(cloud_msg); //创建kd树
    vector<int> indices; //存储查询的近邻点索引
    vector<float> distances; //存储邻近点的距离的平方

    PointT point = cloud_msg->points[0]; //初始化一个查询点

    //初始化 查询最近的k个点

    int k = 10;
    int size = kdtree->nearestKSearch(point,k,indices,distances);

    cout<<"search point: "<<size<<std::endl;
    cout<<"distances: "<<distances[3]<<std::endl;

    indices.clear();
    distances.clear();

    //查询离point点半径为radius邻域球内的点

    double radius = 2.0;
    size = kdtree->radiusSearch(point, radius, indices,distances);
    cout<<"saerch point: "<<size<<std::endl;
}


// k近邻搜索 与 邻域半径搜索
int main(int argc, char** argv) 
{

    MYROS myros(argc, argv, "KdTree_node");

    MYTOPIC cloud_sub("cloud_raw",10);

    cloud_sub.Get<sensor_msgs::PointCloud2>(callback);

    while(ros::ok())
    {
        ros::spin();

    }




    return 0;
}