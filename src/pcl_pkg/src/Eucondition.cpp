#include<base/SYS/MyROS.hpp>
#include<base/PTC/MyTopic.hpp>
#include<base/PTC/MyServer.hpp>
#include<ros/ros.h>

#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/filters/voxel_grid.h>
#include<pcl/filters/crop_box.h>
#include<pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>  // 欧式聚类
#include<pcl/search/kdtree.h>
#include<visualization_msgs/Marker.h>
#include<visualization_msgs/MarkerArray.h>

typedef pcl::PointXYZ PointT;
using namespace std;




int main(int argc, char** argv)
{
    MYROS myros(argc, argv, "Eucondition_node");
    MYTOPIC cloud_raw_sub("/global_map",10);
    MYTOPIC marker_pub_("marker_eu",10);
    auto cloud_callback = [&](const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        pcl::PointCloud<PointT>::Ptr cloud_raw(new pcl::PointCloud<PointT>);
        pcl::fromROSMsg(*msg,*cloud_raw);
        if(cloud_raw->empty())
        {
            cout<<"点云为空"<<std::endl;
            return;
        }

        //ROI
        pcl::PointCloud<PointT>::Ptr cloud_roi(new pcl::PointCloud<PointT>);
        pcl::CropBox<PointT> crop;
        crop.setInputCloud(cloud_raw);
        crop.setMin(Eigen::Vector4f(-30.0f,-30.0f,0.1f,1.0f));
        crop.setMax(Eigen::Vector4f(30.0f,30.0f,2.5f,1.0f));
        crop.filter(*cloud_roi);

        //去除无效点
        vector<int> idx;
        pcl::removeNaNFromPointCloud(*cloud_roi,*cloud_roi,idx);

        //体素滤波
        pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
        pcl::VoxelGrid<PointT> voxel;
        voxel.setLeafSize(0.02f,0.02f,0.02f);
        voxel.setInputCloud(cloud_roi);
        voxel.filter(*cloud_filtered);

        //欧式聚类
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        tree->setInputCloud(cloud_filtered); //构建kd树

        vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(0.5);
        ec.setMinClusterSize(100);
        ec.setMaxClusterSize(100000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_filtered);
        ec.extract(cluster_indices);

        //使用Marker可视化
        visualization_msgs::MarkerArray marker_array;

        int cluster_id = -1;

        for(const auto& indices: cluster_indices)
        {
            visualization_msgs::Marker marker;

            marker.header = msg->header;
            marker.ns = "clusters";
            marker.id = ++cluster_id;
            marker.type = visualization_msgs::Marker::POINTS;
            marker.action = visualization_msgs::Marker::ADD;

            marker.scale.x = 0.02;
            marker.scale.y = 0.02;
            marker.color.a = 1.0;

            std_msgs::ColorRGBA color;
            color.a = 1.0;
            color.r = (cluster_id * 37 % 255) / 255.0f;
            color.g = (cluster_id * 17 % 255) / 255.0f;
            color.b = (cluster_id * 97 % 255) / 255.0f;

            for(const auto& index: indices.indices)
            {
                geometry_msgs::Point p;
                p.x = cloud_filtered->points[index].x;
                p.y = cloud_filtered->points[index].y;
                p.z = cloud_filtered->points[index].z;

                marker.points.push_back(p);
                marker.colors.push_back(color);
            }
            marker_array.markers.push_back(marker);
        }

        //一次性发布
        marker_pub_.Send(marker_array);
    };
    cloud_raw_sub.Get<sensor_msgs::PointCloud2>(cloud_callback);
    while(ros::ok())
    {
        ros::spin();
    }
    return 0;
    
}