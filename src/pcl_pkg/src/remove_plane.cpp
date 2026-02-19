#include<base/SYS/MyROS.hpp>
#include<base/PTC/MyTopic.hpp>
#include<base/PTC/MyServer.hpp>
#include<ros/ros.h>

#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/filters/voxel_grid.h>
#include<pcl/filters/extract_indices.h>
#include<pcl/segmentation/sac_segmentation.h>
#include<pcl/filters/crop_box.h>

typedef pcl::PointXYZ PointT;

using namespace std;


class CloudProcessor
{
private:
    MYTOPIC cloud_sub;
    MYTOPIC cloud_pub_raw;
    MYTOPIC cloud_pub_obj;

    pcl::PointCloud<PointT>::Ptr cloud_raw;
    pcl::PointCloud<PointT>::Ptr cloud_obj;
    pcl::PointCloud<PointT>::Ptr cloud_filtered;
    pcl::PointCloud<PointT>::Ptr cloud_roi_;

public:
    CloudProcessor() :cloud_sub("/scan",10),cloud_pub_raw("cloud_raw",10),
    cloud_pub_obj("cloud_obj",10)
    {
        cloud_sub.Get<sensor_msgs::PointCloud2>
            (std::bind(&CloudProcessor::cloudcallback, this, std::placeholders::_1));
        cloud_raw = boost::make_shared<pcl::PointCloud<PointT>>();
        cloud_obj = boost::make_shared<pcl::PointCloud<PointT>>();
        cloud_filtered = boost::make_shared<pcl::PointCloud<PointT>>();
        cloud_roi_ = boost::make_shared<pcl::PointCloud<PointT>>();


        ROS_INFO("CloudProcessor initialized.");
    }

    void cloudcallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        pcl::fromROSMsg(*msg, *cloud_raw);
        if(cloud_raw->empty())
        {
            cout<<"Empty cloud received."<<std::endl;
            return;
        }
        //ROI
        pcl::CropBox<PointT> crop;
        crop.setInputCloud(cloud_raw);
        crop.setMin(Eigen::Vector4f(0.0f,-3.0f,0.1f,1.0f));
        crop.setMax(Eigen::Vector4f(6.0f,3.0f,2.5f,1.0f));
        crop.filter(*cloud_roi_);

        //下采样
        pcl::VoxelGrid<PointT> voxel;
        voxel.setLeafSize(0.05f,0.05f,0.05f);
        voxel.setInputCloud(cloud_raw);
        voxel.filter(*cloud_filtered);

        //平面分割
        pcl::SACSegmentation<PointT> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setDistanceThreshold(0.02);
        seg.setInputCloud(cloud_filtered);

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

        seg.segment(*inliers, *coefficients);

        if(inliers->indices.empty())
        {
            cout<<"No plane found"<<std::endl;
            return;
        }

        //去掉平面
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(true); //去除地面点
        extract.filter(*cloud_obj);

        //发布原始点云
        sensor_msgs::PointCloud2 raw_msg;
        pcl::toROSMsg(*cloud_raw, raw_msg);
        raw_msg.header = msg->header;
        cloud_pub_raw.Send(raw_msg);

        //发布非平面点云
        sensor_msgs::PointCloud2 obj_msg;
        pcl::toROSMsg(*cloud_obj, obj_msg);
        obj_msg.header = msg->header;
        cloud_pub_obj.Send(obj_msg);
    }
};

int main(int argc, char** argv)
{
    MYROS myros(argc, argv, "remove_plane_node");
    
    CloudProcessor processor;
    ros::spin();
    
    while(ros::ok())
    {

    }
    return 0;
}