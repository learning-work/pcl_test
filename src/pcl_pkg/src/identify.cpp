#include<base/SYS/MyROS.hpp>
#include<base/PTC/MyTopic.hpp>
#include<base/PTC/MyServer.hpp>
#include<ros/ros.h>

#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/filters/filter.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/filters/voxel_grid.h>
#include<pcl/filters/extract_indices.h>
#include<pcl/segmentation/sac_segmentation.h>
#include<pcl/filters/crop_box.h>
#include<pcl/common/common.h>

typedef pcl::PointXYZ PointT;

using namespace std;

class CloudProcessor
{
private:
    MYTOPIC cloud_sub;
    MYTOPIC cloud_pub_raw;
    MYTOPIC cloud_pub_obj;

    pcl::PointCloud<PointT>::Ptr cloud_raw;
    pcl::PointCloud<PointT>::Ptr cloud_msg;
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
        cloud_msg = boost::make_shared<pcl::PointCloud<PointT>>();

        ROS_INFO("CloudProcessor initialized.");
    }

    void cloudcallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        
        //防止累计
        cloud_raw->clear();
        cloud_filtered->clear();
        cloud_obj->clear();
        cloud_roi_->clear();

        pcl::fromROSMsg(*msg, *cloud_raw);
        if(cloud_raw->empty())
        {
            cout<<"Empty cloud received."<<std::endl;
            return;
        }
        //ROI
        pcl::CropBox<PointT> crop;
        crop.setInputCloud(cloud_raw);
        crop.setMin(Eigen::Vector4f(-30.0f,-30.0f,0.01f,1.0f));
        crop.setMax(Eigen::Vector4f(30.0f,30.0f,5.0f,1.0f));
        crop.filter(*cloud_roi_);

        if(cloud_roi_->empty())
        {
            cout<<"cloud_roi_ is empty"<<std::endl;
            return;
        }
        //去除无效点
        vector<int> idx;
        pcl::removeNaNFromPointCloud(*cloud_roi_,*cloud_roi_,idx);

        //下采样
        pcl::VoxelGrid<PointT> voxel;
        voxel.setLeafSize(0.05f,0.05f,0.05f);
        voxel.setInputCloud(cloud_roi_);
        voxel.filter(*cloud_filtered);

        //识别圆柱
        

        //发布点云
        sensor_msgs::PointCloud2 raw_msg;
        pcl::toROSMsg(*cloud_filtered, raw_msg);
        cloud_pub_raw.Send(raw_msg);
    }
};
int main(int argc, char** argv)
{
    MYROS myros(argc, argv, "identify_node");

    CloudProcessor processor;

    ros::spin();

    while(ros::ok())
    {

    }
    return 0;
}