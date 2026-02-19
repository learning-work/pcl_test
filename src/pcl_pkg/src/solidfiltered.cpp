#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/filters/voxel_grid.h>
#include<pcl/filters/crop_box.h>
#include<pcl/filters/extract_indices.h>
#include<pcl/segmentation/extract_clusters.h>  // 欧式聚类
#include<pcl/segmentation/sac_segmentation.h>
#include<pcl/search/kdtree.h>
#include<visualization_msgs/Marker.h>
#include<visualization_msgs/MarkerArray.h>
#include<geometry_msgs/PoseStamped.h>
#include<pcl/features/normal_3d.h>


#include<base/SYS/MyROS.hpp>
#include<base/PTC/MyTopic.hpp>
#include<base/PTC/MyServer.hpp>

typedef pcl::PointXYZ PointT;
using namespace std;

Eigen::Matrix4f lidar_pose;

struct cloudFrame
{
    pcl::PointCloud<PointT>::Ptr cloud;
    Eigen::Matrix4f pose;
};

class SlidingWindowFusion
{
private:
    int window_size_;
    std::deque<cloudFrame> window_;

public:
    SlidingWindowFusion(int window_size)
        :window_size_(window_size)
    {

    }

    void addFrame(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
        const Eigen::Matrix4f& pose)
    {
        cloudFrame frame;
        frame.cloud.reset(new pcl::PointCloud<PointT>(*cloud));
        frame.pose = pose;

        window_.push_back(frame);

        if(window_.size() > window_size_)
        {
            window_.pop_front();
        }
    }

    pcl::PointCloud<PointT>::Ptr getFusedCloud()
    {
        if(window_.empty())
        {
            return pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
        }

        //const Eigen::Matrix4f& T_now = window_.back().pose;

        pcl::PointCloud<PointT>::Ptr fused(new pcl::PointCloud<PointT>);
        

        for(const auto& frame: window_)
        {
            //Eigen::Matrix4f T_i_to_now = T_now.inverse() * frame.pose;
            pcl::PointCloud<PointT> transformed;
            pcl::transformPointCloud(*frame.cloud,transformed,frame.pose);
            *fused += transformed;
        }

        return fused;
    }
};

geometry_msgs::Pose pose_now;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    pose_now = msg->pose;
}

Eigen::Matrix4f poseMsgToEigen(
    const geometry_msgs::Pose& pose)
{
    Eigen::Quaternionf q(
        pose.orientation.w,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z
    );

    Eigen::Matrix3f R = q.toRotationMatrix();

    Eigen::Vector3f t(
        pose.position.x,
        pose.position.y,
        pose.position.z
    );

    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3,3>(0,0) = R;
    T.block<3,1>(0,3) = t;

    return T;
}

int main(int argc, char** argv)
{
    MYROS myros(argc, argv, "solidfilter_node");
    MYTOPIC cloud_raw_sub("/global_map",10);
    MYTOPIC marker_pub("marker_cloud",10);
    MYTOPIC pose_sub("mavros/local_position/pose",10);

    SlidingWindowFusion fusion(10);

    pose_sub.Get<geometry_msgs::PoseStamped>(pose_cb);
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
        crop.setMin(Eigen::Vector4f(-30.0f,-30.0f,0.0f,1.0f));
        crop.setMax(Eigen::Vector4f(30.0f,30.0f,2.5f,1.0f));
        crop.filter(*cloud_roi);

        //体素滤波
        pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
        pcl::VoxelGrid<PointT> voxel;
        voxel.setLeafSize(0.02f,0.02f,0.02f);
        voxel.setInputCloud(cloud_roi);
        voxel.filter(*cloud_filtered);

        //去除无效点
        vector<int> idx;
        pcl::removeNaNFromPointCloud(*cloud_filtered,*cloud_filtered,idx);

        //滑动窗口滤波
        lidar_pose = poseMsgToEigen(pose_now);
        fusion.addFrame(cloud_filtered,lidar_pose);
        pcl::PointCloud<PointT>::Ptr fused_cloud = fusion.getFusedCloud();
        
        //再次体素滤波
        pcl::PointCloud<PointT>::Ptr cloud_filtered2(new pcl::PointCloud<PointT>);
        voxel.setLeafSize(0.02f,0.02f,0.02f);
        voxel.setInputCloud(fused_cloud);
        voxel.filter(*cloud_filtered2);

        //法线估计
        pcl::NormalEstimation<PointT,pcl::Normal> ne;
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        ne.setSearchMethod(tree);
        ne.setInputCloud(cloud_filtered2); 
        ne.setKSearch(50);
        ne.compute(*cloud_normals);

        //识别圆柱
        pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
        pcl::PointIndices::Ptr indices_cyliner(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients_cyliner(new pcl::ModelCoefficients);
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_CYLINDER);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setNormalDistanceWeight(0.1);
        seg.setMaxIterations(10000);
        seg.setDistanceThreshold(0.05);
        seg.setRadiusLimits(0, 0.6);
        seg.setInputCloud(cloud_filtered2);
        seg.setInputNormals(cloud_normals);
        seg.segment(*indices_cyliner, *coefficients_cyliner);
        cout<<"coefficients: "<<*coefficients_cyliner<<std::endl;

        //Marker可视化
        visualization_msgs::Marker marker;

        //marker.header = msg->header;
        marker.header.stamp = msg->header.stamp;
        marker.header.frame_id = "map";
        marker.ns = "cloud";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::ADD;

        marker.scale.x = 0.02;
        marker.scale.y = 0.02;
        marker.color.a = 1.0;

        for(const auto& point: fused_cloud->points)
        {
            geometry_msgs::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = point.z;

            marker.points.push_back(p);
        }
        marker_pub.Send(marker);


        

    };

    cloud_raw_sub.Get<sensor_msgs::PointCloud2>(cloud_callback);
    while(ros::ok())
    {
        ros::spin();
    }
    return 0;
}

      
                                                                                                                                  