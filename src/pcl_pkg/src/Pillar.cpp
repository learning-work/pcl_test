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
#include<unordered_map>
#include<pcl/common/common.h>


#include<base/SYS/MyROS.hpp>
#include<base/PTC/MyTopic.hpp>
#include<base/PTC/MyServer.hpp>

typedef pcl::PointXYZ PointT;
#define MAX_POINT_PRE_VOXEL 10
using namespace std;

Eigen::Matrix4f lidar_pose;


struct  TimePoint
{
    PointT p;
    double stamp;
};

struct Voxel
{
    Eigen::Vector3f sum;
    int count;
    double last_time;
};

struct  VoxelKey
{
    int x, y, z;
    bool operator == (const VoxelKey& other) const
    {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct VoxelKeyHash
{
    std::size_t operator()(const VoxelKey& k) const
    {
        return ((std::hash<int>()(k.x) ^
                (std::hash<int>()(k.y) << 1)) >> 1) ^
                (std::hash<int>()(k.z) << 1);
    }
};

class FindPillar
{
private:
    pcl::PointCloud<PointT>::Ptr cloud_;

    bool isOrthogonal(const Eigen::Vector3f& a,
                  const Eigen::Vector3f& b,
                  float cos_thresh = 0.15f)
    {
        return std::abs(a.dot(b)) < cos_thresh;
    }


public:
    // FindPillar(const pcl::PointCloud<PointT>::Ptr& cloud)
    //     :cloud_(cloud)
    // {

    // }

    struct result
    {
        Eigen::Vector3f center;
        Eigen::Vector3f size;   // dx dy dz
        //float yaw;
    };

    void extractPlanes(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    std::vector<Eigen::Vector3f>& plane_normals)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr rest(
            new pcl::PointCloud<pcl::PointXYZ>(*cloud));


        while (rest->size() > 200)
        {
            pcl::NormalEstimation<PointT,pcl::Normal> ne;
            pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
            pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
            ne.setSearchMethod(tree);
            ne.setInputCloud(rest); 
            ne.setKSearch(50);
            ne.compute(*cloud_normals);

            pcl::SACSegmentationFromNormals<pcl::PointXYZ,pcl::Normal> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(0.1);
            seg.setNormalDistanceWeight(0.2);
            seg.setInputCloud(rest);
            seg.setInputNormals(cloud_normals);

            pcl::PointIndices inliers;
            pcl::ModelCoefficients coeff;
            seg.segment(inliers, coeff);

            if (inliers.indices.size() < 80)
                break;

            Eigen::Vector3f n(coeff.values[0],
                            coeff.values[1],
                            coeff.values[2]);
            plane_normals.push_back(n.normalized());

            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(rest);
            extract.setIndices(
                boost::make_shared<pcl::PointIndices>(inliers));
            extract.setNegative(true);
            extract.filter(*rest);
        }
    }

    //判断是不是一个矩形柱
    bool isRectangularPillar(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster,
    result& pillar)
    {
        // 1. 提取平面法向量
        std::vector<Eigen::Vector3f> normals;
        
        extractPlanes(cluster, normals);

        if (normals.size() < 3 )
            return false;
        
        // 2. 找三组正交平面
        bool found = false;
        for (size_t i = 0; i < normals.size(); ++i)
        {
            for (size_t j = i + 1; j < normals.size(); ++j)
            {
                for (size_t k = j + 1; k < normals.size(); ++k)
                {
                    if (isOrthogonal(normals[i], normals[j]) &&
                        isOrthogonal(normals[i], normals[k]) &&
                        isOrthogonal(normals[j], normals[k]))
                    {
                        found = true;
                    }
                }
            }
        }

        if (!found)
            return false;

        // 3. 尺寸验证
        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D(*cluster, min_pt, max_pt);

        float dx = max_pt.x() - min_pt.x();
        float dy = max_pt.y() - min_pt.y();
        float dz = max_pt.z() - min_pt.z();

        // 根据你环境改
        if (dx < 0.2 || dy < 0.2 || dz < 0.5)
            return false;

        // 4. 填充输出
        pillar.center = Eigen::Vector3f(
            (min_pt.x() + max_pt.x()) * 0.5f,
            (min_pt.y() + max_pt.y()) * 0.5f,
            (min_pt.z() + max_pt.z()) * 0.5f);

        pillar.size = Eigen::Vector3f(dx, dy, dz);
        //pillar.yaw = std::atan2(dy, dx);

        return true;
    }
};


class VoxelFusion
{
private:
    float voxel_size_;
    double window_time_;

    std::unordered_map<VoxelKey,std::deque<TimePoint>,VoxelKeyHash> voxel_map_;

    VoxelKey toVoxelKey(const PointT& p) const
    {
        return {
            static_cast<int>(std::floor(p.x / voxel_size_)),
            static_cast<int>(std::floor(p.y / voxel_size_)),
            static_cast<int>(std::floor(p.z / voxel_size_))
        }; 
    }
    
    void removeExpired(double now)
    {
        for(auto it = voxel_map_.begin(); it != voxel_map_.end(); )
        {
            auto& q = it->second;

            while(!q.empty() && now - q.front().stamp > window_time_)
            {
                q.pop_front();
            }

            if(q.empty())
            {
                it = voxel_map_.erase(it);
            }else
                ++it;
        }   
    }

public:
    VoxelFusion(float voxel_size, double window_time)
        :voxel_size_(voxel_size),window_time_(window_time)
    {

    }

    void addFrame(const pcl::PointCloud<PointT>::Ptr& cloud,const Eigen::Matrix4f& pose,double stamp)
    {
        pcl::PointCloud<PointT> cloud_world;
        pcl::transformPointCloud(*cloud,cloud_world,pose);

        for(const auto& p: cloud_world.points)
        {
            VoxelKey key = toVoxelKey(p);

            if((voxel_map_[key]).size() > MAX_POINT_PRE_VOXEL)
            {
                continue;
            }else{
                voxel_map_[key].push_back({p, stamp});
            }
            
        }

        //removeExpired(stamp);
    }

    pcl::PointCloud<PointT>::Ptr  getvoxel() const
    {
        pcl::PointCloud<PointT>::Ptr cloud_out(new pcl::PointCloud<PointT>);
        for(const auto& kv: voxel_map_)
        {
            for(const auto& tp: kv.second)
            {
                cloud_out->push_back(tp.p);
            }
        }
        return cloud_out;
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

    //SlidingWindowFusion fusion(10);
    VoxelFusion fusion(0.1,2.0);
    FindPillar pillar;
    

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
        voxel.setLeafSize(0.01f,0.01f,0.01f);
        voxel.setInputCloud(cloud_roi);
        voxel.filter(*cloud_filtered);

        //去除无效点
        vector<int> idx;
        pcl::removeNaNFromPointCloud(*cloud_filtered,*cloud_filtered,idx);

        //滑动窗口滤波
        // lidar_pose = poseMsgToEigen(pose_now);
        // fusion.addFrame(cloud_filtered,lidar_pose);
        // pcl::PointCloud<PointT>::Ptr fused_cloud = fusion.getFusedCloud();
        
        //VoxelFusion
        lidar_pose = poseMsgToEigen(pose_now);
        double stamp = ros::WallTime::now().toSec();
        fusion.addFrame(cloud_filtered, lidar_pose, stamp);
        pcl::PointCloud<PointT>::Ptr fused_cloud = fusion.getvoxel();

        //再次体素滤波
        pcl::PointCloud<PointT>::Ptr cloud_filtered2(new pcl::PointCloud<PointT>);
        voxel.setLeafSize(0.01f,0.01f,0.01f);
        voxel.setInputCloud(fused_cloud);
        voxel.filter(*cloud_filtered2);

        //法线估计
        // pcl::NormalEstimation<PointT,pcl::Normal> ne;
        // pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        // pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        // ne.setSearchMethod(tree);
        // ne.setInputCloud(cloud_filtered2); 
        // ne.setKSearch(100);
        // ne.compute(*cloud_normals);

        //欧式聚类
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        tree->setInputCloud(cloud_filtered2);

        vector<pcl::PointIndices> cluster_incices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(0.3);
        ec.setMaxClusterSize(100);
        ec.setMaxClusterSize(20000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_filtered2);
        ec.extract(cluster_incices);

        //识别矩形柱
        vector<FindPillar::result> results;
        pcl::PointCloud<PointT>::Ptr marker_cloud(new pcl::PointCloud<PointT>);
        for(const auto& indices: cluster_incices)
        {
            pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);

            cluster->points.reserve(indices.indices.size());
            for(int idx: indices.indices)
                cluster->points.push_back(cloud_filtered2->points[idx]);
    
                FindPillar::result result;
                if(pillar.isRectangularPillar(cluster, result))
                {
                    results.push_back(result);
                }
        }
        for(const auto& re: results)
        {
            cout<<"矩形柱的位置："<< re.center<<std::endl;
            cout<<"矩形柱的尺寸："<<re.size<<std::endl;
        }
        

        

        // //Marker可视化
        // visualization_msgs::Marker marker;

        // //marker.header = msg->header;
        // marker.header.stamp = msg->header.stamp;
        // marker.header.frame_id = "map";
        // marker.ns = "cloud";
        // marker.id = 0;
        // marker.type = visualization_msgs::Marker::POINTS;
        // marker.action = visualization_msgs::Marker::ADD;

        // marker.scale.x = 0.02;
        // marker.scale.y = 0.02;
        // marker.color.a = 1.0;

        // for(const auto& point: fused_cloud->points)
        // {
        //     geometry_msgs::Point p;
        //     p.x = point.x;
        //     p.y = point.y;
        //     p.z = point.z;

        //     marker.points.push_back(p);
        // }
        // marker_pub.Send(marker);


        

    };

    cloud_raw_sub.Get<sensor_msgs::PointCloud2>(cloud_callback);
    while(ros::ok())
    {
        ros::spin();
    }
    return 0;
}

      
                                                                                                                                  