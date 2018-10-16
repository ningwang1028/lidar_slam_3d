#ifndef MAP_BUILDER_H
#define MAP_BUILDER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <g2o/core/sparse_optimizer.h>
#include <chrono>
#include <mutex>
#include "math_func.h"
#include "key_frame.h"

namespace lidar_slam_3d
{

class MapBuilder
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MapBuilder();
    ~MapBuilder() {}

    void addPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& point_cloud);

    void setMapUpdateDistance(float distance) { map_update_distance_ = distance; }
    void setMapUpdateAngle(float angle) { map_update_angle_ = angle; }
    void setSubmapSize(int size) { submap_size_ = size; }

    Eigen::Matrix4f getTransformation() { return pose_; }
    void getMap(sensor_msgs::PointCloud2& map_msg)
    {
        std::unique_lock<std::mutex> locker(map_mutex_);
        pcl::toROSMsg(map_, map_msg);
    }
    void getPoseGraph(std::vector<Eigen::Vector3d>& nodes,
                      std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& edges);
    void doPoseOptimize();

private:
    void downSample(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr& sampled_cloud);
    void addVertex(const KeyFrame::Ptr& key_frame);
    void addEdge(const KeyFrame::Ptr& source, const Eigen::Matrix4f& source_pose,
                 const KeyFrame::Ptr& target, const Eigen::Matrix4f& target_pose,
                 const Eigen::Matrix<double, 6, 6>& information);
    void updateMap();
    void updateSubmap();
    void detectLoopClosure(const KeyFrame::Ptr& key_frame);
    KeyFrame::Ptr getClosestKeyFrame(const KeyFrame::Ptr& key_frame,
                                     const std::vector<KeyFrame::Ptr>& candidates);
    bool needOptimize();

private:
    pcl::PointCloud<pcl::PointXYZI> map_;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> submap_;
    std::vector<KeyFrame::Ptr> key_frames_;

    pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt_;
    g2o::SparseOptimizer optimizer_;

    Eigen::Matrix4f pose_;
    Eigen::Matrix4f last_update_pose_;
    float voxel_grid_leaf_size_;
    float map_update_distance_;
    float map_update_angle_;
    float loop_search_distance_;
    float loop_min_fitness_score_;
    bool enable_optimize_;
    int loop_keyframe_skip_;
    int loop_min_chain_size_;
    int submap_size_;
    int sequence_num_;
    int loop_constraint_count_;
    int optimize_every_n_constraint_;
    std::chrono::steady_clock::time_point optimize_time_;
    bool first_point_cloud_;
    std::mutex map_mutex_;
};

} // namespace lidar_slam_3d

#endif // MAP_BUILDER_H
