#ifndef KEY_FRAME_H
#define KEY_FRAME_H

#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>

namespace lidar_slam_3d
{

class KeyFrame
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<KeyFrame> Ptr;
    KeyFrame() {}
    KeyFrame(int id, const Eigen::Matrix4f& pose, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
    {
        id_ = id;
        pose_ = pose;
        cloud_ = cloud;
    }
    ~KeyFrame() {}

    void setId(int id) { id_ = id; }
    void setPose(const Eigen::Matrix4f& pose) { pose_ = pose; }
    void setCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) { cloud_ = cloud; }

    int getId() { return id_; }
    Eigen::Matrix4f getPose() { return pose_; }
    pcl::PointCloud<pcl::PointXYZI>::Ptr getCloud() { return cloud_; }

private:
    int id_;
    Eigen::Matrix4f pose_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_;
};

} // namespace lidar_slam_3d

#endif // KEY_FRAME_H
