#include "lidar_slam_3d_ros.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_slam_3d");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    LidarSlam3dRos lidar_slam;

    ros::spin();

    return 0;
}
