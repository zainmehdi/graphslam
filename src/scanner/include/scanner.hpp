#include <ros/ros.h>
#include <ros/console.h>

#include <sstream>
#include <string>

#include <tf/transform_listener.h>

#include <geometry_msgs/Pose2D.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <laser_geometry/laser_geometry.h>

#include <common/Factor.h>
#include <common/Keyframe.h>
#include <common/Registration.h>
#include <common/Pose2DWithCovariance.h>
#include <common/LastKeyframe.h>
#include <common/ClosestKeyframe.h>

#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>

namespace scanner {

/**
 * \brief convert ROS LaserScan message to ROS pointcloud
 */
sensor_msgs::PointCloud2 scan_to_pointcloud(sensor_msgs::LaserScan input) {

  laser_geometry::LaserProjection projector;
  sensor_msgs::PointCloud2 output;
  projector.projectLaser(input, output);

  return output;
}

/**
 * \brief convert ROS pointcloud to PCL pointcloud
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr format_pointcloud(sensor_msgs::PointCloud2 input) {

  pcl::PCLPointCloud2 pcl2_pointcloud;
  pcl_conversions::toPCL(input, pcl2_pointcloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl2_pointcloud, *output);

  return output;
}

/**
 * \brief Alignement output
 */
struct Alignement{
        bool converged;
        float fitness;
        pcl::registration::DefaultConvergenceCriteria<float>::ConvergenceState convergence_state;
        Eigen::Matrix4f transform;
        common::Pose2DWithCovariance Delta;
};


/**
 * \brief Create a human-readable text for the convergence criteria
 */
std::string convergence_text(pcl::registration::DefaultConvergenceCriteria<float>::ConvergenceState state)
{
    std::string text;
    switch (state){ // defined in default_convergence_criteria.h, line 73
        case 0:
            return "Not converged";
        case 1:
            return "Ierations";
        case 2:
            return "Transform";
        case 3:
            return "Abs MSE";
        case 4:
            return "Rel MSE";
        case 5:
            return "No correspondences ";
        default:
            break;
    }
    return text;
}

}

