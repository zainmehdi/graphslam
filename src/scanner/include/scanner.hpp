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

common::Pose2DWithCovariance create_Pose2DWithCovariance_msg(double x, double y, double th, Eigen::MatrixXd Q) {
  common::Pose2DWithCovariance output;
  output.pose.x = x;
  output.pose.y = y;
  output.pose.theta = th;

  if(Q.rows() == 3 && Q.cols() == 3) {
    for(int i = 0; i < Q.rows(); i++) {
      for(int j = 0; j < Q.cols(); j++) {
	output.covariance[( i * Q.rows() ) + j] = Q(i, j);
      }
    }
  }

  return output;
}

common::Pose2DWithCovariance create_Pose2DWithCovariance_msg(geometry_msgs::Pose2D pose, Eigen::MatrixXd Q) {
  common::Pose2DWithCovariance output = create_Pose2DWithCovariance_msg(pose.x, pose.y, pose.theta, Q);
  
  return output;
}

geometry_msgs::Pose2D make_Delta(const Eigen::MatrixXf& T) {
  geometry_msgs::Pose2D Delta;
  Delta.x = T(0, 3);
  Delta.y = T(1, 3);
  Delta.theta = atan2( T(1, 0) , T(0, 0) );
  return Delta;
}

Eigen::Matrix4f make_transform(const geometry_msgs::Pose2D& Delta) {
    Eigen::Matrix4f transform;
    transform.setIdentity();
    float cos_th = cos(Delta.theta);
    float sin_th = sin(Delta.theta);
    transform(0,0) = cos_th;
    transform(0,1) = -sin_th;
    transform(1,0) = sin_th;
    transform(1,1) = cos_th;
    transform(0,3) = Delta.x;
    transform(1,3) = Delta.y;
    return transform;
}

// JS: Isn't it possible to place these functions in a common place for everyone to share?? Like in common/src/ or something? I have made this comment a lot of times :-(
Eigen::MatrixXd compute_covariance(const double k_disp_disp, const double k_rot_disp, const double k_rot_rot, geometry_msgs::Pose2D input)
{
//  double k_disp_disp = 0.1; // JS: I put these as input params for this function. Remove these lines.
//  double k_rot_disp = 0.1;
//  double k_rot_rot = 0.1;
  
  double Dl = sqrt( pow( input.x, 2 ) + pow( input.y, 2) );
  double sigma_xy_squared = k_disp_disp * Dl;
  double sigma_th_squared = ( k_rot_disp * Dl ) + ( k_rot_rot * fabs(input.theta) );
  
  Eigen::MatrixXd Q(3, 3);
  Q(0, 0) = sigma_xy_squared;
  Q(1, 1) = sigma_xy_squared;
  Q(2, 2) = sigma_th_squared;

  return Q;
}

sensor_msgs::PointCloud2 scan_to_pointcloud(sensor_msgs::LaserScan input) {
//  ROS_INFO("SCAN TO POINTCLOUD STARTED.");
  laser_geometry::LaserProjection projector;
  sensor_msgs::PointCloud2 output;
  projector.projectLaser(input, output);

//  ROS_INFO("SCAN TO POINTCLOUD FINISHED.");
  return output;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr format_pointcloud(sensor_msgs::PointCloud2 input) {
//  ROS_INFO("FORMAT POINTCLOUD STARTED.");
  pcl::PCLPointCloud2 pcl2_pointcloud;
  pcl_conversions::toPCL(input, pcl2_pointcloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl2_pointcloud, *output);

//  ROS_INFO("FORMAT POINTCLOUD FINISHED.");
  return output;
}

