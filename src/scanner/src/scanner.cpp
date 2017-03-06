#include <ros/ros.h>

#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <laser_geometry/laser_geometry.h>

#include <common/Factor.h>
#include <common/Keyframe.h>
#include <common/Registration.h>
#include <common/Pose2DWithCovariance.h>

#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>

ros::Publisher registration_pub;

common::Pose2DWithCovariance create_Pose2DWithCovariance_msg(double x, double y, double th, Eigen::MatrixXd m) {
  common::Pose2DWithCovariance output;
  output.pose.x = x;
  output.pose.y = y;
  output.pose.theta = th;

  for(int i = 0; i < m.rows(); i++) {
    for(int j = 0; j < m.cols(); j++) {
      output.covariance[( i * m.rows() ) + j] = m(i, j);
    }
  }

  return output;
}

sensor_msgs::PointCloud2 scan_to_pointcloud(sensor_msgs::LaserScan input) {
  laser_geometry::LaserProjection projector;
  sensor_msgs::PointCloud2 output;
  projector.projectLaser(input, output);
  
  return output;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr format_pointcloud(sensor_msgs::PointCloud2 input) {
  pcl::PCLPointCloud2 pcl2_pointcloud;
  pcl_conversions::toPCL(input, pcl2_pointcloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl2_pointcloud, *output);

  return output;
}

common::Registration gicp(sensor_msgs::PointCloud2 input_1, sensor_msgs::PointCloud2 input_2) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_1 = format_pointcloud(input_1);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_2 = format_pointcloud(input_2);
  
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;

  gicp.setInputSource(pointcloud_1);
  gicp.setInputTarget(pointcloud_2);

  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_transform(new pcl::PointCloud<pcl::PointXYZ>);

  double converged_fitness = gicp.getFitnessScore();
  double converged_fitness_threshold = 0.15;
  bool converged = gicp.hasConverged();
  Eigen::Matrix4f transform = gicp.getFinalTransformation();
  common::Registration output;

  if(converged) {
    if(converged_fitness > converged_fitness_threshold) {
      double Dx = transform(0, 3);
      double Dy = transform(1, 3);
      double Dth = atan2( transform(1, 0), transform(0, 0) );

      double k_disp_disp = 0.1;
      double k_rot_disp = 0.1;
      double k_rot_rot = 0.1;

      double Dl = sqrt( pow( Dx, 2 ) + pow( Dy, 2) );
      double sigma_x_squared = k_disp_disp * Dl;
      double sigma_y_squared = k_disp_disp * Dl;
      double sigma_th_squared = ( k_rot_disp * Dl ) + ( k_rot_rot * Dth );

      Eigen::MatrixXd C_l(6, 6);
      C_l(0, 0) = sigma_x_squared;
      C_l(1, 1) = sigma_y_squared;
      C_l(5, 5) = sigma_th_squared;
 
      common::Pose2DWithCovariance Delta;

      Delta.pose.x = Dx;
      Delta.pose.y = Dy;
      Delta.pose.theta = Dth;
    
      for(int i = 0; i < C_l.rows(); i++) {
	for(int j = 0; j < C_l.cols(); j++) {
	  Delta.covariance[( i * C_l.rows() ) + j] = C_l(i, j);
	}
      }
    
      output.factor_new.delta = Delta;
      output.keyframe_flag = true;
    } else {
      output.keyframe_flag = false;
    }
  } else {
    output.keyframe_flag = false;
  }

  return output;
}

void scanner_callback(const sensor_msgs::LaserScan& input) {
  
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "scanner");
  ros::NodeHandle n;

  ros::Subscriber scanner_sub = n.subscribe("/base_scan", 1, scanner_callback);

  registration_pub  = n.advertise<common::Registration>("/scanner/registration", 1);

  ros::spin();
  return 0;
}