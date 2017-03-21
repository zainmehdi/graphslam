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

/**
 * \brief Create a Pose2DWithCovariance message from numeric input
 */
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

/**
 * \brief Create a Pose2DWithCovariance message from numeric input
 */
common::Pose2DWithCovariance create_Pose2DWithCovariance_msg(geometry_msgs::Pose2D pose, Eigen::MatrixXd Q) {
  common::Pose2DWithCovariance output = create_Pose2DWithCovariance_msg(pose.x, pose.y, pose.theta, Q);
  
  return output;
}

/**
 * \brief Convert a 3D matrix transform to a 2D Delta
 */
geometry_msgs::Pose2D make_Delta(const Eigen::MatrixXf& T) {
  geometry_msgs::Pose2D Delta;
  Delta.x = T(0, 3);
  Delta.y = T(1, 3);
  Delta.theta = atan2( T(1, 0) , T(0, 0) );
  return Delta;
}

/**
 * \brief Convert a 2D Delta to a 3D matrix transform
 */
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

/**
 * \brief Create covariance from sigmas
 */
Eigen::MatrixXd compute_covariance(const double sigma_xy, const double sigma_th)
{
    Eigen::MatrixXd Q(3, 3);
    Q(0, 0) = Q(1, 1) = sigma_xy * sigma_xy;
//    Q(1, 1) = sigma_xy * sigma_xy;
    Q(2, 2) = sigma_th * sigma_th;

    return Q;
}

/**
 * \brief Create covariance from Delta
 */
Eigen::MatrixXd compute_covariance(const double k_disp_disp, const double k_rot_disp, const double k_rot_rot, geometry_msgs::Pose2D input)
{

  double Dl = sqrt( pow( input.x, 2 ) + pow( input.y, 2) );
  double sigma_xy_squared = k_disp_disp * Dl;
  double sigma_th_squared = ( k_rot_disp * Dl ) + ( k_rot_rot * fabs(input.theta) );
  
  Eigen::MatrixXd Q(3, 3);
  Q(0, 0) = sigma_xy_squared;
  Q(1, 1) = sigma_xy_squared;
  Q(2, 2) = sigma_th_squared;

  return Q;
}

/**
 * \brief Compose two 2D poses additively.
 *
 * This corresponds to Pose_end = Pose_init (+) Delta
 */
common::Pose2DWithCovariance compose(common::Pose2DWithCovariance pose, common::Pose2DWithCovariance delta) {
  common::Pose2DWithCovariance output;
  double cos_th = cos( pose.pose.theta );
  double sin_th = sin( pose.pose.theta );

  // Help: composition is:
  //    pose_xy = pose_xy + R(pose.th) * delta_xy; with R(th) = [cos(th) -sin(th); sin(th) cos(th)]
  //    pose_th = pose_th + delta.th;
  output.pose.x = pose.pose.x + ( cos_th * delta.pose.x  +  -sin_th * delta.pose.y );
  output.pose.y = pose.pose.y + ( sin_th * delta.pose.x  +   cos_th * delta.pose.y );
  output.pose.theta = pose.pose.theta + delta.pose.theta;
  output.pose.theta = std::fmod(output.pose.theta + M_PI, 2 * M_PI) - M_PI;

  return output;
}

/**
 * \brief Compute pose increment or Delta.
 *
 * This corresponds to Delta = Pose_end (-) Pose_init
 */
common::Pose2DWithCovariance between(common::Pose2DWithCovariance pose_1, common::Pose2DWithCovariance pose_2) {
  common::Pose2DWithCovariance output;
  double cos_th = cos( pose_1.pose.theta );
  double sin_th = sin( pose_1.pose.theta );

  // Help: composition is:
  //    delta_xy = R(pose1.th).transposed * (pose2_xy - pose1_xy); with R(th) = [cos(th) -sin(th); sin(th) cos(th)]
  //    delta_th = pose2_th - pose1.th;
  double dx = pose_2.pose.x - pose_1.pose.x;
  double dy = pose_2.pose.y - pose_1.pose.y;
  output.pose.x = (  cos_th * dx  + sin_th * dy );
  output.pose.y = ( -sin_th * dx  + cos_th * dy );
  output.pose.theta = pose_2.pose.theta - pose_1.pose.theta;
  output.pose.theta = std::fmod(output.pose.theta + M_PI, 2 * M_PI) - M_PI;

  return output;
}

Eigen::MatrixXd covariance_to_eigen(common::Factor input) {
  Eigen::MatrixXd Q(3, 3);
  Q.row(0) << input.delta.covariance[0],
    input.delta.covariance[1],
    input.delta.covariance[2];
  Q.row(1) << input.delta.covariance[3],
    input.delta.covariance[4],
    input.delta.covariance[5];
  Q.row(2) << input.delta.covariance[6],
    input.delta.covariance[7],
    input.delta.covariance[8];

  return Q;
}

common::Pose2DWithCovariance eigen_to_covariance(common::Pose2DWithCovariance pose, Eigen::MatrixXd Q) {
  for(int i = 0; i < Q.rows(); i++) {
    for(int j = 0; j < Q.cols(); j++) {
      pose.covariance[( i * Q.rows() ) + j] = Q(i, j);
    }
  }

  return pose;
}


