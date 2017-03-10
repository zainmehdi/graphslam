#include <vector>
#include <math.h>
#include <iostream>
#include <string>
#include <sstream>

#include <ros/ros.h>
#include <ros/console.h>

#include <common/Factor.h>
#include <common/Keyframe.h>
#include <common/ClosestKeyframe.h>
#include <common/LastKeyframe.h>
#include <common/Registration.h>
#include <common/Pose2DWithCovariance.h>
#include <common/OdometryBuffer.h>

#include <gtsam/inference/Key.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

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
