#include <scanner.hpp>
#include <nav_msgs/Odometry.h>

ros::Publisher delta_pub;

const double converged_fitness_threshold = 0.5;
double k_disp_disp = 0.1, k_rot_disp = 0.1, k_rot_rot = 0.1;

pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
Eigen::Matrix4f carry_transform;

sensor_msgs::LaserScan robot_0_laserscan;
sensor_msgs::LaserScan robot_1_laserscan;
nav_msgs::Odometry robot_1_gt;
nav_msgs::Odometry robot_0_gt;

common::Pose2DWithCovariance between(const common::Pose2DWithCovariance& start_pose,
                        const common::Pose2DWithCovariance& end_pose) {
  common::Pose2DWithCovariance transform;
  double t_start_th = start_pose.pose.theta;
  double t_end_th = end_pose.pose.theta;
  double cos_th = cos ( t_start_th );
  double sin_th = sin ( t_start_th );
  double dx = end_pose.pose.x - start_pose.pose.x;
  double dy = end_pose.pose.y - start_pose.pose.y;
  double dth = t_end_th - t_start_th;
  dth = std::fmod( dth + M_PI, 2 * M_PI ) - M_PI;
  transform.pose.x = ( cos_th * dx ) + ( sin_th * dy );
  transform.pose.y = ( -1 * sin_th * dx ) + ( cos_th * dy );
  transform.pose.theta = dth;

  return transform;

}

void gicp_register(sensor_msgs::PointCloud2 input_1,
				   sensor_msgs::PointCloud2 input_2,
				   Eigen::Matrix4f& transform) {
  double start = ros::Time::now().toSec();
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_1 = format_pointcloud(input_1);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_2 = format_pointcloud(input_2);

  try {
	gicp.setInputTarget(pointcloud_1);
    gicp.setInputSource(pointcloud_2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_transform(new pcl::PointCloud<pcl::PointXYZ>);
    gicp.align(*pointcloud_transform, transform);
  }
  catch (int e) {
    std::cout << "An exception occurred. Exception Nr. " << e << '\n';
  }

  ROS_INFO("#############################");

  // Ground Truth (GT):

  common::Pose2DWithCovariance pose_0, pose_1;
  pose_0.pose.x = robot_0_gt.pose.pose.position.x;
  pose_0.pose.y = robot_0_gt.pose.pose.position.y;
  pose_0.pose.theta = tf::getYaw(robot_0_gt.pose.pose.orientation);
  pose_1.pose.x = robot_1_gt.pose.pose.position.x;
  pose_1.pose.y = robot_1_gt.pose.pose.position.y;
  pose_1.pose.theta = tf::getYaw(robot_1_gt.pose.pose.orientation);
  common::Pose2DWithCovariance Delta_01 = between(pose_0, pose_1);

  // Register (RG):
  transform = gicp.getFinalTransformation();
  geometry_msgs::Pose2D transform_Delta = make_Delta(transform);
  Eigen::MatrixXd covariance_Delta = compute_covariance(k_disp_disp, k_rot_disp, k_rot_rot, transform_Delta);
  common::Pose2DWithCovariance Delta = create_Pose2DWithCovariance_msg(transform_Delta, covariance_Delta);

  // Error
  common::Pose2DWithCovariance Delta_err = between(Delta_01, Delta);

  // Print
  ROS_INFO("Converged?: %d, Fitness Score: %f", gicp.hasConverged(), gicp.getFitnessScore());
  ROS_INFO("GT: x0 = %f; y1 = %f; th1 = %f", robot_0_gt.pose.pose.position.x, robot_0_gt.pose.pose.position.y, tf::getYaw(robot_0_gt.pose.pose.orientation));
  ROS_INFO("GT: x1 = %f; y1 = %f; th1 = %f", robot_1_gt.pose.pose.position.x, robot_1_gt.pose.pose.position.y, tf::getYaw(robot_1_gt.pose.pose.orientation));
  ROS_INFO("GT: dx = %f; dy = %f; dth = %f", Delta_01.pose.x, Delta_01.pose.y, Delta_01.pose.theta);
  ROS_INFO("RG: dx = %f; dy = %f; dth = %f", Delta.pose.x, Delta.pose.y, Delta.pose.theta);
  ROS_INFO("RG: sx = %f; sy = %f; sth = %f", sqrt(Delta.covariance[0]), sqrt(Delta.covariance[4]), sqrt(Delta.covariance[8]));
  ROS_INFO("RG: ex = %f; ey = %f; eth = %f", Delta_err.pose.x, Delta_err.pose.y, Delta_err.pose.theta);
  double end = ros::Time::now().toSec();
  ROS_INFO("Time to Completion: %f seconds", end - start);
  ROS_INFO("#############################");
  delta_pub.publish(Delta);
}

void gicp_register(sensor_msgs::PointCloud2 input_1, sensor_msgs::PointCloud2 input_2) {
  Eigen::Matrix4f guess_null(Eigen::Matrix4f::Identity());
  gicp_register(input_1, input_2, guess_null);
}

void robot_0_scanner_callback(const sensor_msgs::LaserScan& input) {
  robot_0_laserscan = input;
  ROS_INFO("robot_0_laserscan received.");
}

void robot_1_scanner_callback(const sensor_msgs::LaserScan& input) {
  robot_1_laserscan = input;
  ROS_INFO("robot_1_laserscan received.");
}

void robot_0_gt_callback(const nav_msgs::Odometry& input) {
  robot_0_gt = input;
  ROS_INFO("robot_0_gt received.");
}

void robot_1_gt_callback(const nav_msgs::Odometry& input) {
  robot_1_gt = input;
  ROS_INFO("robot_1_gt received.");
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "scanner");
  ros::NodeHandle n;
  ros::Rate r(100);

  ros::Subscriber robot_0_scan_sub = n.subscribe("/robot_0/base_scan", 1, robot_0_scanner_callback);
  ros::Subscriber robot_1_scan_sub = n.subscribe("/robot_1/base_scan", 1, robot_1_scanner_callback);
  ros::Subscriber ground_truth_0_sub = n.subscribe("/robot_0/base_pose_ground_truth", 1, robot_0_gt_callback);
  ros::Subscriber ground_truth_1_sub = n.subscribe("/robot_1/base_pose_ground_truth", 1, robot_1_gt_callback);
  delta_pub = n.advertise<common::Pose2DWithCovariance>("/scanner/delta", 1);

  // Setup GICP algorithm
//  gicp.setUseReciprocalCorrespondences(true);
  gicp.setMaximumIterations(50); // ICP example 50
    gicp.setMaxCorrespondenceDistance(1); // ICP example 0.05
    gicp.setTransformationEpsilon(1e-8); // ICP example 1e-8
    gicp.setEuclideanFitnessEpsilon(0.1); // ICP example 1
  //  gicp.setCorrespondenceRandomness();
//  gicp.setMaximumOptimizerIterations(50);
  //  gicp.setRotationEpsilon();

  carry_transform.setIdentity();
  int end = ros::Time::now().toSec() + 3;

  while(ros::ok()) {
    sensor_msgs::PointCloud2 robot_0_pointcloud = scan_to_pointcloud(robot_0_laserscan);
    sensor_msgs::PointCloud2 robot_1_pointcloud = scan_to_pointcloud(robot_1_laserscan);

    if(ros::Time::now().toSec() > end) {
      gicp_register(robot_0_pointcloud, robot_1_pointcloud);
    }
    
    r.sleep();
    ros::spinOnce();
  }
  return 0;
}
