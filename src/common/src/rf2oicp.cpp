#include <scanner.hpp>
#include <nav_msgs/Odometry.h>

ros::Publisher delta_pub;

const double converged_fitness_threshold = 0.5;
double k_disp_disp = 0.1, k_rot_disp = 0.1, k_rot_rot = 0.1;

Eigen::Matrix4f carry_transform;

sensor_msgs::LaserScan robot_0_laserscan;
sensor_msgs::LaserScan robot_1_laserscan;
nav_msgs::Odometry robot_1_gt;
nav_msgs::Odometry robot_0_gt;

void sparseicp_register(sensor_msgs::LaserScan input_1,
			 sensor_msgs::LaserScan input_2,
			 Eigen::Matrix4f& transform) {
  double start = ros::Time::now().toSec();

  Vertices vertices_source = scan_to_vertices(input_1);
  Vertices vertices_target = scan_to_vertices(input_2);

  SICP::Parameters pars;
  pars.p = .5;
  pars.max_icp = 15;
  pars.print_icpn = true;
  SICP::point_to_point(vertices_source, vertices_source, pars);
  
  geometry_msgs::Pose2D transform_Delta = make_Delta(transform);
  Eigen::MatrixXd covariance_Delta = compute_covariance(k_disp_disp,
							k_rot_disp,
							k_rot_rot,
							transform_Delta);
  common::Pose2DWithCovariance Delta = create_Pose2DWithCovariance_msg(transform_Delta, covariance_Delta);
  //ROS_INFO("Converged?: %d, Fitness Score: %f", gicp.hasConverged(), gicp.getFitnessScore());
  ROS_INFO("GT: x0 = %f; y1 = %f; th1 = %f",
	   robot_0_gt.pose.pose.position.x,
	   robot_0_gt.pose.pose.position.y,
	   tf::getYaw(robot_0_gt.pose.pose.orientation));
  ROS_INFO("GT: x1 = %f; y1 = %f; th1 = %f",
	   robot_1_gt.pose.pose.position.x,
	   robot_1_gt.pose.pose.position.y,
	   tf::getYaw(robot_1_gt.pose.pose.orientation));
  ROS_INFO("RG: dx = %f; dy = %f; dth = %f",
	   Delta.pose.x, Delta.pose.y, Delta.pose.theta);
  double end = ros::Time::now().toSec();
  ROS_INFO("Time to Completion: %f seconds", end - start);
  ROS_INFO("#############################");
  delta_pub.publish(Delta);
}

void sparseicp_register(sensor_msgs::LaserScan input_1, sensor_msgs::LaserScan input_2) {
  Eigen::Matrix4f guess_null(Eigen::Matrix4f::Identity());
  sparseicp_register(input_1, input_2, guess_null);
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
  ros::init(argc, argv, "sparseicp");
  ros::NodeHandle n;
  ros::Rate r(100);

  ros::Subscriber robot_0_scan_sub = n.subscribe("/robot_0/base_scan", 1, robot_0_scanner_callback);
  ros::Subscriber robot_1_scan_sub = n.subscribe("/robot_1/base_scan", 1, robot_1_scanner_callback);
  ros::Subscriber ground_truth_0_sub = n.subscribe("/robot_0/base_pose_ground_truth", 1, robot_0_gt_callback);
  ros::Subscriber ground_truth_1_sub = n.subscribe("/robot_1/base_pose_ground_truth", 1, robot_1_gt_callback);
  delta_pub = n.advertise<common::Pose2DWithCovariance>("/scanner/delta", 1);

  carry_transform.setIdentity();
  int end = ros::Time::now().toSec() + 3;

  while(ros::ok()) {
    //    sensor_msgs::PointCloud2 robot_0_pointcloud = scan_to_pointcloud(robot_0_laserscan);
    //    sensor_msgs::PointCloud2 robot_1_pointcloud = scan_to_pointcloud(robot_1_laserscan);

    if(ros::Time::now().toSec() > end) {
      sparseicp_register(robot_0_laserscan, robot_1_laserscan);
    }
    
    r.sleep();
    ros::spinOnce();
  }
  return 0;
}
