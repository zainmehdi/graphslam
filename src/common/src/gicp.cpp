#include <scanner.hpp>

ros::Publisher delta_pub;

const double converged_fitness_threshold = 0.5;
double k_disp_disp = 0.1, k_rot_disp = 0.1, k_rot_rot = 0.1;

pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
Eigen::Matrix4f carry_transform;

sensor_msgs::LaserScan robot_0_laserscan;
sensor_msgs::LaserScan robot_1_laserscan;

void gicp_register(sensor_msgs::PointCloud2 input_1,
				   sensor_msgs::PointCloud2 input_2,
				   Eigen::Matrix4f& transform) {
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_1 = format_pointcloud(input_1);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_2 = format_pointcloud(input_2);

  try {
    gicp.setInputSource(pointcloud_1);
    gicp.setInputTarget(pointcloud_2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_transform(new pcl::PointCloud<pcl::PointXYZ>);
    gicp.align(*pointcloud_transform, transform);
  }
  catch (int e) {
    std::cout << "An exception occurred. Exception Nr. " << e << '\n';
  }

  common::Registration output;

  ROS_INFO("Converged?: %d, Fitness Score: %f", gicp.hasConverged(), gicp.getFitnessScore());

  transform = gicp.getFinalTransformation();
  geometry_msgs::Pose2D transform_Delta = make_Delta(transform);

  Eigen::MatrixXd covariance_Delta = compute_covariance(k_disp_disp, k_rot_disp, k_rot_rot, transform_Delta);
  common::Pose2DWithCovariance Delta = create_Pose2DWithCovariance_msg(transform_Delta, covariance_Delta);

  ROS_INFO("x = %f; y = %f; theta = %f", Delta.pose.x, Delta.pose.y, Delta.pose.theta);
  ROS_INFO("[%f %f %f %f %f %f %f %f %f]", Delta.covariance[0],
	   Delta.covariance[1],
	   Delta.covariance[2],
	   Delta.covariance[3],
	   Delta.covariance[4],
	   Delta.covariance[5],
	   Delta.covariance[6],
	   Delta.covariance[7],
	   Delta.covariance[8]);
  
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

int main(int argc, char** argv) {
  ros::init(argc, argv, "scanner");
  ros::NodeHandle n;
  ros::Rate r(1);

  ros::Subscriber robot_0_scan_sub = n.subscribe("/robot_0/base_scan", 1, robot_0_scanner_callback);
  ros::Subscriber robot_1_scan_sub = n.subscribe("/robot_1/base_scan", 1, robot_1_scanner_callback);
  delta_pub = n.advertise<common::Pose2DWithCovariance>("/scanner/delta", 1);

  // Setup GICP algorithm
  gicp.setUseReciprocalCorrespondences(true);
  //  gicp.setMaxCorrespondenceDistance(20.0);
  //  gicp.setEuclideanFitnessEpsilon();
  //  gicp.setCorrespondenceRandomness();
  gicp.setMaximumIterations(500);
  gicp.setMaximumOptimizerIterations(50);
  //  gicp.setTransformationEpsilon(2e-3);
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
