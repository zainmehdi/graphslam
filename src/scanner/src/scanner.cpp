#include <scanner.hpp>

ros::Publisher registration_pub;
ros::Publisher pointcloud_debug_pub;
ros::Publisher delta_pub;
ros::ServiceClient keyframe_last_client;
ros::ServiceClient keyframe_closest_client;

// Tuning constants:
const double converged_fitness_threshold = 0.01; // TODO migrate to rosparams
double k_disp_disp = 0.1, k_rot_disp = 0.1, k_rot_rot = 0.1; // TODO migrate to rosparams

// GICP algorithm
//pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
Eigen::Matrix4f carry_transform;

common::Registration gicp_register(sensor_msgs::PointCloud2 input_1, sensor_msgs::PointCloud2 input_2, Eigen::Matrix4f& transform) {


  // assign inputs
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_1 = format_pointcloud(input_1);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_2 = format_pointcloud(input_2);
  gicp.setInputSource(pointcloud_1);
  gicp.setInputTarget(pointcloud_2);

  // align
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_transform(new pcl::PointCloud<pcl::PointXYZ>);
  gicp.align(*pointcloud_transform, transform);

  common::Registration output;

  output.keyframe_flag = false;
  if (gicp.hasConverged())
  {
      // Get transformation Delta and compute its covariance
      transform = gicp.getFinalTransformation();
      geometry_msgs::Pose2D transform_Delta = make_Delta(transform);

      Eigen::MatrixXd covariance_Delta = compute_covariance(k_disp_disp, k_rot_disp, k_rot_rot, transform_Delta);
      common::Pose2DWithCovariance Delta = create_Pose2DWithCovariance_msg(transform_Delta, covariance_Delta);

      // Assign to outputs
      output.factor_new.delta  = Delta;
      output.factor_loop.delta = Delta;

      if (gicp.getFitnessScore() > converged_fitness_threshold)
      {
          output.keyframe_flag = true;
      }
  }

  return output;
}

common::Registration gicp_register(sensor_msgs::PointCloud2 input_1, sensor_msgs::PointCloud2 input_2) {
    Eigen::Matrix4f guess_null(Eigen::Matrix4f::Identity());
    return gicp_register(input_1, input_2, guess_null);
}

void scanner_callback(const sensor_msgs::LaserScan& input) {

  // clear flags:
  common::Registration output;
  output.first_frame_flag = false;
  output.keyframe_flag = false;
  output.loop_closure_flag = false;

  // request last KF
  common::LastKeyframe keyframe_last_request;
  bool keyframe_last_request_returned = keyframe_last_client.call(keyframe_last_request);

  // Case of first frame
  if(!keyframe_last_request_returned) {
    ROS_INFO("### NO LAST KEYFRAME FOUND ###");
   
    // Set flags, assign pointcloud, and publish
    output.first_frame_flag = true;
    output.keyframe_flag = false;
    output.loop_closure_flag = false;
    output.keyframe_new.scan = input;
    output.keyframe_new.pointcloud = scan_to_pointcloud(input);
    registration_pub.publish(output);
  }

  // Case of other frames
  if(keyframe_last_request_returned) {
    sensor_msgs::PointCloud2 input_pointcloud = scan_to_pointcloud(input);
    sensor_msgs::PointCloud2 keyframe_last_pointcloud = keyframe_last_request.response.keyframe_last.pointcloud;

    double start = ros::Time::now().toSec();

    common::Registration registration_last = gicp_register(input_pointcloud, keyframe_last_pointcloud, carry_transform);

    double end = ros::Time::now().toSec();
    ROS_INFO("align time: %f", end - start);

//    std::cout << "scanner_callback::Transform: \n" << carry_transform << std::endl;
    
    output.keyframe_flag = registration_last.keyframe_flag;
    output.keyframe_new.ts = input.header.stamp;
    output.keyframe_new.pointcloud = input_pointcloud;
    output.keyframe_new.scan = input;
    output.keyframe_last = keyframe_last_request.response.keyframe_last;
    output.factor_new.id_1 = keyframe_last_request.response.keyframe_last.id;
    output.factor_new.id_2 = output.keyframe_new.id;
    output.factor_new.delta = registration_last.factor_new.delta;

    // Check for loop closures only if on Keyframes
    if (registration_last.keyframe_flag){
        carry_transform.setIdentity();
          
      common::ClosestKeyframe keyframe_closest_request;
      keyframe_closest_request.request.keyframe_last = keyframe_last_request.response.keyframe_last;
      bool keyframe_closest_request_returned = keyframe_closest_client.call(keyframe_closest_request);

      if(keyframe_closest_request_returned) {
	// get pointcloud and  register
	sensor_msgs::PointCloud2 keyframe_closest_pointcloud =
	  keyframe_closest_request.response.keyframe_closest.pointcloud;

	common::Registration registration_closest = gicp_register(keyframe_closest_pointcloud, keyframe_last_pointcloud, carry_transform);

	// compute factor things
	output.loop_closure_flag = true;
	output.keyframe_last = keyframe_last_request.response.keyframe_last;
	output.keyframe_loop = keyframe_closest_request.response.keyframe_closest;
	output.factor_loop.id_1 = keyframe_last_request.response.keyframe_last.id;
	output.factor_loop.id_2 = keyframe_closest_request.response.keyframe_closest.id;
	output.factor_loop.delta = registration_closest.factor_loop.delta;
      }
    }
    
    registration_pub.publish(output);
  }

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "scanner");
  ros::NodeHandle n;

  ros::Subscriber scanner_sub = n.subscribe("/base_scan", 1, scanner_callback);
//  keyframe_IDs = 0;

  delta_pub = n.advertise<geometry_msgs::Pose2D>("scanner/delta", 1);
  
  registration_pub  = n.advertise<common::Registration>("/scanner/registration", 1);
  pointcloud_debug_pub = n.advertise<sensor_msgs::PointCloud2>("/scanner/debug_pointcloud", 1);

  keyframe_last_client = n.serviceClient<common::LastKeyframe>("/graph/last_keyframe");
  keyframe_closest_client = n.serviceClient<common::ClosestKeyframe>("/graph/closest_keyframe");

  // Setup ICP algorithm
  gicp.setUseReciprocalCorrespondences(true);
  //  gicp.setMaxCorrespondenceDistance(20.0);
  //  gicp.setEuclideanFitnessEpsilon();
  //  gicp.setCorrespondenceRandomness();
    gicp.setMaximumIterations(500);
  //  gicp.setTransformationEpsilon(2e-3);
  //  gicp.setRotationEpsilon();

    // Setup GICP algorithm
    //    gicp.setMaximumOptimizerIterations(50);

  carry_transform.setIdentity();

  ros::spin();
  return 0;
}
