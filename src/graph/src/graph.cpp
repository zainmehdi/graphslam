#include <graph.hpp>
#include <common/Factor.h>
#include <common/Graph.h>

// #### TUNING CONSTANTS START
double sigma_xy_prior = 0.1; // TODO migrate to rosparams
double sigma_th_prior = 0.1; // TODO migrate to rosparams
int keyframes_to_skip_in_loop_closing = 4; // TODO migrate to rosparams
// #### TUNING CONSTANTS END

gtsam::NonlinearFactorGraph graph;
gtsam::Values poses_initial;
std::vector<common::Keyframe> keyframes; // JS: Better use map<key,Keyframe> where key = ID, as in gtsam::Values
std::vector<common::Factor> factors;
int keyframe_IDs; // Simple ID factory for keyframes.

ros::Publisher graph_pub;


void publish_graph() {
  common::Graph output;
  for(int i = 0; i < keyframes.size(); i++) {
    output.keyframes.push_back(keyframes[i]);
  }

  for(int i = 0; i < factors.size(); i++) {
    output.factors.push_back(factors[i]);
  }

  graph_pub.publish(output);
}

void prior_factor(common::Registration input) {

  // Advance keyframe ID factory
  keyframe_IDs++;

  // Define prior state and noise model
  double x_prior = 0;
  double y_prior = 0;
  double th_prior = 0;

  Eigen::MatrixXd Q(3, 3);
  Q.setZero();
  Q(0, 0) = sigma_xy_prior * sigma_xy_prior;
  Q(1, 1) = sigma_xy_prior * sigma_xy_prior;
  Q(2, 2) = sigma_th_prior * sigma_th_prior;

  gtsam::Pose2 pose_prior(x_prior, y_prior, th_prior);
  gtsam::noiseModel::Gaussian::shared_ptr noise_prior = gtsam::noiseModel::Gaussian::Covariance(Q);

  // Define new KF
  input.keyframe_new.id = keyframe_IDs;
  // input.keyframe_new.pose_odom = // TODO: get odometry pose from odometry_pose service.
  // input.keyframe_new.pose_opti = create_Pose2DWithCovariance_msg(x_prior, y_prior, th_prior, Q); // TODO fix this
  input.keyframe_new.pose_opti.pose.x  = x_prior;
  input.keyframe_new.pose_opti.pose.y  = y_prior;
  input.keyframe_new.pose_opti.pose.theta = th_prior;
  keyframes.push_back(input.keyframe_new);

  // Add factor and prior to the graph
  graph.add(gtsam::PriorFactor<gtsam::Pose2>(input.keyframe_new.id, pose_prior, noise_prior));
  poses_initial.insert(input.keyframe_new.id, pose_prior);

  // print debug info
  ROS_INFO("PRIOR FACTOR ID=%d CREATED. %lu KF, %lu Factor, 0 loops",
	   input.keyframe_new.id, keyframes.size(), graph.nrFactors());
} 

void new_factor(common::Registration input) {

  // Advance keyframe ID factory
  keyframe_IDs++;

  // Compute new KF pose
  common::Pose2DWithCovariance pose_new_msg = compose(input.keyframe_last.pose_opti, input.factor_new.delta);
  gtsam::Pose2 pose_new(pose_new_msg.pose.x, pose_new_msg.pose.y, pose_new_msg.pose.theta);

  // Define new KF
  input.keyframe_new.id = keyframe_IDs;
  input.keyframe_new.pose_opti = pose_new_msg;
  // input.keyframe_new.pose_odom = // TODO: get odometry pose from odometry_pose service.
  keyframes.push_back(input.keyframe_new);

  // Define new factor
  input.factor_new.id_2 = input.keyframe_new.id;
  Eigen::MatrixXd Q = covariance_to_eigen(input.factor_new);
  gtsam::noiseModel::Gaussian::shared_ptr noise_delta = gtsam::noiseModel::Gaussian::Covariance(Q);

  // Add factor and state to the graph
  poses_initial.insert(input.keyframe_new.id, pose_new);
  graph.add(gtsam::BetweenFactor<gtsam::Pose2>(input.factor_new.id_1,
					       input.factor_new.id_2,
					       gtsam::Pose2(input.factor_new.delta.pose.x,
							    input.factor_new.delta.pose.y,
							    input.factor_new.delta.pose.theta),
					       noise_delta));
  common::Factor factor = input.factor_new;
  factor.loop = false;
  factors.push_back(factor);

  // print debug info
//  ROS_INFO("Edge pushed %d % d", factor.id_1, factor.id_2);
  ROS_INFO("MOTION FACTOR %d-->%d. %lu KFs, %lu Factors, %lu Loops",
	   input.factor_new.id_1, input.factor_new.id_2, keyframes.size(), graph.nrFactors(), graph.nrFactors() - keyframes.size());
}

void loop_factor(common::Registration input)
{

    // Define new factor
    Eigen::MatrixXd Q = covariance_to_eigen(input.factor_loop);
    gtsam::noiseModel::Gaussian::shared_ptr noise_delta = gtsam::noiseModel::Gaussian::Covariance(Q);

    // Add factor to the graph
    graph.add(gtsam::BetweenFactor<gtsam::Pose2>(input.factor_loop.id_1,
                                                 input.factor_loop.id_2,
                                                 gtsam::Pose2(input.factor_loop.delta.pose.x,
                                                              input.factor_loop.delta.pose.y,
                                                              input.factor_loop.delta.pose.theta),
                                                 noise_delta));
    common::Factor factor = input.factor_loop;
    factor.loop = true;
    factors.push_back(factor);

    // print debug info
    ROS_INFO("LOOP FACTOR %d-->%d. %lu KFs, %lu Factors, %lu Loops",
	     input.factor_loop.id_1, input.factor_loop.id_2, keyframes.size(), graph.nrFactors(), graph.nrFactors() - keyframes.size());
}

void solve() {

  gtsam::Values poses_optimized = gtsam::LevenbergMarquardtOptimizer(graph, poses_initial).optimize();


  for(int i = 0; i < keyframes.size(); i++) {
    keyframes[i].pose_opti.pose.x = poses_optimized.at<gtsam::Pose2>(keyframes[i].id).x();
    keyframes[i].pose_opti.pose.y = poses_optimized.at<gtsam::Pose2>(keyframes[i].id).y();
    keyframes[i].pose_opti.pose.theta = poses_optimized.at<gtsam::Pose2>(keyframes[i].id).theta();
//    Eigen::MatrixXd pose_opti_covariance = marginals.marginalCovariance(keyframes[i].id);
//    keyframes[i].pose_opti = eigen_to_covariance(keyframes[i].pose_opti, pose_opti_covariance);
  }

  // get ready for next iteration: set next initial to the currently optimized values
  poses_initial = poses_optimized;

  ROS_INFO("SOLVE FINISHED.");
}

bool last_keyframe(common::LastKeyframe::Request &req, common::LastKeyframe::Response &res) {

  if(!keyframes.empty()) {
    res.keyframe_last = keyframes.back();

    return true;
  }

  return false;
}

bool closest_keyframe(common::ClosestKeyframe::Request &req, common::ClosestKeyframe::Response &res) {

  if(!keyframes.empty()) {
    std::vector<double> distances;

    if(keyframes.size() > keyframes_to_skip_in_loop_closing) {
      for(int i = 0; i < keyframes.size() - keyframes_to_skip_in_loop_closing; i++) {
	double x1 = req.keyframe_last.pose_opti.pose.x;
	double y1 = req.keyframe_last.pose_opti.pose.y;
	double x2 = keyframes[i].pose_opti.pose.x;
	double y2 = keyframes[i].pose_opti.pose.y;
	distances.push_back( sqrt( pow( x2 - x1, 2 ) + pow( y2 - y1, 2 ) ) );
      }

      int minimum_keyframe_index = 0;
      for(int i = 0; i < distances.size(); i++) {
	if(distances[i] < distances[minimum_keyframe_index]) {
	  minimum_keyframe_index = i;
	}
      }

      res.keyframe_closest = keyframes[minimum_keyframe_index];
      ROS_INFO("CLOSEST KEYFRAME ID=%d SERVICE FINISHED.", keyframes[minimum_keyframe_index].id);
      return true;
    } else {
      ROS_INFO("CLOSEST KEYFRAME SERVICE FINISHED. Not enough keyframes.");
      return false;
    }
  }

  ROS_INFO("CLOSEST KEYFRAME SERVICE FINISHED. No keyframes available.");
  return false;
}

void registration_callback(const common::Registration& input) {

  if(input.first_frame_flag) {
      ROS_INFO("--------------------------------------------");
      prior_factor(input);
      publish_graph();
      if (!keyframes.empty())
          ROS_INFO("Global pose: %f %f %f", keyframes.back().pose_opti.pose.x,keyframes.back().pose_opti.pose.y,keyframes.back().pose_opti.pose.theta);
      ROS_INFO("--------------------------------------------");
  }

  else if(input.keyframe_flag) {
      new_factor(input);

      if(input.loop_closure_flag) {
          loop_factor(input);
          solve();
      }

      publish_graph();
      ROS_INFO("Laser Delta: %f %f %f", input.factor_new.delta.pose.x, input.factor_new.delta.pose.y, input.factor_new.delta.pose.theta);
      if (!keyframes.empty())
          ROS_INFO("Global pose: %f %f %f", keyframes.back().pose_opti.pose.x,keyframes.back().pose_opti.pose.y,keyframes.back().pose_opti.pose.theta);
      ROS_INFO("--------------------------------------------");
  }

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "graph");
  ros::NodeHandle n;
  
  // Init ID factory
  keyframe_IDs = 0;

  graph_pub = n.advertise<common::Graph>("/graph/graph", 1);
  ros::Subscriber registration_sub = n.subscribe("/scanner/registration", 1, registration_callback);
  ros::ServiceServer last_keyframe_service = n.advertiseService("/graph/last_keyframe", last_keyframe);
  ros::ServiceServer closest_keyframe_service = n.advertiseService("/graph/closest_keyframe", closest_keyframe);

  ros::spin();
  return 0;
}
