#include <scanner.hpp>

ros::Publisher registration_pub;
ros::Publisher pointcloud_debug_pub;
ros::Publisher delta_pub;
ros::ServiceClient keyframe_last_client;
ros::ServiceClient keyframe_closest_client;

// Thresholds for voting for keyframe:
const double converged_fitness_threshold = 999; // [adimensional] TODO migrate to rosparams
const double distance_threshold = 1; // [m] TODO migrate to rosparams
const double rotation_threshold = 1; // [rad] TODO migrate to rosparams
const unsigned int loop_closure_skip = 4;

// Uncertainty model constants
const double k_disp_disp = 0.001, k_rot_disp = 0.002, k_rot_rot = 0.003; // TODO migrate to rosparams

// GICP algorithm
// alignement output
struct Alignement{
        bool converged;
        float fitness;
        pcl::registration::DefaultConvergenceCriteria<float>::ConvergenceState convergence_state;
        Eigen::Matrix4f transform;
        common::Pose2DWithCovariance Delta;
};

//pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;

Eigen::Matrix4f carry_transform; // The transform of the last align which is passed to the next align as initial guess
unsigned int loop_closure_skip_count;

// Helper functions

bool vote_for_keyframe(const common::Pose2DWithCovariance Delta, const double fitness)
{
    if (fitness > converged_fitness_threshold) // fitness
        return true;
    if (fabs(Delta.pose.theta) > rotation_threshold) // rotation
        return true;
    if ((Delta.pose.x*Delta.pose.x+Delta.pose.y*Delta.pose.y) > distance_threshold*distance_threshold) // translation
        return true;

    return false;
}

Alignement gicp_register(const sensor_msgs::PointCloud2 input_1, const sensor_msgs::PointCloud2 input_2, Eigen::Matrix4f& transform){

    // assign inputs
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_1 = format_pointcloud(input_1);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_2 = format_pointcloud(input_2);
    gicp.setInputSource(pointcloud_1);
    gicp.setInputTarget(pointcloud_2);

    // align
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_transform(new pcl::PointCloud<pcl::PointXYZ>);
    gicp.align(*pointcloud_transform, transform);
//    ROS_INFO("Convergence state %d", gicp.getConvergeCriteria()->getConvergenceState());

    Alignement output;
    output.convergence_state = gicp.getConvergeCriteria()->getConvergenceState();
    output.converged = gicp.hasConverged();

    if (gicp.hasConverged())
    {
        transform = gicp.getFinalTransformation();

        output.fitness = gicp.getFitnessScore();

        // Get transformation Delta and compute its covariance
        output.transform = transform;
        geometry_msgs::Pose2D transform_Delta = make_Delta(transform);
        Eigen::MatrixXd covariance_Delta = compute_covariance(k_disp_disp, k_rot_disp, k_rot_rot, transform_Delta);
        output.Delta = create_Pose2DWithCovariance_msg(transform_Delta, covariance_Delta);
    }

    return output;
}

Alignement gicp_register(const sensor_msgs::PointCloud2 input_1, const sensor_msgs::PointCloud2 input_2){
    Eigen::Matrix4f guess_null(Eigen::Matrix4f::Identity());
    return gicp_register(input_1, input_2, guess_null);
}



// Node functions

void scanner_callback(const sensor_msgs::LaserScan& input)
{

    // clear flags:
    common::Registration output;
    output.first_frame_flag = false;
    output.keyframe_flag = false;
    output.loop_closure_flag = false;

    // request last KF
    common::LastKeyframe keyframe_last_request;
    bool keyframe_last_request_returned = keyframe_last_client.call(keyframe_last_request);

    // Case of first frame
    if (!keyframe_last_request_returned)
    {
        ROS_INFO("### NO LAST KEYFRAME FOUND ###");

        // Set flags, assign pointcloud, and publish
        output.first_frame_flag         = true;
        output.keyframe_flag            = false;
        output.loop_closure_flag        = false;
        output.keyframe_new.scan        = input;
        output.keyframe_new.pointcloud  = scan_to_pointcloud(input);
        registration_pub.publish(output);
    }

    // Case of other frames
    if (keyframe_last_request_returned)
    {
        sensor_msgs::PointCloud2 input_pointcloud = scan_to_pointcloud(input);
        sensor_msgs::PointCloud2 keyframe_last_pointcloud = keyframe_last_request.response.keyframe_last.pointcloud;

        double start = ros::Time::now().toSec();

        gicp.setMaxCorrespondenceDistance(1); // 1m for close range
        Alignement alignement_last = gicp_register(input_pointcloud, keyframe_last_pointcloud, carry_transform);


        double end = ros::Time::now().toSec();

        //    ROS_INFO("align time: %f", end - start);
        //    std::cout << "scanner_callback::Transform: \n" << carry_transform << std::endl;

        output.keyframe_flag            = vote_for_keyframe(alignement_last.Delta, alignement_last.fitness);
        output.keyframe_new.ts          = input.header.stamp;
        output.keyframe_new.pointcloud  = input_pointcloud;
        output.keyframe_new.scan        = input;
        output.keyframe_last            = keyframe_last_request.response.keyframe_last;
        output.factor_new.id_1          = keyframe_last_request.response.keyframe_last.id;
        output.factor_new.id_2          = output.keyframe_new.id;
        output.factor_new.delta         = alignement_last.Delta;

        // Check for loop closures only if on Keyframes
        if (output.keyframe_flag)
        {
            carry_transform.setIdentity();

            loop_closure_skip_count++;
            if ((loop_closure_skip_count % loop_closure_skip) == 0) // only try once in a while
            {
//                loop_closure_skip_count = 0;

                common::ClosestKeyframe keyframe_closest_request;
                keyframe_closest_request.request.keyframe_last = keyframe_last_request.response.keyframe_last;
                bool keyframe_closest_request_returned = keyframe_closest_client.call(keyframe_closest_request);

                if (keyframe_closest_request_returned)
                {
                    // compute prior transform from 2 keyframes
                    Eigen::Matrix4f T_last = make_transform(keyframe_last_request.response.keyframe_last.pose_opti.pose);
                    Eigen::Matrix4f T_loop = make_transform(keyframe_closest_request.response.keyframe_closest.pose_opti.pose);
                    Eigen::Matrix4f loop_transform = T_last.inverse()*T_loop;

                    // get pointcloud and  register
                    sensor_msgs::PointCloud2 keyframe_closest_pointcloud =
                            keyframe_closest_request.response.keyframe_closest.pointcloud;

                    gicp.setMaxCorrespondenceDistance(5); // 5m for loop closure
                    Alignement alignement_loop = gicp_register(keyframe_closest_pointcloud, keyframe_last_pointcloud,
                                                                loop_transform);

                    ROS_INFO("LC: convergence state: %d", alignement_loop.convergence_state);
                    ROS_INFO("LC: Delta: %f %f %f", alignement_loop.Delta.pose.x, alignement_loop.Delta.pose.y, alignement_loop.Delta.pose.theta);

                    // compute factor things
                    output.loop_closure_flag        = alignement_loop.converged;
                    if (alignement_loop.converged)
                    {
                        output.keyframe_last        = keyframe_last_request.response.keyframe_last;
                        output.keyframe_loop        = keyframe_closest_request.response.keyframe_closest;
                        output.factor_loop.id_1     = keyframe_last_request.response.keyframe_last.id;
                        output.factor_loop.id_2     = keyframe_closest_request.response.keyframe_closest.id;
                        output.factor_loop.delta    = alignement_loop.Delta;
                    }
                }
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
  //  gicp.setCorrespondenceRandomness();
  //  gicp.setRotationEpsilon();
  gicp.setMaximumIterations(50); // ICP example 50
  gicp.setMaxCorrespondenceDistance(1); // ICP example 0.05
  gicp.setTransformationEpsilon(1e-8); // ICP example 1e-8
  gicp.setEuclideanFitnessEpsilon(0.1); // ICP example 1

  // Setup GICP algorithm
  //    gicp.setMaximumOptimizerIterations(50);

  carry_transform.setIdentity();
  loop_closure_skip_count = 0;

  ros::spin();
  return 0;
}
