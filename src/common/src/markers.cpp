#include <ros/ros.h>
#include <common/Keyframe.h>
#include <common/Keyframes.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

geometry_msgs::PoseArray pose_optis;

void keyframes_callback(const common::Keyframes& input) {
  pose_optis.poses.clear();
  pose_optis.header.frame_id = "odom";
  
  for(int i = 0; i < input.keyframes.size(); i++) {
    geometry_msgs::Pose pose;
    pose.position.x = input.keyframes[i].pose_opti.pose.x;
    pose.position.y = input.keyframes[i].pose_opti.pose.y;
    pose.orientation = tf::createQuaternionMsgFromYaw(input.keyframes[i].pose_opti.pose.theta);
    pose_optis.poses.push_back(pose);
  }  
}

int main( int argc, char** argv ) {
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate(100);
  ros::Publisher pose_array_pub = n.advertise<geometry_msgs::PoseArray>("/keyframe/poses", 50);
  ros::Subscriber keyframe_sub = n.subscribe("/graph/keyframes", 1, keyframes_callback);

  while(ros::ok()) {
    pose_array_pub.publish(pose_optis);
    ros::spinOnce();
  }

  return 0;
}

