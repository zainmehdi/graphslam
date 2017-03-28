#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <common/Factor.h>
#include <common/Keyframe.h>
#include <common/Graph.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

//#######################
//#### Add Map Saving Service
//#######################

geometry_msgs::PoseArray pose_optis;

void graph_callback(const common::Graph& input) {
  pose_optis.poses.clear();
  
  for(int i = 0; i < input.keyframes.size(); i++) {
    for(int j = 0; j < input.keyframes[i].scan.ranges.size(); j+=25) {
      geometry_msgs::Point pnt;
      double th = std::fmod(input.keyframes[i].pose_opti.pose.theta +
			    input.keyframes[i].scan.angle_min +
       			    ( j * input.keyframes[i].scan.angle_increment ) + M_PI, 2 * M_PI ) - M_PI;
      pnt.x = input.keyframes[i].pose_opti.pose.x + ( input.keyframes[i].scan.ranges[j] * cos( th ) );
      pnt.y = input.keyframes[i].pose_opti.pose.y + ( input.keyframes[i].scan.ranges[j] * sin( th ) );
      scan_marker_point.points.push_back(pnt);
    }
  }
}

int main( int argc, char** argv ) {
  ros::init(argc, argv, "map_builder");
  ros::NodeHandle n;
  ros::Rate(100);
  ros::Subscriber graph_sub = n.subscribe("/graph/graph", 1, graph_callback);
  
  while(ros::ok()) {
    loop_marker_pub.publish(loop_line_list);
    ros::spinOnce();
  }

  return 0;
}

