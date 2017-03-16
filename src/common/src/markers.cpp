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

geometry_msgs::PoseArray pose_optis;
visualization_msgs::Marker keyframe_points, keyframe_line_strip; //, keyframe_line_list;
visualization_msgs::Marker loop_points, loop_line_list;

void graph_callback(const common::Graph& input) {
  pose_optis.poses.clear();
  keyframe_points.points.clear();
  keyframe_line_strip.points.clear();
  loop_points.points.clear();
  loop_line_list.points.clear();
  
  // loop all keyframes
  for(int i = 0; i < input.keyframes.size(); i++) {
      // create pose
    geometry_msgs::Pose pose;
    pose.position.x = input.keyframes[i].pose_opti.pose.x;
    pose.position.y = input.keyframes[i].pose_opti.pose.y;
    pose.orientation = tf::createQuaternionMsgFromYaw(input.keyframes[i].pose_opti.pose.theta);

    // append arrow marker
    pose_optis.poses.push_back(pose);

    // create point
    geometry_msgs::Point pnt;
    pnt.x = pose.position.x;
    pnt.y = pose.position.y;

    // append position and motion factor markers to the line strip of all motions
    keyframe_line_strip.points.push_back(pnt);
    keyframe_points.points.push_back(pnt);
  }

  // loop all factors looking for loop closures
  for(int i = 0; i < input.factors.size(); i++) {
      if(input.factors[i].loop) {
    geometry_msgs::Pose2D pose_1, pose_2;
    bool p_1_found = false;
    bool p_2_found = false;
      
    // find 1st keyframe
    for(int j = 0; j < input.keyframes.size(); j++) {
      if(input.keyframes[j].id == input.factors[i].id_1) {
	pose_1 = input.keyframes[j].pose_opti.pose;
	p_1_found = true;
      }
    }

    // find 2nd keyframe
    for(int j = 0; j < input.keyframes.size(); j++) {
      if(input.keyframes[j].id == input.factors[i].id_2) {
	pose_2 = input.keyframes[j].pose_opti.pose;
	p_2_found = true;
      }
    }

    // append loop pints and factors
    if(p_1_found && p_2_found && input.factors[i].id_1 != input.factors[i].id_2 ) {
      geometry_msgs::Point pnt_1;
      pnt_1.x = pose_1.x;
      pnt_1.y = pose_1.y;
      geometry_msgs::Point pnt_2;
      pnt_2.x = pose_2.x;
      pnt_2.y = pose_2.y;
      

      loop_points.points.push_back(pnt_1);
      loop_points.points.push_back(pnt_2);
      loop_line_list.points.push_back(pnt_1);
      loop_line_list.points.push_back(pnt_2);
      }
    }
  }
}

int main( int argc, char** argv ) {
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate(100);
  ros::Publisher keyframe_marker_pub = n.advertise<visualization_msgs::Marker>("keyframe_marker", 50);
  ros::Publisher loop_marker_pub = n.advertise<visualization_msgs::Marker>("loop_marker", 50);
  ros::Publisher pose_array_pub = n.advertise<geometry_msgs::PoseArray>("/keyframe/poses", 50);
  ros::Subscriber graph_sub = n.subscribe("/graph/graph", 1, graph_callback);

  // Keyframe poses arrow markers
  pose_optis.header.frame_id = "odom";

  // Keyframe position markers
  keyframe_points.id = 0;
  keyframe_points.type = visualization_msgs::Marker::SPHERE_LIST;
  keyframe_points.action = visualization_msgs::Marker::ADD;
  keyframe_points.scale.x = 0.1;
  keyframe_points.scale.y = 0.1;
  keyframe_points.scale.z = 0.1;
  keyframe_points.color.g = 1.0f;
  keyframe_points.color.a = 1.0;
  keyframe_points.header.frame_id = "odom";
  keyframe_points.header.stamp = ros::Time::now();
  keyframe_points.ns = "keyframe_points_and_lines";
  keyframe_points.pose.orientation.w = 1.0;

  // Motion factor segments
  keyframe_line_strip.id = 1;
  keyframe_line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  keyframe_line_strip.action = visualization_msgs::Marker::ADD;
  keyframe_line_strip.scale.x = 0.02;
  keyframe_line_strip.color.r = 0.1;
  keyframe_line_strip.color.g = 0.1;
  keyframe_line_strip.color.b = 1.0;
  keyframe_line_strip.color.a = 1.0;
  keyframe_line_strip.header.frame_id = "odom";
  keyframe_line_strip.pose.orientation.w = 1.0;
  keyframe_line_strip.header.stamp = ros::Time::now();
  keyframe_line_strip.ns = "keyframe_points_and_lines";

  // loop closure position markers
  loop_points.id = 1;
  loop_points.type = visualization_msgs::Marker::SPHERE_LIST;
  loop_points.action = visualization_msgs::Marker::ADD;
  loop_points.scale.x = 0.1;
  loop_points.scale.y = 0.1;
  loop_points.scale.z = 0.1;
  loop_points.color.r = 1.0f;
  loop_points.color.g = 1.0f;
  loop_points.color.b = 1.0f;
  loop_points.color.a = 1.0;
  loop_points.header.frame_id = "odom";
  loop_points.header.stamp = ros::Time::now();
  loop_points.ns = "loop_points_and_lines";
  loop_points.pose.orientation.w = 1.0;

  // loop closure factor segments
  loop_line_list.id = 0;
  loop_line_list.type = visualization_msgs::Marker::LINE_LIST;
  loop_line_list.scale.x = 0.02;
  loop_line_list.color.r = 1.0;
  loop_line_list.color.a = 1.0;
  loop_line_list.header.frame_id = "odom";
  loop_line_list.header.stamp = ros::Time::now();
  loop_line_list.ns = "loop_points_and_lines";
  loop_line_list.pose.orientation.w = 1.0;
  
  while(ros::ok()) {
    pose_array_pub.publish(pose_optis);
    keyframe_marker_pub.publish(keyframe_points);
    loop_marker_pub.publish(loop_points);
    keyframe_marker_pub.publish(keyframe_line_strip);
    loop_marker_pub.publish(loop_line_list);
    ros::spinOnce();
  }

  return 0;
}

