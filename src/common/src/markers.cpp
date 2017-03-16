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
visualization_msgs::Marker keyframe_points, keyframe_line_strip, keyframe_line_list;
visualization_msgs::Marker loop_points, loop_line_list;

void graph_callback(const common::Graph& input) {
  pose_optis.poses.clear();
  keyframe_points.points.clear();
  keyframe_line_strip.points.clear();
  keyframe_line_list.points.clear();
  loop_points.points.clear();
  loop_line_list.points.clear();
  
  for(int i = 0; i < input.keyframes.size(); i++) {
    geometry_msgs::Pose pose;
    pose.position.x = input.keyframes[i].pose_opti.pose.x;
    pose.position.y = input.keyframes[i].pose_opti.pose.y;
    pose.orientation = tf::createQuaternionMsgFromYaw(input.keyframes[i].pose_opti.pose.theta);
    pose_optis.poses.push_back(pose);
  }

  for(int i = 0; i < input.factors.size(); i++) {
    geometry_msgs::Pose2D id_1_pose, id_2_pose;
    bool p_1_found = false;
    bool p_2_found = false;
      
    for(int j = 0; j < input.keyframes.size(); j++) {
      if(input.keyframes[j].id == input.factors[i].id_1) {
	id_1_pose = input.keyframes[j].pose_opti.pose;
	p_1_found = true;
      }
    }

    for(int j = 0; j < input.keyframes.size(); j++) {
      if(input.keyframes[j].id == input.factors[i].id_2) {
	id_2_pose = input.keyframes[j].pose_opti.pose;
	p_2_found = true;
      }
    }

    if(p_1_found && p_2_found && input.factors[i].id_1 != input.factors[i].id_2 ) {
      geometry_msgs::Point p_1;
      p_1.x = id_1_pose.x;
      p_1.y = id_1_pose.y;
      geometry_msgs::Point p_2;
      p_2.x = id_2_pose.x;
      p_2.y = id_2_pose.y;
      
      if(input.factors[i].loop) {
	loop_points.points.push_back(p_1);
	loop_points.points.push_back(p_2);
	loop_line_list.points.push_back(p_1);
	loop_line_list.points.push_back(p_2);
      } else {
	keyframe_points.points.push_back(p_1);
	keyframe_points.points.push_back(p_2);
	keyframe_line_strip.points.push_back(p_1);
	keyframe_line_strip.points.push_back(p_2);
	keyframe_line_list.points.push_back(p_1);
	keyframe_line_list.points.push_back(p_2);
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
  pose_optis.header.frame_id = "odom";
  keyframe_points.header.frame_id = keyframe_line_strip.header.frame_id = keyframe_line_list.header.frame_id = "odom";
  loop_points.header.frame_id = loop_line_list.header.frame_id = "odom";
  keyframe_points.header.stamp = keyframe_line_strip.header.stamp = keyframe_line_list.header.stamp = ros::Time::now();
  loop_points.header.stamp = loop_line_list.header.stamp = ros::Time::now();
  keyframe_points.ns = keyframe_line_strip.ns = keyframe_line_list.ns = "keyframe_points_and_lines";
  loop_points.ns = loop_line_list.ns = "loop_points_and_lines";
  keyframe_points.action = keyframe_line_strip.action = keyframe_line_list.action = visualization_msgs::Marker::ADD;
  loop_points.action = loop_line_list.action = visualization_msgs::Marker::ADD;
  keyframe_points.pose.orientation.w = keyframe_line_strip.pose.orientation.w = keyframe_line_list.pose.orientation.w = 1.0;
  loop_points.pose.orientation.w = loop_line_list.pose.orientation.w = 1.0;
  keyframe_points.id = 0;
  loop_points.id = 1;
  keyframe_line_strip.id = 1;
  loop_line_list.id = 0;
  keyframe_line_list.id = 2;
  keyframe_points.type = visualization_msgs::Marker::POINTS;
  loop_points.type = visualization_msgs::Marker::POINTS;
  keyframe_line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  loop_line_list.type = visualization_msgs::Marker::LINE_LIST;
  keyframe_line_list.type = visualization_msgs::Marker::LINE_LIST;
  keyframe_points.scale.x = loop_points.scale.x = 0.1;
  keyframe_points.scale.y = loop_points.scale.y = 0.1;
  keyframe_line_strip.scale.x = loop_line_list.scale.x = 0.03;
  keyframe_line_list.scale.x = 0.03;
  keyframe_points.color.g = 1.0f;
  keyframe_points.color.a = 1.0;
  keyframe_line_strip.color.b = 1.0;
  keyframe_line_strip.color.a = 1.0;
  keyframe_line_list.color.b = 1.0;
  keyframe_line_list.color.a = 1.0;
  loop_points.color.r = 1.0f;
  loop_points.color.g = 1.0f;
  loop_points.color.b = 1.0f;
  loop_points.color.a = 1.0;
  loop_line_list.color.r = 1.0;
//  loop_line_list.color.g = 1.0;
//  loop_line_list.color.b = 1.0;
  loop_line_list.color.a = 1.0;
  
  while(ros::ok()) {
    pose_array_pub.publish(pose_optis);
    keyframe_marker_pub.publish(keyframe_points);
    loop_marker_pub.publish(loop_points);
    keyframe_marker_pub.publish(keyframe_line_strip);
    loop_marker_pub.publish(loop_line_list);
//    keyframe_marker_pub.publish(keyframe_line_list);
    ros::spinOnce();
  }

  return 0;
}

