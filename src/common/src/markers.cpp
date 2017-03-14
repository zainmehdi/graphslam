#include <ros/ros.h>
#include <common/Keyframe.h>
#include <common/Keyframes.h>
#include <common/Graph.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

geometry_msgs::PoseArray pose_optis;
visualization_msgs::Marker points, line_strip, line_list;

void graph_callback(const common::Graph& input) {
  //  ROS_INFO("%d", input.edges.size());
  pose_optis.poses.clear();
  points.points.clear();
  line_strip.points.clear();
  line_list.points.clear();
  
  for(int i = 0; i < input.keyframes.size(); i++) {
    geometry_msgs::Pose pose;
    pose.position.x = input.keyframes[i].pose_opti.pose.x;
    pose.position.y = input.keyframes[i].pose_opti.pose.y;
    pose.orientation = tf::createQuaternionMsgFromYaw(input.keyframes[i].pose_opti.pose.theta);
    pose_optis.poses.push_back(pose);
  }

  for(int i = 0; i < input.edges.size(); i++) {
    geometry_msgs::Pose2D id_1_pose, id_2_pose;
    bool p_1_found = false;
    bool p_2_found = false;
    
    for(int j = 0; j < input.keyframes.size(); j++) {
      if(input.keyframes[j].id == input.edges[i].id_1) {
	id_1_pose = input.keyframes[i].pose_opti.pose;
	p_1_found = true;
      }
    }

    for(int j = 0; j < input.keyframes.size(); j++) {
      if(input.keyframes[j].id == input.edges[i].id_2) {
	id_2_pose = input.keyframes[i].pose_opti.pose;
	p_2_found = true;
      }
    }

    if(p_1_found && p_2_found && input.edges[i].id_1 != input.edges[i].id_2 ) {
      geometry_msgs::Point p_1;
      p_1.x = id_1_pose.x;
      p_1.y = id_1_pose.y;
      ROS_INFO("p_1 %f %f", id_1_pose.x, id_1_pose.y);
      points.points.push_back(p_1);
      line_strip.points.push_back(p_1);

      geometry_msgs::Point p_2;
      p_2.x = id_2_pose.x;
      p_2.y = id_2_pose.y;
      ROS_INFO("p_2 %f %f", id_2_pose.x, id_2_pose.y);
      points.points.push_back(p_2);
      line_strip.points.push_back(p_2);
      line_list.points.push_back(p_2);
    }
  }
}

int main( int argc, char** argv ) {
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate(100);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  ros::Publisher pose_array_pub = n.advertise<geometry_msgs::PoseArray>("/keyframe/poses", 50);
  ros::Subscriber graph_sub = n.subscribe("/graph/graph", 1, graph_callback);
  pose_optis.header.frame_id = "odom";
  points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "odom";
  points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
  points.ns = line_strip.ns = line_list.ns = "points_and_lines";
  points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;
  points.id = 0;
  line_strip.id = 1;
  line_list.id = 2;
  points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  points.scale.x = 0.2;
  points.scale.y = 0.2;
  line_strip.scale.x = 0.1;
  line_list.scale.x = 0.1;
  points.color.g = 1.0f;
  points.color.a = 1.0;
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;
  line_list.color.r = 1.0;
  line_list.color.a = 1.0;
  

  while(ros::ok()) {
    pose_array_pub.publish(pose_optis);
    marker_pub.publish(points);
    marker_pub.publish(line_strip);
    marker_pub.publish(line_list);
    ros::spinOnce();
  }

  return 0;
}

