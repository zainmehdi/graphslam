#include <math.h>
#include <vector>
#include <deque>

#include <ros/ros.h>
#include <ros/geometry_msgs/Twist.h>

#include <geo>

#include "common/Odometry.h"
#include "common/Pose2DWithCovariance.h"
#include "common/OdometryBuffer.h"

#include "utils.hpp"


// Tuning constants:
// double k_d_d = 0.1, k_r_d = 0.1, k_r_r = 0.1; // TODO migrate to rosparams
double sigma_vx = 0.1, sigma_vy = 0.1, sigma_vth = 0.1; // TODO migrate to rosparams


struct Twist2D{
        double vx, vy, vth;
        void setZero() { vx = vy = vth = 0; }
        static Twist2D Zero() { Twist2D tw; tw.setZero(); return tw; }
} twist;


std::deque<common::Odometry> buffer_odom; // JS: changed vector --> deque for an efficient pop_front().


void vel_callback(const geometry_msgs::Twist::ConstPtr& input) {
  twist.vx  = input->linear.x;
  twist.vy  = input->linear.y;
  twist.vth = input->angular.z;
}

void add_to_buffer(const common::Odometry& input) {
  if(buffer_odom.size() > 1000) {
      buffer_odom.pop_front();
  }

  buffer_odom.push_back(input);
}

// JS: Rename and adjust API to 'between(keyframe , keyframe)' as in the document
// JS: or, otherwise, just leave as a buffer request, in such case return only one odometry instance, not the increment between two instances.
bool odometry_buffer_request(common::OdometryBuffer::Request &req, common::OdometryBuffer::Response &res) {
  int buffer_search_position = 0;
  int t_start_buffer_position = 0;
  int t_end_buffer_position = 0;
  int t_start = (int) req.t_start.toSec();
  int t_end = (int) req.t_end.toSec();
  bool t_start_found = false;
  bool t_end_found = false;
  std::vector<common::Odometry> odometry_buffer_frozen = buffer_odom;

  for(int i = 0; i < buffer_odom.size(); i++) {
    buffer_search_position = (int) odometry_buffer_frozen[i].ts.toSec();

    if(buffer_search_position == t_start) {
      t_start_buffer_position = i;
      t_start_found = true;
    }

    if(buffer_search_position == t_end) {
      t_end_buffer_position = i;
      t_end_found = true;
    }
  }

  if(t_start_found && t_end_found) {
    common::Pose2DWithCovariance t_start_pose   = odometry_buffer_frozen[t_start_buffer_position].pose;
    common::Pose2DWithCovariance t_end_pose     = odometry_buffer_frozen[t_end_buffer_position].pose;
    res.delta = between(t_start_pose, t_end_pose); // JS: we want one pose returned, not the pose increment
    return true;
  }

  return false;
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "odometry");
    ros::NodeHandle n;

    ros::Subscriber vel_sub = n.subscribe("/cmd_vel_modified", 1, vel_callback);

    ros::Publisher odom_pub = n.advertise < common::Odometry > ("/odometry/odometry", 1);

    ros::Time current_time  = ros::Time::now();
    ros::Time last_time     = current_time;
    ros::Rate loop_rate(100);

    twist.setZero();

    geometry_msgs::Pose2D pose_odom, delta;
    pose_odom.x = pose_odom.y = pose_odom.theta = 0;

    while (ros::ok())
    {
        // time integration into small delta
        current_time    = ros::Time::now();
        double delta_t  = (current_time - last_time).toSec();
        delta.x         = twist.vx  * delta_t;
        delta.y         = twist.vy  * delta_t;
        delta.theta     = twist.vth * delta_t;

        // integrate into global odometry pose
        pose_odom       = compose(pose_odom, delta);

        // construct Odometry message
        common::Odometry odometry;
        odometry.ts         = current_time;
        odometry.pose.pose  = pose_odom;
        // odometry.pose.covariance = [...] // TODO Only if necessary ! JS: By now, do not do it.

        // Add to buffer and publish
        add_to_buffer(odometry);
        odom_pub.publish(odometry);

        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::spin();
    return 0;
}
