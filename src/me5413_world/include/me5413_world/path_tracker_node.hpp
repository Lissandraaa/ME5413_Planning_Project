#ifndef PATH_TRACKER_NODE_H_
#define PATH_TRACKER_NODE_H_

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <dynamic_reconfigure/server.h>
#include <me5413_world/path_trackerConfig.h>

#include "pid.hpp"

namespace me5413_world {

class PathTrackerNode {
 public:
  PathTrackerNode();
  virtual ~PathTrackerNode() {}

 private:
  // Callback functions
  void robotOdomCallback(const nav_msgs::Odometry::ConstPtr& odom);
  void controlLoopCallback(const ros::TimerEvent&);
  
  // Utility functions
  geometry_msgs::Twist computeControlOutputs(const nav_msgs::Odometry& odom);

  // ROS-related members
  ros::NodeHandle nh_;
  ros::Timer control_loop_timer_;
  ros::Subscriber odom_subscriber_;
  ros::Publisher cmd_vel_publisher_;

  // Dynamic reconfigure server
  dynamic_reconfigure::Server<me5413_world::path_trackerConfig> config_server_;

  // Robot state and parameters
  nav_msgs::Odometry current_odom_;
  double look_ahead_distance_;

  // PID controllers for speed and steering control
  PID speed_pid_;
  PID steering_pid_;
};

PathTrackerNode::PathTrackerNode() :
  look_ahead_distance_(1.0), // This value should be set based on the robot size and speed
  speed_pid_(0.1, 0.01, 0.0), // Initialize with some default PID values
  steering_pid_(0.1, 0.01, 0.0) { // Initialize with some default PID values

  odom_subscriber_ = nh_.subscribe("odom", 10, &PathTrackerNode::robotOdomCallback, this);
  cmd_vel_publisher_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  control_loop_timer_ = nh_.createTimer(ros::Duration(0.1), &PathTrackerNode::controlLoopCallback, this);

  // Initialize dynamic reconfigure server
  config_server_.setCallback(boost::bind(&PathTrackerNode::reconfigureCallback, this, _1, _2));
}

void PathTrackerNode::robotOdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
  current_odom_ = *odom;
}

void PathTrackerNode::controlLoopCallback(const ros::TimerEvent&) {
  geometry_msgs::Twist cmd_vel = computeControlOutputs(current_odom_);
  cmd_vel_publisher_.publish(cmd_vel);
}

geometry_msgs::Twist PathTrackerNode::computeControlOutputs(const nav_msgs::Odometry& odom) {
  geometry_msgs::Twist cmd_vel;

  // Example implementation
  double target_speed = speed_pid_.calculate(look_ahead_distance_, odom.twist.twist.linear.x);
  double steering_angle = steering_pid_.calculate(0.0, odom.pose.pose.orientation.z); // Placeholder for actual calculation

  cmd_vel.linear.x = target_speed;
  cmd_vel.angular.z = steering_angle;

  return cmd_vel;
}

// Dynamic reconfigure callback function
void PathTrackerNode::reconfigureCallback(me5413_world::path_trackerConfig& config, uint32_t level) {
  look_ahead_distance_ = config.look_ahead_distance;
  speed_pid_.setParams(config.speed_pid_p, config.speed_pid_i, config.speed_pid_d);
  steering_pid_.setParams(config.steering_pid_p, config.steering_pid_i, config.steering_pid_d);
}

}  // namespace me5413_world

#endif  // PATH_TRACKER_NODE_H_
