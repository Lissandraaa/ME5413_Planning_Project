/** path_tracker_node.cpp
 *
 * Copyright (C) 2024 Shuo SUN & Advanced Robotics Center, National University of Singapore
 *
 * MIT License
 *
 * ROS Node for robot to track a given path using Pure Pursuit algorithm
 */

#include "me5413_world/math_utils.hpp"
#include "me5413_world/path_tracker_node.hpp"

namespace me5413_world {

// Dynamic Parameters
double SPEED_TARGET; // Target speed for the robot
double PID_Kp, PID_Ki, PID_Kd; // PID controller parameters
bool PARAMS_UPDATED; // Flag to check if parameters are updated

void dynamicParamCallback(const me5413_world::path_trackerConfig& config, uint32_t level) {
    // Update control parameters from the dynamic reconfigure server
    SPEED_TARGET = config.speed_target;
    PID_Kp = config.PID_Kp;
    PID_Ki = config.PID_Ki;
    PID_Kd = config.PID_Kd;
    PARAMS_UPDATED = true;
}

PathTrackerNode::PathTrackerNode() : tf2_listener_(tf2_buffer_) {
    // Setup callback for dynamic reconfigure server
    f = boost::bind(&dynamicParamCallback, _1, _2);
    server.setCallback(f);

    // Subscribe to the robot's odometry and local path topics
    sub_robot_odom_ = nh_.subscribe("/gazebo/ground_truth/state", 1, &PathTrackerNode::robotOdomCallback, this);
    sub_local_path_ = nh_.subscribe("/me5413_world/planning/local_path", 1, &PathTrackerNode::localPathCallback, this);
    // Publish velocity commands
    pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/jackal_velocity_controller/cmd_vel", 1);

    // Initialization
    robot_frame_ = "base_link";
    world_frame_ = "world";
    pid_ = control::PID(0.1, 1.0, -1.0, PID_Kp, PID_Ki, PID_Kd);
}

void PathTrackerNode::localPathCallback(const nav_msgs::Path::ConstPtr& path) {
    // Update the robot's goal pose and publish control commands
    pose_world_goal_ = path->poses.back().pose;
    pub_cmd_vel_.publish(computeControlOutputs(this->odom_world_robot_));
}

void PathTrackerNode::robotOdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
    // Update the robot's odometry
    odom_world_robot_ = *odom;
}

geometry_msgs::Twist PathTrackerNode::computeControlOutputs(const nav_msgs::Odometry& odom_robot) {
    geometry_msgs::Twist cmd_vel;

    if (PARAMS_UPDATED) {
        // Update PID controller settings if the parameters were updated
        pid_.updateSettings(PID_Kp, PID_Ki, PID_Kd);
        PARAMS_UPDATED = false;
    }

    // Calculate angle to goal
    double angle_to_goal = calculateAngleToGoal(odom_robot.pose.pose, pose_world_goal_);
    double angular_z = pid_.calculate(0, angle_to_goal); // Assuming target angle difference is 0

    // Set the robot's speed and angular velocity
    cmd_vel.linear.x = SPEED_TARGET;
    cmd_vel.angular.z = angular_z;

    return cmd_vel;
}

double PathTrackerNode::calculateAngleToGoal(const geometry_msgs::Pose& current_pose, const geometry_msgs::Pose& goal_pose) {
    // Assuming this function calculates the angle from the robot's current pose to the goal pose
    // Placeholder for actual calculation
    double dy = goal_pose.position.y - current_pose.position.y;
    double dx = goal_pose.position.x - current_pose.position.x;
    double target_angle = atan2(dy, dx);
    double current_angle = tf2::getYaw(current_pose.orientation);
    double angle_diff = normalizeAngle(target_angle - current_angle);
    return angle_diff;
}

// Placeholder for normalizeAngle function
double normalizeAngle(double angle) {
    // Normalize the angle to the range [-pi, pi]
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}



} // namespace me5413_world

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_tracker_node");
    me5413_world::PathTrackerNode path_tracker_node;
    ros::spin();  // Keep the node running
    return 0;
}
