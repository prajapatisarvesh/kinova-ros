#include <ros/ros.h>
#include <kinova_msgs/JointVelocity.h>
#include <kinova_msgs/ClearTrajectories.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <iostream>
#include <sensor_msgs/JointState.h>
#include <signal.h>
#include <stdio.h>
#include <vector>
#include <Eigen/Dense>
#include <fstream>
#include <nlohmann/json.hpp>
#include <iostream>
#include <string>
#include <chrono>
using json = nlohmann::json;
json data;

void write_data(void);
// Total Joints
int joints_ = 6;
bool start_pid = false;
// Global variable current position
sensor_msgs::JointState jaco_position;
Eigen::VectorXd jaco_state(joints_);
Eigen::VectorXd mrt_target(joints_);
// Global variable target position
trajectory_msgs::JointTrajectory jaco_trajectory;
// Error Vector
Eigen::VectorXd error_(joints_);
// Prev_Error Vector
Eigen::VectorXd prev_error_(joints_);
// Integral Vector
Eigen::VectorXd integral_(joints_);
// PID vector
Eigen::VectorXd pid_(joints_);
// Output Vector  
Eigen::VectorXd output_(joints_);
Eigen::VectorXd propotional(joints_);
Eigen::VectorXd derivative(joints_);
Eigen::VectorXd integral(joints_);
//PID Parameters
double p = 100.0;
double i = 0.3;
double d = 0;
double dt = 100;
// Kinova_velocity msg
kinova_msgs::JointVelocity jaco_velocity;
double max_ = 35.0;
double min_ = -35.0;
ros::Publisher velocity_publisher;
void position_listener(trajectory_msgs::JointTrajectory trajectory);
void jaco_feedback(sensor_msgs::JointState joint_state);
void shutdown_handler(int sig);
void pid_callback(const ros::TimerEvent &);
void velocity_listener(kinova_msgs::JointVelocity target_velocity);
// MRT Fail Safe
ros::Time time_;
ros::Time data_start_time;
// JSON Vars
std::vector<double> target_joint_velocity(6, 0);
std::vector<double> current_velocity(6, 0);
std::vector<double> data_time_;
std::string file_name;
kinova_msgs::JointVelocity target_vel;
kinova_msgs::ClearTrajectories clear_trajectory;
ros::ServiceClient clear_trajectory_service;