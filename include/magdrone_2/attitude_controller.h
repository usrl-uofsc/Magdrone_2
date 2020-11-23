#pragma once

#include <string>

// ROS
#include <ros/ros.h>

// ROS messages
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Joy.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>

class px4_attitude_controller
{
public:
    px4_attitude_controller(ros::NodeHandle &nh);
    ~px4_attitude_controller();

    void publishCommand();

private:
    // ROS Subscribers
    ros::Subscriber joy_sub;
    ros::Subscriber state_sub;
    ros::Subscriber pose_sub;

    // ROS Publishers
    ros::Publisher raw_att_control_pub;

    // ROS Services
    ros::ServiceClient arm_client;
    ros::ServiceClient set_mode_client;

    // Callbacks
    void joyCallback(const sensor_msgs::Joy::ConstPtr &msg);
    void stateCallback(const mavros_msgs::State::ConstPtr &msg);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    // Functions
    bool initializeDrone();
    void changeMode(std::string &mode);
    void callArm(bool &state);

    // Data
    mavros_msgs::State current_state;
    double current_yaw;
    sensor_msgs::Joy joy_command;
    mavros_msgs::AttitudeTarget cmd;
}; // px4_attitude_controller