#pragma once

#include <string>
#include <thread>
#include <mutex>

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
    px4_attitude_controller(ros::NodeHandle &nh, double publisher_rate);
    ~px4_attitude_controller();

    void publishCommand(double publisher_rate);

    std::thread t_worker;

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
    void changeMode(std::string &mode);
    void callArm(bool &state);

    // Data
    mavros_msgs::State current_state;
    double current_yaw;
    sensor_msgs::Joy joy_command;
    mavros_msgs::AttitudeTarget cmd;

    std::unique_ptr<std::mutex> status_mutex;
    std::unique_ptr<std::mutex> pose_mutex;
}; // px4_attitude_controller