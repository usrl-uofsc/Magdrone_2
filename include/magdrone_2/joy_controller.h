#pragma once

// ROS
#include <ros/ros.h>

// ROS messages
#include <mavros_msgs/State.h>
#include <sensor_msgs/Joy.h>

class joy_controller
{
public:
    joy_controller(ros::NodeHandle &nh);
    ~joy_controller();

    void publishCommand();

private:
    // ROS Subscribers
    ros::Subscriber joy_sub;
    ros::Subscriber state_sub;

    // ROS Publishers
    /** raw_vel_control_pub
     * Publishes a velocity command on the body FLU frame
     */
    ros::Publisher raw_vel_control_pub;

    // ROS Services
    ros::ServiceClient arm_client;
    ros::ServiceClient set_mode_client;

    // Callbacks
    void joyCallback(const sensor_msgs::Joy::ConstPtr &msg);
    void stateCallback(const mavros_msgs::State::ConstPtr &msg);

    // Functions
    bool initializeDrone();

    // Data
    sensor_msgs::Joy joy_command;
    bool run_test;
    ros::Time test_start;
    int loop_it;

    mavros_msgs::State current_state;
};