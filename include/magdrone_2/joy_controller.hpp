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
    /** vel_control_pub
     * Publishes a velocity command on the world ENU frame
     */
    ros::Publisher vel_control_pub;
    /** raw_vel_control_pub
     * Publishes a velocity command on the body FLU frame
     */
    ros::Publisher raw_vel_control_pub;
    /** raw_att_control_pub
     * Publishes an attitude command
     */
    ros::Publisher raw_att_control_pub;
    /** rc_overide_control_pub
     * Publishes an override of the RC (This might be a bad idea)
     */
    ros::Publisher rc_overide_control_pub;

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

    mavros_msgs::State current_state;
};