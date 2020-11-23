#include "magdrone_2/joy_controller.hpp"

// ROS
#include <tf2/LinearMath/Quaternion.h>

// ROS messages
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>

joy_controller::joy_controller(ros::NodeHandle &nh)
{
    // Set up Subscribers
    joy_sub = nh.subscribe("/joy", 1, &joy_controller::joyCallback, this);
    state_sub = nh.subscribe("/mavros/state", 1, &joy_controller::stateCallback, this);

    // Set up Publishers
    pose_control_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1);
    //vel_control_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 1);
    raw_vel_control_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
    //raw_att_control_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1);
    //rc_overide_control_pub = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1);

    // Set up Service Clients
    arm_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    joy_command.axes.clear();
    for (int i = 0; i < 4; ++i)
        joy_command.axes.push_back(0.0);
    run_test = false;

    initializeDrone();
}

joy_controller::~joy_controller() {}

bool joy_controller::initializeDrone()
{
    ros::Rate rate(5);

    // Check that FCU is connected
    for (int i = 25; ros::ok && i > 0; --i)
    {
        ros::spinOnce();
        rate.sleep();

        if (current_state.connected)
            break;
    }

    if (current_state.connected)
    {
        ROS_INFO("Vehicle is connected");
    }
    else
    {
        ROS_INFO("Vehicle not connected");
        return false;
    }

    // Send a few commands before starting
//    ROS_INFO("Initiallizing");
//    for (int i = 25; ros::ok && i > 0; --i)
//    {
//        ros::spinOnce();
//        publishCommand();
//        rate.sleep();
//    }

    // Set control mode to offboard
    mavros_msgs::SetMode offboard_mode;
    offboard_mode.request.custom_mode = "OFFBOARD";

    ROS_INFO("Setting mode to Offboard");
    for (int i = 25; ros::ok && i > 0; --i)
    {
        if (set_mode_client.call(offboard_mode) && offboard_mode.response.mode_sent)
        {
            ROS_INFO("Offboard enabled");
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }

    // Arm drone
//    mavros_msgs::CommandBool arm_cmd;
//    arm_cmd.request.value = true;

//    ROS_INFO("Arming...");
//    for (int i = 25; ros::ok && i > 0; --i)
//    {
//        if (arm_client.call(arm_cmd) && arm_cmd.response.success)
//        {
//            ROS_INFO("Armed");
//            break;
//        }
//        ros::spinOnce();
//        rate.sleep();
//    }

    // // Take off and center at (0, 0, 1)
    // geometry_msgs::PoseStamped pose;
    // pose.pose.position.x = 0;
    // pose.pose.position.y = 0;
    // pose.pose.position.z = 1;

    // ROS_INFO("Initiallizing");
    // for (int i = 50; ros::ok && i > 0; --i)
    // {
    //     ros::spinOnce();
    //     pose_control_pub.publish(pose);
    //     rate.sleep();
    // }

    return true;
}

// Callbacks
void joy_controller::joyCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
    joy_command.axes.clear();
    joy_command.axes.push_back(msg->axes[1]);
    joy_command.axes.push_back(msg->axes[2]);
    joy_command.axes.push_back(msg->axes[3]);
    joy_command.axes.push_back(msg->axes[0]);

    if (msg->buttons[0] == 1) {
        run_test = not run_test;
        test_start = ros::Time::now();
        loop_it = 0;

        if (run_test)
            ROS_INFO("Starting the velocity tuning test");
        else
            ROS_INFO("Stoping the velocity tuning test");
    }
}

void joy_controller::stateCallback(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

// Publishers
void joy_controller::publishCommand()
{
    mavros_msgs::PositionTarget cmd;

    cmd.header.stamp = ros::Time::now();

    cmd.coordinate_frame = cmd.FRAME_BODY_NED;
    cmd.type_mask = cmd.IGNORE_PX |
                    cmd.IGNORE_PY |
                    cmd.IGNORE_PZ |
                    cmd.IGNORE_AFX |
                    cmd.IGNORE_AFY |
                    cmd.IGNORE_AFZ |
                    cmd.IGNORE_YAW;

    cmd.velocity.z = 0.0;
    cmd.velocity.y = 0.0;
    cmd.velocity.x = 0.0;

    if (run_test)
    {
        double dt = (ros::Time::now() - test_start).toSec();

        if (dt < (4 + 16 * loop_it))
            cmd.velocity.x = 0.5;
        else if (dt < (8 + 16 * loop_it))
            cmd.velocity.x = 0.0;
        else if (dt < (12 + 16 * loop_it))
            cmd.velocity.x = -0.5;
        else if (dt < (16 + 16 * loop_it))
            cmd.velocity.x = 0.0;
        else
            ++loop_it;
    }
    raw_vel_control_pub.publish(cmd);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_controller_node");
    ros::NodeHandle nh;

    joy_controller joy_pilot(nh);

    ros::Rate rate(30);

    while (ros::ok())
    {
        ros::spinOnce();
        joy_pilot.publishCommand();
        rate.sleep();
    }

    return 0;
}
