#include "magdrone_2/joy_controller.hpp"

// ROS
#include <tf2/LinearMath/Quaternion.h>

// ROS messages
#include <geometry_msgs/TwistStamped.h>

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>

joy_controller::joy_controller(ros::NodeHandle &nh)
{
    // Set up Subscribers
    joy_sub = nh.subscribe("/joy", 1, &joy_controller::joyCallback, this);
    state_sub = nh.subscribe("/mavros/state", 1, &joy_controller::stateCallback, this);

    // Set up Publishers
    vel_control_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 1);
    raw_vel_control_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
    raw_att_control_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1);
    rc_overide_control_pub = nh.advertise < mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1);

                                                // Set up Service Clients
                             arm_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    joy_command.axes.clear();
    for (int i = 0; i < 4; ++i)
        joy_command.axes.push_back(0.0);

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
    ROS_INFO("Initiallizing");
    for (int i = 25; ros::ok && i > 0; --i)
    {
        ros::spinOnce();
        publishCommand();
        rate.sleep();
    }

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
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ROS_INFO("Arming...");
    for (int i = 25; ros::ok && i > 0; --i)
    {
        if (arm_client.call(arm_cmd) && arm_cmd.response.success)
        {
            ROS_INFO("Armed");
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }

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
}

void joy_controller::stateCallback(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

// Publishers
void joy_controller::publishCommand()
{
    if (false)
    {
        geometry_msgs::TwistStamped cmd;

        cmd.twist.linear.x = 5.0 * joy_command.axes[2];
        cmd.twist.linear.y = 5.0 * joy_command.axes[1];
        cmd.twist.linear.z = 5.0 * joy_command.axes[0];

        cmd.twist.angular.z = 5.0 * joy_command.axes[3];

        vel_control_pub.publish(cmd);
    }
    else if (false)
    {
        mavros_msgs::PositionTarget cmd;

        // cmd.coordinate_frame = FRAME_BODY_NED;
        // cmd.type_mask = IGNORE_PX |
        //                 IGNORE_PY |
        //                 IGNORE_PZ |
        //                 IGNORE_AFX |
        //                 IGNORE_AFY |
        //                 IGNORE_AFZ |
        //                 IGNORE_YAW;
        cmd.coordinate_frame = 8;
        cmd.type_mask = 1479;
        cmd.velocity.x = 5.0 * joy_command.axes[2];
        cmd.velocity.y = 5.0 * joy_command.axes[1];
        cmd.velocity.z = 5.0 * joy_command.axes[0];
        cmd.yaw_rate = 5.0 * joy_command.axes[3];

        raw_vel_control_pub.publish(cmd);
    }
    else if (false)
    {
        tf2::Quaternion q;
        mavros_msgs::AttitudeTarget cmd;

        // cmd.type_mask = IGNORE_ROLL_RATE |
        //                 IGNORE_PITCH_RATE;
        cmd.type_mask = 3;

        // I need to rotate by vehicle's yaw if I want to control in body frame
        q.setRPY(-0.5*joy_command.axes[1], 0.5*joy_command.axes[2], 0);
        q.normalize();

        cmd.orientation.x = q[0];
        cmd.orientation.y = q[1];
        cmd.orientation.z = q[2];
        cmd.orientation.w = q[3];
        // cmd.body_rate.x = 1.0 * joy_command.axes[1];
        // cmd.body_rate.y = 1.0 * joy_command.axes[2];
        cmd.body_rate.z = 1.0 * joy_command.axes[3];
        cmd.thrust = 0.71 * (1.0 + joy_command.axes[0]);

        raw_att_control_pub.publish(cmd);
    }
    else if (true)
    {
        mavros_msgs::OverrideRCIn cmd;

        cmd.channels[0] = (uint16_t)(1500 + 600 * joy_command.axes[0]);
        cmd.channels[1] = (uint16_t)(1500 - 600 * joy_command.axes[1]);
        cmd.channels[2] = (uint16_t)(1500 + 600 * joy_command.axes[2]);
        cmd.channels[3] = (uint16_t)(1500 - 600 * joy_command.axes[3]);
        cmd.channels[4] = (uint16_t)(1500);
        cmd.channels[5] = (uint16_t)(1500);
        cmd.channels[6] = (uint16_t)(1500);
        cmd.channels[7] = (uint16_t)(1500);

        rc_overide_control_pub.publish(cmd);
    }
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