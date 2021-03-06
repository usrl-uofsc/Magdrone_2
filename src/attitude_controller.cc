#include "magdrone_2/attitude_controller.h"

#include <cmath>

// ROS
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

px4_attitude_controller::px4_attitude_controller(ros::NodeHandle &nh, double publisher_rate)
{
    // Set up Subscribers
    joy_sub = nh.subscribe("/joy", 1, &px4_attitude_controller::joyCallback, this);
    state_sub = nh.subscribe("/mavros/state", 1, &px4_attitude_controller::stateCallback, this);
    pose_sub = nh.subscribe("/mavros/local_position/pose", 1, &px4_attitude_controller::poseCallback, this);

    // Set up Publishers
    raw_att_control_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1);

    // Set up Service Clients
    arm_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // Initialize Parameters
    joy_command.axes.clear();
    for (int i = 0; i < 4; ++i)
        joy_command.axes.push_back(0.0);

    // Command flag:
    // - Attitude for roll and pitch
    // - Yaw rate for orientation
    // - Normalized thrust
    cmd.type_mask = cmd.IGNORE_ROLL_RATE |
                    cmd.IGNORE_PITCH_RATE;

    status_mutex.reset(new std::mutex);
    pose_mutex.reset(new std::mutex);
    t_worker = std::thread(&px4_attitude_controller::publishCommand, this, 30);
}

px4_attitude_controller::~px4_attitude_controller() {}

void px4_attitude_controller::changeMode(std::string &mode)
{
    ros::Rate rate(5);

    // Set control mode to offboard
    mavros_msgs::SetMode offboard_mode;
    offboard_mode.request.custom_mode = mode;

    ROS_INFO("Changing mode to %s", mode.c_str());
    for (int i = 25; ros::ok && i > 0; --i)
    {
        if (set_mode_client.call(offboard_mode) && offboard_mode.response.mode_sent)
        {
            ROS_INFO("Offboard enabled");
            break;
        }
        rate.sleep();
    }
}

void px4_attitude_controller::callArm(bool &state)
{
    ros::Rate rate(5);

    // Arm drone
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = state;

    if (state)
        ROS_INFO("Arming...");
    else
        ROS_INFO("Disarming...");

    for (int i = 25; ros::ok && i > 0; --i)
    {
        if (arm_client.call(arm_cmd) && arm_cmd.response.success)
        {
            ROS_INFO("Done");
            break;
        }
        rate.sleep();
    }
}

// Callbacks
void px4_attitude_controller::joyCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
    joy_command.axes.clear();
    joy_command.axes.push_back(msg->axes[1]);
    joy_command.axes.push_back(msg->axes[2]);
    joy_command.axes.push_back(msg->axes[3]);
    joy_command.axes.push_back(msg->axes[0]);

    // Set mode to Offboard
    if (msg->buttons[0] == 1)
    {
        std::string mode;
        if (current_state.mode != "OFFBOARD")
            mode = "OFFBOARD";
        else
            mode = "POSCTL";

        changeMode(mode);
    }

    // Send Arm call
    if (msg->buttons[1] == 1)
    {
        bool arm_call = !current_state.armed;
        callArm(arm_call);
    }
}

void px4_attitude_controller::stateCallback(const mavros_msgs::State::ConstPtr &msg)
{
    std::lock_guard<std::mutex> status_guard(*(status_mutex));
    current_state = *msg;
}

void px4_attitude_controller::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    double q_w = msg->pose.orientation.w;
    double q_x = msg->pose.orientation.x;
    double q_y = msg->pose.orientation.y;
    double q_z = msg->pose.orientation.z;

    double sycp = 2.0 * (q_w * q_z + q_x * q_y);
    double cycp = 1.0 - 2.0 * (q_y * q_y + q_z * q_z);

    std::lock_guard<std::mutex> pose_guard(*(pose_mutex));
    current_yaw = std::atan2(sycp, cycp);
}

// Publishers
void px4_attitude_controller::publishCommand(double publisher_rate)
{
    ros::Rate rate(publisher_rate);

    // Make sure vehicle is connected
    ROS_INFO("Connecting to vehicle");
    while (true)
    {
        bool status_ok = false;
        { // Lock status mutex
            std::lock_guard<std::mutex> status_guard(*(status_mutex));
            status_ok = current_state.connected;
        }
        if (status_ok)
            break;
    }
    ROS_INFO("Vehicle is connected");

    while (ros::ok())
    {
        bool status_ok;
        { // Lock status mutex
            std::lock_guard<std::mutex> status_guard(*(status_mutex));
            status_ok = current_state.mode == "OFFBOARD" && current_state.connected;
        }
        if (status_ok)
        {
            tf2::Quaternion q;
            { // Lock pose_mutex
                std::lock_guard<std::mutex> pose_guard(*(pose_mutex));
                q.setRPY(-0.5 * joy_command.axes[1], 0.5 * joy_command.axes[2], current_yaw);
            }
            q.normalize();

            cmd.orientation.x = q[0];
            cmd.orientation.y = q[1];
            cmd.orientation.z = q[2];
            cmd.orientation.w = q[3];

            cmd.body_rate.z = 1.0 * joy_command.axes[3];

            cmd.thrust = 0.35 * (1.0 + joy_command.axes[0]);
        }
        raw_att_control_pub.publish(cmd);
        rate.sleep();
    }
}