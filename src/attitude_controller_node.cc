#include "magdrone_2/attitude_controller.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "attitude_controller_node");
    ros::NodeHandle nh;
    px4_attitude_controller px4_pilot(nh, 30.0);

    ros::spin();

    px4_pilot.t_worker.join();

    return 0;
}