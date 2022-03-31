#include "ur_speed_control/ur_speed_control.h"

int main(int argc, char **argv) {

    ros::init(argc, argv, "ur_speed_control");

    ros::NodeHandle nh;
    ros::Rate loop_rate = 500;

    URSpeedControl *usc = new URSpeedControl(nh, loop_rate);

    while (ros::ok()) {

        usc -> spinner();

    }

    delete usc;

return 0;

}
