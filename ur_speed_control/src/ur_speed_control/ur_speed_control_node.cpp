#include "ur_speed_control/ur_speed_control.h"

void UR_Stop_Signal_Handler (int sig) {

	// ---- UR RTDE LIBRARY ---- //
	ur_rtde::RTDEControlInterface *rtde_control_ = new ur_rtde::RTDEControlInterface("192.168.2.30");

	rtde_control_ -> stopL(2.0);
	rtde_control_ -> disconnect();

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "ur_speed_control", ros::init_options::NoSigintHandler);

    // Sig-Int Function (CTRL+C)
    signal(SIGINT, UR_Stop_Signal_Handler);

    ros::NodeHandle nh;
    ros::Rate loop_rate = 500;

    URSpeedControl *usc = new URSpeedControl(nh, loop_rate);

    while (ros::ok()) {

        usc -> spinner();

    }

    delete usc;

return 0;

}
