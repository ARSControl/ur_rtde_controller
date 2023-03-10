#include "cartesian_controller/cartesian_controller.h"

int main(int argc, char **argv) {

    ros::init(argc, argv, "cartesian_controller");

    ros::NodeHandle nh;
    ros::Rate loop_rate = 500;

    CartesianController *cc = new CartesianController(nh, loop_rate);

    while (ros::ok()) {

        cc -> spinner();

    }

    delete cc;

return 0;

}
