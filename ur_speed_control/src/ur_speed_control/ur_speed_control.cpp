#include "ros/ros.h"
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>

geometry_msgs::TwistStamped desired_twist;

void twist_callback(const geometry_msgs::TwistStamped msg){desired_twist = msg;}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "ur_speed_control");
	ros::NodeHandle n;
	ros::Rate loop_rate(500);

	ros::Subscriber twist_sub = n.subscribe("twist_cmd", 1, &twist_callback);
	ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("act_pose", 1);

	ur_rtde::RTDEControlInterface* rtde_control = new ur_rtde::RTDEControlInterface("192.168.2.30");
	ur_rtde::RTDEReceiveInterface* rtde_receive = new ur_rtde::RTDEReceiveInterface("192.168.2.30");

	desired_twist.twist.linear.x = 0.0;
	desired_twist.twist.linear.y = 0.0;
	desired_twist.twist.linear.z = 0.0;

	desired_twist.twist.angular.x = 0.0;
	desired_twist.twist.angular.y = 0.0;
	desired_twist.twist.angular.z = 0.0;

	std::vector<double> desired_twist_dbl;
	desired_twist_dbl.resize(6);

	std::vector<double> pose_dbl;
	double angle;
	Eigen::Vector3d axis;
	geometry_msgs::PoseStamped pose;
	pose_dbl.resize(6);

	// TODO: da launchfile
	double max_acc = 0.25; // 2.5


	while (ros::ok()){

		desired_twist_dbl[0] = desired_twist.twist.linear.x;
		desired_twist_dbl[1] = desired_twist.twist.linear.y;
		desired_twist_dbl[2] = desired_twist.twist.linear.z;
		desired_twist_dbl[3] = desired_twist.twist.angular.x;
		desired_twist_dbl[4] = desired_twist.twist.angular.y;
		desired_twist_dbl[5] = desired_twist.twist.angular.z;

		// xd: tool speed [m/s] (spatial vector)
		// acceleration: tool position acceleration [m/s^2]
		// time: time [s] before the function returns (optional)
		rtde_control -> speedL(desired_twist_dbl, max_acc, 0.002);

		pose_dbl = rtde_receive->getTargetTCPPose();
		angle = sqrt(pow(pose_dbl[3],2) + pow(pose_dbl[4],2) + pow(pose_dbl[5],2));
		axis << pose_dbl[3], pose_dbl[4], pose_dbl[5]; axis = axis.normalized(); 

		pose.pose.position.x = pose_dbl[0];
		pose.pose.position.y = pose_dbl[1];
		pose.pose.position.z = pose_dbl[2];

		pose.pose.orientation.x = Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis)).x();
		pose.pose.orientation.y = Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis)).y();
		pose.pose.orientation.z = Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis)).z();
		pose.pose.orientation.w = Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis)).w();

		pose_pub.publish(pose);

		ros::spinOnce();
		loop_rate.sleep();

	}

	rtde_control->disconnect();
	return 0;
}


