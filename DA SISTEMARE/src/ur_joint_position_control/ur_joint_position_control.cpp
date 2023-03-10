#include "ros/ros.h"
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include "trajectory_msgs/JointTrajectory.h"
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>

trajectory_msgs::JointTrajectory desired_joint_position;

void joint_position_callback(const trajectory_msgs::JointTrajectory msg){desired_joint_position = msg;}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "ur_joint_position_control");
	ros::NodeHandle n;
	ros::Rate loop_rate(500);

	ros::Subscriber joint_position_sub = n.subscribe("joint_position_cmd", 1, &joint_position_callback);

	ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("pose", 1);
	ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("joint_state", 1);

	ur_rtde::RTDEControlInterface* rtde_control = new ur_rtde::RTDEControlInterface("192.168.2.30");
	ur_rtde::RTDEReceiveInterface* rtde_receive = new ur_rtde::RTDEReceiveInterface("192.168.2.30");

	std::vector<double> desired_joint_position_dbl, joint_position_dbl;
	desired_joint_position_dbl.resize(6);
	joint_position_dbl.resize(6);

	sensor_msgs::JointState joint_state;
	joint_state.position.resize(6);

	std::vector<double> pose_dbl;
	double angle;
	Eigen::Vector3d axis;
	geometry_msgs::PoseStamped pose;
	pose_dbl.resize(6);

	desired_joint_position.points.resize(1);
	desired_joint_position.points[0].positions.resize(6);

	joint_position_dbl = rtde_receive -> getActualQ();

	desired_joint_position.points[0].positions[0] = joint_position_dbl[0];
	desired_joint_position.points[0].positions[1] = joint_position_dbl[1];
	desired_joint_position.points[0].positions[2] = joint_position_dbl[2];
	desired_joint_position.points[0].positions[3] = joint_position_dbl[3];
	desired_joint_position.points[0].positions[4] = joint_position_dbl[4];
	desired_joint_position.points[0].positions[5] = joint_position_dbl[5];

	while (ros::ok()){

		desired_joint_position_dbl[0] = desired_joint_position.points[0].positions[0];
		desired_joint_position_dbl[1] = desired_joint_position.points[0].positions[1];
		desired_joint_position_dbl[2] = desired_joint_position.points[0].positions[2];
		desired_joint_position_dbl[3] = desired_joint_position.points[0].positions[3];
		desired_joint_position_dbl[4] = desired_joint_position.points[0].positions[4];
		desired_joint_position_dbl[5] = desired_joint_position.points[0].positions[5];

		rtde_control -> servoJ(desired_joint_position_dbl, 0.0, 0.0, 0.002, 0.1, 100.0);

		joint_position_dbl = rtde_receive -> getActualQ();

		joint_state.position[0] = joint_position_dbl[0];
		joint_state.position[1] = joint_position_dbl[1];
		joint_state.position[2] = joint_position_dbl[2];
		joint_state.position[3] = joint_position_dbl[3];
		joint_state.position[4] = joint_position_dbl[4];
		joint_state.position[5] = joint_position_dbl[5];

		pose_dbl = rtde_receive -> getTargetTCPPose();
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
		joint_state_pub.publish(joint_state);

		ros::spinOnce();
		loop_rate.sleep();

	}

	rtde_control->disconnect();
	return 0;
}


