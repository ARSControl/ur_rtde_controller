#include "ros/ros.h"
#include <ur_rtde/rtde_control_interface.h>

//#include <ur_rtde/rtde_receive_interface.h>
//#include <geometry_msgs/TwistStamped.h>
//#include <geometry_msgs/PoseStamped.h>
//#include <Eigen/Dense>

//geometry_msgs::TwistStamped desired_twist;

//void twist_callback(const geometry_msgs::TwistStamped msg){desired_twist = msg;}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "ur_force_control");
	ros::NodeHandle n;
	ros::Rate loop_rate(500);

//	ros::Subscriber twist_sub = n.subscribe("twist_cmd", 1, &twist_callback);
//	ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("pose", 1);

	ur_rtde::RTDEControlInterface* rtde_control = new ur_rtde::RTDEControlInterface("192.168.2.30");
//	ur_rtde::RTDEReceiveInterface* rtde_receive = new ur_rtde::RTDEReceiveInterface("192.168.2.30");

//	desired_twist.twist.linear.x = 0.0;
//	desired_twist.twist.linear.y = 0.0;
//	desired_twist.twist.linear.z = 0.0;

//	desired_twist.twist.angular.x = 0.0;
//	desired_twist.twist.angular.y = 0.0;
//	desired_twist.twist.angular.z = 0.0;

//	std::vector<double> desired_twist_dbl;
//	desired_twist_dbl.resize(6);

	std::vector<double> task_frame;
	task_frame.resize(6);

	task_frame[0] = 0.0;
	task_frame[1] = 0.0;
	task_frame[2] = 0.0;
	task_frame[3] = 0.0;
	task_frame[4] = 0.0;
	task_frame[5] = 0.0;

	std::vector<int> selection_vector;
	selection_vector.resize(6);

	selection_vector[0] = 1;
	selection_vector[1] = 1;
	selection_vector[2] = 1;
	selection_vector[3] = 1;
	selection_vector[4] = 1;
	selection_vector[5] = 1;

	std::vector<double> wrench;
	wrench.resize(6);

	wrench[0] = 0.0;
	wrench[1] = 0.0;
	wrench[2] = 0.0;
	wrench[3] = 0.0;
	wrench[4] = 0.0;
	wrench[5] = 0.0;

	std::vector<double> limits;
	limits.resize(6);

	limits[0] = 1.0;
	limits[1] = 1.0;
	limits[2] = 1.0;
	limits[3] = 0.2;
	limits[4] = 0.2;
	limits[5] = 0.2;

//	double angle;
//	Eigen::Vector3d axis;
//	geometry_msgs::PoseStamped pose;
//	pose_dbl.resize(6);

	rtde_control -> forceModeSetDamping(0.0);
	rtde_control -> forceMode(task_frame, selection_vector, wrench, 2, limits);
	rtde_control -> forceModeSetGainScaling(1.0);

	while (ros::ok()){




//		desired_twist_dbl[0] = desired_twist.twist.linear.x;
//		desired_twist_dbl[1] = desired_twist.twist.linear.y;
//		desired_twist_dbl[2] = desired_twist.twist.linear.z;
//		desired_twist_dbl[3] = desired_twist.twist.angular.x;
//		desired_twist_dbl[4] = desired_twist.twist.angular.y;
//		desired_twist_dbl[5] = desired_twist.twist.angular.z;

//		rtde_control -> speedL(desired_twist_dbl, 0.25, 0.0);

//		pose_dbl = rtde_receive->getTargetTCPPose();
//		angle = sqrt(pow(pose_dbl[3],2) + pow(pose_dbl[4],2) + pow(pose_dbl[5],2));
//		axis << pose_dbl[3], pose_dbl[4], pose_dbl[5]; axis = axis.normalized(); 

//		pose.pose.position.x = pose_dbl[0];
//		pose.pose.position.y = pose_dbl[1];
//		pose.pose.position.z = pose_dbl[2];

//		pose.pose.orientation.x = Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis)).x();
//		pose.pose.orientation.y = Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis)).y();
//		pose.pose.orientation.z = Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis)).z();
//		pose.pose.orientation.w = Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis)).w();

//		pose_pub.publish(pose);

		ros::spinOnce();
		loop_rate.sleep();

	}

	rtde_control -> forceModeStop();
	rtde_control->disconnect();
	return 0;
}


