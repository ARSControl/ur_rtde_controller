#include "ur_speed_control/ur_speed_control.h"

URSpeedControl::URSpeedControl(ros::NodeHandle &nh, ros::Rate ros_rate): nh_(nh), ros_rate_(ros_rate)
{

	// ---- LOAD PARAMETERS ---- //
	if(!nh_.param<std::string>("/ur_speed_control/ROBOT_IP", ROBOT_IP, "192.168.2.30")) {ROS_ERROR_STREAM("Failed To Get \"ROBOT_IP\" Param. Usinge Default: " << ROBOT_IP);}
	if(!nh_.param<double>("/ur_speed_control/max_acc", max_acc_, 0.25)) {ROS_ERROR_STREAM("Failed To Get \"max_acc\" Param. Usinge Default: " << max_acc_);}

	// ---- UR RTDE LIBRARY ---- //
	rtde_control_ = new ur_rtde::RTDEControlInterface(ROBOT_IP);
	rtde_receive_ = new ur_rtde::RTDEReceiveInterface(ROBOT_IP);
	rtde_io_ = new ur_rtde::RTDEIOInterface(ROBOT_IP);

    // ---- ROS - PUBLISHERS ---- //
	pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/act_pose", 1);
	robot_status_pub_ = nh_.advertise<ur_speed_control::robot_status>("/ur_rtde/safety_status", 1);

    // ---- ROS - SUBSCRIBERS ---- //
	twist_sub_ = nh_.subscribe("/twist_cmd", 1, &URSpeedControl::twistCallback, this);

	// ---- ROS - SERVICE SERVERS ---- //
	onrobot_gripper_service_ = nh_.advertiseService("/ur_rtde/onrobot_gripper_control", &URSpeedControl::onRobotGripperCallback, this);

    // ---- INITIALIZING CLASS VARIABLES ---- //
	desired_twist_.twist.linear.x = 0.0;
	desired_twist_.twist.linear.y = 0.0;
	desired_twist_.twist.linear.z = 0.0;

	desired_twist_.twist.angular.x = 0.0;
	desired_twist_.twist.angular.y = 0.0;
	desired_twist_.twist.angular.z = 0.0;

	desired_twist_dbl_.resize(6);
	pose_dbl_.resize(6);

	ros::Duration(1).sleep();
	ROS_INFO("UR Speed Controller - Inizialized");

}

URSpeedControl::~URSpeedControl()
{

	rtde_control_ -> disconnect();

}

void URSpeedControl::twistCallback(const geometry_msgs::TwistStamped msg)
{
	
	desired_twist_ = msg;
	
}

bool URSpeedControl::onRobotGripperCallback(ur_speed_control::command_gripper::Request  &req, ur_speed_control::command_gripper::Response &res) {

    // Pin 17 = Tool Digital Opuput 1 (Set Velocity)
	if (rtde_io_ -> setToolDigitalOut(1, req.velocity)) {ROS_INFO_STREAM("Gripper Velocity Setted: " << (req.velocity ? "SLOW":"FAST"));}
	else 
	{
		ROS_ERROR("ERROR: Failed To Set Gripper Velocity");
		res.success = false;
		return false;
	}
    
    // Pin 16 = Tool Digital Opuput 0 (Open / Close)
	if (rtde_io_ -> setToolDigitalOut(0, req.movement)) {ROS_INFO_STREAM("Gripper Movement Started: " << (req.movement ? "CLOSE":"OPEN"));}
	else
	{
		ROS_ERROR("ERROR: Failed To Start Gripper Movement");
		res.success = false;
		return false;
	}
    
	res.success = true;
	return res.success;

}

void URSpeedControl::readRobotSafetyStatus ()
{
	/************************************************
	 * 												*
	 *  Safety status bits Bits 0-10:				*
	 * 												*
	 * 		0 = Is normal mode						*
	 * 		1 = Is reduced mode						*
	 * 		2 = Is protective stopped				*
	 * 		3 = Is recovery mode					*
	 * 		4 = Is safeguard stopped				*
	 * 		5 = Is system emergency stopped			*
	 * 		6 = Is robot emergency stopped			*
	 * 		7 = Is emergency stopped				*
	 * 		8 = Is violation						*
	 * 		9 = Is fault							*
	 * 	   10 = Is stopped due to safety 			*
	 * 												*
	 ***********************************************/

	/************************************************
	 * 												*
	 *  Robot mode									*
	 * 												*
	 *	   -1 = ROBOT_MODE_NO_CONTROLLER			*
	 *		0 = ROBOT_MODE_DISCONNECTED 			*
	 *		1 = ROBOT_MODE_CONFIRM_SAFETY 			*
	 *		2 = ROBOT_MODE_BOOTING					*
	 *		3 = ROBOT_MODE_POWER_OFF				*
	 *		4 = ROBOT_MODE_POWER_ON					*
	 * 		5 = ROBOT_MODE_IDLE						*
	 * 		6 = ROBOT_MODE_BACKDRIVE				*
	 *		7 = ROBOT_MODE_RUNNING					*
	 *		8 = ROBOT_MODE_UPDATING_FIRMWARE		*
	 * 												*
	 ***********************************************/


	ur_speed_control::robot_status robot_status;
	std::vector<std::string> robot_mode_msg = {"ROBOT_MODE_NO_CONTROLLER", "ROBOT_MODE_DISCONNECTED", "ROBOT_MODE_CONFIRM_SAFETY", "ROBOT_MODE_BOOTING", "ROBOT_MODE_POWER_OFF", "ROBOT_MODE_POWER_ON", "ROBOT_MODE_IDLE", "ROBOT_MODE_BACKDRIVE", "ROBOT_MODE_RUNNING", "ROBOT_MODE_UPDATING_FIRMWARE"};
	std::vector<std::string> safety_mode_msg = {"Is normal mode", "Is reduced mode", "Is protective stopped", "Is recovery mode", "Is safeguard stopped", "Is system emergency stopped", "Is robot emergency stopped", "Is emergency stopped", "Is violation", "Is fault", "Is stopped due to safety"};
	std::vector<std::string> safety_status_bits_msg = {"Is normal mode", "Is reduced mode", "Is protective stopped", "Is recovery mode", "Is safeguard stopped", "Is system emergency stopped", "Is robot emergency stopped", "Is emergency stopped", "Is violation", "Is fault", "Is stopped due to safety"};
	
	// Get Robot Mode
	robot_status.robot_mode = rtde_receive_ -> getRobotMode();
	robot_status.robot_mode_msg = robot_mode_msg[robot_status.robot_mode + 1];

	// Get Safety Mode
	robot_status.safety_mode = rtde_receive_ -> getSafetyMode();
	robot_status.safety_mode_msg = safety_mode_msg[robot_status.safety_mode - 1];

	// Get Safety Status Bits
	robot_status.safety_status_bits = rtde_receive_ -> getSafetyStatusBits();
	robot_status.safety_status_bits_msg = safety_status_bits_msg[robot_status.safety_status_bits];

	robot_status_pub_.publish(robot_status);

}

void URSpeedControl::spinner()
{
	// Read and Publish Robot Status
	readRobotSafetyStatus();

	desired_twist_dbl_[0] = desired_twist_.twist.linear.x;
	desired_twist_dbl_[1] = desired_twist_.twist.linear.y;
	desired_twist_dbl_[2] = desired_twist_.twist.linear.z;
	desired_twist_dbl_[3] = desired_twist_.twist.angular.x;
	desired_twist_dbl_[4] = desired_twist_.twist.angular.y;
	desired_twist_dbl_[5] = desired_twist_.twist.angular.z;

	// xd: tool speed [m/s] (spatial vector) | acceleration: tool position acceleration [m/s^2] | time: time [s] before the function returns (optional)
	rtde_control_ -> speedL(desired_twist_dbl_, max_acc_, 0.002);

	pose_dbl_ = rtde_receive_ -> getTargetTCPPose();
	angle_ = sqrt(pow(pose_dbl_[3],2) + pow(pose_dbl_[4],2) + pow(pose_dbl_[5],2));
	axis_ << pose_dbl_[3], pose_dbl_[4], pose_dbl_[5];
	axis_ = axis_.normalized(); 

	pose_.pose.position.x = pose_dbl_[0];
	pose_.pose.position.y = pose_dbl_[1];
	pose_.pose.position.z = pose_dbl_[2];

	pose_.pose.orientation.x = Eigen::Quaterniond(Eigen::AngleAxisd(angle_, axis_)).x();
	pose_.pose.orientation.y = Eigen::Quaterniond(Eigen::AngleAxisd(angle_, axis_)).y();
	pose_.pose.orientation.z = Eigen::Quaterniond(Eigen::AngleAxisd(angle_, axis_)).z();
	pose_.pose.orientation.w = Eigen::Quaterniond(Eigen::AngleAxisd(angle_, axis_)).w();

	pose_pub_.publish(pose_);
	
	ros::spinOnce();
	ros_rate_.sleep();

}
