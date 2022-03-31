#include "ur_speed_control/ur_speed_control.h"

URSpeedControl::URSpeedControl(ros::NodeHandle &nh, ros::Rate ros_rate): nh_(nh), ros_rate_(ros_rate)
{

	// ---- LOAD PARAMETERS ---- //
	if(!nh_.param<std::string>("ROBOT_IP", ROBOT_IP, "192.168.2.30")) {ROS_ERROR_STREAM("Failed To Get \"ROBOT_IP\" Param. Usinge Default: " << ROBOT_IP);}
	if(!nh_.param<double>("max_acc", max_acc_, 0.25)) {ROS_ERROR_STREAM("Failed To Get \"max_acc\" Param. Usinge Default: " << max_acc_);}

	// ---- UR RTDE LIBRARY ---- //
	rtde_control_ = new ur_rtde::RTDEControlInterface(ROBOT_IP);
	rtde_receive_ = new ur_rtde::RTDEReceiveInterface(ROBOT_IP);
	rtde_io_ = new ur_rtde::RTDEIOInterface(ROBOT_IP);

    // ---- ROS PUBLISHERS ---- //
	pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/act_pose", 1);

    // ---- ROS SUBSCRIBERS ---- //
	twist_sub_ = nh_.subscribe("/twist_cmd", 1, &URSpeedControl::twistCallback, this);

	// ---- ROS - SERVICE SERVERS ---- //
	onrobot_gripper_service_ = nh_.advertiseService("/onrobot_gripper_control", &URSpeedControl::onRobotGripperCallback, this);

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


	//   // fun = 1 to set output pin
    // setIO_srv.request.fun = 1;
    
    // // pin 17 = Tool Digital Opuput 1 (set velocity)
    // setIO_srv.request.pin = 17;

    // // change pin status, FAST = 0, SLOW = 1
    // if (fast_slow) {setIO_srv.request.state = 0.0;}
    // else {setIO_srv.request.state = 1.0;}
    
    // // Set Velocity
    // if (setIO_client.call(setIO_srv)) {}
    // else {ROS_ERROR("Failed to Call Service: \"/ur_hardware_interface/set_io\"");}

    // // fun = 1 to set output pin
    // setIO_srv.request.fun = 1;
    
    // // pin 16 = Tool Digital Opuput 0 (open/close)
    // setIO_srv.request.pin = 16;

    // // change pin status, OPEN = 0, CLOSE = 1
    // if (open_close) {setIO_srv.request.state = 0.0;}
    // else {setIO_srv.request.state = 1.0;}
    
    // // Open / Close Gripper
    // if (setIO_client.call(setIO_srv)) {}
    // else {ROS_ERROR("Failed to Call Service: \"/ur_hardware_interface/set_io\"");}
	return true;

}

void URSpeedControl::spinner()
{

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
