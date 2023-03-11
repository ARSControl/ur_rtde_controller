#include "rtde_controller/velocity_controller.h"

RTDEController::RTDEController(ros::NodeHandle &nh, ros::Rate ros_rate): nh_(nh), ros_rate_(ros_rate)
{

	// Load Parameters
	if(!nh_.param<std::string>("/rtde_controller/ROBOT_IP", ROBOT_IP, "192.168.2.30")) {ROS_ERROR_STREAM("Failed To Get \"ROBOT_IP\" Param. Using Default: " << ROBOT_IP);}
	if(!nh_.param<bool>("/rtde_controller/enable_gripper", enable_gripper, "False")) {ROS_ERROR_STREAM("Failed To Get \"gripper_enabled\" Param. Using Default: " << enable_gripper);}
	if(!nh_.param<double>("/rtde_controller/max_acc", max_acc_, 0.25)) {ROS_ERROR_STREAM("Failed To Get \"max_acc\" Param. Using Default: " << max_acc_);}

	// RTDE Library
	rtde_control_ = new ur_rtde::RTDEControlInterface(ROBOT_IP);
	rtde_receive_ = new ur_rtde::RTDEReceiveInterface(ROBOT_IP);
	rtde_io_  	  = new ur_rtde::RTDEIOInterface(ROBOT_IP);

	// RobotiQ Gripper
	if (enable_gripper) {

		robotiq_gripper_ = new ur_rtde::RobotiqGripper(ROBOT_IP, 63352, true);
		robotiq_gripper_ -> connect();

	}

    // ROS - Publishers
	joint_state_pub_  		 = nh_.advertise<sensor_msgs::JointState>("/ur_rtde/joint_state", 1);
	tcp_pose_pub_     		 = nh_.advertise<geometry_msgs::PoseStamped>("/ur_rtde/cartesian_pose", 1);
	robot_status_pub_ 		 = nh_.advertise<ur_rtde_controller::RobotStatus>("/ur_rtde/ur_safety_status", 1);
	trajectory_executed_pub_ = nh_.advertise<std_msgs::Bool>("/ur_rtde/trajectory_executed", 1);

    // ROS - Subscribers
	joint_velocity_command_sub_      = nh_.subscribe("/ur_rtde/controllers/joint_velocity_controller/command", 1, &RTDEController::jointVelocityCallback, this);
	cartesian_velocity_command_sub_  = nh_.subscribe("/ur_rtde/controllers/cartesian_velocity_controller/command", 1, &RTDEController::cartesianVelocityCallback, this);
	trajectory_command_sub_     	 = nh_.subscribe("/ur_rtde/controllers/trajectory_controller/command", 1, &RTDEController::jointTrajectoryCallback, this);
	joint_goal_command_sub_     	 = nh_.subscribe("/ur_rtde/controllers/joint_space_controller/command", 1, &RTDEController::jointGoalCallback, this);
	cartesian_goal_command_sub_ 	 = nh_.subscribe("/ur_rtde/controllers/cartesian_space_controller/command", 1, &RTDEController::cartesianGoalCallback, this);

	// ROS - Service Servers
	robotiq_gripper_server_ = nh_.advertiseService("/ur_rtde/robotiq_gripper/command", &RTDEController::RobotiQGripperCallback, this);
	stop_robot_server_ = nh_.advertiseService("/ur_rtde/controllers/stop_robot", &RTDEController::stopRobotCallback, this);

	ros::Duration(1).sleep();
	ROS_INFO("UR RTDE Controller - Initialized");

}

RTDEController::~RTDEController()
{

	rtde_control_ -> stopJ(2.0);
	rtde_control_ -> disconnect();

}

void RTDEController::jointVelocityCallback(const trajectory_msgs::JointTrajectory msg)
{

	// Joint Velocity Publisher
	rtde_control_ -> speedJ(msg.points[0].velocities, max_acc_, 0.002);

}

void RTDEController::cartesianVelocityCallback(const geometry_msgs::Twist msg)
{

	// Create Desired Velocity Vector
	std::vector<double> desired_cartesian_velocity;
	desired_cartesian_velocity.push_back(msg.linear.x);
	desired_cartesian_velocity.push_back(msg.linear.y);
	desired_cartesian_velocity.push_back(msg.linear.z);
	desired_cartesian_velocity.push_back(msg.angular.x);
	desired_cartesian_velocity.push_back(msg.angular.y);
	desired_cartesian_velocity.push_back(msg.angular.z);

	// Cartesian Velocity Publisher
	rtde_control_ -> speedL(desired_cartesian_velocity, max_acc_, 0.002);

}

void RTDEController::jointTrajectoryCallback(const trajectory_msgs::JointTrajectory msg)
{

	desired_trajectory_ = msg;
	new_trajectory_received_ = true;

}

void RTDEController::jointGoalCallback(const trajectory_msgs::JointTrajectoryPoint msg)
{

	// Received New Joint Goal
	desired_joint_pose_ = msg.positions;

	new_joint_pose_received_ = true;

}

void RTDEController::cartesianGoalCallback(const geometry_msgs::PoseStamped msg)
{
	// Convert Received New Pose to Eigen Vector
	desired_cartesian_pose_ = pose2eigen(msg);

	new_cartesian_pose_received_ = true;

}

bool RTDEController::stopRobotCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{

	// Stop Robot in Joint Space
	rtde_control_ -> stopJ(2.0);

	// Reset Booleans
	new_trajectory_received_ = false;
	new_joint_pose_received_ = false;
	new_cartesian_pose_received_ = false;

	res.success = true;
	return res.success;

}

bool RTDEController::RobotiQGripperCallback(ur_rtde_controller::RobotiQGripperControl::Request  &req, ur_rtde_controller::RobotiQGripperControl::Response &res)
{
	// Normalize Received Values
	float position 	= req.position / 100;
	float speed		= req.speed / 100;
	float force		= req.force / 100;

	// Move Gripper - Normalized Values (0.0 - 1.0)
	res.status = robotiq_gripper_ -> move(position, speed, force, ur_rtde::RobotiqGripper::WAIT_FINISHED);

	/************************************************************************************************
	 *																								*
	 * Object Detection Status																		*
	 * 																								*
	 * 	MOVING = 0                | Gripper is Opening or Closing									*
	 * 	STOPPED_OUTER_OBJECT = 1  | Outer Object Detected while Opening the Gripper					*
	 * 	STOPPED_INNER_OBJECT = 2  | Inner Object Detected while Closing the Gripper					*
	 * 	AT_DEST = 3               | Requested Target Position Reached - No Object Detected			*
	 * 																								*
 	 ***********************************************************************************************/

	res.success = true;
	return res.success;

}

void RTDEController::publishJointState()
{

	sensor_msgs::JointState joint_state;
	joint_state.position.resize(6);
	joint_state.velocity.resize(6);

	// Read Joint Position and Velocity
	actual_joint_position_ = rtde_receive_ -> getActualQ();
	actual_joint_velocity_ = rtde_receive_ -> getActualQd();

	// Write Joint Position and Velocity in JointState Message
	for(int i = 0; i < 6; i++)
	{
		joint_state.position[i] = actual_joint_position_[i];
		joint_state.velocity[i] = actual_joint_velocity_[i];
	}

	// Publish JointState
	joint_state_pub_.publish(joint_state);

}

void RTDEController::publishTCPPose()
{

	// Read Joint Position
	std::vector<double> tcp_pose = rtde_receive_ -> getActualTCPPose();

	// Compute AngleAxis from rx,ry,rz
	double angle = sqrt(pow(tcp_pose[3],2) + pow(tcp_pose[4],2) + pow(tcp_pose[5],2));
	Eigen::Vector3d axis(tcp_pose[3], tcp_pose[4], tcp_pose[5]);
	axis = axis.normalized();

	// Write TCP Pose in PoseStamped Message
	actual_cartesian_pose_.pose.position.x = tcp_pose[0];
	actual_cartesian_pose_.pose.position.y = tcp_pose[1];
	actual_cartesian_pose_.pose.position.z = tcp_pose[2];
	actual_cartesian_pose_.pose.orientation.x = Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis)).x();
	actual_cartesian_pose_.pose.orientation.y = Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis)).y();
	actual_cartesian_pose_.pose.orientation.z = Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis)).z();
	actual_cartesian_pose_.pose.orientation.w = Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis)).w();

	// Publish TCP Pose
	tcp_pose_pub_.publish(actual_cartesian_pose_);

}

void RTDEController::publishTrajectoryExecuted()
{

	// Publish Trajectory Executed Message
	std_msgs::Bool trajectory_executed;
	trajectory_executed.data = true;
	trajectory_executed_pub_.publish(trajectory_executed);

}

void RTDEController::readRobotSafetyStatus()
{
	/************************************************
	 * 												*
	 *  Safety Status Bits 0-10:					*
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
	 *  Safety Mode									*
	 * 												*
	 *		0 = NORMAL					 			*
	 *		1 = REDUCED					 			*
	 *		2 = PROTECTIVE_STOP						*
	 *		3 = RECOVERY							*
	 *		4 = SAFEGUARD_STOP						*
	 * 		5 = SYSTEM_EMERGENCY_STOP				*
	 * 		6 = ROBOT_EMERGENCY_STOP				*
	 *		7 = VIOLATION							*
	 *		8 = FAULT								*
	 * 												*
	 ***********************************************/

	/************************************************
	 * 												*
	 *  Robot Mode									*
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

	ur_rtde_controller::RobotStatus robot_status;
	std::vector<std::string> robot_mode_msg = {"ROBOT_MODE_NO_CONTROLLER", "ROBOT_MODE_DISCONNECTED", "ROBOT_MODE_CONFIRM_SAFETY", "ROBOT_MODE_BOOTING", "ROBOT_MODE_POWER_OFF", "ROBOT_MODE_POWER_ON", "ROBOT_MODE_IDLE", "ROBOT_MODE_BACKDRIVE", "ROBOT_MODE_RUNNING", "ROBOT_MODE_UPDATING_FIRMWARE"};
	std::vector<std::string> safety_mode_msg = {"NORMAL", "REDUCED", "PROTECTIVE_STOP", "RECOVERY", "SAFEGUARD_STOP", "SYSTEM_EMERGENCY_STOP", "ROBOT_EMERGENCY_STOP", "VIOLATION" "FAULT"};
	std::vector<std::string> safety_status_bits_msg = {"Is normal mode", "Is reduced mode", "Is protective stopped", "Is recovery mode", "Is safeguard stopped", "Is system emergency stopped", "Is robot emergency stopped", "Is emergency stopped", "Is violation", "Is fault", "Is stopped due to safety"};

	// Get Robot Mode
	robot_status.robot_mode = rtde_receive_ -> getRobotMode();
	robot_status.robot_mode_msg = robot_mode_msg[robot_status.robot_mode + 1];

	// Get Safety Mode
	robot_status.safety_mode = rtde_receive_ -> getSafetyMode();
	robot_status.safety_mode_msg = safety_mode_msg[robot_status.safety_mode];

	// Get Safety Status Bits
	robot_status.safety_status_bits = int(rtde_receive_ -> getSafetyStatusBits());
	robot_status.safety_status_bits_msg = safety_status_bits_msg[robot_status.safety_status_bits];

	robot_status_pub_.publish(robot_status);

}

Eigen::Matrix<double, 4, 4> RTDEController::pose2eigen(geometry_msgs::PoseStamped pose)
{

	Eigen::Matrix<double, 4, 4> T = Eigen::Matrix<double, 4, 4>::Identity();

	T(0, 3) = pose.pose.position.x;
	T(1, 3) = pose.pose.position.y;
	T(2, 3) = pose.pose.position.z;

	Eigen::Quaterniond q(pose.pose.orientation.w, pose.pose.orientation.x,
    	                 pose.pose.orientation.y, pose.pose.orientation.z);

	T.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();

	return T;

}

Eigen::VectorXd RTDEController::computePoseError(Eigen::Matrix<double, 4, 4> T_des, Eigen::Matrix<double, 4, 4> T)
{

	Eigen::Matrix<double, 6, 1> err;

	err.block<3, 1>(0, 0) = T.block<3, 1>(0, 3) - T_des.block<3, 1>(0, 3);

	Eigen::Quaterniond orientation_quat_des = Eigen::Quaterniond(T_des.block<3, 3>(0, 0));
	Eigen::Quaterniond orientation_quat = Eigen::Quaterniond(T.block<3, 3>(0, 0));

	if (orientation_quat_des.coeffs().dot(orientation_quat.coeffs()) < 0.0) {orientation_quat.coeffs() << -orientation_quat.coeffs();}

	Eigen::Quaterniond orientation_quat_error(orientation_quat.inverse() * orientation_quat_des);

	err.block<3, 1>(3, 0) << orientation_quat_error.x(), orientation_quat_error.y(), orientation_quat_error.z();
	err.block<3, 1>(3, 0) << -T.block<3, 3>(0, 0) * err.block<3, 1>(3, 0);

	return err.col(0);

}

bool RTDEController::isPoseReached(Eigen::VectorXd position_error, double movement_precision)
{

	if ((Eigen::abs(position_error.array()) < movement_precision).all())
	{

		std::cout << std::endl;
		ROS_INFO_STREAM("Position Reached - Movement Completed | Position Error: ");
		std::cout << Eigen::abs(position_error.array()) << std::endl << std::endl;

		return true;

	} else {return false;}

}

void RTDEController::spinner()
{

	ros::spinOnce();

	// Publish JointState and TCPPose
	publishJointState();
	publishTCPPose();
	readRobotSafetyStatus();

	// Move to New Trajectory Goal
	if (new_trajectory_received_)
	{

		// Create Next Point
		std::vector<std::vector<double>> next_point;
		next_point.push_back(desired_trajectory_.points[0].positions);
		next_point.push_back(desired_trajectory_.points[0].velocities);
		next_point.push_back(desired_trajectory_.points[0].accelerations);
		next_point.push_back(desired_trajectory_.points[0].effort);

		// Move to the First Trajectory Point
		rtde_control_ -> moveJ(desired_trajectory_.points[0].positions);

		// Erase Point from Trajectory
		desired_trajectory_.points.erase(desired_trajectory_.points.begin());

		// Check for Trajectory Ending
		if (desired_trajectory_.points.size() == 0) 
		{

			new_trajectory_received_ = false;

			// Publish Trajectory Executed
			publishTrajectoryExecuted();

			// Stop Robot
			rtde_control_ -> stopJ(2.0);

		}

	}

	else if (new_joint_pose_received_)
	{

		// Compute Position Error
        Eigen::VectorXd position_error = Eigen::VectorXd::Map(desired_joint_pose_.data(), desired_joint_pose_.size()) 
									   - Eigen::VectorXd::Map(actual_joint_position_.data(), actual_joint_position_.size());

		// Compute Velocity Setpoint
  		Eigen::VectorXd velocity_setpoint = - 2.0 * position_error;

		// Create Desired Velocity Vector
		std::vector<double> desired_velocity(velocity_setpoint.data(), velocity_setpoint.data() + velocity_setpoint.size());

		// Move Robot in Joint
		rtde_control_ -> speedJ(desired_velocity, max_acc_, 0.002);

		// Check for Position Reached (movement_precision = 0.0001)
		if (isPoseReached(position_error, 0.0001)) 
		{
			// Stop Robot
			rtde_control_ -> speedStop(2.0);

			new_joint_pose_received_ = false;
		}

	}

	else if (new_cartesian_pose_received_)
	{

		// Compute Position Error
        Eigen::VectorXd position_error = computePoseError(desired_cartesian_pose_, pose2eigen(actual_cartesian_pose_));

		// Compute Velocity Setpoint
  		Eigen::VectorXd velocity_setpoint = - 2.0 * position_error;

		// Create Desired Velocity Vector
		std::vector<double> desired_velocity(velocity_setpoint.data(), velocity_setpoint.data() + velocity_setpoint.size());

		// Move Robot in Linear
		rtde_control_ -> speedL(desired_velocity, max_acc_, 0.002);

		// Check for Position Reached (movement_precision = 0.0001)
		if (isPoseReached(position_error, 0.0001)) 
		{
			// Stop Robot
			rtde_control_ -> speedStop(2.0);

			new_cartesian_pose_received_ = false;
		}

	}

	// Sleep until ROS Rate
	ros_rate_.sleep();

}

int main(int argc, char **argv) {

	ros::init(argc, argv, "ur_rtde_controller");

	ros::NodeHandle nh;
	ros::Rate loop_rate = 500;

	RTDEController *rtde = new RTDEController(nh, loop_rate);

	while (ros::ok()) {

		rtde -> spinner();

	}

	delete rtde;

return 0;

}
