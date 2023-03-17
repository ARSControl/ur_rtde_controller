#include "rtde_controller/rtde_controller.h"

RTDEController::RTDEController(ros::NodeHandle &nh, ros::Rate ros_rate): nh_(nh), ros_rate_(ros_rate)
{
	// Load Parameters
	if(!nh_.param<std::string>("/ur_rtde_controller/ROBOT_IP", ROBOT_IP, "192.168.2.30")) {ROS_ERROR_STREAM("Failed To Get \"ROBOT_IP\" Param. Using Default: " << ROBOT_IP);}
	if(!nh_.param<bool>("/ur_rtde_controller/enable_gripper", enable_gripper_, "False")) {ROS_ERROR_STREAM("Failed To Get \"gripper_enabled\" Param. Using Default: " << enable_gripper_);}
	if(!nh_.param<bool>("/ur_rtde_controller/asynchronous", asynchronous_, "False")) {ROS_ERROR_STREAM("Failed To Get \"asynchronous\" Param. Using Default: " << asynchronous_);}

	// RTDE Library
	rtde_control_ = new ur_rtde::RTDEControlInterface(ROBOT_IP);
	rtde_receive_ = new ur_rtde::RTDEReceiveInterface(ROBOT_IP);
	rtde_io_  	  = new ur_rtde::RTDEIOInterface(ROBOT_IP);

	// RobotiQ Gripper
	if (enable_gripper_) {

		// Initialize Gripper
		robotiq_gripper_ = new ur_rtde::RobotiqGripper(ROBOT_IP, 63352, true);
		robotiq_gripper_ -> connect();

		// Gripper Service Server
		robotiq_gripper_server_ = nh_.advertiseService("/ur_rtde/robotiq_gripper/command", &RTDEController::RobotiQGripperCallback, this);

	}

    // ROS - Publishers
	joint_state_pub_		 = nh_.advertise<sensor_msgs::JointState> ("/joint_states", 1);
	tcp_pose_pub_			 = nh_.advertise<geometry_msgs::Pose>	  ("/ur_rtde/cartesian_pose", 1);
	ft_sensor_pub_			 = nh_.advertise<geometry_msgs::Wrench>   ("/ur_rtde/ft_sensor", 1);
	trajectory_executed_pub_ = nh_.advertise<std_msgs::Bool>		  ("/ur_rtde/trajectory_executed", 1);

    // ROS - Subscribers
	// trajectory_command_sub_			= nh_.subscribe("/ur_rtde/controllers/trajectory_controller/command", 		  1, &RTDEController::jointTrajectoryCallback, this);
	joint_goal_command_sub_			= nh_.subscribe("/ur_rtde/controllers/joint_space_controller/command",        1, &RTDEController::jointGoalCallback, this);
	cartesian_goal_command_sub_		= nh_.subscribe("/ur_rtde/controllers/cartesian_space_controller/command",    1, &RTDEController::cartesianGoalCallback, this);
	joint_velocity_command_sub_		= nh_.subscribe("/ur_rtde/controllers/joint_velocity_controller/command",     1, &RTDEController::jointVelocityCallback, this);
	cartesian_velocity_command_sub_	= nh_.subscribe("/ur_rtde/controllers/cartesian_velocity_controller/command", 1, &RTDEController::cartesianVelocityCallback, this);

	// ROS - Service Servers
	stop_robot_server_			= nh_.advertiseService("/ur_rtde/controllers/stop_robot", &RTDEController::stopRobotCallback, this);
    start_FreedriveMode_server_	= nh_.advertiseService("/ur_rtde/FreedriveMode/start", 	  &RTDEController::startFreedriveModeCallback, this);
    stop_FreedriveMode_server_	= nh_.advertiseService("/ur_rtde/FreedriveMode/stop",  	  &RTDEController::stopFreedriveModeCallback, this);
	zeroFT_sensor_server_		= nh_.advertiseService("/ur_rtde/zeroFTSensor", 		  &RTDEController::zeroFTSensorCallback, this);
    get_FK_server_				= nh_.advertiseService("/ur_rtde/getFK", 				  &RTDEController::getForwardKinematicCallback, this);
    get_IK_server_				= nh_.advertiseService("/ur_rtde/getIK", 				  &RTDEController::getInverseKinematicCallback, this);
	get_safety_status_server_	= nh_.advertiseService("/ur_rtde/getSafetyStatus",  	  &RTDEController::getSafetyStatusCallback, this);

	ros::Duration(1).sleep();
	std::cout << std::endl;
	ROS_WARN("UR RTDE Controller - Connected\n");
}

RTDEController::~RTDEController()
{
	rtde_control_ -> stopJ(2.0);
	rtde_control_ -> disconnect();
	std::cout << std::endl;
	ROS_WARN("UR RTDE Controller - Disconnected\n");
}

void RTDEController::jointTrajectoryCallback(const trajectory_msgs::JointTrajectory msg)
{
	//TODO: Spline Interpolation and more... 
	desired_trajectory_ = msg;
	new_trajectory_received_ = true;
}

void RTDEController::jointGoalCallback(const trajectory_msgs::JointTrajectoryPoint msg)
{
	// Check Input Data Size
	if (msg.positions.size() != 6) {ROS_ERROR("ERROR: Received Joint Position Goal Size != 6"); return;}
	if (msg.time_from_start.toSec() == 0) {ROS_ERROR("ERROR: Desired Time = 0"); return;}

	// Get Desired and Actual Joint Pose
	Eigen::VectorXd desired_pose = Eigen::VectorXd::Map(msg.positions.data(), msg.positions.size());
	Eigen::VectorXd actual_pose  = Eigen::VectorXd::Map(actual_joint_position_.data(), actual_joint_position_.size());

	// Check Joint Limits
	if ((desired_pose.array().abs() > JOINT_LIMITS).any()) {ROS_ERROR("ERROR: Received Joint Position Outside Joint Limits"); return;}

	// Path Length
	double LP = (desired_pose - actual_pose).array().abs().maxCoeff();
	double velocity = 1.0, acceleration = 4.0;
	double T = msg.time_from_start.toSec();

	// Check Acceleration is Sufficient to Reach the Goal in the Desired Time
	if (acceleration < 4 * LP / std::pow(T,2))
	{
		T = std::sqrt(4 * LP / acceleration);
		ROS_WARN_STREAM("Robot Acceleration is Not Sufficient to Reach the Goal in the Desired Time | Used the Minimum Time: " << T);
	}

	// Compute Velocity
	double ta = T/2.0 - 0.5 * std::sqrt((std::pow(T,2) * acceleration - 4.0 * LP) / acceleration + 10e-12);
	velocity = ta * acceleration;

	// Check Velocity Limits
	if (velocity > JOINT_VELOCITY_MAX) {ROS_ERROR("Requested Velocity > Maximum Velocity"); return;}

	// Move to Joint Goal
	rtde_control_ -> moveJ(msg.positions, velocity, acceleration, asynchronous_);

	// Publish Trajectory Executed
	if (!asynchronous_) publishTrajectoryExecuted();
	else new_async_joint_pose_received_ = true;
}

void RTDEController::cartesianGoalCallback(const ur_rtde_controller::CartesianPoint msg)
{
	// Convert Geometry Pose to RTDE Pose
	std::vector<double> desired_pose = Pose2RTDE(msg.cartesian_pose);

	// Check Tool Velocity Limits
	if (msg.velocity > TOOL_VELOCITY_MAX) {ROS_ERROR("Requested Velocity > Maximum Velocity"); return;}

	// Move to Linear Goal
	rtde_control_ -> moveL(desired_pose, msg.velocity, 1.20, asynchronous_);

	// Publish Trajectory Executed
	if (!asynchronous_) publishTrajectoryExecuted();
    else new_async_cartesian_pose_received_ = true;
}

void RTDEController::jointVelocityCallback(const std_msgs::Float64MultiArray msg)
{
	// Check Input Data Size
	if (msg.data.size() != 6) {ROS_ERROR("ERROR: Received Joint Velocity Size != 6"); return;}

	// Get Current and Desired Joint Velocity
	std::vector<double> current_velocity = rtde_receive_ -> getActualQd();
	std::vector<double> desired_velocity = msg.data;

	// Compute Velocity Difference
	Eigen::VectorXd velocity_difference = Eigen::VectorXd::Map(desired_velocity.data(), desired_velocity.size()) 
										- Eigen::VectorXd::Map(current_velocity.data(), current_velocity.size());

	// Compute MAX Acceleration
	double acceleration = velocity_difference.array().abs().maxCoeff() / ros_rate_.expectedCycleTime().toSec();

	// Check Acceleration Limits
	if (acceleration > JOINT_ACCELERATION_MAX) {ROS_ERROR("Requested Acceleration > Maximum Acceleration"); return;}

	// Joint Velocity Publisher
	rtde_control_ -> speedJ(desired_velocity, acceleration);
}

void RTDEController::cartesianVelocityCallback(const geometry_msgs::Twist msg)
{
	// Get Current Cartesian Velocity
	std::vector<double> current_velocity = rtde_receive_ -> getActualTCPSpeed();

	// Create Desired Velocity Vector
	std::vector<double> desired_cartesian_velocity;
	desired_cartesian_velocity.push_back(msg.linear.x);
	desired_cartesian_velocity.push_back(msg.linear.y);
	desired_cartesian_velocity.push_back(msg.linear.z);
	desired_cartesian_velocity.push_back(msg.angular.x);
	desired_cartesian_velocity.push_back(msg.angular.y);
	desired_cartesian_velocity.push_back(msg.angular.z);

	// TODO: Compute Velocity Difference

	// TODO: Compute MAX Acceleration
	// double acceleration = velocity_difference.array().abs().maxCoeff() / ros_rate_.expectedCycleTime().toSec();
	double acceleration = 0.25;

	// Check Acceleration Limits
	if (acceleration > TOOL_ACCELERATION_MAX) {ROS_ERROR("Requested Acceleration > Maximum Acceleration"); return;}

	// Cartesian Velocity Publisher
	rtde_control_ -> speedL(desired_cartesian_velocity, acceleration);
}

bool RTDEController::stopRobotCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
	// Stop Robot in Velocity
	res.success = rtde_control_ -> speedStop(2.0);

	// Reset Booleans Variables
	resetBooleans();

	return res.success;
}

bool RTDEController::startFreedriveModeCallback(ur_rtde_controller::StartFreedriveMode::Request  &req, ur_rtde_controller::StartFreedriveMode::Response &res)
{
	// freeAxes = [1,0,0,0,0,0] 	-> The robot is compliant in the x direction relative to the feature.
	// freeAxes: A 6 dimensional vector that contains 0’s and 1’s, these indicates in which axes movement is allowed. The first three values represents the cartesian directions along x, y, z, and the last three defines the rotation axis, rx, ry, rz. All relative to the selected feature

	// Start FreeDrive Mode
	res.success = rtde_control_ -> freedriveMode(req.free_axes);
	return res.success;
}

bool RTDEController::stopFreedriveModeCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
	// Exit from FreeDrive Mode
	res.success = rtde_control_ -> endFreedriveMode();
	return res.success;
}

bool RTDEController::zeroFTSensorCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
	// Reset Force-Torque Sensor
	res.success = rtde_control_ -> zeroFtSensor();
	return res.success;
}

bool RTDEController::getForwardKinematicCallback(ur_rtde_controller::GetForwardKinematic::Request  &req, ur_rtde_controller::GetForwardKinematic::Response &res)
{
	// Compute Forward Kinematic
	std::vector<double> tcp_pose = rtde_control_ -> getForwardKinematics(req.joint_position, {0.0,0.0,0.0,0.0,0.0,0.0});

	// Convert RTDE Pose to Geometry Pose
	res.tcp_position = RTDE2Pose(tcp_pose);

	res.success = true;
	return res.success;
}

bool RTDEController::getInverseKinematicCallback(ur_rtde_controller::GetInverseKinematic::Request  &req, ur_rtde_controller::GetInverseKinematic::Response &res)
{
	// Convert Geometry Pose to RTDE Pose
	std::vector<double> tcp_pose = Pose2RTDE(req.tcp_position);

	// Compute Inverse Kinematic
	res.joint_position = rtde_control_ -> getInverseKinematics(tcp_pose);

	res.success = true;
	return res.success;
}

bool RTDEController::getSafetyStatusCallback(ur_rtde_controller::GetRobotStatus::Request  &req, ur_rtde_controller::GetRobotStatus::Response &res)
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

	std::vector<std::string> robot_mode_msg = {"ROBOT_MODE_NO_CONTROLLER", "ROBOT_MODE_DISCONNECTED", "ROBOT_MODE_CONFIRM_SAFETY", "ROBOT_MODE_BOOTING", "ROBOT_MODE_POWER_OFF", "ROBOT_MODE_POWER_ON", "ROBOT_MODE_IDLE", "ROBOT_MODE_BACKDRIVE", "ROBOT_MODE_RUNNING", "ROBOT_MODE_UPDATING_FIRMWARE"};
	std::vector<std::string> safety_mode_msg = {"NORMAL", "REDUCED", "PROTECTIVE_STOP", "RECOVERY", "SAFEGUARD_STOP", "SYSTEM_EMERGENCY_STOP", "ROBOT_EMERGENCY_STOP", "VIOLATION" "FAULT"};
	std::vector<std::string> safety_status_bits_msg = {"Is normal mode", "Is reduced mode", "Is protective stopped", "Is recovery mode", "Is safeguard stopped", "Is system emergency stopped", "Is robot emergency stopped", "Is emergency stopped", "Is violation", "Is fault", "Is stopped due to safety"};

	// Get Robot Mode
	res.robot_mode = rtde_receive_ -> getRobotMode();
	res.robot_mode_msg = robot_mode_msg[res.robot_mode + 1];

	// Get Safety Mode
	res.safety_mode = rtde_receive_ -> getSafetyMode();
	res.safety_mode_msg = safety_mode_msg[res.safety_mode];

	// Get Safety Status Bits
	res.safety_status_bits = int(rtde_receive_ -> getSafetyStatusBits());
	res.safety_status_bits_msg = safety_status_bits_msg[res.safety_status_bits];

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
	try {res.status = robotiq_gripper_ -> move(position, speed, force, ur_rtde::RobotiqGripper::WAIT_FINISHED);}
	catch (const std::exception &e) {return false;}

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
	while (ros::ok() && !shutdown_)
	{
		// Create JointState Message
		sensor_msgs::JointState joint_state;

		// Read Joint Position and Velocity
		joint_state.position = rtde_receive_ -> getActualQ();
		joint_state.velocity = rtde_receive_ -> getActualQd();

		// Publish JointState
		joint_state_pub_.publish(joint_state);

		// Sleep to ROS Rate
		ros_rate_.sleep();
	}
}

void RTDEController::publishTCPPose()
{
	while (ros::ok() && !shutdown_)
	{
		// Read TCP Position
		std::vector<double> tcp_pose = rtde_receive_ -> getActualTCPPose();

		// Convert RTDE Pose to Geometry Pose
		geometry_msgs::Pose pose = RTDE2Pose(tcp_pose);

		// Publish TCP Pose
		tcp_pose_pub_.publish(pose);

		// Sleep to ROS Rate
		ros_rate_.sleep();
	}
}

void RTDEController::publishFTSensor()
{
	while (ros::ok() && !shutdown_)
	{
		// Read FT Sensor Forces
		std::vector<double> tcp_forces = rtde_receive_ -> getActualTCPForce();

		// Create Wrench Message
		geometry_msgs::Wrench forces;
		forces.force.x = tcp_forces[0];
		forces.force.y = tcp_forces[1];
		forces.force.z = tcp_forces[2];
		forces.torque.x = tcp_forces[3];
		forces.torque.y = tcp_forces[4];
		forces.torque.z = tcp_forces[5];

		// Publish FTSensor Forces
		ft_sensor_pub_.publish(forces);

		// Sleep to ROS Rate
		ros_rate_.sleep();
	}
}

void RTDEController::resetBooleans()
{
	// Reset Booleans Variables
	new_trajectory_received_ = false;
    new_async_joint_pose_received_ = false;
    new_async_cartesian_pose_received_ = false;
}

void RTDEController::publishTrajectoryExecuted()
{
	// Publish Trajectory Executed Message
	std_msgs::Bool trajectory_executed;
	trajectory_executed.data = true;
	trajectory_executed_pub_.publish(trajectory_executed);

	// Reset Booleans Variables
	resetBooleans();
}

std::vector<double> RTDEController::Pose2RTDE(geometry_msgs::Pose pose)
{
	// Create a Quaternion from Pose Orientation
	Eigen::Quaterniond quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);

    // Convert from Quaternion to Euler Angles
    Eigen::Vector3d axis = Eigen::AngleAxisd(quaternion).axis();
    double angle = Eigen::AngleAxisd(quaternion).angle();
	Eigen::Vector3d euler_orientation = axis * angle;

	// Create TCP Pose Message
	std::vector<double> tcp_pose;
	tcp_pose.push_back(pose.position.x);
	tcp_pose.push_back(pose.position.y);
	tcp_pose.push_back(pose.position.z);
	tcp_pose.push_back(euler_orientation[0]);
	tcp_pose.push_back(euler_orientation[1]);
	tcp_pose.push_back(euler_orientation[2]);

	return tcp_pose;
}

geometry_msgs::Pose RTDEController::RTDE2Pose(std::vector<double> rtde_pose)
{
	// Compute AngleAxis from rx,ry,rz
    double angle = sqrt(pow(rtde_pose[3],2) + pow(rtde_pose[4],2) + pow(rtde_pose[5],2));
	Eigen::Vector3d axis(rtde_pose[3], rtde_pose[4], rtde_pose[5]);
	axis = axis.normalized();

	// Convert Euler to Quaternion
	Eigen::Quaterniond quaternion(Eigen::AngleAxisd(angle, axis));

	// Write TCP Pose in Geometry Pose Message
	geometry_msgs::Pose pose;
	pose.position.x = rtde_pose[0];
	pose.position.y = rtde_pose[1];
	pose.position.z = rtde_pose[2];
	pose.orientation.x = quaternion.x();
	pose.orientation.y = quaternion.y();
	pose.orientation.z = quaternion.z();
	pose.orientation.w = quaternion.w();

	return pose;
}

Eigen::Matrix<double, 4, 4> RTDEController::pose2eigen(geometry_msgs::Pose pose)
{
	Eigen::Matrix<double, 4, 4> T = Eigen::Matrix<double, 4, 4>::Identity();

	T(0, 3) = pose.position.x;
	T(1, 3) = pose.position.y;
	T(2, 3) = pose.position.z;

	Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x,
    	                 pose.orientation.y, pose.orientation.z);

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
	if ((Eigen::abs(position_error.array()) < movement_precision).all()) return true;
	else return false;
}

void RTDEController::moveTrajectory()
{
	// Return if No Trajectory Received
	if (!new_trajectory_received_) return;

	// Create Next Point
	std::vector<std::vector<double>> next_point;
	next_point.push_back(desired_trajectory_.points[0].positions);
	next_point.push_back(desired_trajectory_.points[0].velocities);
	next_point.push_back(desired_trajectory_.points[0].accelerations);
	next_point.push_back(desired_trajectory_.points[0].effort);

	// Move to New Trajectory Goal
	// TODO: Move to the First Trajectory Point
	rtde_control_ -> moveJ(desired_trajectory_.points[0].positions);

	// Erase Point from Trajectory
	desired_trajectory_.points.erase(desired_trajectory_.points.begin());

	// Check for Trajectory Ending
	if (desired_trajectory_.points.size() == 0)
	{
		// Stop Robot
		rtde_control_ -> stopJ(2.0);

		// Publish Trajectory Executed
		publishTrajectoryExecuted();
	}

}

/* 
void RTDEController::spinner()
{
	// Callback Readings
	ros::spinOnce();

	// Update Actual Joint and TCP Positions
	actual_joint_position_ = rtde_receive_ -> getActualQ();
	actual_joint_velocity_ = rtde_receive_ -> getActualQd();
	actual_cartesian_pose_ = RTDE2Pose(rtde_receive_ -> getActualTCPPose());

	// Move to New Trajectory Goal
	// if (new_trajectory_received_ && (ros::Time::now() - trajectory_timer_) >= desired_trajectory_.points[0].time_from_start)
	if (new_trajectory_received_)
	{

		double velocity = 0.5;
		double acceleration = 0.5;

		// Path Creation
		std::vector<std::vector<double>> path;
		
		std::vector<double> first_point = desired_trajectory_.points[0].positions;
		first_point.push_back(velocity);
		first_point.push_back(acceleration);
		first_point.push_back(0.01);
		
		std::vector<double> second_point = desired_trajectory_.points[1].positions;
		second_point.push_back(velocity);
		second_point.push_back(acceleration);
		second_point.push_back(0.01);
		
		path.push_back(first_point);
		path.push_back(second_point);

		// Move to the First Trajectory Point
		// rtde_control_ -> moveJ(desired_trajectory_.points[0].positions, velocity, acceleration, true);
		// rtde_control_ -> movePath()
		// rtde_control_ -> moveJ(desired_trajectory_.points[0].positions, velocity, acceleration, true);
		rtde_control_ -> moveJ(path, true);

		// // Erase Point from Trajectory
		// desired_trajectory_.points.erase(desired_trajectory_.points.begin());

		// // Check for Trajectory Ending
		// if (desired_trajectory_.points.size() == 0) 
		// {

			new_trajectory_received_ = false;

		// 	// Publish Trajectory Executed
		// 	publishTrajectoryExecuted();

		// 	// Stop Robot
		// 	rtde_control_ -> stopJ(2.0);

		// }

		// Reset Trajectory Timer
		// trajectory_timer_ = ros::Time::now();

	}

	// Sleep until ROS Rate
	ros_rate_.sleep();

}
 */

void RTDEController::checkAsyncMovements()
{
	// Return if No Async Movement Received
	if (!new_async_joint_pose_received_ and !new_async_cartesian_pose_received_) return;

	// Check if Async Operation is Ended -> Trajectory Executed
	if (rtde_control_ -> getAsyncOperationProgress() < 0) publishTrajectoryExecuted();
}

void RTDEController::spinner()
{
	// Callback Readings
	ros::spinOnce();

	// Update Actual Joint and TCP Positions
	actual_joint_position_ = rtde_receive_ -> getActualQ();
	actual_joint_velocity_ = rtde_receive_ -> getActualQd();
	actual_cartesian_pose_ = RTDE2Pose(rtde_receive_ -> getActualTCPPose());

	// Trajectory Controller
	moveTrajectory();

	// Check Async Movements Status
	checkAsyncMovements();

	// Sleep until ROS Rate
	ros_rate_.sleep();
}

// Create Null-Pointers to the RTDE Class and the Threads
RTDEController *rtde = nullptr;
std::thread *publishJointState = nullptr;
std::thread *publishTCPPose    = nullptr;
std::thread *publishFTSensor   = nullptr;

void signalHandler(int signal)
{
	std::cout << "\nKeyboard Interrupt Received\n";

	// Set Shutdown Trigger
	rtde -> shutdown_ = true;

	// Join Threads on Main
	publishJointState -> join();
	publishTCPPose    -> join();
	publishFTSensor   -> join();

	// Call Destructor
	delete rtde;
	exit(signal);
}

int main(int argc, char **argv) {

    // ros::init(argc, argv, "ur_rtde_controller", ros::init_options::NoSigintHandler);
    ros::init(argc, argv, "ur_rtde_controller");

    ros::NodeHandle nh;
    ros::Rate loop_rate = 500;

	// Create a New RTDEController
    rtde = new RTDEController(nh, loop_rate);

	// Create a SIGINT Handler
	struct sigaction sa;
    sa.sa_handler = signalHandler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
	sigaction(SIGINT, &sa, NULL);

	// Publish JointState, TCPPose, FTSensor in separate Threads
	publishJointState = new std::thread(&RTDEController::publishJointState, rtde);
	publishTCPPose    = new std::thread(&RTDEController::publishTCPPose,    rtde);
	publishFTSensor   = new std::thread(&RTDEController::publishFTSensor,   rtde);
	ros::Duration(1).sleep();

	// Main Spinner
    while (ros::ok()) {rtde -> spinner();}

	// Set Shutdown Trigger
	rtde -> shutdown_ = true;

	// Join Threads on Main
	publishJointState -> join();
	publishTCPPose    -> join();
	publishFTSensor   -> join();

	// Call Destructor
    delete rtde;

return 0;

}
