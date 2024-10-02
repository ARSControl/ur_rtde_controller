#include "rtde_controller/rtde_controller.h"

int sign(double value) {
    if (value > 0) {return 1;}
    else if (value < 0) {return -1;}
    else {return 0;}
}

double durationToSec(rclcpp::Duration duration) {
    return duration.seconds();
}

RTDEController::RTDEController(): Node ("ur_rtde_controller") {

    // Declare Parameters
    declare_parameter<std::string>("ROBOT_IP", std::string("192.168.2.30"));
    declare_parameter<bool>("enable_gripper", false);
    declare_parameter<bool>("asynchronous", false);
    declare_parameter<bool>("limit_acc", false);
    declare_parameter<bool>("ft_sensor", true);

    // Load Parameters
    if(!get_parameter_or("ROBOT_IP", ROBOT_IP, std::string("192.168.2.30"))) {RCLCPP_ERROR_STREAM(get_logger(), "Failed To Get \"ROBOT_IP\" Param. Using Default: " << ROBOT_IP);}
    if(!get_parameter_or("enable_gripper", enable_gripper_, false)) {RCLCPP_ERROR_STREAM(get_logger(), "Failed To Get \"gripper_enabled\" Param. Using Default: " << enable_gripper_);}
    if(!get_parameter_or("asynchronous", asynchronous_, false)) {RCLCPP_ERROR_STREAM(get_logger(), "Failed To Get \"asynchronous\" Param. Using Default: " << asynchronous_);}
    if(!get_parameter_or("limit_acc", limit_acc_, false)) {RCLCPP_ERROR_STREAM(get_logger(), "Failed To Get \"limit_acc\" Param. Using Default: " << limit_acc_);}
    if(!get_parameter_or("ft_sensor", ft_sensor_, true)) {RCLCPP_ERROR_STREAM(get_logger(), "Failed To Get \"ft_sensor\" Param. Using Default: " << ft_sensor_);}

    // Initialize Robot
    while (rclcpp::ok() && !robot_initialized) {

        // Initialize Dashboard
        if (!rtde_dashboard_initialized) {try {rtde_dashboard_ = new ur_rtde::DashboardClient(ROBOT_IP); rtde_dashboard_initialized = true;}
        catch (const std::exception &e) {RCLCPP_ERROR_STREAM(get_logger(), "Failed to Initialize the Dashboard Client:\n" << e.what());}}

        // Check Remote Control Status
        if (rtde_dashboard_initialized && !rtde_dashboard_connected) {try {rtde_dashboard_ -> connect(); rtde_dashboard_connected = true;
        while (!rtde_dashboard_ -> isInRemoteControl()) {RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "ERROR: Robot Not in RemoteControl Mode\n");}}
        catch (const std::exception &e) {RCLCPP_ERROR_STREAM(get_logger(), "Failed to Connect to the Dashboard Server:\n" << e.what());}}

        // RTDE Control Library
        if (rtde_dashboard_connected && !rtde_control_initialized) try {rtde_control_ = new ur_rtde::RTDEControlInterface(ROBOT_IP); rtde_control_initialized = true;}
        catch (const std::exception &e) {RCLCPP_ERROR_STREAM(get_logger(), "Failed to Initialize the RTDE Control Interface:\n" << e.what());}

        // RTDE Receive Library
        if (rtde_dashboard_connected && !rtde_receive_initialized) try {rtde_receive_ = new ur_rtde::RTDEReceiveInterface(ROBOT_IP); rtde_receive_initialized = true;}
        catch (const std::exception &e) {RCLCPP_ERROR_STREAM(get_logger(), "Failed to Initialize the RTDE Receive Interface:\n" << e.what());}

        // RTDE IO Library
        if (rtde_dashboard_connected && !rtde_io_initialized) try {rtde_io_ = new ur_rtde::RTDEIOInterface(ROBOT_IP); rtde_io_initialized = true;}
        catch (const std::exception &e) {RCLCPP_ERROR_STREAM(get_logger(), "Failed to Initialize the RTDE IO Interface:\n" << e.what());}

        // Reupload RTDE Control Script if Needed
        if (rtde_dashboard_initialized && !rtde_dashboard_ -> running()) try {rtde_control_ -> reuploadScript(); rtde_dashboard_ -> disconnect();}
        catch (const std::exception &e) {RCLCPP_ERROR_STREAM(get_logger(), "Failed to Reupload the RTDE Control Script:\n" << e.what());}

        // Robot Initialized
        if (rtde_dashboard_initialized && rtde_dashboard_connected && rtde_control_initialized && rtde_receive_initialized && rtde_io_initialized) robot_initialized = true;

    }

    // RobotiQ Gripper
    if (enable_gripper_) {

        try {

            // Initialize Gripper
            robotiq_gripper_ = new ur_rtde::RobotiqGripper(ROBOT_IP, 63352, false);
            robotiq_gripper_ -> connect();
            robotiq_gripper_ -> activate();

            // Gripper Service Server
            robotiq_gripper_server_ = create_service<ur_rtde_controller::srv::RobotiQGripperControl>("/ur_rtde/robotiq_gripper/command", std::bind(&RTDEController::RobotiQGripperCallback, this, std::placeholders::_1, std::placeholders::_2));

            // Gripper Enable/Disable Service Servers
            enable_gripper_server_  = create_service<std_srvs::srv::Trigger>("/ur_rtde/robotiq_gripper/enable", std::bind(&RTDEController::enableRobotiQGripperCallback, this,std::placeholders::_1, std::placeholders::_2));
            disable_gripper_server_ = create_service<std_srvs::srv::Trigger>("/ur_rtde/robotiq_gripper/disable", std::bind(&RTDEController::disableRobotiQGripperCallback, this,std::placeholders::_1, std::placeholders::_2));

			// Gripper Current Position Service Server
			gripper_current_position_server_ = create_service<ur_rtde_controller::srv::GetGripperPosition>("/ur_rtde/robotiq_gripper/current_position", std::bind(&RTDEController::currentPositionRobotiQGripperCallback, this,std::placeholders::_1, std::placeholders::_2));

        } catch(const std::exception& e) {std::cerr << "Error: " << e.what() << std::endl; RCLCPP_ERROR(get_logger(), "Failed to Start the RobotiQ 2F Gripper");}

    }

    if (ft_sensor_) {

        // Zero FT Sensor
        rtde_control_ -> zeroFtSensor();

        // FT Sensor Publisher
        ft_sensor_pub_ = create_publisher<geometry_msgs::msg::Wrench>("/ur_rtde/ft_sensor", 1);

        // Zero FT Sensor Service Server
        zeroFT_sensor_server_ = create_service<std_srvs::srv::Trigger>("/ur_rtde/zeroFTSensor", std::bind(&RTDEController::zeroFTSensorCallback, this,std::placeholders::_1, std::placeholders::_2));

    }

    // ROS - Publishers
    joint_state_pub_         = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 1);
    tcp_pose_pub_            = create_publisher<geometry_msgs::msg::Pose>("/ur_rtde/cartesian_pose", 1);
    trajectory_executed_pub_ = create_publisher<std_msgs::msg::Bool>("/ur_rtde/trajectory_executed", 1);

    // ROS - Subscribers
    trajectory_command_sub_         = create_subscription<trajectory_msgs::msg::JointTrajectory>("/ur_rtde/controllers/trajectory_controller/command", 1, std::bind(&RTDEController::jointTrajectoryCallback, this, std::placeholders::_1));
    joint_goal_command_sub_         = create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>("/ur_rtde/controllers/joint_space_controller/command", 1, std::bind(&RTDEController::jointGoalCallback, this, std::placeholders::_1));
    cartesian_goal_command_sub_     = create_subscription<ur_rtde_controller::msg::CartesianPoint>("/ur_rtde/controllers/cartesian_space_controller/command", 1, std::bind(&RTDEController::cartesianGoalCallback, this, std::placeholders::_1));
    joint_velocity_command_sub_     = create_subscription<std_msgs::msg::Float64MultiArray>("/ur_rtde/controllers/joint_velocity_controller/command", 1, std::bind(&RTDEController::jointVelocityCallback, this, std::placeholders::_1));
    cartesian_velocity_command_sub_ = create_subscription<geometry_msgs::msg::Twist>("/ur_rtde/controllers/cartesian_velocity_controller/command", 1, std::bind(&RTDEController::cartesianVelocityCallback, this, std::placeholders::_1));
	digital_io_set_sub_				= create_subscription<std_msgs::msg::Int8>("/ur_rtde/digitalIO/command", 1, std::bind(&RTDEController::digitalIOSetCallback, this, std::placeholders::_1));

    // ROS - Service Servers
    stop_robot_server_          = create_service<std_srvs::srv::Trigger>("/ur_rtde/controllers/stop_robot", std::bind(&RTDEController::stopRobotCallback, this,std::placeholders::_1, std::placeholders::_2));
    set_async_parameter_server_ = create_service<std_srvs::srv::SetBool>("/ur_rtde/param/set_asynchronous", std::bind(&RTDEController::setAsyncParameterCallback, this,std::placeholders::_1, std::placeholders::_2));
    start_FreedriveMode_server_ = create_service<ur_rtde_controller::srv::StartFreedriveMode>("/ur_rtde/FreedriveMode/start", std::bind(&RTDEController::startFreedriveModeCallback, this,std::placeholders::_1, std::placeholders::_2));
    stop_FreedriveMode_server_  = create_service<std_srvs::srv::Trigger>("/ur_rtde/FreedriveMode/stop", std::bind(&RTDEController::stopFreedriveModeCallback, this,std::placeholders::_1, std::placeholders::_2));
	zeroFT_sensor_server_ 		= create_service<std_srvs::srv::Trigger>("/ur_rtde/zeroFTSensor", std::bind(&RTDEController::zeroFTSensorCallback, this,std::placeholders::_1, std::placeholders::_2));
    get_FK_server_              = create_service<ur_rtde_controller::srv::GetForwardKinematic>("/ur_rtde/getFK", std::bind(&RTDEController::getForwardKinematicCallback, this,std::placeholders::_1, std::placeholders::_2));
    get_IK_server_              = create_service<ur_rtde_controller::srv::GetInverseKinematic>("/ur_rtde/getIK", std::bind(&RTDEController::getInverseKinematicCallback, this,std::placeholders::_1, std::placeholders::_2));
    get_safety_status_server_   = create_service<ur_rtde_controller::srv::GetRobotStatus>("/ur_rtde/getSafetyStatus", std::bind(&RTDEController::getSafetyStatusCallback, this,std::placeholders::_1, std::placeholders::_2));

    rclcpp::sleep_for(std::chrono::seconds(1));
    std::cout << std::endl;
    RCLCPP_WARN(get_logger(), "UR RTDE Controller - Connected\n");

}

RTDEController::~RTDEController()
{
    // Stop Robot
    stopRobot();

    // Disconnect RTDE Control Interface
    rtde_control_ -> disconnect();
    std::cout << std::endl;
    RCLCPP_WARN(get_logger(), "UR RTDE Controller - Disconnected\n");
}

// TODO: FIX Trajectory Function -> Doesn't Work with Dynamic Planner
void RTDEController::jointTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
    // Initialize Error
    double err = 0.0;

    // Check if the Initial Point == Actual Joint Position
    for (uint i = 0; i < msg->points.begin()->positions.size(); i++)
        err = std::max(std::fabs(msg->points.begin()->positions[i] - actual_joint_position_[i]), err);

    // Return Error If Trajectory Starting Point != First Trajectory Point 
    if (err > SENSOR_ERROR) {RCLCPP_ERROR(get_logger(), "Trajectory Not Starting from the Actual Configuration.\n"); return;}

    // Ensure Initial and Final Point Velocity = 0
    if ((Eigen::ArrayXd::Map(msg->points.front().velocities.data(), msg->points.front().velocities.size()) >= Eigen::ArrayXd::Constant(6, SENSOR_ERROR)).any()
    ||  (Eigen::ArrayXd::Map(msg->points.back().velocities.data(), msg->points.back().velocities.size()) >= Eigen::ArrayXd::Constant(6, SENSOR_ERROR)).any())
        {RCLCPP_ERROR(get_logger(), "Trajectory Starting/Final Velocity != 0\n"); return;}

    // Polynomial Interpolation Object
    PolyFit::trajectory trajectory;
    trajectory.points.resize(msg->points.size());

    // Compose Trajectory Message
    for (uint i = 0; i < msg->points.size(); i++)
    {
        trajectory.points[i].position = msg->points[i].positions;
        if (msg->points[i].velocities.size())    trajectory.points[i].velocity = msg->points[i].velocities;
        if (msg->points[i].accelerations.size()) trajectory.points[i].acceleration = msg->points[i].accelerations;
        // trajectory.points[i].time = msg->points[i].time_from_start;
        trajectory.points[i].time = durationToSec(msg->points[i].time_from_start);
    }

    // Compute Polynomial Fitting
    if (polynomial_fit_.computePolynomials(trajectory))
    {
        // Check if the Resulting Trajectory Comply with the Limits. 
        if (polynomial_fit_.evaluateMaxPolynomials(0.002) > JOINT_LIMITS || polynomial_fit_.evaluateMaxPolynomialsDer(0.002) > JOINT_VELOCITY_MAX || polynomial_fit_.evaluateMaxPolynomialsDDer(0.002) > JOINT_ACCELERATION_MAX)
            {RCLCPP_ERROR(get_logger(), "ERROR: Joint Limit Not Satisfied.\n"); return;}

        // New Trajectory Received
        trajectory_time_ = 0.0;
        new_trajectory_received_ = true;
        RCLCPP_INFO(get_logger(), "New Trajectory Received\n");

    } else {RCLCPP_ERROR(get_logger(), "ERROR: Unable to Fit the Trajectory! | Check Data Points.\n");}
}

void RTDEController::jointGoalCallback(const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg)
{
    // Check Input Data Size
    if (msg->positions.size() != 6) {RCLCPP_ERROR(get_logger(), "ERROR: Received Joint Position Goal Size != 6\n"); return;}
    if (durationToSec(msg->time_from_start) == 0 && msg->velocities.size() == 0) {RCLCPP_ERROR(get_logger(), "ERROR: Desired Time = 0\n"); return;}
    else if (durationToSec(msg->time_from_start) == 0 && msg->velocities[0] <= 0.0) {RCLCPP_ERROR(get_logger(), "ERROR: Desired Time = 0 | Desired Velocity <= 0\n"); return;}

    // Get Desired and Actual Joint Pose
    Eigen::VectorXd desired_pose = Eigen::VectorXd::Map(msg->positions.data(), msg->positions.size());
    Eigen::VectorXd actual_pose  = Eigen::VectorXd::Map(actual_joint_position_.data(), actual_joint_position_.size());

    // Check Joint Limits
    if (!rtde_control_ -> isJointsWithinSafetyLimits(msg->positions)) {RCLCPP_ERROR(get_logger(), "ERROR: Received Joint Position Outside Safety Limits\n"); return;}

    // Initialize Velocity and Acceleration
    double velocity, acceleration = 4.0;

    // Compute Velocity Using Time
    if (durationToSec(msg->time_from_start) != 0)
    {
        // Path Length
        double LP = (desired_pose - actual_pose).array().abs().maxCoeff();
        double T = durationToSec(msg->time_from_start);

        // Check Acceleration is Sufficient to Reach the Goal in the Desired Time
        if (acceleration < 4 * LP / std::pow(T,2))
        {
            T = std::sqrt(4 * LP / acceleration);
            RCLCPP_WARN_STREAM(get_logger(), "Robot Acceleration is Not Sufficient to Reach the Goal in the Desired Time | Used the Minimum Time: " << T << std::endl);
        }

        // Compute Velocity
        double ta = T/2.0 - 0.5 * std::sqrt((std::pow(T,2) * acceleration - 4.0 * LP) / acceleration + 10e-12);
        velocity = ta * acceleration;

    // Use Given Velocity
    } else {velocity = msg->velocities[0];}

    // Check Velocity Limits
    if (velocity > JOINT_VELOCITY_MAX) {RCLCPP_ERROR(get_logger(), "Requested Velocity > Maximum Velocity\n"); return;}

    // Move to Joint Goal
    rtde_control_ -> moveJ(msg->positions, velocity, acceleration, asynchronous_);

    // Publish Trajectory Executed
    if (!asynchronous_) publishTrajectoryExecuted();
    else new_async_joint_pose_received_ = true;
}

void RTDEController::cartesianGoalCallback(const ur_rtde_controller::msg::CartesianPoint::SharedPtr msg)
{
    // Convert Geometry Pose to RTDE Pose
    std::vector<double> desired_pose = Pose2RTDE(msg->cartesian_pose);

    // Check Pose Limits
    if (!rtde_control_ -> isPoseWithinSafetyLimits(desired_pose)) {RCLCPP_ERROR(get_logger(), "ERROR: Received Cartesian Position Outside Safety Limits\n"); return;}

    // TODO: Convert Desired Time to Velocity

    // Check Tool Velocity Limits
    if (msg->velocity > TOOL_VELOCITY_MAX) {RCLCPP_ERROR(get_logger(), "Requested Velocity > Maximum Velocity\n"); return;}

    // Move to Linear Goal
    rtde_control_ -> moveL(desired_pose, msg->velocity, 1.20, asynchronous_);

    // Publish Trajectory Executed
    if (!asynchronous_) publishTrajectoryExecuted();
    else new_async_cartesian_pose_received_ = true;
}

void RTDEController::jointVelocityCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    // Check Input Data Size
    if (msg->data.size() != 6) {RCLCPP_ERROR(get_logger(), "ERROR: Received Joint Velocity Size != 6\n"); return;}

    // Get Current and Desired Joint Velocity
    std::vector<double> current_velocity = rtde_receive_ -> getActualQd();
    std::vector<double> desired_velocity = msg->data;

    // Compute Velocity Difference
    Eigen::VectorXd velocity_difference = Eigen::VectorXd::Map(desired_velocity.data(), desired_velocity.size()) 
                                        - Eigen::VectorXd::Map(current_velocity.data(), current_velocity.size());

    // Compute MAX Acceleration
    double acceleration = velocity_difference.array().abs().maxCoeff() / ros_rate_;
    acceleration = sign(acceleration) * std::max(std::fabs(acceleration), 1.0);
    acceleration = 10.0;

    // Check Acceleration Limits
    if (limit_acc_ && acceleration > JOINT_ACCELERATION_MAX) {RCLCPP_ERROR(get_logger(), "Requested Acceleration > Maximum Acceleration\n"); return;}

    // Joint Velocity Publisher
    rtde_control_ -> speedJ(desired_velocity, acceleration, 0.002);
}

void RTDEController::cartesianVelocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // Get Current Cartesian Velocity
    std::vector<double> current_velocity = rtde_receive_ -> getActualTCPSpeed();

    // Create Desired Velocity Vector
    std::vector<double> desired_cartesian_velocity;
    desired_cartesian_velocity.push_back(msg->linear.x);
    desired_cartesian_velocity.push_back(msg->linear.y);
    desired_cartesian_velocity.push_back(msg->linear.z);
    desired_cartesian_velocity.push_back(msg->angular.x);
    desired_cartesian_velocity.push_back(msg->angular.y);
    desired_cartesian_velocity.push_back(msg->angular.z);

    // TODO: Compute Velocity Difference

    // TODO: Compute MAX Acceleration
    // double acceleration = velocity_difference.array().abs().maxCoeff() / ros_rate_.expectedCycleTime().seconds();
    double acceleration = 0.25;

    // Check Acceleration Limits
    if (acceleration > TOOL_ACCELERATION_MAX) {RCLCPP_ERROR(get_logger(), "Requested Acceleration > Maximum Acceleration\n"); return;}

    // Cartesian Velocity Publisher
    rtde_control_ -> speedL(desired_cartesian_velocity, acceleration, 0.002);
}

void RTDEController::digitalIOSetCallback(const std_msgs::msg::Int8::SharedPtr msg)
{
	// Function to set a boolean value in a digital port of the UR IO network
	// output boolean = sign(msg)
	// output id	  = abs(msg)
	uint8_t output_id = abs(msg->data);
	bool signal_level = false;
	if (msg->data > 0) {signal_level = true;}
	rtde_io_ -> setStandardDigitalOut(output_id,signal_level);
}

bool RTDEController::stopRobotCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    auto req = request;

    // Stop Robot
    stopRobot();

    if (!asynchronous_)
    {
        // StopRobot Not Working with Synchronous Operations
        RCLCPP_WARN(get_logger(), "StopRobot Not Working without the Asynchronous Flag");
        response->success = false;

    } else {response->success = true;}

    return true;
}

bool RTDEController::setAsyncParameterCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    // Set Asynchronous Parameter
    asynchronous_ = request->data;

    response->success = true;
    return true;
}

bool RTDEController::startFreedriveModeCallback(const std::shared_ptr<ur_rtde_controller::srv::StartFreedriveMode::Request> request, std::shared_ptr<ur_rtde_controller::srv::StartFreedriveMode::Response> response)
{
    // freeAxes = [1,0,0,0,0,0]     -> The robot is compliant in the x direction relative to the feature.
    // freeAxes: A 6 dimensional vector that contains 0’s and 1’s, these indicates in which axes movement is allowed. The first three values represents the cartesian directions along x, y, z, and the last three defines the rotation axis, rx, ry, rz. All relative to the selected feature

    // Start FreeDrive Mode
    response->success = rtde_control_ -> freedriveMode(request->free_axes);
    return true;
}

bool RTDEController::stopFreedriveModeCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    // Exit from FreeDrive Mode
    auto req = request;
    response->success = rtde_control_ -> endFreedriveMode();
    return true;
}

bool RTDEController::zeroFTSensorCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    // Reset Force-Torque Sensor
    auto req = request;
    response->success = rtde_control_ -> zeroFtSensor();
    return true;
}

bool RTDEController::getForwardKinematicCallback(const std::shared_ptr<ur_rtde_controller::srv::GetForwardKinematic::Request> request, std::shared_ptr<ur_rtde_controller::srv::GetForwardKinematic::Response> response)
{
    // Compute Forward Kinematic
    std::vector<double> tcp_pose = rtde_control_ -> getForwardKinematics(request->joint_position, {0.0,0.0,0.0,0.0,0.0,0.0});

    // Convert RTDE Pose to Geometry Pose
    response->tcp_position = RTDE2Pose(tcp_pose);

    response->success = true;
    return true;
}

bool RTDEController::getInverseKinematicCallback(const std::shared_ptr<ur_rtde_controller::srv::GetInverseKinematic::Request> request, std::shared_ptr<ur_rtde_controller::srv::GetInverseKinematic::Response> response)
{
    // Convert Geometry Pose to RTDE Pose
    std::vector<double> tcp_pose = Pose2RTDE(request->tcp_position);

    // Compute Inverse Kinematic
    if (request->near_position.size() == 6) response->joint_position = rtde_control_ -> getInverseKinematics(tcp_pose, request->near_position);
    else response->joint_position = rtde_control_ -> getInverseKinematics(tcp_pose);

    response->success = true;
    return true;
}

bool RTDEController::getSafetyStatusCallback(const std::shared_ptr<ur_rtde_controller::srv::GetRobotStatus::Request> request, std::shared_ptr<ur_rtde_controller::srv::GetRobotStatus::Response> response)
{
    /************************************************
     *                                              *
     *  Safety Status Bits 0-10:                    *
     *                                              *
     *        0 = Is normal mode                    *
     *        1 = Is reduced mode                   *
     *        2 = Is protective stopped             *
     *        3 = Is recovery mode                  *
     *        4 = Is safeguard stopped              *
     *        5 = Is system emergency stopped       *
     *        6 = Is robot emergency stopped        *
     *        7 = Is emergency stopped              *
     *        8 = Is violation                      *
     *        9 = Is fault                          *
     *        10 = Is stopped due to safety         *
     *                                              *
     ***********************************************/

    /************************************************
     *                                              *
     *  Safety Mode                                 *
     *                                              *
     *        0 = NORMAL                            *
     *        1 = REDUCED                           *
     *        2 = PROTECTIVE_STOP                   *
     *        3 = RECOVERY                          *
     *        4 = SAFEGUARD_STOP                    *
     *        5 = SYSTEM_EMERGENCY_STOP             *
     *        6 = ROBOT_EMERGENCY_STOP              *
     *        7 = VIOLATION                         *
     *        8 = FAULT                             *
     *                                              *
     ***********************************************/

    /************************************************
     *                                              *
     *  Robot Mode                                  *
     *                                              *
     *       -1 = ROBOT_MODE_NO_CONTROLLER          *
     *        0 = ROBOT_MODE_DISCONNECTED           *
     *        1 = ROBOT_MODE_CONFIRM_SAFETY         *
     *        2 = ROBOT_MODE_BOOTING                *
     *        3 = ROBOT_MODE_POWER_OFF              *
     *        4 = ROBOT_MODE_POWER_ON               *
     *        5 = ROBOT_MODE_IDLE                   *
     *        6 = ROBOT_MODE_BACKDRIVE              *
     *        7 = ROBOT_MODE_RUNNING                *
     *        8 = ROBOT_MODE_UPDATING_FIRMWARE      *
     *                                              *
     ***********************************************/

    std::vector<std::string> robot_mode_msg = {"ROBOT_MODE_NO_CONTROLLER", "ROBOT_MODE_DISCONNECTED", "ROBOT_MODE_CONFIRM_SAFETY", "ROBOT_MODE_BOOTING", "ROBOT_MODE_POWER_OFF", "ROBOT_MODE_POWER_ON", "ROBOT_MODE_IDLE", "ROBOT_MODE_BACKDRIVE", "ROBOT_MODE_RUNNING", "ROBOT_MODE_UPDATING_FIRMWARE"};
    std::vector<std::string> safety_mode_msg = {"NORMAL", "REDUCED", "PROTECTIVE_STOP", "RECOVERY", "SAFEGUARD_STOP", "SYSTEM_EMERGENCY_STOP", "ROBOT_EMERGENCY_STOP", "VIOLATION" "FAULT"};
    std::vector<std::string> safety_status_bits_msg = {"Is normal mode", "Is reduced mode", "Is protective stopped", "Is recovery mode", "Is safeguard stopped", "Is system emergency stopped", "Is robot emergency stopped", "Is emergency stopped", "Is violation", "Is fault", "Is stopped due to safety"};

    // Get Robot Mode
    auto req = request;
    response->robot_mode = rtde_receive_ -> getRobotMode();
    response->robot_mode_msg = robot_mode_msg[response->robot_mode + 1];

    // Get Safety Mode
    response->safety_mode = rtde_receive_ -> getSafetyMode();
    response->safety_mode_msg = safety_mode_msg[response->safety_mode];

    // Get Safety Status Bits
    response->safety_status_bits = int(rtde_receive_ -> getSafetyStatusBits());
    response->safety_status_bits_msg = safety_status_bits_msg[response->safety_status_bits];

    response->success = true;
    return true;
}

bool RTDEController::RobotiQGripperCallback(const std::shared_ptr<ur_rtde_controller::srv::RobotiQGripperControl::Request> request, std::shared_ptr<ur_rtde_controller::srv::RobotiQGripperControl::Response> response)
{
    // Normalize Received Values
    float position = request->position / 100;
    float speed    = request->speed / 100;
    float force    = request->force / 100;

    // Move Gripper - Normalized Values (0.0 - 1.0)
    try {response->status = robotiq_gripper_ -> move(position, speed, force, ur_rtde::RobotiqGripper::WAIT_FINISHED);}
    catch (const std::exception &e) {return false;}

    /************************************************************************************************
     *                                                                                              *
     * Object Detection Status                                                                      *
     *                                                                                              *
     *    MOVING = 0                  |    Gripper is Opening or Closing                            *
     *    STOPPED_OUTER_OBJECT = 1    |    Outer Object Detected while Opening the Gripper          *
     *    STOPPED_INNER_OBJECT = 2    |    Inner Object Detected while Closing the Gripper          *
     *    AT_DEST = 3                 |    Requested Target Position Reached - No Object Detected   *
     *                                                                                              *
      ***********************************************************************************************/

    response->success = true;
    return true;
}

bool RTDEController::enableRobotiQGripperCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    // Enable RobotiQ Gripper
    auto req = request;
    enable_gripper_ = true;

    // Create the Gripper Class if Doesn't Exist
    if (robotiq_gripper_ == nullptr) robotiq_gripper_ = new ur_rtde::RobotiqGripper(ROBOT_IP, 63352, false);

    // Connect the Gripper if Not Connected
    if (!robotiq_gripper_ -> isConnected()) robotiq_gripper_ -> connect();

    // Activate the Gripper
    robotiq_gripper_ -> activate();

    // Create the Gripper Service Server if Doesn't Exist
    if (robotiq_gripper_server_ == nullptr) robotiq_gripper_server_ = create_service<ur_rtde_controller::srv::RobotiQGripperControl>("/ur_rtde/robotiq_gripper/command", std::bind(&RTDEController::RobotiQGripperCallback, this, std::placeholders::_1, std::placeholders::_2));

    response->success = true;
    return true;
}

bool RTDEController::disableRobotiQGripperCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    // Disable RobotiQ Gripper
    auto req = request;
    enable_gripper_ = false;
    response->success = true;

    // Return if the Gripper Class Doesn't Exist
    if (robotiq_gripper_ == nullptr) return true;

    // Disconnect if the Gripper is Connected
    if (robotiq_gripper_ -> isConnected()) robotiq_gripper_ -> disconnect();

    // Shutdown the Gripper Service Server if Exist
    if (robotiq_gripper_server_ != nullptr) robotiq_gripper_server_ = nullptr;

	response->success = true;
    return true;
}

bool RTDEController::currentPositionRobotiQGripperCallback(const std::shared_ptr<ur_rtde_controller::srv::GetGripperPosition::Request> request, std::shared_ptr<ur_rtde_controller::srv::GetGripperPosition::Response> response)
{
	// Get Current RobotiQ Gripper Position
	response->current_position = robotiq_gripper_ -> getCurrentPosition();

	response->success = true;
	return true;
}

void RTDEController::publishJointState()
{
    while (rclcpp::ok() && !shutdown_)
    {
        // Create JointState Message
        sensor_msgs::msg::JointState joint_state;
        joint_state.name = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
        joint_state.header.stamp = now();

        // Read Joint Position and Velocity
        joint_state.position = rtde_receive_ -> getActualQ();
        joint_state.velocity = rtde_receive_ -> getActualQd();

        // Publish JointState
        joint_state_pub_ -> publish(joint_state);

        // Sleep to ROS Rate
        ros_rate.sleep();

    }
}

void RTDEController::publishTCPPose()
{
    while (rclcpp::ok() && !shutdown_)
    {
        // Read TCP Position
        std::vector<double> tcp_pose = rtde_receive_ -> getActualTCPPose();

        // Convert RTDE Pose to Geometry Pose
        geometry_msgs::msg::Pose pose = RTDE2Pose(tcp_pose);

        // Publish TCP Pose
        tcp_pose_pub_ -> publish(pose);

        // Sleep to ROS Rate
        ros_rate.sleep();

    }
}

void RTDEController::publishFTSensor()
{

    // Return if the FT Sensor is Disabled
    if (!ft_sensor_) return;

    while (rclcpp::ok() && !shutdown_)
    {
        // Read FT Sensor Forces
        std::vector<double> tcp_forces = rtde_receive_ -> getActualTCPForce();

        // Create Wrench Message
        geometry_msgs::msg::Wrench forces;
        forces.force.x = tcp_forces[0];
        forces.force.y = tcp_forces[1];
        forces.force.z = tcp_forces[2];
        forces.torque.x = tcp_forces[3];
        forces.torque.y = tcp_forces[4];
        forces.torque.z = tcp_forces[5];

        // Publish FTSensor Forces
        ft_sensor_pub_ -> publish(forces);

        // Sleep to ROS Rate
        ros_rate.sleep();

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
    std_msgs::msg::Bool trajectory_executed;
    trajectory_executed.data = true;
    trajectory_executed_pub_ -> publish(trajectory_executed);

    // Reset Booleans Variables
    resetBooleans();
}

std::vector<double> RTDEController::Pose2RTDE(geometry_msgs::msg::Pose pose)
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

geometry_msgs::msg::Pose RTDEController::RTDE2Pose(std::vector<double> rtde_pose)
{
    // Compute AngleAxis from rx,ry,rz
    double angle = sqrt(pow(rtde_pose[3],2) + pow(rtde_pose[4],2) + pow(rtde_pose[5],2));
    Eigen::Vector3d axis(rtde_pose[3], rtde_pose[4], rtde_pose[5]);
    axis = axis.normalized();

    // Convert Euler to Quaternion
    Eigen::Quaterniond quaternion(Eigen::AngleAxisd(angle, axis));

    // Write TCP Pose in Geometry Pose Message
    geometry_msgs::msg::Pose pose;
    pose.position.x = rtde_pose[0];
    pose.position.y = rtde_pose[1];
    pose.position.z = rtde_pose[2];
    pose.orientation.x = quaternion.x();
    pose.orientation.y = quaternion.y();
    pose.orientation.z = quaternion.z();
    pose.orientation.w = quaternion.w();

    return pose;
}

Eigen::Matrix<double, 4, 4> RTDEController::pose2eigen(geometry_msgs::msg::Pose pose)
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

bool RTDEController::isJointReached()
{
    // Compute Joint Error
    Eigen::VectorXd error = polynomial_fit_.getLastPoint() - Eigen::Map<Eigen::VectorXd>(actual_joint_position_.data(), actual_joint_position_.size());
    if (error.cwiseAbs().maxCoeff() < SENSOR_ERROR && trajectory_time_ > polynomial_fit_.getFinalTime()) return true;
    else return false;
}

void RTDEController::moveTrajectory()
{
    // Return if No Trajectory Received
    if (!new_trajectory_received_) return;

    // Check if Trajectory is Ended
    if (isJointReached())
    {
        // Stop Speed Mode
        rtde_control_ -> speedStop();

        // Publish Trajectory Executed
        publishTrajectoryExecuted();
        resetBooleans();
        return;
    }

    // Create the Desired Velocity Vector
    Eigen::VectorXd trajectory_vel = polynomial_fit_.evaluatePolynomialsDer(trajectory_time_);
    trajectory_vel += (polynomial_fit_.evaluatePolynomials(trajectory_time_) - Eigen::Map<Eigen::VectorXd>(actual_joint_position_.data(), actual_joint_position_.size()));
    std::vector<double> desired_velocity(trajectory_vel.data(), trajectory_vel.data() + trajectory_vel.size());

    // Create the Desired Acceleration Vector
    Eigen::VectorXd acc = polynomial_fit_.evaluatePolynomialsDDer(trajectory_time_);

    // Move Robot with Velocity Commands
    rtde_control_ -> speedJ(desired_velocity, (acc.cwiseAbs()).maxCoeff(), 5e-4);

    // Increase trajectory_time_
    trajectory_time_ += 0.002;
}

void RTDEController::checkAsyncMovements()
{
    // Return if No Async Movement Received
    if (!new_async_joint_pose_received_ and !new_async_cartesian_pose_received_) return;

    // Check if Async Operation is Ended -> Trajectory Executed
    if (rtde_control_ -> getAsyncOperationProgress() < 0) publishTrajectoryExecuted();
}

void RTDEController::stopRobot()
{
    // Stop Robot
    rtde_control_ -> stopJ(2.0);

    // Wait
    rclcpp::sleep_for(std::chrono::milliseconds(100));

    // Clear Dashboard Warning Pop-Up
    rtde_dashboard_ -> connect();
    rtde_dashboard_ -> closePopup();
    rtde_dashboard_ -> disconnect();

    // Reset Booleans Variables
    resetBooleans();
}

void RTDEController::checkRobotStatus()
{
    // Init Flags
    bool eStop = false, protectiveStop = false;

    // Print Robot Emergency and Protective Stop
    while (rtde_receive_ -> isEmergencyStopped())  {RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 5000, "EMERGENCY STOP PRESSED"); eStop = true;}
    while (rtde_receive_ -> isProtectiveStopped()) {RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 5000, "PROTECTIVE STOP");         protectiveStop = true;}

    // Check Flags
    if (eStop)
    {
        // Info Prints
        RCLCPP_WARN(get_logger(), "EMERGENCY STOP RELEASED\n");

        // Check if Robot Mode is ROBOT_MODE_RUNNING
        while (rtde_receive_ -> getRobotMode() != ROBOT_MODE_RUNNING) {RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 5000, "Wait for Robot Recovery...");}
    
    } else if (protectiveStop) {RCLCPP_WARN(get_logger(), "PROTECTIVE STOP RECOVERED\n");}
        
    if (eStop || protectiveStop)
    {
        // Re-Upload RTDE Control Script
        rtde_control_ -> reuploadScript();
        rtde_control_ -> disconnect();
        rtde_control_ -> reconnect();

        // Wait Time For Connection
        rclcpp::sleep_for(std::chrono::seconds(1));

        // Print Robot Ready
        std::cout << std::endl;
        RCLCPP_WARN(get_logger(), "Robot Ready to Receive New Commands\n");

        // Reset Booleans
        resetBooleans();
    }

    // Check Robot Connection Status
    if (!rtde_control_ -> isConnected()) RCLCPP_ERROR(get_logger(), "ROBOT DISCONNECTED\n");
}

void RTDEController::spinner()
{
    // Callback Readings
    rclcpp::spin_some(this->get_node_base_interface());

    // Check UR Status
    checkRobotStatus();

    // Update Actual Joint and TCP Positions
    actual_joint_position_ = rtde_receive_ -> getActualQ();
    actual_joint_velocity_ = rtde_receive_ -> getActualQd();
    actual_cartesian_pose_ = RTDE2Pose(rtde_receive_ -> getActualTCPPose());

    // Trajectory Controller
    moveTrajectory();

    // Check Async Movements Status
    checkAsyncMovements();

    // Sleep to ROS Rate
    ros_rate.sleep();

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

    // Initialize ROS
    rclcpp::init(argc, argv);

    // Create a SIGINT Handler
    struct sigaction sa;
    sa.sa_handler = signalHandler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, NULL);
    std::cout << std::endl;

    // Create a New RTDEController
    rtde = new RTDEController();

    // Publish JointState, TCPPose, FTSensor in separate Threads
    publishJointState = new std::thread(&RTDEController::publishJointState, rtde);
    publishTCPPose    = new std::thread(&RTDEController::publishTCPPose,    rtde);
    publishFTSensor   = new std::thread(&RTDEController::publishFTSensor,   rtde);
    rclcpp::sleep_for(std::chrono::seconds(1));

    // Main Spinner
    while (rclcpp::ok()) {rtde -> spinner();}

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
