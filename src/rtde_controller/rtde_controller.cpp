#include "rtde_controller/rtde_controller.h"
#include <cmath>

int sign(double value) {
    if (value > 0) {return 1;}
    else if (value < 0) {return -1;}
    else {return 0;}
}

double durationToSec(rclcpp::Duration duration) {
    return duration.seconds();
}

RTDEController::RTDEController(): Node ("ur_rtde_controller") {
    // All users of RTDEControlInterface share this mutually exclusive group.
    control_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Declare Parameters
    declare_parameter<std::string>("ROBOT_IP", std::string("192.168.2.30"));
    declare_parameter<bool>("enable_gripper", false);
    declare_parameter<bool>("asynchronous", false);
    declare_parameter<bool>("limit_acc", false);
    declare_parameter<bool>("ft_sensor", true);
    declare_parameter<double>("rate", 500.0);

    // Load Parameters
    if(!get_parameter_or("ROBOT_IP", ROBOT_IP, std::string("192.168.2.30"))) {RCLCPP_ERROR_STREAM(get_logger(), "Failed To Get \"ROBOT_IP\" Param. Using Default: " << ROBOT_IP);}
    if(!get_parameter_or("enable_gripper", enable_gripper_, false)) {RCLCPP_ERROR_STREAM(get_logger(), "Failed To Get \"gripper_enabled\" Param. Using Default: " << enable_gripper_);}
    if(!get_parameter_or("asynchronous", asynchronous_, false)) {RCLCPP_ERROR_STREAM(get_logger(), "Failed To Get \"asynchronous\" Param. Using Default: " << asynchronous_);}
    if(!get_parameter_or("limit_acc", limit_acc_, false)) {RCLCPP_ERROR_STREAM(get_logger(), "Failed To Get \"limit_acc\" Param. Using Default: " << limit_acc_);}
    if(!get_parameter_or("ft_sensor", ft_sensor_, true)) {RCLCPP_ERROR_STREAM(get_logger(), "Failed To Get \"ft_sensor\" Param. Using Default: " << ft_sensor_);}
    if(!get_parameter_or("rate", rate_, 500.0)) {RCLCPP_ERROR_STREAM(get_logger(), "Failed To Get \"rate\" Param. Using Default: " << rate_);}

    // Log Parameters
    std::ostringstream oss; oss << std::boolalpha;
    oss << "PARAMETERS LIST:\nROBOT_IP:           " << ROBOT_IP << "\n";
    oss << "Enable Gripper:     " << enable_gripper_ << "\n";
    oss << "Asynchronous:       " << asynchronous_ << "\n";
    oss << "Limit Acceleration: " << limit_acc_ << "\n";
    oss << "FT Sensor:          " << ft_sensor_ << "\n";
    oss << "Rate:               " << rate_ << " Hz";
    RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());

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
        if (rtde_dashboard_initialized && rtde_control_initialized && !rtde_dashboard_ -> running()) try {rtde_control_ -> reuploadScript(); rtde_dashboard_ -> disconnect();}
        catch (const std::exception &e) {RCLCPP_ERROR_STREAM(get_logger(), "Failed to Reupload the RTDE Control Script:\n" << e.what());}

        // Robot Initialized
        if (rtde_dashboard_initialized && rtde_dashboard_connected && rtde_control_initialized && rtde_receive_initialized && rtde_io_initialized) robot_initialized = true;

    }

    if (!robot_initialized) { throw std::runtime_error("Robot initialization interrupted"); }

    // RobotiQ Gripper
    if (enable_gripper_) {

        try {

            // Initialize Gripper
            robotiq_gripper_ = new ur_rtde::RobotiqGripper(ROBOT_IP, 63352, false);
            robotiq_gripper_ -> connect();
            robotiq_gripper_ -> activate();

            // Gripper Service Server
            robotiq_gripper_server_ = create_service<ur_rtde_controller::srv::RobotiQGripperControl>("/ur_rtde/robotiq_gripper/command", std::bind(&RTDEController::RobotiQGripperCallback, this, std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS(), control_callback_group_);

            // Gripper Enable/Disable Service Servers
            enable_gripper_server_  = create_service<std_srvs::srv::Trigger>("/ur_rtde/robotiq_gripper/enable", std::bind(&RTDEController::enableRobotiQGripperCallback, this,std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS(), control_callback_group_);
            disable_gripper_server_ = create_service<std_srvs::srv::Trigger>("/ur_rtde/robotiq_gripper/disable", std::bind(&RTDEController::disableRobotiQGripperCallback, this,std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS(), control_callback_group_);

			// Gripper Current Position Service Server
			gripper_current_position_server_ = create_service<ur_rtde_controller::srv::GetGripperPosition>("/ur_rtde/robotiq_gripper/current_position", std::bind(&RTDEController::currentPositionRobotiQGripperCallback, this,std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS(), control_callback_group_);

        } catch(const std::exception& e) {std::cerr << "Error: " << e.what() << std::endl; RCLCPP_ERROR(get_logger(), "Failed to Start the RobotiQ 2F Gripper");}

    }

    if (ft_sensor_) {

        // Zero FT Sensor
        rtde_control_ -> zeroFtSensor();

        // FT Sensor Publisher
        ft_sensor_pub_ = create_publisher<geometry_msgs::msg::Wrench>("/ur_rtde/ft_sensor", 1);

        // Zero FT Sensor Service Server
        zeroFT_sensor_server_ = create_service<std_srvs::srv::Trigger>("/ur_rtde/zeroFTSensor", std::bind(&RTDEController::zeroFTSensorCallback, this,std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS(), control_callback_group_);

    }

    // ROS - Publishers
    joint_state_.name = {"shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"};
    joint_state_.position.resize(6, 0.0);
    joint_state_.velocity.resize(6, 0.0);

    joint_state_pub_         = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 1);
    tcp_pose_pub_            = create_publisher<geometry_msgs::msg::Pose>("/ur_rtde/cartesian_pose", 1);
    trajectory_executed_pub_ = create_publisher<std_msgs::msg::Bool>("/ur_rtde/trajectory_executed", 1);

    // Commands are reliable and volatile: never replay a previous motion to a late subscriber.
    const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    // ROS - Subscribers
    trajectory_callback_group_ = control_callback_group_;
    rclcpp::SubscriptionOptions sub_options; sub_options.callback_group = trajectory_callback_group_;
    trajectory_command_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>("/ur_rtde/controllers/trajectory_controller/command",
                                command_qos, std::bind(&RTDEController::jointTrajectoryCallback, this, std::placeholders::_1),sub_options);


    joint_goal_callback_group_ = control_callback_group_;
    sub_options.callback_group = joint_goal_callback_group_;
    joint_goal_command_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>("/ur_rtde/controllers/joint_space_controller/command",
                                command_qos, std::bind(&RTDEController::jointGoalCallback, this, std::placeholders::_1),sub_options);
    
    
    cartesian_goal_callback_group_ = control_callback_group_;
    sub_options.callback_group = cartesian_goal_callback_group_;
    cartesian_goal_command_sub_ = create_subscription<ur_rtde_controller::msg::CartesianPoint>("/ur_rtde/controllers/cartesian_space_controller/command",
                                    command_qos, std::bind(&RTDEController::cartesianGoalCallback, this, std::placeholders::_1),sub_options);
    

    joint_velocity_callback_group_ = control_callback_group_;
    sub_options.callback_group = joint_velocity_callback_group_;
    joint_velocity_command_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>("/ur_rtde/controllers/joint_velocity_controller/command",
                                    command_qos, std::bind(&RTDEController::jointVelocityCallback, this, std::placeholders::_1),sub_options);
    
    
    cartesian_velocity_callback_group_ = control_callback_group_;
    sub_options.callback_group = cartesian_velocity_callback_group_;
    cartesian_velocity_command_sub_ = create_subscription<geometry_msgs::msg::Twist>("/ur_rtde/controllers/cartesian_velocity_controller/command",
                                        command_qos, std::bind(&RTDEController::cartesianVelocityCallback, this, std::placeholders::_1),sub_options);
	
    
    digital_io_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sub_options.callback_group = digital_io_callback_group_;
    digital_io_set_sub_	= create_subscription<std_msgs::msg::Int8>("/ur_rtde/digitalIO/command", 1,
                            std::bind(&RTDEController::digitalIOSetCallback, this, std::placeholders::_1),sub_options);
    
    
    tool_digital_io_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sub_options.callback_group = tool_digital_io_callback_group_;
    tool_digital_io_set_sub_ = create_subscription<std_msgs::msg::Int8>("/ur_rtde/tool_digitalIO/command", 1,
                                std::bind(&RTDEController::toolDigitalIOSetCallback, this, std::placeholders::_1),sub_options);

    // ROS - Service Servers
    set_velocity_commands_server_ = create_service<std_srvs::srv::SetBool>(
        "/ur_rtde/controllers/set_velocity_commands",
        std::bind(&RTDEController::setVelocityCommandsCallback, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(), control_callback_group_);
    stop_robot_server_          = create_service<std_srvs::srv::Trigger>("/ur_rtde/controllers/stop_robot", std::bind(&RTDEController::stopRobotCallback, this,std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS(), control_callback_group_);
    set_async_parameter_server_ = create_service<std_srvs::srv::SetBool>("/ur_rtde/param/set_asynchronous", std::bind(&RTDEController::setAsyncParameterCallback, this,std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS(), control_callback_group_);
    start_FreedriveMode_server_ = create_service<ur_rtde_controller::srv::StartFreedriveMode>("/ur_rtde/FreedriveMode/start", std::bind(&RTDEController::startFreedriveModeCallback, this,std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS(), control_callback_group_);
    stop_FreedriveMode_server_  = create_service<std_srvs::srv::Trigger>("/ur_rtde/FreedriveMode/stop", std::bind(&RTDEController::stopFreedriveModeCallback, this,std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS(), control_callback_group_);
	zeroFT_sensor_server_ 		= create_service<std_srvs::srv::Trigger>("/ur_rtde/zeroFTSensor", std::bind(&RTDEController::zeroFTSensorCallback, this,std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS(), control_callback_group_);
    get_FK_server_              = create_service<ur_rtde_controller::srv::GetForwardKinematic>("/ur_rtde/getFK", std::bind(&RTDEController::getForwardKinematicCallback, this,std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS(), control_callback_group_);
    get_IK_server_              = create_service<ur_rtde_controller::srv::GetInverseKinematic>("/ur_rtde/getIK", std::bind(&RTDEController::getInverseKinematicCallback, this,std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS(), control_callback_group_);
    get_safety_status_server_   = create_service<ur_rtde_controller::srv::GetRobotStatus>("/ur_rtde/getSafetyStatus", std::bind(&RTDEController::getSafetyStatusCallback, this,std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS(), control_callback_group_);

    rclcpp::sleep_for(std::chrono::seconds(1));
    std::cout << std::endl;
    RCLCPP_WARN(get_logger(), "UR RTDE Controller - Connected\n");
}

RTDEController::~RTDEController()
{
    // spinner() has returned and its executor threads have joined before destruction.
    try
    {
        if (rtde_control_ && rtde_control_->isConnected())
        {
            if (rtde_control_->isProgramRunning())
            {
                if (motion_mode_ == MotionMode::JointVelocity || motion_mode_ == MotionMode::CartesianVelocity ||
                    motion_mode_ == MotionMode::Trajectory) { rtde_control_->speedStop(2.0); }
                else if (motion_mode_ == MotionMode::Freedrive) { rtde_control_->endFreedriveMode(); }
                else if (motion_mode_ == MotionMode::CartesianGoal) { rtde_control_->stopL(2.0); }
                else if (motion_mode_ == MotionMode::JointGoal) { rtde_control_->stopJ(2.0); }
            }
            rtde_control_->disconnect();
        }
    }
    catch (const std::exception & error)
    {
        RCLCPP_ERROR(get_logger(), "RTDE shutdown: %s", error.what());
    }
    delete robotiq_gripper_;
    delete rtde_io_;
    delete rtde_receive_;
    delete rtde_control_;
    delete rtde_dashboard_;
}

bool RTDEController::motionBusy() const
{
    return motion_mode_ == MotionMode::Trajectory || motion_mode_ == MotionMode::JointGoal ||
           motion_mode_ == MotionMode::CartesianGoal || motion_mode_ == MotionMode::Fault;
}

void RTDEController::publishMotionResult(bool success)
{
    if (!rclcpp::ok()) { return; }
    std_msgs::msg::Bool result;
    result.data = success;
    trajectory_executed_pub_->publish(result);
}

void RTDEController::failMotion(const std::string & reason)
{
    velocity_commands_enabled_ = false;
    motion_mode_ = MotionMode::Fault;
    resetBooleans();
    RCLCPP_ERROR(get_logger(), "%s Velocity commands remain disabled.", reason.c_str());
    publishMotionResult(false);
}

bool RTDEController::stopVelocityAndFreedrive()
{
    try
    {
        if (!rtde_control_->isConnected() || !rtde_control_->isProgramRunning())
        {
            RCLCPP_ERROR(get_logger(), "Motion handoff refused: RTDE control is disconnected or its program is stopped.");
            return false;
        }
        if ((motion_mode_ == MotionMode::JointVelocity || motion_mode_ == MotionMode::CartesianVelocity) &&
            !rtde_control_->speedStop(2.0))
        {
            failMotion("Motion handoff failed: speedStop returned false.");
            return false;
        }
        // Also exit freedrive started through the pendant, rather than only the ROS service.
        if (!rtde_control_->endFreedriveMode())
        {
            failMotion("Motion handoff failed: endFreedriveMode returned false.");
            return false;
        }
        motion_mode_ = MotionMode::Idle;
        return true;
    }
    catch (const std::exception & error)
    {
        failMotion(std::string("Motion handoff failed: ") + error.what());
        return false;
    }
}

bool RTDEController::beginPositionMotion(MotionMode mode)
{
    velocity_commands_enabled_ = false;
    if (motionBusy())
    {
        RCLCPP_ERROR(get_logger(), "Motion command rejected: another position/trajectory motion is active or the controller is faulted.");
        return false;
    }
    if (!stopVelocityAndFreedrive()) { return false; }
    motion_mode_ = mode;
    return true;
}

bool RTDEController::setVelocityCommandsCallback(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response)
{
    try
    {
        if (!request->data)
        {
            // Close the gate before stopping the current mode; queued messages will be discarded.
            velocity_commands_enabled_ = false;
            response->success = !motionBusy() && stopVelocityAndFreedrive();
            response->message = response->success ? "Velocity commands disabled; speed/freedrive stopped." :
                                                   "Handoff refused; velocity commands remain disabled.";
        }
        else
        {
            response->success = motion_mode_ == MotionMode::Idle && rtde_control_->isConnected() &&
                                rtde_control_->isProgramRunning();
            if (response->success)
            {
                // These volatile messages were queued before the resume acknowledgement.
                std_msgs::msg::Float64MultiArray joint_velocity;
                geometry_msgs::msg::Twist cartesian_velocity;
                rclcpp::MessageInfo info;
                while (joint_velocity_command_sub_->take(joint_velocity, info)) {}
                while (cartesian_velocity_command_sub_->take(cartesian_velocity, info)) {}
                velocity_commands_enabled_ = true;
            }
            response->message = response->success ? "Velocity commands enabled." :
                                                   "Cannot enable velocity commands while motion is active or control is unavailable.";
        }
    }
    catch (const std::exception & error)
    {
        velocity_commands_enabled_ = false;
        motion_mode_ = MotionMode::Fault;
        response->success = false;
        response->message = error.what();
    }
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    return true;
}

// TODO: FIX Trajectory Function -> Doesn't Work with Dynamic Planner
void RTDEController::jointTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
    if (motionBusy() || msg->points.empty())
    {
        RCLCPP_ERROR(get_logger(), "Trajectory rejected: empty input or another motion is active.");
        return;
    }
    const auto actual_joint_position = rtde_receive_->getActualQ();
    // Initialize Error
    double err = 0.0;

    // Check if the Initial Point == Actual Joint Position
    for (uint i = 0; i < msg->points.begin()->positions.size(); i++)
        err = std::max(std::fabs(msg->points.begin()->positions[i] - actual_joint_position[i]), err);

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

        if (!beginPositionMotion(MotionMode::Trajectory)) { return; }
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

    if (motionBusy()) { RCLCPP_ERROR(get_logger(), "Joint goal rejected: another motion is active."); return; }
    const auto actual_joint_position = rtde_receive_->getActualQ();
    // Get Desired and Actual Joint Pose
    Eigen::VectorXd desired_pose = Eigen::VectorXd::Map(msg->positions.data(), msg->positions.size());
    Eigen::VectorXd actual_pose  = Eigen::Map<const Eigen::VectorXd>(actual_joint_position.data(), actual_joint_position.size());

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

    if (!beginPositionMotion(MotionMode::JointGoal)) { return; }
    try
    {
        if (!rtde_control_->moveJ(msg->positions, velocity, acceleration, asynchronous_))
        {
            failMotion("Joint moveJ returned false.");
            return;
        }
        if (!asynchronous_) { publishTrajectoryExecuted(); }
        else { new_async_joint_pose_received_ = true; }
    }
    catch (const std::exception & error) { failMotion(error.what()); }
}

void RTDEController::cartesianGoalCallback(const ur_rtde_controller::msg::CartesianPoint::SharedPtr msg)
{
    auto pose = msg->cartesian_pose;
    Eigen::Quaterniond orientation(pose.orientation.w, pose.orientation.x,
                                   pose.orientation.y, pose.orientation.z);
    if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
        !std::isfinite(pose.position.z) || !orientation.coeffs().allFinite() ||
        !std::isfinite(orientation.norm()) || orientation.norm() < 1e-12)
    {
        RCLCPP_ERROR(get_logger(), "Cartesian goal rejected: invalid position or quaternion.");
        publishMotionResult(false);
        return;
    }
    if (!std::isfinite(msg->velocity) || msg->velocity <= TOOL_VELOCITY_MIN ||
        msg->velocity > TOOL_VELOCITY_MAX)
    {
        RCLCPP_ERROR(get_logger(), "Cartesian goal rejected: velocity must be in (0, %.3f] m/s.",
                     static_cast<double>(TOOL_VELOCITY_MAX));
        publishMotionResult(false);
        return;
    }
    orientation.normalize();
    pose.orientation.w = orientation.w();
    pose.orientation.x = orientation.x();
    pose.orientation.y = orientation.y();
    pose.orientation.z = orientation.z();
    const auto desired_pose = Pose2RTDE(pose);
    const bool asynchronous = asynchronous_;
    RCLCPP_INFO(get_logger(),
        "Cartesian goal received: xyz=[%.6f, %.6f, %.6f], rotation-vector=[%.6f, %.6f, %.6f], velocity=%.3f, asynchronous=%s",
        desired_pose[0], desired_pose[1], desired_pose[2], desired_pose[3], desired_pose[4],
        desired_pose[5], msg->velocity, asynchronous ? "true" : "false");
    try
    {
        if (!rtde_control_->isConnected() || !rtde_control_->isProgramRunning())
        {
            RCLCPP_ERROR(get_logger(), "Cartesian goal rejected: RTDE control is disconnected or its robot program is not running.");
            publishMotionResult(false);
            return;
        }
        if (!rtde_control_->isPoseWithinSafetyLimits(desired_pose))
        {
            RCLCPP_ERROR(get_logger(), "Cartesian goal rejected: pose outside robot safety limits.");
            publishMotionResult(false);
            return;
        }
        if (!beginPositionMotion(MotionMode::CartesianGoal))
        {
            publishMotionResult(false);
            return;
        }
        RCLCPP_INFO(get_logger(), "Cartesian motion owns control; velocity commands are disabled.");
        if (!rtde_control_->moveL(desired_pose, msg->velocity, 1.20, asynchronous))
        {
            failMotion("Cartesian moveL returned false.");
            return;
        }
        RCLCPP_INFO(get_logger(), "Cartesian moveL %s.", asynchronous ? "accepted asynchronously" : "completed");
        if (!asynchronous) publishTrajectoryExecuted();
        else new_async_cartesian_pose_received_ = true;
    }
    catch (const std::exception & error)
    {
        failMotion(std::string("Cartesian moveL failed: ") + error.what());
    }
}

void RTDEController::jointVelocityCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (!velocity_commands_enabled_ || motionBusy() || motion_mode_ == MotionMode::Freedrive) { return; }
    if (motion_mode_ == MotionMode::CartesianVelocity && !rtde_control_->speedStop(2.0))
    {
        failMotion("Could not stop the previous velocity mode.");
        return;
    }

    // Check Input Data Size
    if (msg->data.size() != 6) {RCLCPP_ERROR(get_logger(), "ERROR: Received Joint Velocity Size != 6\n"); return;}

    // Get Current and Desired Joint Velocity
    std::vector<double> desired_velocity = msg->data;
    std::vector<double> current_velocity = rtde_receive_->getActualQd();

    // // Compute Velocity Difference
    Eigen::VectorXd velocity_difference = Eigen::VectorXd::Map(desired_velocity.data(), desired_velocity.size()) 
                                        - Eigen::VectorXd::Map(current_velocity.data(), current_velocity.size());

    // Compute MAX Acceleration
    double acceleration = velocity_difference.array().abs().maxCoeff() / rate_;
    acceleration = sign(acceleration) * std::max(std::fabs(acceleration), 1.0);

    // Set Acceleration to a Fixed (Maximum) Value
    // double acceleration = 10.0; 

    // Check Acceleration Limits
    if (limit_acc_ && acceleration > JOINT_ACCELERATION_MAX) {RCLCPP_ERROR(get_logger(), "Requested Acceleration > Maximum Acceleration\n"); 
                                                              acceleration = JOINT_ACCELERATION_MAX;}

    // Joint Velocity Publisher
    try
    {
        if (!rtde_control_->speedJ(desired_velocity, acceleration, 0.0001))
        {
            failMotion("speedJ returned false.");
            return;
        }
        motion_mode_ = MotionMode::JointVelocity;
    }
    catch (const std::exception & error) { failMotion(error.what()); }
}

void RTDEController::cartesianVelocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    if (!velocity_commands_enabled_ || motionBusy() || motion_mode_ == MotionMode::Freedrive) { return; }
    if (motion_mode_ == MotionMode::JointVelocity && !rtde_control_->speedStop(2.0))
    {
        failMotion("Could not stop the previous velocity mode.");
        return;
    }

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
    try
    {
        if (!rtde_control_->speedL(desired_cartesian_velocity, acceleration, 0.002))
        {
            failMotion("speedL returned false.");
            return;
        }
        motion_mode_ = MotionMode::CartesianVelocity;
    }
    catch (const std::exception & error) { failMotion(error.what()); }
}

void RTDEController::digitalIOSetCallback(const std_msgs::msg::Int8::SharedPtr msg)
{
	// Function to set a boolean value in a digital port of the UR IO network
    //NOTE: As 0 id wouldn't be able to specify the value, ids are shifted by 1
	// output boolean = sign(msg)
	// output id	  = abs(msg) -1 
	uint8_t output_id = abs(msg->data) - 1;
	bool signal_level = false;
	if (msg->data > 0) {signal_level = true;}
	rtde_io_ -> setStandardDigitalOut(output_id,signal_level);
}

void RTDEController::toolDigitalIOSetCallback(const std_msgs::msg::Int8::SharedPtr msg)
{
    // Function to set a boolean value in a digital port of the UR IO network
    //NOTE: As 0 id wouldn't be able to specify the value, ids are shifted by 1
    // output boolean = sign(msg)
    // output id	  = abs(msg) - 1
    uint8_t output_id = abs(msg->data) - 1;
    bool signal_level = false;
    if (msg->data >= 0) {signal_level = true;}
    rtde_io_ -> setToolDigitalOut(output_id, signal_level);
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
    if (!beginPositionMotion(MotionMode::Freedrive)) { response->success = false; return true; }
    response->success = rtde_control_->freedriveMode(request->free_axes);
    if (!response->success) { failMotion("Could not start freedrive."); }
    return true;
}

bool RTDEController::stopFreedriveModeCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    // Exit from FreeDrive Mode
    auto req = request;
    response->success = rtde_control_->endFreedriveMode();
    if (response->success && motion_mode_ == MotionMode::Freedrive) { motion_mode_ = MotionMode::Idle; }
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
    float position = double(request->position) / 100.0;
    float speed    = double(request->speed) / 100.0;
    float force    = double(request->force) / 100.0;

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
    if (robotiq_gripper_server_ == nullptr) robotiq_gripper_server_ = create_service<ur_rtde_controller::srv::RobotiQGripperControl>("/ur_rtde/robotiq_gripper/command", std::bind(&RTDEController::RobotiQGripperCallback, this, std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS(), control_callback_group_);

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

bool RTDEController::currentPositionRobotiQGripperCallback(
    [[maybe_unused]] const std::shared_ptr<ur_rtde_controller::srv::GetGripperPosition::Request> request,
                           std::shared_ptr<ur_rtde_controller::srv::GetGripperPosition::Response> response)
{
	// Get Current RobotiQ Gripper Position
	response->current_position = robotiq_gripper_ -> getCurrentPosition();

	response->success = true;
	return true;
}

void RTDEController::publishJointState()
{
    // Create JointState Message
    joint_state_.header.stamp = now();

    // Read Joint Position and Velocity
    joint_state_.position = rtde_receive_ -> getActualQ();
    joint_state_.velocity = rtde_receive_ -> getActualQd();

    actual_joint_position_ = joint_state_.position;
    actual_joint_velocity_ = joint_state_.velocity;

    // Publish JointState
    joint_state_pub_ -> publish(joint_state_);
}

void RTDEController::publishTCPPose()
{
    // Read TCP Position
    actual_cartesian_pose_ = RTDE2Pose(rtde_receive_ -> getActualTCPPose());

    // Publish TCP Pose
    tcp_pose_pub_ -> publish(actual_cartesian_pose_);
}

void RTDEController::publishFTSensor()
{
    // Return if the FT Sensor is Disabled
    if (!ft_sensor_) return;

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
    motion_mode_ = MotionMode::Idle;
    publishMotionResult(true);

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
    const auto actual_joint_position = rtde_receive_->getActualQ();
    Eigen::VectorXd error = polynomial_fit_.getLastPoint() - Eigen::Map<const Eigen::VectorXd>(actual_joint_position.data(), actual_joint_position.size());
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
        if (!rtde_control_->speedStop(2.0)) { failMotion("Could not stop trajectory speed mode."); return; }

        // Publish Trajectory Executed
        publishTrajectoryExecuted();
        resetBooleans();
        return;
    }

    // Create the Desired Velocity Vector
    Eigen::VectorXd trajectory_vel = polynomial_fit_.evaluatePolynomialsDer(trajectory_time_);
    const auto actual_joint_position = rtde_receive_->getActualQ();
    trajectory_vel += (polynomial_fit_.evaluatePolynomials(trajectory_time_) - Eigen::Map<const Eigen::VectorXd>(actual_joint_position.data(), actual_joint_position.size()));
    std::vector<double> desired_velocity(trajectory_vel.data(), trajectory_vel.data() + trajectory_vel.size());

    // Create the Desired Acceleration Vector
    Eigen::VectorXd acc = polynomial_fit_.evaluatePolynomialsDDer(trajectory_time_);

    // Move Robot with Velocity Commands
    if (!rtde_control_->speedJ(desired_velocity, (acc.cwiseAbs()).maxCoeff(), 5e-4))
    {
        failMotion("Trajectory speedJ returned false.");
        return;
    }

    // Increase trajectory_time_
    trajectory_time_ += 0.002;
}

void RTDEController::checkAsyncMovements()
{
    // Return if No Async Movement Received
    if (!new_async_joint_pose_received_ and !new_async_cartesian_pose_received_) return;

    // Check if Async Operation is Ended -> Trajectory Executed
    if (!rtde_control_->isProgramRunning()) { failMotion("RTDE program stopped during asynchronous motion."); return; }
    if (!rtde_control_->getAsyncOperationProgressEx().isAsyncOperationRunning()) { publishTrajectoryExecuted(); }
}

void RTDEController::stopRobot()
{
    velocity_commands_enabled_ = false;
    if (motion_mode_ == MotionMode::JointVelocity || motion_mode_ == MotionMode::CartesianVelocity ||
        motion_mode_ == MotionMode::Trajectory) { rtde_control_->speedStop(2.0); }
    else if (motion_mode_ == MotionMode::Freedrive) { rtde_control_->endFreedriveMode(); }
    else if (motion_mode_ == MotionMode::CartesianGoal) { rtde_control_->stopL(2.0); }
    else { rtde_control_->stopJ(2.0); }
    motion_mode_ = MotionMode::Idle;

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
    while (rclcpp::ok() && rtde_receive_ -> isEmergencyStopped())  {RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 5000, "EMERGENCY STOP PRESSED"); eStop = true;}
    while (rclcpp::ok() && rtde_receive_ -> isProtectiveStopped()) {RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 5000, "PROTECTIVE STOP");         protectiveStop = true;}

    if (!rclcpp::ok()) { return; }
    // Check Flags
    if (eStop)
    {
        // Info Prints
        RCLCPP_WARN(get_logger(), "EMERGENCY STOP RELEASED\n");

        // Check if Robot Mode is ROBOT_MODE_RUNNING
        while (rclcpp::ok() && rtde_receive_ -> getRobotMode() != ROBOT_MODE_RUNNING) {RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 5000, "Wait for Robot Recovery...");}
    
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

        velocity_commands_enabled_ = false;
        motion_mode_ = MotionMode::Idle;
        // Reset Booleans
        resetBooleans();
    }

    // Check Robot Connection Status
    if (!rtde_control_ -> isConnected()) RCLCPP_ERROR(get_logger(), "ROBOT DISCONNECTED\n");
}

void RTDEController::checkRobot()
{
    try
    {
        // Check UR Status
        checkRobotStatus();

        if (!rclcpp::ok()) { return; }
        // Trajectory Controller
        moveTrajectory();

        // Check Async Movements Status
        checkAsyncMovements();
    }
    catch (const std::exception & error)
    {
        if (rclcpp::ok()) { failMotion(error.what()); }
    }
}

void RTDEController::spinner()
{
    // Main Spinner
    executor.add_node(this->get_node_base_interface());

    // Add timers
    joint_state_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    jointState_timer_ = create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000./rate_)), std::bind(&RTDEController::publishJointState, this), joint_state_callback_group_);

    tcp_pose_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    tcpPose_timer_ = create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000./rate_)), std::bind(&RTDEController::publishTCPPose, this), tcp_pose_callback_group_);

    force_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    force_timer_ = create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000./rate_)), std::bind(&RTDEController::publishFTSensor, this), force_callback_group_);

    robot_check_callback_group_ = control_callback_group_;
    checkRobot_timer_ = create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000./rate_)), std::bind(&RTDEController::checkRobot, this), robot_check_callback_group_);

    // Spin the Executor
    executor.spin();

    // Set Shutdown Trigger
    shutdown_ = true;
}

// Create Null-Pointers to the RTDE Class and the Threads
// RTDEController *rtde = nullptr;
// std::thread *publishJointState = nullptr;
// std::thread *publishTCPPose    = nullptr;
// std::thread *publishFTSensor   = nullptr;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try
    {
        auto rtde = std::make_shared<RTDEController>();
        rtde->spinner();
        rtde.reset();
    }
    catch (const std::exception & error)
    {
        if (rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("ur_rtde_controller"), "%s", error.what());
            rclcpp::shutdown();
            return 1;
        }
    }
    rclcpp::shutdown();
    return 0;
}
