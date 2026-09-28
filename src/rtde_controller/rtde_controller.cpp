#include "rtde_controller/rtde_controller.h"

#include <algorithm>
#include <stdexcept>

double durationToSec(rclcpp::Duration duration) {
    return duration.seconds();
}

RTDEController::RTDEController(): Node ("ur_rtde_controller") {

    // Declare Parameters
    declare_parameter<std::string>("ROBOT_IP", std::string("192.168.2.30"));
    declare_parameter<bool>("enable_gripper", false);
    declare_parameter<bool>("asynchronous", false);
    declare_parameter<bool>("limit_acc", true);
    declare_parameter<bool>("ft_sensor", true);
    declare_parameter<double>("rate", 500.0);
    declare_parameter<double>("trajectory_start_tolerance", 1e-3);
    declare_parameter<double>("trajectory_goal_tolerance", 1e-3);
    declare_parameter<std::vector<double>>("torque_limits", {50.0, 50.0, 25.0, 10.0, 10.0, 10.0});
    declare_parameter<int>("torque_watchdog_cycles", 5);
    declare_parameter<bool>("torque_friction_compensation", true);

    // Load Parameters
    bool asynchronous;
    if(!get_parameter_or("ROBOT_IP", ROBOT_IP, std::string("192.168.2.30"))) {RCLCPP_ERROR_STREAM(get_logger(), "Failed To Get \"ROBOT_IP\" Param. Using Default: " << ROBOT_IP);}
    if(!get_parameter_or("enable_gripper", enable_gripper_, false)) {RCLCPP_ERROR_STREAM(get_logger(), "Failed To Get \"gripper_enabled\" Param. Using Default: " << enable_gripper_);}
    if(!get_parameter_or("asynchronous", asynchronous, false)) {RCLCPP_ERROR_STREAM(get_logger(), "Failed To Get \"asynchronous\" Param. Using Default: " << asynchronous);}
    if(!get_parameter_or("limit_acc", limit_acc_, true)) {RCLCPP_ERROR_STREAM(get_logger(), "Failed To Get \"limit_acc\" Param. Using Default: " << limit_acc_);}
    if(!get_parameter_or("ft_sensor", ft_sensor_, true)) {RCLCPP_ERROR_STREAM(get_logger(), "Failed To Get \"ft_sensor\" Param. Using Default: " << ft_sensor_);}
    if(!get_parameter_or("rate", rate_, 500.0)) {RCLCPP_ERROR_STREAM(get_logger(), "Failed To Get \"rate\" Param. Using Default: " << rate_);}
    if(!get_parameter_or("trajectory_start_tolerance", trajectory_start_tolerance_, 1e-3)) {RCLCPP_ERROR_STREAM(get_logger(), "Failed To Get \"trajectory_start_tolerance\" Param. Using Default: " << trajectory_start_tolerance_);}
    if(!get_parameter_or("trajectory_goal_tolerance", trajectory_goal_tolerance_, 1e-3)) {RCLCPP_ERROR_STREAM(get_logger(), "Failed To Get \"trajectory_goal_tolerance\" Param. Using Default: " << trajectory_goal_tolerance_);}
    if(!get_parameter_or("torque_limits", torque_limits_, std::vector<double>{50.0, 50.0, 25.0, 10.0, 10.0, 10.0})) {RCLCPP_ERROR(get_logger(), "Failed To Get \"torque_limits\" Param. Using Default");}
    if(!get_parameter_or("torque_watchdog_cycles", torque_watchdog_cycles_, 5)) {RCLCPP_ERROR_STREAM(get_logger(), "Failed To Get \"torque_watchdog_cycles\" Param. Using Default: " << torque_watchdog_cycles_);}
    bool torque_friction_compensation;
    if(!get_parameter_or("torque_friction_compensation", torque_friction_compensation, true)) {RCLCPP_ERROR_STREAM(get_logger(), "Failed To Get \"torque_friction_compensation\" Param. Using Default: " << torque_friction_compensation);}
    asynchronous_ = asynchronous;
    torque_friction_compensation_ = torque_friction_compensation;

    // Check Torque Mode Params
    if (torque_limits_.size() != 6 || std::any_of(torque_limits_.begin(), torque_limits_.end(), [](double limit) {return !(limit > 0.0);}))
        throw std::invalid_argument("\"torque_limits\" Param Must Contain 6 Positive Values");
    if (torque_watchdog_cycles_ < 0) throw std::invalid_argument("\"torque_watchdog_cycles\" Param Must be >= 0");

    // Check Rate
    if (rate_ <= 0.0) {RCLCPP_ERROR_STREAM(get_logger(), "Invalid \"rate\" Param: " << rate_ << ". Using Default: 500 Hz"); rate_ = 500.0;}

    // Log Parameters
    std::ostringstream oss; oss << std::boolalpha;
    oss << "PARAMETERS LIST:\nROBOT_IP:           " << ROBOT_IP << "\n";
    oss << "Enable Gripper:     " << enable_gripper_ << "\n";
    oss << "Asynchronous:       " << asynchronous_ << "\n";
    oss << "Limit Acceleration: " << limit_acc_ << "\n";
    oss << "FT Sensor:          " << ft_sensor_ << "\n";
    oss << "Rate:               " << rate_ << " Hz\n";
    oss << "Torque Limits:      [" << torque_limits_[0] << ", " << torque_limits_[1] << ", " << torque_limits_[2] << ", " << torque_limits_[3] << ", " << torque_limits_[4] << ", " << torque_limits_[5] << "] Nm\n";
    oss << "Torque Watchdog:    " << torque_watchdog_cycles_ << " Cycles\n";
    oss << "Friction Comp.:     " << torque_friction_compensation_;
    RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());

    // Initialize Robot
    while (rclcpp::ok() && !robot_initialized) {

        // Initialize Dashboard
        if (!rtde_dashboard_initialized) {try {rtde_dashboard_ = std::make_unique<ur_rtde::DashboardClient>(ROBOT_IP); rtde_dashboard_initialized = true;}
        catch (const std::exception &e) {RCLCPP_ERROR_STREAM(get_logger(), "Failed to Initialize the Dashboard Client:\n" << e.what());}}

        // Check Remote Control Status
        if (rtde_dashboard_initialized && !rtde_dashboard_connected) {try {rtde_dashboard_ -> connect(); rtde_dashboard_connected = true;
        while (rclcpp::ok() && !rtde_dashboard_ -> isInRemoteControl()) {RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "ERROR: Robot Not in RemoteControl Mode\n"); rclcpp::sleep_for(std::chrono::milliseconds(100));}}
        catch (const std::exception &e) {RCLCPP_ERROR_STREAM(get_logger(), "Failed to Connect to the Dashboard Server:\n" << e.what());}}

        // RTDE Control Library
        if (rtde_dashboard_connected && !rtde_control_initialized) try {rtde_control_ = std::make_unique<ur_rtde::RTDEControlInterface>(ROBOT_IP); rtde_control_initialized = true;}
        catch (const std::exception &e) {RCLCPP_ERROR_STREAM(get_logger(), "Failed to Initialize the RTDE Control Interface:\n" << e.what());}

        // RTDE Receive Library
        if (rtde_dashboard_connected && !rtde_receive_initialized) try {rtde_receive_ = std::make_unique<ur_rtde::RTDEReceiveInterface>(ROBOT_IP); rtde_receive_initialized = true;}
        catch (const std::exception &e) {RCLCPP_ERROR_STREAM(get_logger(), "Failed to Initialize the RTDE Receive Interface:\n" << e.what());}

        // RTDE IO Library
        if (rtde_dashboard_connected && !rtde_io_initialized) try {rtde_io_ = std::make_unique<ur_rtde::RTDEIOInterface>(ROBOT_IP); rtde_io_initialized = true;}
        catch (const std::exception &e) {RCLCPP_ERROR_STREAM(get_logger(), "Failed to Initialize the RTDE IO Interface:\n" << e.what());}

        // Reupload RTDE Control Script if Needed
        if (rtde_dashboard_connected && rtde_control_initialized && !rtde_dashboard_ -> running()) try {rtde_control_ -> reuploadScript(); rtde_dashboard_ -> disconnect();}
        catch (const std::exception &e) {RCLCPP_ERROR_STREAM(get_logger(), "Failed to Reupload the RTDE Control Script:\n" << e.what());}

        // Robot Initialized
        if (rtde_dashboard_initialized && rtde_dashboard_connected && rtde_control_initialized && rtde_receive_initialized && rtde_io_initialized) robot_initialized = true;
        else rclcpp::sleep_for(std::chrono::seconds(1));

    }

    // Shutdown Requested Before the Robot Initialization
    if (!robot_initialized) throw std::runtime_error("Shutdown Requested Before the Robot Initialization");

    // Initialize Actual Joint State
    actual_joint_position_ = rtde_receive_ -> getActualQ();
    actual_joint_velocity_ = rtde_receive_ -> getActualQd();

    // RobotiQ Gripper
    if (enable_gripper_) {

        try {

            // Initialize Gripper
            robotiq_gripper_ = std::make_unique<ur_rtde::RobotiqGripper>(ROBOT_IP, 63352, false);
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
    joint_state_.name = {"shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"};
    joint_state_.position.resize(6, 0.0);
    joint_state_.velocity.resize(6, 0.0);

    joint_state_pub_         = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 1);
    tcp_pose_pub_            = create_publisher<geometry_msgs::msg::Pose>("/ur_rtde/cartesian_pose", 1);
    trajectory_executed_pub_ = create_publisher<std_msgs::msg::Bool>("/ur_rtde/trajectory_executed", 1);
    robot_dynamics_pub_      = create_publisher<ur_rtde_controller::msg::RobotDynamics>("/ur_rtde/dynamics", 1);
    torque_mode_active_pub_  = create_publisher<std_msgs::msg::Bool>("/ur_rtde/torque_mode/active", rclcpp::QoS(1).transient_local());
    publishTorqueModeActive(false);

    // ROS - Subscribers -> TODO: ADD CALLBACK GROUPS
    auto cb_group_sub1 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_options; sub_options.callback_group = cb_group_sub1;
    trajectory_command_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>("/ur_rtde/controllers/trajectory_controller/command",
                                1, std::bind(&RTDEController::jointTrajectoryCallback, this, std::placeholders::_1),sub_options);


    auto cb_group_sub2 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sub_options.callback_group = cb_group_sub2;    
    joint_goal_command_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>("/ur_rtde/controllers/joint_space_controller/command",
                                1, std::bind(&RTDEController::jointGoalCallback, this, std::placeholders::_1),sub_options);
    
    
    auto cb_group_sub3 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sub_options.callback_group = cb_group_sub3;
    cartesian_goal_command_sub_ = create_subscription<ur_rtde_controller::msg::CartesianPoint>("/ur_rtde/controllers/cartesian_space_controller/command",
                                    1, std::bind(&RTDEController::cartesianGoalCallback, this, std::placeholders::_1),sub_options);
    

    auto cb_group_sub4 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sub_options.callback_group = cb_group_sub4;
    joint_velocity_command_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>("/ur_rtde/controllers/joint_velocity_controller/command",
                                    1, std::bind(&RTDEController::jointVelocityCallback, this, std::placeholders::_1),sub_options);
    
    
    auto cb_group_sub5 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sub_options.callback_group = cb_group_sub5;
    cartesian_velocity_command_sub_ = create_subscription<geometry_msgs::msg::Twist>("/ur_rtde/controllers/cartesian_velocity_controller/command",
                                        1, std::bind(&RTDEController::cartesianVelocityCallback, this, std::placeholders::_1),sub_options);
	
    
    auto cb_group_sub6 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sub_options.callback_group = cb_group_sub6;
    digital_io_set_sub_	= create_subscription<std_msgs::msg::Int8>("/ur_rtde/digitalIO/command", 1,
                            std::bind(&RTDEController::digitalIOSetCallback, this, std::placeholders::_1),sub_options);
    
    
    auto cb_group_sub7 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sub_options.callback_group = cb_group_sub7;
    tool_digital_io_set_sub_ = create_subscription<std_msgs::msg::Int8>("/ur_rtde/tool_digitalIO/command", 1,
                                std::bind(&RTDEController::toolDigitalIOSetCallback, this, std::placeholders::_1),sub_options);

    auto cb_group_sub8 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sub_options.callback_group = cb_group_sub8;
    torque_command_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>("/ur_rtde/controllers/torque_controller/command", 1,
                                std::bind(&RTDEController::torqueCommandCallback, this, std::placeholders::_1),sub_options);

    // ROS - Service Servers
    stop_robot_server_          = create_service<std_srvs::srv::Trigger>("/ur_rtde/controllers/stop_robot", std::bind(&RTDEController::stopRobotCallback, this,std::placeholders::_1, std::placeholders::_2));
    set_async_parameter_server_ = create_service<std_srvs::srv::SetBool>("/ur_rtde/param/set_asynchronous", std::bind(&RTDEController::setAsyncParameterCallback, this,std::placeholders::_1, std::placeholders::_2));
    start_FreedriveMode_server_ = create_service<ur_rtde_controller::srv::StartFreedriveMode>("/ur_rtde/FreedriveMode/start", std::bind(&RTDEController::startFreedriveModeCallback, this,std::placeholders::_1, std::placeholders::_2));
    stop_FreedriveMode_server_  = create_service<std_srvs::srv::Trigger>("/ur_rtde/FreedriveMode/stop", std::bind(&RTDEController::stopFreedriveModeCallback, this,std::placeholders::_1, std::placeholders::_2));
    get_FK_server_              = create_service<ur_rtde_controller::srv::GetForwardKinematic>("/ur_rtde/getFK", std::bind(&RTDEController::getForwardKinematicCallback, this,std::placeholders::_1, std::placeholders::_2));
    get_IK_server_              = create_service<ur_rtde_controller::srv::GetInverseKinematic>("/ur_rtde/getIK", std::bind(&RTDEController::getInverseKinematicCallback, this,std::placeholders::_1, std::placeholders::_2));
    get_safety_status_server_   = create_service<ur_rtde_controller::srv::GetRobotStatus>("/ur_rtde/getSafetyStatus", std::bind(&RTDEController::getSafetyStatusCallback, this,std::placeholders::_1, std::placeholders::_2));
    start_torque_mode_server_   = create_service<std_srvs::srv::Trigger>("/ur_rtde/torque_mode/start", std::bind(&RTDEController::startTorqueModeCallback, this,std::placeholders::_1, std::placeholders::_2));
    stop_torque_mode_server_    = create_service<std_srvs::srv::Trigger>("/ur_rtde/torque_mode/stop", std::bind(&RTDEController::stopTorqueModeCallback, this,std::placeholders::_1, std::placeholders::_2));
    set_friction_compensation_server_ = create_service<std_srvs::srv::SetBool>("/ur_rtde/torque_mode/set_friction_compensation", std::bind(&RTDEController::setFrictionCompensationCallback, this,std::placeholders::_1, std::placeholders::_2));

    rclcpp::sleep_for(std::chrono::seconds(1));
    std::cout << std::endl;
    RCLCPP_WARN(get_logger(), "UR RTDE Controller - Connected\n");
}

RTDEController::~RTDEController()
{
    // Stop Robot
    stopRobot();

    // Disconnect RTDE Control Interface
    if (rtde_control_) rtde_control_ -> disconnect();
    std::cout << std::endl;
    RCLCPP_WARN(get_logger(), "UR RTDE Controller - Disconnected\n");
}

std::vector<double> RTDEController::getActualJointPosition()
{
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    return actual_joint_position_;
}

std::vector<double> RTDEController::getActualJointVelocity()
{
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    return actual_joint_velocity_;
}

void RTDEController::jointTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
    // Reject if Torque Mode is Active
    if (isTorqueModeActive("Joint Trajectory Command")) return;

    // Joint Order: Message joint_names (if Given) -> Controller Joint Order
    const size_t n_joints = joint_state_.name.size();
    std::vector<size_t> joint_index(n_joints);
    for (size_t j = 0; j < n_joints; j++) joint_index[j] = j;

    if (!msg->joint_names.empty())
    {
        if (msg->joint_names.size() != n_joints) {RCLCPP_ERROR(get_logger(), "ERROR: Trajectory joint_names Size != %zu\n", n_joints); return;}

        for (size_t j = 0; j < n_joints; j++)
        {
            auto it = std::find(msg->joint_names.begin(), msg->joint_names.end(), joint_state_.name[j]);
            if (it == msg->joint_names.end()) {RCLCPP_ERROR(get_logger(), "ERROR: Joint \"%s\" Missing in the Trajectory\n", joint_state_.name[j].c_str()); return;}
            joint_index[j] = std::distance(msg->joint_names.begin(), it);
        }
    }

    auto reorder = [&](const std::vector<double> &values) {
        if (values.size() != n_joints) return values;
        std::vector<double> ordered(n_joints);
        for (size_t j = 0; j < n_joints; j++) ordered[j] = values[joint_index[j]];
        return ordered;
    };

    // Convert Trajectory Message
    std::vector<trajectory::Waypoint> waypoints;
    for (const auto &point : msg->points)
        waypoints.push_back({durationToSec(point.time_from_start), reorder(point.positions), reorder(point.velocities), reorder(point.accelerations)});

    // Check Trajectory Consistency
    std::string error = trajectory::validate(waypoints, n_joints);
    if (!error.empty()) {RCLCPP_ERROR(get_logger(), "ERROR: Invalid Trajectory | %s\n", error.c_str()); return;}

    // Sample the Trajectory at the Controller Rate -> Resample if Not Already Sampled at 1/rate
    bool resampled = false;
    trajectory::SampledTrajectory sampled = trajectory::sample(waypoints, 1.0 / rate_, &resampled);
    if (resampled) RCLCPP_WARN(get_logger(), "Trajectory Not Sampled at %.1f ms (or Without Velocities) -> Resampled\n", 1000.0 / rate_);

    // Check the Trajectory Starts from the Actual State
    std::vector<double> actual_position = getActualJointPosition(), actual_velocity = getActualJointVelocity();
    Eigen::VectorXd q  = Eigen::Map<Eigen::VectorXd>(actual_position.data(), actual_position.size());
    Eigen::VectorXd dq = Eigen::Map<Eigen::VectorXd>(actual_velocity.data(), actual_velocity.size());

    double start_error = (sampled.position.front() - q).cwiseAbs().maxCoeff();
    if (start_error > trajectory_start_tolerance_)
        {RCLCPP_ERROR(get_logger(), "ERROR: Trajectory Not Starting from the Actual Configuration | Error: %.5f rad > %.5f rad\n", start_error, trajectory_start_tolerance_); return;}

    double start_velocity_error = (sampled.velocity.front() - dq).cwiseAbs().maxCoeff();
    if (start_velocity_error > TRAJECTORY_VELOCITY_TOLERANCE)
        {RCLCPP_ERROR(get_logger(), "ERROR: Trajectory Starting Velocity != Actual Velocity | Error: %.5f rad/s\n", start_velocity_error); return;}

    if (sampled.velocity.back().cwiseAbs().maxCoeff() > TRAJECTORY_VELOCITY_TOLERANCE)
        {RCLCPP_ERROR(get_logger(), "ERROR: Trajectory Final Velocity != 0\n"); return;}

    // Check Joint, Velocity and Acceleration Limits
    double max_position = 0.0, max_velocity = 0.0, max_acceleration = 0.0;
    for (size_t k = 0; k < sampled.size(); k++)
    {
        max_position     = std::max(max_position,     sampled.position[k].cwiseAbs().maxCoeff());
        max_velocity     = std::max(max_velocity,     sampled.velocity[k].cwiseAbs().maxCoeff());
        max_acceleration = std::max(max_acceleration, sampled.acceleration[k].cwiseAbs().maxCoeff());
    }

    if (max_position > JOINT_LIMITS || max_velocity > JOINT_VELOCITY_MAX || max_acceleration > JOINT_ACCELERATION_MAX)
        {RCLCPP_ERROR(get_logger(), "ERROR: Joint Limit Not Satisfied | Max Position: %.3f, Max Velocity: %.3f, Max Acceleration: %.3f\n", max_position, max_velocity, max_acceleration); return;}

    std::vector<double> final_position(sampled.position.back().data(), sampled.position.back().data() + n_joints);
    if (!rtde_control_ -> isJointsWithinSafetyLimits(final_position)) {RCLCPP_ERROR(get_logger(), "ERROR: Trajectory Final Position Outside Safety Limits\n"); return;}

    // New Trajectory -> Replaces the One in Execution
    const double duration = (sampled.size() - 1) / rate_;
    {
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        trajectory_ = std::move(sampled);
        trajectory_index_ = 0;
        trajectory_settling_cycles_ = 0;
        trajectory_active_ = true;
    }

    RCLCPP_INFO(get_logger(), "New Trajectory Received | Duration: %.3f s\n", duration);
}

void RTDEController::jointGoalCallback(const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg)
{
    // Reject if Torque Mode is Active
    if (isTorqueModeActive("Joint Goal Command")) return;

    // Check Input Data Size
    if (msg->positions.size() != 6) {RCLCPP_ERROR(get_logger(), "ERROR: Received Joint Position Goal Size != 6\n"); return;}
    if (durationToSec(msg->time_from_start) == 0 && msg->velocities.size() == 0) {RCLCPP_ERROR(get_logger(), "ERROR: Desired Time = 0\n"); return;}
    else if (durationToSec(msg->time_from_start) == 0 && msg->velocities[0] <= 0.0) {RCLCPP_ERROR(get_logger(), "ERROR: Desired Time = 0 | Desired Velocity <= 0\n"); return;}

    // Get Desired and Actual Joint Pose
    Eigen::VectorXd desired_pose = Eigen::VectorXd::Map(msg->positions.data(), msg->positions.size());
    std::vector<double> actual_joint_position = getActualJointPosition();
    Eigen::VectorXd actual_pose  = Eigen::VectorXd::Map(actual_joint_position.data(), actual_joint_position.size());

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
    // Reject if Torque Mode is Active
    if (isTorqueModeActive("Cartesian Goal Command")) return;

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
    // Reject if Torque Mode is Active
    if (isTorqueModeActive("Joint Velocity Command")) return;

    // Check Input Data Size
    if (msg->data.size() != 6) {RCLCPP_ERROR(get_logger(), "ERROR: Received Joint Velocity Size != 6\n"); return;}

    // Get Current and Desired Joint Velocity
    std::vector<double> desired_velocity = msg->data;
    std::vector<double> current_velocity = getActualJointVelocity();

    // Compute Velocity Difference
    Eigen::VectorXd velocity_difference = Eigen::VectorXd::Map(desired_velocity.data(), desired_velocity.size()) 
                                        - Eigen::VectorXd::Map(current_velocity.data(), current_velocity.size());

    // Compute MAX Acceleration -> Reach the Desired Velocity in One Control Cycle (dv / dt)
    double acceleration = std::max(velocity_difference.array().abs().maxCoeff() * rate_, 1.0);

    // Set Acceleration to a Fixed (Maximum) Value
    // double acceleration = 10.0; 

    // Check Acceleration Limits
    if (limit_acc_ && acceleration > JOINT_ACCELERATION_MAX) {RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Requested Acceleration > Maximum Acceleration -> Clamped to %.1f rad/s^2\n", JOINT_ACCELERATION_MAX);
                                                              acceleration = JOINT_ACCELERATION_MAX;}

    // Joint Velocity Publisher
    rtde_control_ -> speedJ(desired_velocity, acceleration,0.0001); // Use a Small Sleep Time to Avoid RTDE Control Timeout
}

void RTDEController::cartesianVelocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // Reject if Torque Mode is Active
    if (isTorqueModeActive("Cartesian Velocity Command")) return;

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
    //NOTE: As 0 id wouldn't be able to specify the value, ids are shifted by 1
	// output boolean = sign(msg)
	// output id	  = abs(msg) -1 
	if (msg->data == 0) {RCLCPP_ERROR(get_logger(), "ERROR: Digital Output Command = 0 -> Use +/-(id + 1)\n"); return;}
	uint8_t output_id = abs(msg->data) - 1;
	bool signal_level = msg->data > 0;
	rtde_io_ -> setStandardDigitalOut(output_id,signal_level);
}

void RTDEController::toolDigitalIOSetCallback(const std_msgs::msg::Int8::SharedPtr msg)
{
    // Function to set a boolean value in a digital port of the UR IO network
    //NOTE: As 0 id wouldn't be able to specify the value, ids are shifted by 1
    // output boolean = sign(msg)
    // output id	  = abs(msg) - 1
    if (msg->data == 0) {RCLCPP_ERROR(get_logger(), "ERROR: Tool Digital Output Command = 0 -> Use +/-(id + 1)\n"); return;}
    uint8_t output_id = abs(msg->data) - 1;
    bool signal_level = msg->data > 0;
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
    // Reject if Torque Mode is Active -> RTDE Control Interface Busy
    if (isTorqueModeActive("Start Freedrive Mode")) {response->success = false; return true;}

    // freeAxes = [1,0,0,0,0,0]     -> The robot is compliant in the x direction relative to the feature.
    // freeAxes: A 6 dimensional vector that contains 0’s and 1’s, these indicates in which axes movement is allowed. The first three values represents the cartesian directions along x, y, z, and the last three defines the rotation axis, rx, ry, rz. All relative to the selected feature

    // Start FreeDrive Mode
    response->success = rtde_control_ -> freedriveMode(request->free_axes);
    return true;
}

bool RTDEController::stopFreedriveModeCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    // Reject if Torque Mode is Active -> RTDE Control Interface Busy
    if (isTorqueModeActive("Stop Freedrive Mode")) {response->success = false; return true;}

    // Exit from FreeDrive Mode
    auto req = request;
    response->success = rtde_control_ -> endFreedriveMode();
    return true;
}

bool RTDEController::zeroFTSensorCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    // Reject if Torque Mode is Active -> RTDE Control Interface Busy
    if (isTorqueModeActive("Zero FT Sensor")) {response->success = false; return true;}

    // Reset Force-Torque Sensor
    auto req = request;
    response->success = rtde_control_ -> zeroFtSensor();
    return true;
}

bool RTDEController::getForwardKinematicCallback(const std::shared_ptr<ur_rtde_controller::srv::GetForwardKinematic::Request> request, std::shared_ptr<ur_rtde_controller::srv::GetForwardKinematic::Response> response)
{
    // Reject if Torque Mode is Active -> RTDE Control Interface Busy
    if (isTorqueModeActive("Forward Kinematic")) {response->success = false; return true;}

    // Compute Forward Kinematic
    std::vector<double> tcp_pose = rtde_control_ -> getForwardKinematics(request->joint_position, {0.0,0.0,0.0,0.0,0.0,0.0});

    // Convert RTDE Pose to Geometry Pose
    response->tcp_position = RTDE2Pose(tcp_pose);

    response->success = true;
    return true;
}

bool RTDEController::getInverseKinematicCallback(const std::shared_ptr<ur_rtde_controller::srv::GetInverseKinematic::Request> request, std::shared_ptr<ur_rtde_controller::srv::GetInverseKinematic::Response> response)
{
    // Reject if Torque Mode is Active -> RTDE Control Interface Busy
    if (isTorqueModeActive("Inverse Kinematic")) {response->success = false; return true;}

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
    std::vector<std::string> safety_mode_msg = {"NORMAL", "REDUCED", "PROTECTIVE_STOP", "RECOVERY", "SAFEGUARD_STOP", "SYSTEM_EMERGENCY_STOP", "ROBOT_EMERGENCY_STOP", "VIOLATION", "FAULT"};
    std::vector<std::string> safety_status_bits_msg = {"Is normal mode", "Is reduced mode", "Is protective stopped", "Is recovery mode", "Is safeguard stopped", "Is system emergency stopped", "Is robot emergency stopped", "Is emergency stopped", "Is violation", "Is fault", "Is stopped due to safety"};

    // Get Robot Mode
    auto req = request;
    response->robot_mode = rtde_receive_ -> getRobotMode();
    size_t robot_mode_index = response->robot_mode + 1;
    response->robot_mode_msg = (robot_mode_index < robot_mode_msg.size()) ? robot_mode_msg[robot_mode_index] : "UNKNOWN";

    // Get Safety Mode
    response->safety_mode = rtde_receive_ -> getSafetyMode();
    size_t safety_mode_index = response->safety_mode;
    response->safety_mode_msg = (safety_mode_index < safety_mode_msg.size()) ? safety_mode_msg[safety_mode_index] : "UNKNOWN";

    // Get Safety Status Bits -> Bitmask, List All the Active Bits
    response->safety_status_bits = rtde_receive_ -> getSafetyStatusBits();
    for (size_t bit = 0; bit < safety_status_bits_msg.size(); bit++)
    {
        if (!(response->safety_status_bits & (1u << bit))) continue;
        if (!response->safety_status_bits_msg.empty()) response->safety_status_bits_msg += ", ";
        response->safety_status_bits_msg += safety_status_bits_msg[bit];
    }

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
    if (!robotiq_gripper_ || !robotiq_gripper_ -> isConnected()) {RCLCPP_ERROR(get_logger(), "ERROR: RobotiQ Gripper Not Connected\n"); response->success = false; return true;}

    try {response->status = robotiq_gripper_ -> move(position, speed, force, ur_rtde::RobotiqGripper::WAIT_FINISHED);}
    catch (const std::exception &e) {RCLCPP_ERROR(get_logger(), "ERROR: RobotiQ Gripper Move Failed: %s\n", e.what()); response->success = false; return true;}

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

    try {

        // Create the Gripper Class if Doesn't Exist
        if (!robotiq_gripper_) robotiq_gripper_ = std::make_unique<ur_rtde::RobotiqGripper>(ROBOT_IP, 63352, false);

        // Connect the Gripper if Not Connected
        if (!robotiq_gripper_ -> isConnected()) robotiq_gripper_ -> connect();

        // Activate the Gripper
        robotiq_gripper_ -> activate();

    } catch (const std::exception &e) {RCLCPP_ERROR(get_logger(), "ERROR: Failed to Enable the RobotiQ Gripper: %s\n", e.what()); response->success = false; return true;}

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

bool RTDEController::currentPositionRobotiQGripperCallback(
    [[maybe_unused]] const std::shared_ptr<ur_rtde_controller::srv::GetGripperPosition::Request> request,
                           std::shared_ptr<ur_rtde_controller::srv::GetGripperPosition::Response> response)
{
	// Get Current RobotiQ Gripper Position
	if (!robotiq_gripper_ || !robotiq_gripper_ -> isConnected()) {RCLCPP_ERROR(get_logger(), "ERROR: RobotiQ Gripper Not Connected\n"); response->success = false; return true;}
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

    {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        actual_joint_position_ = joint_state_.position;
        actual_joint_velocity_ = joint_state_.velocity;
    }

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
    new_async_joint_pose_received_ = false;
    new_async_cartesian_pose_received_ = false;

    // Abort the Trajectory in Execution
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    trajectory_active_ = false;
}

void RTDEController::publishTrajectoryExecuted(bool success)
{
    // Publish Trajectory Executed Message
    std_msgs::msg::Bool trajectory_executed;
    trajectory_executed.data = success;
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

void RTDEController::moveTrajectory()
{
    std::unique_lock<std::mutex> lock(trajectory_mutex_);

    // Return if No Trajectory in Execution
    if (!trajectory_active_) return;

    std::vector<double> actual_joint_position = getActualJointPosition();
    Eigen::VectorXd q = Eigen::Map<Eigen::VectorXd>(actual_joint_position.data(), actual_joint_position.size());

    Eigen::VectorXd desired_velocity;
    double acceleration = TRAJECTORY_MIN_ACCELERATION;

    if (trajectory_index_ < trajectory_.size())
    {
        // Tracking -> Feedforward Velocity + Position Correction
        const size_t k = trajectory_index_++;
        desired_velocity = trajectory_.velocity[k] + TRAJECTORY_POSITION_GAIN * (trajectory_.position[k] - q);
        acceleration = std::max(trajectory_.acceleration[k].cwiseAbs().maxCoeff(), TRAJECTORY_MIN_ACCELERATION);
    }
    else
    {
        // Settling -> Position Correction on the Final Point until Goal Tolerance or Timeout
        Eigen::VectorXd error = trajectory_.position.back() - q;
        bool goal_reached = error.cwiseAbs().maxCoeff() < trajectory_goal_tolerance_;

        if (goal_reached || ++trajectory_settling_cycles_ > TRAJECTORY_SETTLING_TIME * rate_)
        {
            // Stop Speed Mode
            rtde_control_ -> speedStop();
            trajectory_active_ = false;
            lock.unlock();

            if (!goal_reached) RCLCPP_ERROR(get_logger(), "ERROR: Trajectory Goal Not Reached | Error: %.5f rad\n", error.cwiseAbs().maxCoeff());

            // Publish Trajectory Executed
            publishTrajectoryExecuted(goal_reached);
            return;
        }

        desired_velocity = TRAJECTORY_POSITION_GAIN * error;
    }

    // Move Robot with Velocity Commands
    std::vector<double> velocity_command(desired_velocity.data(), desired_velocity.data() + desired_velocity.size());
    rtde_control_ -> speedJ(velocity_command, acceleration, 5e-4);
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
    // Robot Not Initialized
    if (!rtde_control_ || !rtde_dashboard_) return;

    // Exit Torque Mode
    stopTorqueMode();

    // Reset Booleans Variables -> Abort the Trajectory in Execution Before Stopping
    resetBooleans();

    // Stop Robot
    rtde_control_ -> stopJ(2.0);

    // Wait
    rclcpp::sleep_for(std::chrono::milliseconds(100));

    // Clear Dashboard Warning Pop-Up
    rtde_dashboard_ -> connect();
    rtde_dashboard_ -> closePopup();
    rtde_dashboard_ -> disconnect();
}

bool RTDEController::isTorqueModeActive(const std::string &command)
{
    if (!torque_mode_active_) return false;
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "ERROR: %s Rejected -> Torque Mode Active\n", command.c_str());
    return true;
}

void RTDEController::torqueCommandCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (!torque_mode_active_) {RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Torque Command Ignored -> Torque Mode Not Active\n"); return;}

    // Check Input Data
    if (msg->data.size() != 6) {RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "ERROR: Received Joint Torque Size != 6\n"); return;}
    if (!std::all_of(msg->data.begin(), msg->data.end(), [](double tau) {return std::isfinite(tau);})) {RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "ERROR: Received Non-Finite Joint Torque\n"); return;}

    // Store the Latest Command -> Consumed by the Torque Control Loop
    std::lock_guard<std::mutex> lock(torque_command_mutex_);
    torque_command_ = msg->data;
    torque_command_new_ = true;
}

bool RTDEController::startTorqueModeCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    auto req = request;

    std::lock_guard<std::mutex> thread_lock(torque_thread_mutex_);
    if (torque_mode_active_) {response->success = false; response->message = "Torque Mode Already Active"; return true;}

    // Join the Previous Torque Thread (Exited by Itself on Watchdog / Stop)
    if (torque_thread_.joinable()) torque_thread_.join();

    // Abort Trajectories and Async Movements
    resetBooleans();

    // Discard Old Commands
    {
        std::lock_guard<std::mutex> lock(torque_command_mutex_);
        torque_command_new_ = false;
    }

    // Start the Torque Control Loop
    torque_mode_active_ = true;
    torque_thread_ = std::thread(&RTDEController::torqueControlLoop, this);
    publishTorqueModeActive(true);

    RCLCPP_WARN(get_logger(), "Torque Mode Started -> Waiting for Commands on /ur_rtde/controllers/torque_controller/command\n");
    response->success = true;
    return true;
}

bool RTDEController::stopTorqueModeCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    auto req = request;
    stopTorqueMode();
    response->success = true;
    return true;
}

bool RTDEController::setFrictionCompensationCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    // Enable / Disable the UR Internal Friction Compensation -> Applied from the Next Torque Command
    torque_friction_compensation_ = request->data;
    RCLCPP_WARN(get_logger(), "Torque Mode Friction Compensation: %s\n", request->data ? "Enabled" : "Disabled");

    response->success = true;
    return true;
}

void RTDEController::stopTorqueMode()
{
    // Request the Loop Exit and Wait for It
    std::lock_guard<std::mutex> thread_lock(torque_thread_mutex_);
    torque_mode_active_ = false;
    if (torque_thread_.joinable() && torque_thread_.get_id() != std::this_thread::get_id()) torque_thread_.join();
}

void RTDEController::publishTorqueModeActive(bool active)
{
    std_msgs::msg::Bool msg;
    msg.data = active;
    torque_mode_active_pub_ -> publish(msg);
}

void RTDEController::publishRobotDynamics()
{
    ur_rtde_controller::msg::RobotDynamics msg;
    msg.header.stamp = now();

    // Joint State from the RTDE Receive Interface
    std::vector<double> q   = rtde_receive_ -> getActualQ();
    std::vector<double> dq  = rtde_receive_ -> getActualQd();
    std::vector<double> tau = rtde_receive_ -> getActualCurrentAsTorque();

    // Dynamics from the Robot Controller, Computed on the Same Joint State
    std::vector<double> M    = rtde_control_ -> getMassMatrix(q);
    std::vector<double> C    = rtde_control_ -> getCoriolisAndCentrifugalTorques(q, dq);
    std::vector<double> J    = rtde_control_ -> getJacobian(q);
    std::vector<double> Jdot = rtde_control_ -> getJacobianTimeDerivative(q, dq);

    if (q.size() != 6 || dq.size() != 6 || tau.size() != 6 || M.size() != 36 || C.size() != 6 || J.size() != 36 || Jdot.size() != 36)
        throw std::runtime_error("Unexpected Robot Dynamics Size (PolyScope >= 5.23 Required)");

    std::copy(q.begin(), q.end(), msg.position.begin());
    std::copy(dq.begin(), dq.end(), msg.velocity.begin());
    std::copy(tau.begin(), tau.end(), msg.effort.begin());
    std::copy(C.begin(), C.end(), msg.coriolis.begin());

    // UR Matrices are Column-Major -> Message Matrices are Row-Major
    for (size_t row = 0; row < 6; row++)
    {
        for (size_t col = 0; col < 6; col++)
        {
            msg.mass_matrix[row * 6 + col]  = M[col * 6 + row];
            msg.jacobian[row * 6 + col]     = J[col * 6 + row];
            msg.jacobian_dot[row * 6 + col] = Jdot[col * 6 + row];
        }
    }

    // TCP Pose and Wrench
    msg.tcp_pose = RTDE2Pose(rtde_receive_ -> getActualTCPPose());
    std::vector<double> wrench = rtde_receive_ -> getActualTCPForce();
    msg.tcp_wrench.force.x  = wrench[0]; msg.tcp_wrench.force.y  = wrench[1]; msg.tcp_wrench.force.z  = wrench[2];
    msg.tcp_wrench.torque.x = wrench[3]; msg.tcp_wrench.torque.y = wrench[4]; msg.tcp_wrench.torque.z = wrench[5];

    robot_dynamics_pub_ -> publish(msg);
}

void RTDEController::torqueControlLoop()
{
    const auto start_time = std::chrono::steady_clock::now();
    std::vector<double> torque(6, 0.0);
    bool command_received = false;
    int missed_cycles = 0;
    std::string exit_reason;

    try {

        while (torque_mode_active_ && rclcpp::ok())
        {
            // Start of the Robot Control Cycle
            auto t_cycle_start = rtde_control_ -> initPeriod();

            // Exit on Emergency / Protective Stop
            if (rtde_receive_ -> isEmergencyStopped() || rtde_receive_ -> isProtectiveStopped()) {exit_reason = "Emergency / Protective Stop"; break;}

            // Publish the Robot State and Dynamics for the External Controller
            publishRobotDynamics();

            // Get the Latest Command
            bool new_command = false;
            {
                std::lock_guard<std::mutex> lock(torque_command_mutex_);
                if (torque_command_new_)
                {
                    new_command = true;
                    torque_command_new_ = false;
                    torque = torque_command_;
                }
            }

            // Watchdog -> Hold the Last Command for torque_watchdog_cycles, then Exit
            if (new_command) {command_received = true; missed_cycles = 0;}
            else if (command_received && ++missed_cycles > torque_watchdog_cycles_) {exit_reason = "Watchdog -> No Torque Command for " + std::to_string(missed_cycles) + " Cycles"; break;}

            // Wait for the First Command -> Robot Stays in Position Control
            if (!command_received)
            {
                if (std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count() > TORQUE_MODE_START_TIMEOUT) {exit_reason = "No Torque Command Received after Start"; break;}
                rtde_control_ -> waitPeriod(t_cycle_start);
                continue;
            }

            // Saturate and Send the Torque Command
            std::vector<double> torque_command(6);
            for (size_t j = 0; j < 6; j++)
            {
                torque_command[j] = std::clamp(torque[j], -torque_limits_[j], torque_limits_[j]);
                if (torque_command[j] != torque[j]) RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Joint %zu Torque Saturated: %.2f -> %.2f Nm\n", j, torque[j], torque_command[j]);
            }

            if (!rtde_control_ -> directTorque(torque_command, torque_friction_compensation_)) {exit_reason = "directTorque Failed"; break;}

            // Wait the End of the Robot Control Cycle
            rtde_control_ -> waitPeriod(t_cycle_start);
        }

    } catch (const std::exception &e) {exit_reason = std::string("Exception: ") + e.what();}

    torque_mode_active_ = false;

    // Stop the Robot -> Back to Position Control
    try {rtde_control_ -> stopJ(2.0);}
    catch (const std::exception &e) {RCLCPP_ERROR(get_logger(), "Failed to Stop the Robot after Torque Mode: %s\n", e.what());}

    publishTorqueModeActive(false);
    if (exit_reason.empty()) RCLCPP_WARN(get_logger(), "Torque Mode Stopped\n");
    else RCLCPP_ERROR(get_logger(), "Torque Mode Stopped | %s\n", exit_reason.c_str());
}

void RTDEController::checkRobotStatus()
{
    // Init Flags
    bool eStop = false, protectiveStop = false;

    // Print Robot Emergency and Protective Stop
    while (rclcpp::ok() && rtde_receive_ -> isEmergencyStopped())  {RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 5000, "EMERGENCY STOP PRESSED"); eStop = true;          rclcpp::sleep_for(std::chrono::milliseconds(10));}
    while (rclcpp::ok() && rtde_receive_ -> isProtectiveStopped()) {RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 5000, "PROTECTIVE STOP");         protectiveStop = true; rclcpp::sleep_for(std::chrono::milliseconds(10));}

    // Check Flags
    if (eStop)
    {
        // Info Prints
        RCLCPP_WARN(get_logger(), "EMERGENCY STOP RELEASED\n");

        // Check if Robot Mode is ROBOT_MODE_RUNNING
        while (rclcpp::ok() && rtde_receive_ -> getRobotMode() != ROBOT_MODE_RUNNING) {RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 5000, "Wait for Robot Recovery..."); rclcpp::sleep_for(std::chrono::milliseconds(10));}
    
    } else if (protectiveStop) {RCLCPP_WARN(get_logger(), "PROTECTIVE STOP RECOVERED\n");}
        
    if (eStop || protectiveStop)
    {
        // Exit Torque Mode (the Loop Already Stops on Emergency/Protective Stop)
        stopTorqueMode();

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

void RTDEController::checkRobot()
{
    // Check UR Status
    checkRobotStatus();

    // Trajectory Controller
    moveTrajectory();

    // Check Async Movements Status
    checkAsyncMovements();
}

void RTDEController::spinner()
{
    // Main Spinner
    executor.add_node(this->get_node_base_interface());

    // Add timers
    const auto period = std::chrono::duration<double>(1.0 / rate_);
    auto cb_group_timer1 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    jointState_timer_ = create_wall_timer(period, std::bind(&RTDEController::publishJointState, this), cb_group_timer1);

    auto cb_group_timer2 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    tcpPose_timer_ = create_wall_timer(period, std::bind(&RTDEController::publishTCPPose, this), cb_group_timer2);

    auto cb_group_timer3 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    force_timer_ = create_wall_timer(period, std::bind(&RTDEController::publishFTSensor, this), cb_group_timer3);

    auto cb_group_timer4 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    checkRobot_timer_ = create_wall_timer(period, std::bind(&RTDEController::checkRobot, this), cb_group_timer4);

    // Spin the Executor
    executor.spin();
}

int main(int argc, char **argv) {

    // Initialize ROS -> Default SIGINT Handler Stops the Executor
    rclcpp::init(argc, argv);
    std::cout << std::endl;

    try {

        // Create a New RTDEController
        auto rtde = std::make_shared<RTDEController>();

        // Main Spinner
        rtde->spinner();

        // Stop the Robot and Disconnect (Destructor) Before Shutting Down ROS
        rtde.reset();

    } catch (const std::exception &e) {std::cerr << "UR RTDE Controller: " << e.what() << std::endl;}

    // Shutdown ROS
    rclcpp::shutdown();

    return 0;

}
