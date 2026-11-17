#include "rtde_controller/rtde_controller.h"

#include <algorithm>
#include <sstream>
#include <stdexcept>

RTDEController::RTDEController(): nh_(), private_nh_("~") {

    // Load Parameters
    bool asynchronous;
    if(!private_nh_.param<std::string>("ROBOT_IP", ROBOT_IP, "192.168.2.30")) {ROS_WARN_STREAM("Param \"ROBOT_IP\" Not Set. Using Default: " << ROBOT_IP);}
    if(!private_nh_.param<bool>("enable_gripper", enable_gripper_, false)) {ROS_WARN_STREAM("Param \"enable_gripper\" Not Set. Using Default: " << enable_gripper_);}
    if(!private_nh_.param<bool>("asynchronous", asynchronous, false)) {ROS_WARN_STREAM("Param \"asynchronous\" Not Set. Using Default: " << asynchronous);}
    if(!private_nh_.param<bool>("limit_acc", limit_acc_, true)) {ROS_WARN_STREAM("Param \"limit_acc\" Not Set. Using Default: " << limit_acc_);}
    if(!private_nh_.param<bool>("ft_sensor", ft_sensor_, true)) {ROS_WARN_STREAM("Param \"ft_sensor\" Not Set. Using Default: " << ft_sensor_);}
    if(!private_nh_.param<double>("rate", rate_, 500.0)) {ROS_WARN_STREAM("Param \"rate\" Not Set. Using Default: " << rate_);}
    if(!private_nh_.param<double>("trajectory_start_tolerance", trajectory_start_tolerance_, 1e-3)) {ROS_WARN_STREAM("Param \"trajectory_start_tolerance\" Not Set. Using Default: " << trajectory_start_tolerance_);}
    if(!private_nh_.param<double>("trajectory_goal_tolerance", trajectory_goal_tolerance_, 1e-3)) {ROS_WARN_STREAM("Param \"trajectory_goal_tolerance\" Not Set. Using Default: " << trajectory_goal_tolerance_);}
    asynchronous_ = asynchronous;

    // Check Rate
    if (rate_ <= 0.0) {ROS_ERROR_STREAM("Invalid \"rate\" Param: " << rate_ << ". Using Default: 500 Hz"); rate_ = 500.0;}

    // Log Parameters
    std::ostringstream oss; oss << std::boolalpha;
    oss << "PARAMETERS LIST:\nROBOT_IP:           " << ROBOT_IP << "\n";
    oss << "Enable Gripper:     " << enable_gripper_ << "\n";
    oss << "Asynchronous:       " << asynchronous_ << "\n";
    oss << "Limit Acceleration: " << limit_acc_ << "\n";
    oss << "FT Sensor:          " << ft_sensor_ << "\n";
    oss << "Rate:               " << rate_ << " Hz";
    ROS_INFO("%s", oss.str().c_str());

    // Initialize Robot
    while (ros::ok() && !robot_initialized) {

        // Initialize Dashboard
        if (!rtde_dashboard_initialized) {try {rtde_dashboard_ = std::make_unique<ur_rtde::DashboardClient>(ROBOT_IP); rtde_dashboard_initialized = true;}
        catch (const std::exception &e) {ROS_ERROR_STREAM("Failed to Initialize the Dashboard Client:\n" << e.what());}}

        // Check Remote Control Status
        if (rtde_dashboard_initialized && !rtde_dashboard_connected) {try {rtde_dashboard_ -> connect(); rtde_dashboard_connected = true;
        while (ros::ok() && !rtde_dashboard_ -> isInRemoteControl()) {ROS_ERROR_THROTTLE(5.0, "ERROR: Robot Not in RemoteControl Mode\n"); std::this_thread::sleep_for(std::chrono::milliseconds(100));}}
        catch (const std::exception &e) {ROS_ERROR_STREAM("Failed to Connect to the Dashboard Server:\n" << e.what());}}

        // RTDE Control Library
        if (rtde_dashboard_connected && !rtde_control_initialized) try {rtde_control_ = std::make_unique<ur_rtde::RTDEControlInterface>(ROBOT_IP); rtde_control_initialized = true;}
        catch (const std::exception &e) {ROS_ERROR_STREAM("Failed to Initialize the RTDE Control Interface:\n" << e.what());}

        // RTDE Receive Library
        if (rtde_dashboard_connected && !rtde_receive_initialized) try {rtde_receive_ = std::make_unique<ur_rtde::RTDEReceiveInterface>(ROBOT_IP); rtde_receive_initialized = true;}
        catch (const std::exception &e) {ROS_ERROR_STREAM("Failed to Initialize the RTDE Receive Interface:\n" << e.what());}

        // RTDE IO Library
        if (rtde_dashboard_connected && !rtde_io_initialized) try {rtde_io_ = std::make_unique<ur_rtde::RTDEIOInterface>(ROBOT_IP); rtde_io_initialized = true;}
        catch (const std::exception &e) {ROS_ERROR_STREAM("Failed to Initialize the RTDE IO Interface:\n" << e.what());}

        // Reupload RTDE Control Script if Needed
        if (rtde_dashboard_connected && rtde_control_initialized && !rtde_dashboard_ -> running()) try {rtde_control_ -> reuploadScript(); rtde_dashboard_ -> disconnect();}
        catch (const std::exception &e) {ROS_ERROR_STREAM("Failed to Reupload the RTDE Control Script:\n" << e.what());}

        // Robot Initialized
        if (rtde_dashboard_initialized && rtde_dashboard_connected && rtde_control_initialized && rtde_receive_initialized && rtde_io_initialized) robot_initialized = true;
        else std::this_thread::sleep_for(std::chrono::seconds(1));

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
            robotiq_gripper_server_ = nh_.advertiseService("/ur_rtde/robotiq_gripper/command", &RTDEController::RobotiQGripperCallback, this);

            // Gripper Enable/Disable Service Servers
            enable_gripper_server_  = nh_.advertiseService("/ur_rtde/robotiq_gripper/enable", &RTDEController::enableRobotiQGripperCallback, this);
            disable_gripper_server_ = nh_.advertiseService("/ur_rtde/robotiq_gripper/disable", &RTDEController::disableRobotiQGripperCallback, this);

            // Gripper Current Position Service Server
            gripper_current_position_server_ = nh_.advertiseService("/ur_rtde/robotiq_gripper/current_position", &RTDEController::currentPositionRobotiQGripperCallback, this);

        } catch(const std::exception& e) {std::cerr << "Error: " << e.what() << std::endl; ROS_ERROR("Failed to Start the RobotiQ 2F Gripper");}

    }

    if (ft_sensor_) {

        // Zero FT Sensor
        rtde_control_ -> zeroFtSensor();

        // FT Sensor Publisher
        ft_sensor_pub_ = nh_.advertise<geometry_msgs::Wrench>("/ur_rtde/ft_sensor", 1);

        // Zero FT Sensor Service Server
        zeroFT_sensor_server_ = nh_.advertiseService("/ur_rtde/zeroFTSensor", &RTDEController::zeroFTSensorCallback, this);

    }

    // ROS - Publishers
    joint_state_.name = {"shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"};
    joint_state_.position.resize(6, 0.0);
    joint_state_.velocity.resize(6, 0.0);

    joint_state_pub_         = nh_.advertise<sensor_msgs::JointState>("/joint_states", 1);
    tcp_pose_pub_            = nh_.advertise<geometry_msgs::Pose>("/ur_rtde/cartesian_pose", 1);
    trajectory_executed_pub_ = nh_.advertise<std_msgs::Bool>("/ur_rtde/trajectory_executed", 1);

    // ROS - Subscribers -> Executed by the AsyncSpinner, Callbacks of the Same Subscriber are Not Concurrent
    trajectory_command_sub_         = nh_.subscribe("/ur_rtde/controllers/trajectory_controller/command", 1, &RTDEController::jointTrajectoryCallback, this);
    joint_goal_command_sub_         = nh_.subscribe("/ur_rtde/controllers/joint_space_controller/command", 1, &RTDEController::jointGoalCallback, this);
    cartesian_goal_command_sub_     = nh_.subscribe("/ur_rtde/controllers/cartesian_space_controller/command", 1, &RTDEController::cartesianGoalCallback, this);
    joint_velocity_command_sub_     = nh_.subscribe("/ur_rtde/controllers/joint_velocity_controller/command", 1, &RTDEController::jointVelocityCallback, this);
    cartesian_velocity_command_sub_ = nh_.subscribe("/ur_rtde/controllers/cartesian_velocity_controller/command", 1, &RTDEController::cartesianVelocityCallback, this);
    digital_io_set_sub_             = nh_.subscribe("/ur_rtde/digitalIO/command", 1, &RTDEController::digitalIOSetCallback, this);
    tool_digital_io_set_sub_        = nh_.subscribe("/ur_rtde/tool_digitalIO/command", 1, &RTDEController::toolDigitalIOSetCallback, this);

    // ROS - Service Servers
    stop_robot_server_          = nh_.advertiseService("/ur_rtde/controllers/stop_robot", &RTDEController::stopRobotCallback, this);
    set_async_parameter_server_ = nh_.advertiseService("/ur_rtde/param/set_asynchronous", &RTDEController::setAsyncParameterCallback, this);
    start_FreedriveMode_server_ = nh_.advertiseService("/ur_rtde/FreedriveMode/start", &RTDEController::startFreedriveModeCallback, this);
    stop_FreedriveMode_server_  = nh_.advertiseService("/ur_rtde/FreedriveMode/stop", &RTDEController::stopFreedriveModeCallback, this);
    get_FK_server_              = nh_.advertiseService("/ur_rtde/getFK", &RTDEController::getForwardKinematicCallback, this);
    get_IK_server_              = nh_.advertiseService("/ur_rtde/getIK", &RTDEController::getInverseKinematicCallback, this);
    get_safety_status_server_   = nh_.advertiseService("/ur_rtde/getSafetyStatus", &RTDEController::getSafetyStatusCallback, this);

    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << std::endl;
    ROS_WARN("UR RTDE Controller - Connected\n");
}

RTDEController::~RTDEController()
{
    // Stop Robot
    stopRobot();

    // Disconnect RTDE Control Interface
    if (rtde_control_) rtde_control_ -> disconnect();
    std::cout << std::endl;
    ROS_WARN("UR RTDE Controller - Disconnected\n");
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

void RTDEController::jointTrajectoryCallback(const trajectory_msgs::JointTrajectory::ConstPtr &msg)
{
    // Joint Order: Message joint_names (if Given) -> Controller Joint Order
    const size_t n_joints = joint_state_.name.size();
    std::vector<size_t> joint_index(n_joints);
    for (size_t j = 0; j < n_joints; j++) joint_index[j] = j;

    if (!msg->joint_names.empty())
    {
        if (msg->joint_names.size() != n_joints) {ROS_ERROR("ERROR: Trajectory joint_names Size != %zu\n", n_joints); return;}

        for (size_t j = 0; j < n_joints; j++)
        {
            auto it = std::find(msg->joint_names.begin(), msg->joint_names.end(), joint_state_.name[j]);
            if (it == msg->joint_names.end()) {ROS_ERROR("ERROR: Joint \"%s\" Missing in the Trajectory\n", joint_state_.name[j].c_str()); return;}
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
        waypoints.push_back({point.time_from_start.toSec(), reorder(point.positions), reorder(point.velocities), reorder(point.accelerations)});

    // Check Trajectory Consistency
    std::string error = trajectory::validate(waypoints, n_joints);
    if (!error.empty()) {ROS_ERROR("ERROR: Invalid Trajectory | %s\n", error.c_str()); return;}

    // Sample the Trajectory at the Controller Rate -> Resample if Not Already Sampled at 1/rate
    bool resampled = false;
    trajectory::SampledTrajectory sampled = trajectory::sample(waypoints, 1.0 / rate_, &resampled);
    if (resampled) ROS_WARN("Trajectory Not Sampled at %.1f ms (or Without Velocities) -> Resampled\n", 1000.0 / rate_);

    // Check the Trajectory Starts from the Actual State
    std::vector<double> actual_position = getActualJointPosition(), actual_velocity = getActualJointVelocity();
    Eigen::VectorXd q  = Eigen::Map<Eigen::VectorXd>(actual_position.data(), actual_position.size());
    Eigen::VectorXd dq = Eigen::Map<Eigen::VectorXd>(actual_velocity.data(), actual_velocity.size());

    double start_error = (sampled.position.front() - q).cwiseAbs().maxCoeff();
    if (start_error > trajectory_start_tolerance_)
        {ROS_ERROR("ERROR: Trajectory Not Starting from the Actual Configuration | Error: %.5f rad > %.5f rad\n", start_error, trajectory_start_tolerance_); return;}

    double start_velocity_error = (sampled.velocity.front() - dq).cwiseAbs().maxCoeff();
    if (start_velocity_error > TRAJECTORY_VELOCITY_TOLERANCE)
        {ROS_ERROR("ERROR: Trajectory Starting Velocity != Actual Velocity | Error: %.5f rad/s\n", start_velocity_error); return;}

    if (sampled.velocity.back().cwiseAbs().maxCoeff() > TRAJECTORY_VELOCITY_TOLERANCE)
        {ROS_ERROR("ERROR: Trajectory Final Velocity != 0\n"); return;}

    // Check Joint, Velocity and Acceleration Limits
    double max_position = 0.0, max_velocity = 0.0, max_acceleration = 0.0;
    for (size_t k = 0; k < sampled.size(); k++)
    {
        max_position     = std::max(max_position,     sampled.position[k].cwiseAbs().maxCoeff());
        max_velocity     = std::max(max_velocity,     sampled.velocity[k].cwiseAbs().maxCoeff());
        max_acceleration = std::max(max_acceleration, sampled.acceleration[k].cwiseAbs().maxCoeff());
    }

    if (max_position > JOINT_LIMITS || max_velocity > JOINT_VELOCITY_MAX || max_acceleration > JOINT_ACCELERATION_MAX)
        {ROS_ERROR("ERROR: Joint Limit Not Satisfied | Max Position: %.3f, Max Velocity: %.3f, Max Acceleration: %.3f\n", max_position, max_velocity, max_acceleration); return;}

    std::vector<double> final_position(sampled.position.back().data(), sampled.position.back().data() + n_joints);
    if (!rtde_control_ -> isJointsWithinSafetyLimits(final_position)) {ROS_ERROR("ERROR: Trajectory Final Position Outside Safety Limits\n"); return;}

    // New Trajectory -> Replaces the One in Execution
    const double duration = (sampled.size() - 1) / rate_;
    {
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        trajectory_ = std::move(sampled);
        trajectory_index_ = 0;
        trajectory_duration_ = duration;
        trajectory_active_ = true;
    }

    ROS_INFO("New Trajectory Received | Duration: %.3f s\n", duration);
}

void RTDEController::jointGoalCallback(const trajectory_msgs::JointTrajectoryPoint::ConstPtr &msg)
{
    // Check Input Data Size
    if (msg->positions.size() != 6) {ROS_ERROR("ERROR: Received Joint Position Goal Size != 6\n"); return;}
    if (msg->time_from_start.toSec() == 0 && msg->velocities.size() == 0) {ROS_ERROR("ERROR: Desired Time = 0\n"); return;}
    else if (msg->time_from_start.toSec() == 0 && msg->velocities[0] <= 0.0) {ROS_ERROR("ERROR: Desired Time = 0 | Desired Velocity <= 0\n"); return;}

    // Get Desired and Actual Joint Pose
    Eigen::VectorXd desired_pose = Eigen::VectorXd::Map(msg->positions.data(), msg->positions.size());
    std::vector<double> actual_joint_position = getActualJointPosition();
    Eigen::VectorXd actual_pose  = Eigen::VectorXd::Map(actual_joint_position.data(), actual_joint_position.size());

    // Check Joint Limits
    if (!rtde_control_ -> isJointsWithinSafetyLimits(msg->positions)) {ROS_ERROR("ERROR: Received Joint Position Outside Safety Limits\n"); return;}

    // Initialize Velocity and Acceleration
    double velocity, acceleration = 4.0;

    // Compute Velocity Using Time
    if (msg->time_from_start.toSec() != 0)
    {
        // Path Length
        double LP = (desired_pose - actual_pose).array().abs().maxCoeff();
        double T = msg->time_from_start.toSec();

        // Check Acceleration is Sufficient to Reach the Goal in the Desired Time
        if (acceleration < 4 * LP / std::pow(T,2))
        {
            T = std::sqrt(4 * LP / acceleration);
            ROS_WARN_STREAM("Robot Acceleration is Not Sufficient to Reach the Goal in the Desired Time | Used the Minimum Time: " << T << std::endl);
        }

        // Compute Velocity
        double ta = T/2.0 - 0.5 * std::sqrt((std::pow(T,2) * acceleration - 4.0 * LP) / acceleration + 10e-12);
        velocity = ta * acceleration;

    // Use Given Velocity
    } else {velocity = msg->velocities[0];}

    // Check Velocity Limits
    if (velocity > JOINT_VELOCITY_MAX) {ROS_ERROR("Requested Velocity > Maximum Velocity\n"); return;}

    // Exit Speed Mode (After Velocity Commands) Before the Position Movement
    rtde_control_ -> speedStop();

    // Move to Joint Goal
    rtde_control_ -> moveJ(msg->positions, velocity, acceleration, asynchronous_);

    // Publish Trajectory Executed
    if (!asynchronous_) publishTrajectoryExecuted();
    else new_async_joint_pose_received_ = true;
}

void RTDEController::cartesianGoalCallback(const ur_rtde_controller::CartesianPoint::ConstPtr &msg)
{
    // Convert Geometry Pose to RTDE Pose
    std::vector<double> desired_pose = Pose2RTDE(msg->cartesian_pose);

    // Check Pose Limits
    if (!rtde_control_ -> isPoseWithinSafetyLimits(desired_pose)) {ROS_ERROR("ERROR: Received Cartesian Position Outside Safety Limits\n"); return;}

    // TODO: Convert Desired Time to Velocity

    // Check Tool Velocity Limits
    if (msg->velocity > TOOL_VELOCITY_MAX) {ROS_ERROR("Requested Velocity > Maximum Velocity\n"); return;}

    // Exit Speed Mode (After Velocity Commands) Before the Position Movement
    rtde_control_ -> speedStop();

    // Move to Linear Goal
    rtde_control_ -> moveL(desired_pose, msg->velocity, 1.20, asynchronous_);

    // Publish Trajectory Executed
    if (!asynchronous_) publishTrajectoryExecuted();
    else new_async_cartesian_pose_received_ = true;
}

void RTDEController::jointVelocityCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    // Check Input Data Size
    if (msg->data.size() != 6) {ROS_ERROR("ERROR: Received Joint Velocity Size != 6\n"); return;}

    // Get Current and Desired Joint Velocity
    std::vector<double> desired_velocity = msg->data;
    std::vector<double> current_velocity = getActualJointVelocity();

    // Compute Velocity Difference
    Eigen::VectorXd velocity_difference = Eigen::VectorXd::Map(desired_velocity.data(), desired_velocity.size())
                                        - Eigen::VectorXd::Map(current_velocity.data(), current_velocity.size());

    // Compute MAX Acceleration -> Reach the Desired Velocity in One Control Cycle (dv / dt)
    double acceleration = std::max(velocity_difference.array().abs().maxCoeff() * rate_, 1.0);

    // Check Acceleration Limits
    if (limit_acc_ && acceleration > JOINT_ACCELERATION_MAX) {ROS_WARN_THROTTLE(1.0, "Requested Acceleration > Maximum Acceleration -> Clamped to %.1f rad/s^2\n", JOINT_ACCELERATION_MAX);
                                                              acceleration = JOINT_ACCELERATION_MAX;}

    // Joint Velocity Publisher
    rtde_control_ -> speedJ(desired_velocity, acceleration, 0.0001); // Use a Small Sleep Time to Avoid RTDE Control Timeout
}

void RTDEController::cartesianVelocityCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    // Create Desired Velocity Vector
    std::vector<double> desired_cartesian_velocity = {msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z};

    // TODO: Compute Velocity Difference and MAX Acceleration
    double acceleration = 0.25;

    // Check Acceleration Limits
    if (acceleration > TOOL_ACCELERATION_MAX) {ROS_ERROR("Requested Acceleration > Maximum Acceleration\n"); return;}

    // Cartesian Velocity Publisher
    rtde_control_ -> speedL(desired_cartesian_velocity, acceleration, 0.002);
}

void RTDEController::digitalIOSetCallback(const std_msgs::Int8::ConstPtr &msg)
{
    // Function to set a boolean value in a digital port of the UR IO network
    // NOTE: As 0 id wouldn't be able to specify the value, ids are shifted by 1
    // output boolean = sign(msg)
    // output id      = abs(msg) - 1
    if (msg->data == 0) {ROS_ERROR("ERROR: Digital Output Command = 0 -> Use +/-(id + 1)\n"); return;}
    uint8_t output_id = abs(msg->data) - 1;
    bool signal_level = msg->data > 0;
    rtde_io_ -> setStandardDigitalOut(output_id, signal_level);
}

void RTDEController::toolDigitalIOSetCallback(const std_msgs::Int8::ConstPtr &msg)
{
    // Function to set a boolean value in a tool digital port of the UR IO network
    // NOTE: As 0 id wouldn't be able to specify the value, ids are shifted by 1
    // output boolean = sign(msg)
    // output id      = abs(msg) - 1
    if (msg->data == 0) {ROS_ERROR("ERROR: Tool Digital Output Command = 0 -> Use +/-(id + 1)\n"); return;}
    uint8_t output_id = abs(msg->data) - 1;
    bool signal_level = msg->data > 0;
    rtde_io_ -> setToolDigitalOut(output_id, signal_level);
}

bool RTDEController::stopRobotCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    // Stop Robot
    stopRobot();

    if (!asynchronous_)
    {
        // StopRobot Not Working with Synchronous Operations
        ROS_WARN("StopRobot Not Working without the Asynchronous Flag");
        res.success = false;

    } else {res.success = true;}

    return true;
}

bool RTDEController::setAsyncParameterCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    // Set Asynchronous Parameter
    asynchronous_ = req.data;

    res.success = true;
    return true;
}

bool RTDEController::startFreedriveModeCallback(ur_rtde_controller::StartFreedriveMode::Request &req, ur_rtde_controller::StartFreedriveMode::Response &res)
{
    // freeAxes = [1,0,0,0,0,0]     -> The robot is compliant in the x direction relative to the feature.
    // freeAxes: A 6 dimensional vector that contains 0’s and 1’s, these indicates in which axes movement is allowed. The first three values represents the cartesian directions along x, y, z, and the last three defines the rotation axis, rx, ry, rz. All relative to the selected feature

    // Exit Speed Mode (After Velocity Commands) Before the Freedrive Mode
    rtde_control_ -> speedStop();

    // Start FreeDrive Mode
    res.success = rtde_control_ -> freedriveMode(req.free_axes);
    return true;
}

bool RTDEController::stopFreedriveModeCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    // Exit from FreeDrive Mode
    res.success = rtde_control_ -> endFreedriveMode();
    return true;
}

bool RTDEController::zeroFTSensorCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    // Reset Force-Torque Sensor
    res.success = rtde_control_ -> zeroFtSensor();
    return true;
}

bool RTDEController::getForwardKinematicCallback(ur_rtde_controller::GetForwardKinematic::Request &req, ur_rtde_controller::GetForwardKinematic::Response &res)
{
    // Compute Forward Kinematic
    std::vector<double> tcp_pose = rtde_control_ -> getForwardKinematics(req.joint_position, {0.0,0.0,0.0,0.0,0.0,0.0});

    // Convert RTDE Pose to Geometry Pose
    res.tcp_position = RTDE2Pose(tcp_pose);

    res.success = true;
    return true;
}

bool RTDEController::getInverseKinematicCallback(ur_rtde_controller::GetInverseKinematic::Request &req, ur_rtde_controller::GetInverseKinematic::Response &res)
{
    // Convert Geometry Pose to RTDE Pose
    std::vector<double> tcp_pose = Pose2RTDE(req.tcp_position);

    // Compute Inverse Kinematic
    if (req.near_position.size() == 6) res.joint_position = rtde_control_ -> getInverseKinematics(tcp_pose, req.near_position);
    else res.joint_position = rtde_control_ -> getInverseKinematics(tcp_pose);

    res.success = true;
    return true;
}

bool RTDEController::getSafetyStatusCallback(ur_rtde_controller::GetRobotStatus::Request &req, ur_rtde_controller::GetRobotStatus::Response &res)
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
    res.robot_mode = rtde_receive_ -> getRobotMode();
    size_t robot_mode_index = res.robot_mode + 1;
    res.robot_mode_msg = (robot_mode_index < robot_mode_msg.size()) ? robot_mode_msg[robot_mode_index] : "UNKNOWN";

    // Get Safety Mode
    res.safety_mode = rtde_receive_ -> getSafetyMode();
    size_t safety_mode_index = res.safety_mode;
    res.safety_mode_msg = (safety_mode_index < safety_mode_msg.size()) ? safety_mode_msg[safety_mode_index] : "UNKNOWN";

    // Get Safety Status Bits -> Bitmask, List All the Active Bits
    res.safety_status_bits = rtde_receive_ -> getSafetyStatusBits();
    for (size_t bit = 0; bit < safety_status_bits_msg.size(); bit++)
    {
        if (!(res.safety_status_bits & (1u << bit))) continue;
        if (!res.safety_status_bits_msg.empty()) res.safety_status_bits_msg += ", ";
        res.safety_status_bits_msg += safety_status_bits_msg[bit];
    }

    res.success = true;
    return true;
}

bool RTDEController::RobotiQGripperCallback(ur_rtde_controller::RobotiQGripperControl::Request &req, ur_rtde_controller::RobotiQGripperControl::Response &res)
{
    // Normalize Received Values
    float position = double(req.position) / 100.0;
    float speed    = double(req.speed) / 100.0;
    float force    = double(req.force) / 100.0;

    // Move Gripper - Normalized Values (0.0 - 1.0)
    if (!robotiq_gripper_ || !robotiq_gripper_ -> isConnected()) {ROS_ERROR("ERROR: RobotiQ Gripper Not Connected\n"); res.success = false; return true;}

    try {res.status = robotiq_gripper_ -> move(position, speed, force, ur_rtde::RobotiqGripper::WAIT_FINISHED);}
    catch (const std::exception &e) {ROS_ERROR("ERROR: RobotiQ Gripper Move Failed: %s\n", e.what()); res.success = false; return true;}

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

    res.success = true;
    return true;
}

bool RTDEController::enableRobotiQGripperCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    // Enable RobotiQ Gripper
    enable_gripper_ = true;

    try {

        // Create the Gripper Class if Doesn't Exist
        if (!robotiq_gripper_) robotiq_gripper_ = std::make_unique<ur_rtde::RobotiqGripper>(ROBOT_IP, 63352, false);

        // Connect the Gripper if Not Connected
        if (!robotiq_gripper_ -> isConnected()) robotiq_gripper_ -> connect();

        // Activate the Gripper
        robotiq_gripper_ -> activate();

    } catch (const std::exception &e) {ROS_ERROR("ERROR: Failed to Enable the RobotiQ Gripper: %s\n", e.what()); res.success = false; return true;}

    // Create the Gripper Service Server if Doesn't Exist
    if (!robotiq_gripper_server_) robotiq_gripper_server_ = nh_.advertiseService("/ur_rtde/robotiq_gripper/command", &RTDEController::RobotiQGripperCallback, this);

    res.success = true;
    return true;
}

bool RTDEController::disableRobotiQGripperCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    // Disable RobotiQ Gripper
    enable_gripper_ = false;
    res.success = true;

    // Return if the Gripper Class Doesn't Exist
    if (!robotiq_gripper_) return true;

    // Disconnect if the Gripper is Connected
    if (robotiq_gripper_ -> isConnected()) robotiq_gripper_ -> disconnect();

    // Shutdown the Gripper Service Server if Exist
    if (robotiq_gripper_server_) robotiq_gripper_server_.shutdown();

    return true;
}

bool RTDEController::currentPositionRobotiQGripperCallback(ur_rtde_controller::GetGripperPosition::Request &req, ur_rtde_controller::GetGripperPosition::Response &res)
{
    // Get Current RobotiQ Gripper Position
    if (!robotiq_gripper_ || !robotiq_gripper_ -> isConnected()) {ROS_ERROR("ERROR: RobotiQ Gripper Not Connected\n"); res.success = false; return true;}
    res.current_position = robotiq_gripper_ -> getCurrentPosition();

    res.success = true;
    return true;
}

void RTDEController::publishJointState()
{
    // Create JointState Message
    joint_state_.header.stamp = ros::Time::now();

    // Read Joint Position and Velocity
    joint_state_.position = rtde_receive_ -> getActualQ();
    joint_state_.velocity = rtde_receive_ -> getActualQd();

    {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        actual_joint_position_ = joint_state_.position;
        actual_joint_velocity_ = joint_state_.velocity;
    }

    // Publish JointState
    joint_state_pub_.publish(joint_state_);
}

void RTDEController::publishTCPPose()
{
    // Read TCP Position
    actual_cartesian_pose_ = RTDE2Pose(rtde_receive_ -> getActualTCPPose());

    // Publish TCP Pose
    tcp_pose_pub_.publish(actual_cartesian_pose_);
}

void RTDEController::publishFTSensor()
{
    // Return if the FT Sensor is Disabled
    if (!ft_sensor_) return;

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
    std_msgs::Bool trajectory_executed;
    trajectory_executed.data = success;
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

        // Last Sample Sent -> Start the Settling Timer (Wall Clock, Independent of Loop Overruns)
        if (trajectory_index_ == trajectory_.size()) trajectory_settling_start_ = std::chrono::steady_clock::now();
    }
    else
    {
        // Settling -> Position Correction on the Final Point until Goal Tolerance or Timeout (max(0.1 x Trajectory Duration, TRAJECTORY_MIN_SETTLING_TIME))
        Eigen::VectorXd error = trajectory_.position.back() - q;
        bool goal_reached = error.cwiseAbs().maxCoeff() < trajectory_goal_tolerance_;
        const double max_settling_time = std::max((TRAJECTORY_TIMEOUT_FACTOR - 1.0) * trajectory_duration_, TRAJECTORY_MIN_SETTLING_TIME);
        const double settling_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - trajectory_settling_start_).count();

        if (goal_reached || settling_time > max_settling_time)
        {
            // Stop Speed Mode
            rtde_control_ -> speedStop();
            trajectory_active_ = false;
            lock.unlock();

            if (!goal_reached) ROS_ERROR("ERROR: Trajectory Goal Not Reached within %.3f s Settling Time | Error: %.5f rad\n", max_settling_time, error.cwiseAbs().maxCoeff());

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

    // Reset Booleans Variables -> Abort the Trajectory in Execution Before Stopping
    resetBooleans();

    // Stop Robot
    rtde_control_ -> stopJ(2.0);

    // Wait
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Clear Dashboard Warning Pop-Up
    rtde_dashboard_ -> connect();
    rtde_dashboard_ -> closePopup();
    rtde_dashboard_ -> disconnect();
}

void RTDEController::checkRobotStatus()
{
    // Init Flags
    bool eStop = false, protectiveStop = false;

    // Print Robot Emergency and Protective Stop
    while (ros::ok() && rtde_receive_ -> isEmergencyStopped())  {ROS_WARN_STREAM_THROTTLE(5.0, "EMERGENCY STOP PRESSED"); eStop = true;          std::this_thread::sleep_for(std::chrono::milliseconds(10));}
    while (ros::ok() && rtde_receive_ -> isProtectiveStopped()) {ROS_WARN_STREAM_THROTTLE(5.0, "PROTECTIVE STOP");         protectiveStop = true; std::this_thread::sleep_for(std::chrono::milliseconds(10));}

    // Check Flags
    if (eStop)
    {
        // Info Prints
        ROS_WARN("EMERGENCY STOP RELEASED\n");

        // Check if Robot Mode is ROBOT_MODE_RUNNING
        while (ros::ok() && rtde_receive_ -> getRobotMode() != ROBOT_MODE_RUNNING) {ROS_INFO_STREAM_THROTTLE(5.0, "Wait for Robot Recovery..."); std::this_thread::sleep_for(std::chrono::milliseconds(10));}

    } else if (protectiveStop) {ROS_WARN("PROTECTIVE STOP RECOVERED\n");}

    if (eStop || protectiveStop)
    {
        // Re-Upload RTDE Control Script
        rtde_control_ -> reuploadScript();
        rtde_control_ -> disconnect();
        rtde_control_ -> reconnect();

        // Wait Time For Connection
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Print Robot Ready
        std::cout << std::endl;
        ROS_WARN("Robot Ready to Receive New Commands\n");

        // Reset Booleans
        resetBooleans();
    }

    // Check Robot Connection Status
    if (!rtde_control_ -> isConnected()) ROS_ERROR("ROBOT DISCONNECTED\n");
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
    // Timers at the Controller Rate -> Each Timer Callback is Not Concurrent with Itself
    const ros::WallDuration period(1.0 / rate_);
    jointState_timer_ = nh_.createWallTimer(period, [this](const ros::WallTimerEvent &) {publishJointState();});
    tcpPose_timer_    = nh_.createWallTimer(period, [this](const ros::WallTimerEvent &) {publishTCPPose();});
    force_timer_      = nh_.createWallTimer(period, [this](const ros::WallTimerEvent &) {publishFTSensor();});
    checkRobot_timer_ = nh_.createWallTimer(period, [this](const ros::WallTimerEvent &) {checkRobot();});

    // Multi-Threaded Spinner -> Blocking Callbacks (e.g. Synchronous moveJ) Do Not Stop the Timers
    ros::AsyncSpinner async_spinner(16);
    async_spinner.start();

    // Spin Until Shutdown (Default SIGINT Handler)
    ros::waitForShutdown();

    // Stop the Callbacks Before the Destructor
    async_spinner.stop();
    jointState_timer_.stop();
    tcpPose_timer_.stop();
    force_timer_.stop();
    checkRobot_timer_.stop();
}

int main(int argc, char **argv) {

    // Initialize ROS -> Default SIGINT Handler Calls ros::shutdown()
    ros::init(argc, argv, "ur_rtde_controller");
    std::cout << std::endl;

    try {

        // Create a New RTDEController
        auto rtde = std::make_unique<RTDEController>();

        // Main Spinner
        rtde->spinner();

        // Stop the Robot and Disconnect (Destructor)
        rtde.reset();

    } catch (const std::exception &e) {std::cerr << "UR RTDE Controller: " << e.what() << std::endl;}

    return 0;

}
