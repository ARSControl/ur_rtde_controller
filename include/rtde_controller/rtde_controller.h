#ifndef RTDE_CONTROLLER_H
#define RTDE_CONTROLLER_H

#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <memory>
#include <mutex>
#include <thread>

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_io_interface.h>
#include <ur_rtde/dashboard_client.h>
#include <ur_rtde/robotiq_gripper.h>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int8.hpp>

#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "ur_rtde_controller/msg/cartesian_point.hpp"
#include "ur_rtde_controller/msg/robot_dynamics.hpp"
#include "ur_rtde_controller/srv/roboti_q_gripper_control.hpp"
#include "ur_rtde_controller/srv/get_forward_kinematic.hpp"
#include "ur_rtde_controller/srv/get_inverse_kinematic.hpp"
#include "ur_rtde_controller/srv/start_freedrive_mode.hpp"
#include "ur_rtde_controller/srv/get_robot_status.hpp"
#include "ur_rtde_controller/srv/get_gripper_position.hpp"

#include <Eigen/Dense>

#include "trajectory/joint_trajectory_sampler.h"

#define JOINT_LIMITS 6.28
#define JOINT_VELOCITY_MAX 3.14
#define JOINT_VELOCITY_MIN 0.0
#define JOINT_ACCELERATION_MAX 40.0
#define JOINT_ACCELERATION_MIN 0.0
#define TOOL_VELOCITY_MAX 3.0
#define TOOL_VELOCITY_MIN 0
#define TOOL_ACCELERATION_MAX 150.0
#define TOOL_ACCELERATION_MIN 0
#define SERVO_LOOKAHEAD_TIME_MAX 0.2
#define SERVO_LOOKAHEAD_TIME_MIN 0.03
#define SERVO_GAIN_MAX 2000
#define SERVO_GAIN_MIN 100
#define BLEND_MAX 2.0
#define BLEND_MIN 0.0

#define ROBOT_MODE_NO_CONTROLLER - 1
#define ROBOT_MODE_DISCONNECTED 0
#define ROBOT_MODE_CONFIRM_SAFETY 1
#define ROBOT_MODE_BOOTING 2
#define ROBOT_MODE_POWER_OFF 3
#define ROBOT_MODE_POWER_ON 4
#define ROBOT_MODE_IDLE 5
#define ROBOT_MODE_BACKDRIVE 6
#define ROBOT_MODE_RUNNING 7
#define ROBOT_MODE_UPDATING_FIRMWARE 8

#define TRAJECTORY_POSITION_GAIN 1.0
#define TRAJECTORY_VELOCITY_TOLERANCE 0.01
#define TRAJECTORY_MIN_ACCELERATION 1.0
#define TRAJECTORY_SETTLING_TIME 1.0

#define TORQUE_MODE_START_TIMEOUT 1.0

class RTDEController : public rclcpp::Node {

    public:

        RTDEController();
        ~RTDEController();

        void spinner(void);

        // ROS2 Publishers Functions
        void publishJointState();
        void publishTCPPose();
        void publishFTSensor();
        void checkRobot();

        // ROS2 Rate
        double rate_;

    private:

        // Parameters
        std::string ROBOT_IP;
        bool enable_gripper_;
        std::atomic<bool> asynchronous_;
        bool limit_acc_;
        bool ft_sensor_;
        double trajectory_start_tolerance_;
        double trajectory_goal_tolerance_;
        std::vector<double> torque_limits_;
        std::atomic<bool> torque_friction_compensation_;
        int torque_watchdog_cycles_;

        // Initialization Variables
        bool rtde_dashboard_initialized = false;
        bool rtde_dashboard_connected = false;
        bool rtde_control_initialized = false;
        bool rtde_receive_initialized = false;
        bool rtde_io_initialized = false;
        bool robot_initialized = false;

        // Global Variables
        sensor_msgs::msg::JointState joint_state_;
        geometry_msgs::msg::Pose actual_cartesian_pose_;
        std::atomic<bool> new_async_joint_pose_received_{false};
        std::atomic<bool> new_async_cartesian_pose_received_{false};

        // Actual Joint State -> Written by publishJointState, Read by the Command Callbacks
        std::mutex joint_state_mutex_;
        std::vector<double> actual_joint_position_;
        std::vector<double> actual_joint_velocity_;
        std::vector<double> getActualJointPosition();
        std::vector<double> getActualJointVelocity();

        // Trajectory Variables -> Written by jointTrajectoryCallback, Executed by moveTrajectory
        std::mutex trajectory_mutex_;
        trajectory::SampledTrajectory trajectory_;
        bool trajectory_active_ = false;
        size_t trajectory_index_ = 0;
        size_t trajectory_settling_cycles_ = 0;

        // Torque Mode -> Dedicated Real-Time Thread Synchronized with the Robot Control Cycle
        std::atomic<bool> torque_mode_active_{false};
        std::thread torque_thread_;
        std::mutex torque_thread_mutex_;
        std::mutex torque_command_mutex_;
        std::vector<double> torque_command_;
        bool torque_command_new_ = false;

        // UR RTDE Library
        std::unique_ptr<ur_rtde::RTDEControlInterface> rtde_control_;
        std::unique_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive_;
        std::unique_ptr<ur_rtde::RTDEIOInterface> rtde_io_;
        std::unique_ptr<ur_rtde::DashboardClient> rtde_dashboard_;

        // RobotiQ Gripper
        std::unique_ptr<ur_rtde::RobotiqGripper> robotiq_gripper_;

        // ROS2 Publishers
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr tcp_pose_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr ft_sensor_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr trajectory_executed_pub_;
        rclcpp::Publisher<ur_rtde_controller::msg::RobotDynamics>::SharedPtr robot_dynamics_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr torque_mode_active_pub_;

        // ROS2 Timer
        rclcpp::TimerBase::SharedPtr checkRobot_timer_;
        rclcpp::TimerBase::SharedPtr tcpPose_timer_;
        rclcpp::TimerBase::SharedPtr force_timer_;
        rclcpp::TimerBase::SharedPtr jointState_timer_;
        rclcpp::executors::MultiThreadedExecutor executor;

        // ROS2 Subscribers and Callbacks
        rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_command_sub_;
        rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr joint_goal_command_sub_;
        rclcpp::Subscription<ur_rtde_controller::msg::CartesianPoint>::SharedPtr cartesian_goal_command_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_velocity_command_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cartesian_velocity_command_sub_;
        rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr digital_io_set_sub_;
        rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr tool_digital_io_set_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr torque_command_sub_;
        void jointTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
        void jointGoalCallback(const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg);
        void cartesianGoalCallback(const ur_rtde_controller::msg::CartesianPoint::SharedPtr msg);
        void jointVelocityCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
        void cartesianVelocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
        void digitalIOSetCallback(const std_msgs::msg::Int8::SharedPtr msg);
        void toolDigitalIOSetCallback(const std_msgs::msg::Int8::SharedPtr msg);
        void torqueCommandCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

        // ROS2 Service Servers and Callbacks
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_robot_server_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_async_parameter_server_;
        rclcpp::Service<ur_rtde_controller::srv::StartFreedriveMode>::SharedPtr start_FreedriveMode_server_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_FreedriveMode_server_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zeroFT_sensor_server_;
        rclcpp::Service<ur_rtde_controller::srv::GetForwardKinematic>::SharedPtr get_FK_server_;
        rclcpp::Service<ur_rtde_controller::srv::GetInverseKinematic>::SharedPtr get_IK_server_;
        rclcpp::Service<ur_rtde_controller::srv::GetRobotStatus>::SharedPtr get_safety_status_server_;
        rclcpp::Service<ur_rtde_controller::srv::RobotiQGripperControl>::SharedPtr robotiq_gripper_server_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_gripper_server_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_gripper_server_;
        rclcpp::Service<ur_rtde_controller::srv::GetGripperPosition>::SharedPtr gripper_current_position_server_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_torque_mode_server_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_torque_mode_server_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_friction_compensation_server_;
        bool stopRobotCallback(const std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res);
        bool setAsyncParameterCallback(const std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr res);
        bool startFreedriveModeCallback(const ur_rtde_controller::srv::StartFreedriveMode::Request::SharedPtr req, ur_rtde_controller::srv::StartFreedriveMode::Response::SharedPtr res);
        bool stopFreedriveModeCallback(const std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res);
        bool zeroFTSensorCallback(const std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res);
        bool getForwardKinematicCallback(const ur_rtde_controller::srv::GetForwardKinematic::Request::SharedPtr req, ur_rtde_controller::srv::GetForwardKinematic::Response::SharedPtr res);
        bool getInverseKinematicCallback(const ur_rtde_controller::srv::GetInverseKinematic::Request::SharedPtr req, ur_rtde_controller::srv::GetInverseKinematic::Response::SharedPtr res);
        bool getSafetyStatusCallback(const ur_rtde_controller::srv::GetRobotStatus::Request::SharedPtr req, ur_rtde_controller::srv::GetRobotStatus::Response::SharedPtr res);
        bool RobotiQGripperCallback(const ur_rtde_controller::srv::RobotiQGripperControl::Request::SharedPtr req, ur_rtde_controller::srv::RobotiQGripperControl::Response::SharedPtr res);
        bool enableRobotiQGripperCallback(const std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res);
        bool disableRobotiQGripperCallback(const std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res);
        bool startTorqueModeCallback(const std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res);
        bool stopTorqueModeCallback(const std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res);
        bool setFrictionCompensationCallback(const std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr res);
        bool currentPositionRobotiQGripperCallback(const ur_rtde_controller::srv::GetGripperPosition::Request::SharedPtr req, ur_rtde_controller::srv::GetGripperPosition::Response::SharedPtr res);

        // Movement Functions
        void moveTrajectory();
        void checkAsyncMovements();
        void stopRobot();

        // Torque Mode Functions
        void torqueControlLoop();
        void publishRobotDynamics();
        void stopTorqueMode();
        void publishTorqueModeActive(bool active);
        bool isTorqueModeActive(const std::string &command);

        // Utilities Functions
        void resetBooleans();
        void publishTrajectoryExecuted(bool success = true);
        void checkRobotStatus();
        std::vector<double> Pose2RTDE(geometry_msgs::msg::Pose pose);
        geometry_msgs::msg::Pose RTDE2Pose(std::vector<double> rtde_pose);

        // Eigen Functions
        Eigen::Matrix<double, 4, 4> pose2eigen(geometry_msgs::msg::Pose pose);
        Eigen::VectorXd computePoseError(Eigen::Matrix<double, 4, 4> T_des, Eigen::Matrix<double, 4, 4> T);
        bool isPoseReached(Eigen::VectorXd position_error, double movement_precision);

};

#endif /* RTDE_CONTROLLER_H */
