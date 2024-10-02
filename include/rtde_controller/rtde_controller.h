#ifndef RTDE_CONTROLLER_H
#define RTDE_CONTROLLER_H

#include <ros/ros.h>
#include <thread>
#include <signal.h>

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_io_interface.h>
#include <ur_rtde/dashboard_client.h>
#include <ur_rtde/robotiq_gripper.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>

#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>

#include "ur_rtde_controller/RobotiQGripperControl.h"
#include "ur_rtde_controller/CartesianPoint.h"
#include "ur_rtde_controller/GetForwardKinematic.h"
#include "ur_rtde_controller/GetInverseKinematic.h"
#include "ur_rtde_controller/StartFreedriveMode.h"
#include "ur_rtde_controller/GetRobotStatus.h"
#include "ur_rtde_controller/GetGripperPosition.h"

#include <Eigen/Dense>

#include "polyfit/polyfit.h"

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

#define SENSOR_ERROR 10e-5

class RTDEController {

	public:

        RTDEController(ros::NodeHandle &nh, ros::Rate ros_rate);
        ~RTDEController();

        void spinner();

        // ROS2 Publishers Functions
        void publishJointState();
        void publishTCPPose();
        void publishFTSensor();
        bool shutdown_ = false;

	private:

        // ROS - Node Handle & Rate
        ros::NodeHandle nh_;
        ros::Rate ros_rate_;

        // Parameters
        std::string ROBOT_IP;
        bool enable_gripper_;
        bool asynchronous_;
        bool limit_acc_;
        bool ft_sensor_;

        // Initialization Variables
        bool rtde_dashboard_initialized = false;
        bool rtde_dashboard_connected = false;
        bool rtde_control_initialized = false;
        bool rtde_receive_initialized = false;
        bool rtde_io_initialized = false;
        bool robot_initialized = false;

        // Global Variables
        std::vector<double> actual_joint_position_;
        std::vector<double> actual_joint_velocity_;
        geometry_msgs::Pose actual_cartesian_pose_;
        bool new_trajectory_received_ = false;
        bool new_async_joint_pose_received_ = false;
        bool new_async_cartesian_pose_received_ = false;

        // Trajectory Variables
        PolyFit fitting;
        PolyFit polynomial_fit_;
        double trajectory_time_;

        // UR RTDE Library
        ur_rtde::RTDEControlInterface *rtde_control_;
        ur_rtde::RTDEReceiveInterface *rtde_receive_;
        ur_rtde::RTDEIOInterface *rtde_io_;
        ur_rtde::DashboardClient *rtde_dashboard_;

        // RobotiQ Gripper
        ur_rtde::RobotiqGripper *robotiq_gripper_;

        // ROS Publishers
        ros::Publisher joint_state_pub_;
        ros::Publisher tcp_pose_pub_;
        ros::Publisher ft_sensor_pub_;
        ros::Publisher trajectory_executed_pub_;

    	// ROS Subscribers and Callbacks
        ros::Subscriber trajectory_command_sub_;
        ros::Subscriber joint_goal_command_sub_;
        ros::Subscriber cartesian_goal_command_sub_;
        ros::Subscriber joint_velocity_command_sub_;
        ros::Subscriber cartesian_velocity_command_sub_;
        ros::Subscriber digital_io_set_sub_;

        void jointTrajectoryCallback(const trajectory_msgs::JointTrajectory msg);
        void jointGoalCallback(const trajectory_msgs::JointTrajectoryPoint msg);
        void cartesianGoalCallback(const ur_rtde_controller::CartesianPoint msg);
        void jointVelocityCallback(const std_msgs::Float64MultiArray msg);
        void cartesianVelocityCallback(const geometry_msgs::Twist msg);
		void digitalIOSetCallback(const std_msgs::Int8 msg);

    	// ROS Service Servers and Callbacks
        ros::ServiceServer stop_robot_server_;
        ros::ServiceServer set_async_parameter_server_;
        ros::ServiceServer start_FreedriveMode_server_;
        ros::ServiceServer stop_FreedriveMode_server_;
        ros::ServiceServer zeroFT_sensor_server_;
        ros::ServiceServer get_FK_server_;
        ros::ServiceServer get_IK_server_;
        ros::ServiceServer get_safety_status_server_;
        ros::ServiceServer robotiq_gripper_server_;
        ros::ServiceServer enable_gripper_server_;
        ros::ServiceServer disable_gripper_server_;
        ros::ServiceServer gripper_current_position_server_;
        bool stopRobotCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
        bool setAsyncParameterCallback(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res);
        bool startFreedriveModeCallback(ur_rtde_controller::StartFreedriveMode::Request  &req, ur_rtde_controller::StartFreedriveMode::Response &res);
        bool stopFreedriveModeCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
        bool zeroFTSensorCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
        bool getForwardKinematicCallback(ur_rtde_controller::GetForwardKinematic::Request  &req, ur_rtde_controller::GetForwardKinematic::Response &res);
        bool getInverseKinematicCallback(ur_rtde_controller::GetInverseKinematic::Request  &req, ur_rtde_controller::GetInverseKinematic::Response &res);
        bool getSafetyStatusCallback(ur_rtde_controller::GetRobotStatus::Request  &req, ur_rtde_controller::GetRobotStatus::Response &res);
        bool RobotiQGripperCallback(ur_rtde_controller::RobotiQGripperControl::Request  &req, ur_rtde_controller::RobotiQGripperControl::Response &res);
        bool enableRobotiQGripperCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
        bool disableRobotiQGripperCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
        bool currentPositionRobotiQGripperCallback(ur_rtde_controller::GetGripperPosition::Request &req, ur_rtde_controller::GetGripperPosition::Response &res);

        // Movement Functions
        void moveTrajectory();
        void checkAsyncMovements();
        void stopRobot();

        // Utilities Functions
        void resetBooleans();
        void publishTrajectoryExecuted();
        void checkRobotStatus();
        std::vector<double> Pose2RTDE(geometry_msgs::Pose pose);
        geometry_msgs::Pose RTDE2Pose(std::vector<double> rtde_pose);

        // Eigen Functions
        Eigen::Matrix<double, 4, 4> pose2eigen(geometry_msgs::Pose pose);
        Eigen::VectorXd computePoseError(Eigen::Matrix<double, 4, 4> T_des, Eigen::Matrix<double, 4, 4> T);
        bool isPoseReached(Eigen::VectorXd position_error, double movement_precision);
        bool isJointReached();

};

#endif /* RTDE_CONTROLLER_H */
