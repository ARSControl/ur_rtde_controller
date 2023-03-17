#ifndef POSITION_CONTROLLER_H
#define POSITION_CONTROLLER_H

#include <ros/ros.h>
#include <thread>

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_io_interface.h>
#include <ur_rtde/robotiq_gripper.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>

#include <std_srvs/Trigger.h>

#include "ur_rtde_controller/RobotiQGripperControl.h"
#include "ur_rtde_controller/CartesianPoint.h"
#include "ur_rtde_controller/GetForwardKinematic.h"
#include "ur_rtde_controller/GetInverseKinematic.h"
#include "ur_rtde_controller/StartFreedriveMode.h"
#include "ur_rtde_controller/GetRobotStatus.h"

#include <Eigen/Dense>

#define UR_JOINT_LIMITS 6.28
#define UR_JOINT_VELOCITY_MAX 3.14
#define UR_JOINT_VELOCITY_MIN 0.0
#define UR_JOINT_ACCELERATION_MAX 40.0
#define UR_JOINT_ACCELERATION_MIN 0.0
#define UR_TOOL_VELOCITY_MAX 3.0
#define UR_TOOL_VELOCITY_MIN 0
#define UR_TOOL_ACCELERATION_MAX 150.0
#define UR_TOOL_ACCELERATION_MIN 0
#define UR_SERVO_LOOKAHEAD_TIME_MAX 0.2
#define UR_SERVO_LOOKAHEAD_TIME_MIN 0.03
#define UR_SERVO_GAIN_MAX 2000
#define UR_SERVO_GAIN_MIN 100
#define UR_BLEND_MAX 2.0
#define UR_BLEND_MIN 0.0

class RTDEController {

  public:

    RTDEController(ros::NodeHandle &nh, ros::Rate ros_rate);

    ~RTDEController();

    void spinner();

    // ---- ROS PUBLISHER FUNCTIONS ---- //
    void publishJointState();
    void publishTCPPose();
    void publishFTSensor();
    bool shutdown_ = false;

  private:

    // ---- ROS - NODE HANDLE & RATE ---- //
    ros::NodeHandle nh_;
    ros::Rate ros_rate_;

    // ---- PARAMETERS ---- //
    std::string ROBOT_IP;
    bool enable_gripper;

    // ---- GLOBAL VARIABLES ---- //
    std::vector<double> actual_joint_position_;
    std::vector<double> actual_joint_velocity_;
    geometry_msgs::Pose actual_cartesian_pose_;
    trajectory_msgs::JointTrajectory desired_trajectory_;
    bool new_trajectory_received_ = false;

    // ---- UR RTDE LIBRARY ---- //
    ur_rtde::RTDEControlInterface* rtde_control_;
    ur_rtde::RTDEReceiveInterface* rtde_receive_;
    ur_rtde::RTDEIOInterface*      rtde_io_;

    // ---- ROBOTIQ GRIPPER ---- //
    ur_rtde::RobotiqGripper*       robotiq_gripper_;

    // ---- ROS - PUBLISHERS ---- //
    ros::Publisher joint_state_pub_;
    ros::Publisher tcp_pose_pub_;
    ros::Publisher ft_sensor_pub_;
    ros::Publisher trajectory_executed_pub_;

    // ---- ROS - SUBSCRIBERS & CALLBACKS ---- //
    ros::Subscriber trajectory_command_sub_;
    ros::Subscriber joint_goal_command_sub_;
    ros::Subscriber cartesian_goal_command_sub_;
    ros::Subscriber joint_velocity_command_sub_;
    ros::Subscriber cartesian_velocity_command_sub_;
    void jointTrajectoryCallback(const trajectory_msgs::JointTrajectory msg);
    void jointGoalCallback(const trajectory_msgs::JointTrajectoryPoint msg);
    void cartesianGoalCallback(const ur_rtde_controller::CartesianPoint msg);
    void jointVelocityCallback(const std_msgs::Float64MultiArray msg);
    void cartesianVelocityCallback(const geometry_msgs::Twist msg);

    // ---- ROS - SERVICE SERVERS & CALLBACKS ---- //
    ros::ServiceServer stop_robot_server_;
    ros::ServiceServer start_FreedriveMode_server_;
    ros::ServiceServer stop_FreedriveMode_server_;
    ros::ServiceServer zeroFT_sensor_server_;
    ros::ServiceServer get_FK_server_;
    ros::ServiceServer get_IK_server_;
    ros::ServiceServer get_safety_status_server_;
    ros::ServiceServer robotiq_gripper_server_;
    bool stopRobotCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool startFreedriveModeCallback(ur_rtde_controller::StartFreedriveMode::Request  &req, ur_rtde_controller::StartFreedriveMode::Response &res);
    bool stopFreedriveModeCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool zeroFTSensorCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool getForwardKinematicCallback(ur_rtde_controller::GetForwardKinematic::Request  &req, ur_rtde_controller::GetForwardKinematic::Response &res);
    bool getInverseKinematicCallback(ur_rtde_controller::GetInverseKinematic::Request  &req, ur_rtde_controller::GetInverseKinematic::Response &res);
    bool getSafetyStatusCallback(ur_rtde_controller::GetRobotStatus::Request  &req, ur_rtde_controller::GetRobotStatus::Response &res);
    bool RobotiQGripperCallback(ur_rtde_controller::RobotiQGripperControl::Request  &req, ur_rtde_controller::RobotiQGripperControl::Response &res);

    // ---- MOVEMENT FUNCTIONS ---- //
    void moveTrajectory();

    // ---- UTILITIES FUNCTIONS ---- //
    void resetBooleans();
    void publishTrajectoryExecuted();
    std::vector<double> Pose2RTDE(geometry_msgs::Pose pose);
    geometry_msgs::Pose RTDE2Pose(std::vector<double> rtde_pose);

    // ---- EIGEN FUNCTIONS ---- //
    Eigen::Matrix<double, 4, 4> pose2eigen(geometry_msgs::Pose pose);
    Eigen::VectorXd computePoseError(Eigen::Matrix<double, 4, 4> T_des, Eigen::Matrix<double, 4, 4> T);
    bool isPoseReached(Eigen::VectorXd position_error, double movement_precision);

};

#endif /* POSITION_CONTROLLER_H */
