#ifndef VELOCITY_CONTROLLER_H
#define VELOCITY_CONTROLLER_H

#include <ros/ros.h>

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_io_interface.h>
#include <ur_rtde/robotiq_gripper.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>

#include <std_srvs/Trigger.h>

#include "ur_rtde_controller/RobotiQGripperControl.h"
#include "ur_rtde_controller/RobotStatus.h"

#include <Eigen/Dense>

class RTDEController {

    public:

      RTDEController(ros::NodeHandle &nh, ros::Rate ros_rate);

      ~RTDEController();

      void spinner();

    private:

        // ---- ROS - NODE HANDLE & RATE ---- //
        ros::NodeHandle nh_;
        ros::Rate ros_rate_;

        // ---- PARAMETERS ---- //
    	  std::string ROBOT_IP;
        bool enable_gripper;
        double max_acc_;

        // ---- GLOBAL VARIABLES ---- //
        std::vector<double> actual_joint_position_;
        std::vector<double> actual_joint_velocity_;
	      geometry_msgs::PoseStamped actual_cartesian_pose_;
        trajectory_msgs::JointTrajectory desired_trajectory_;
        std::vector<double> desired_joint_pose_;
	      Eigen::Matrix<double, 4, 4> desired_cartesian_pose_ = Eigen::Matrix<double, 4, 4>::Identity();
        bool new_trajectory_received_ = false;
	      bool new_joint_pose_received_ = false;
        bool new_cartesian_pose_received_ = false;

        // ---- UR RTDE LIBRARY ---- //
        ur_rtde::RTDEControlInterface* rtde_control_;
        ur_rtde::RTDEReceiveInterface* rtde_receive_;
        ur_rtde::RTDEIOInterface*      rtde_io_;

        // ---- ROBOTIQ GRIPPER ---- //
        ur_rtde::RobotiqGripper*       robotiq_gripper_;

        // ---- ROS - PUBLISHERS ---- //
        ros::Publisher joint_state_pub_;
        ros::Publisher tcp_pose_pub_;
        ros::Publisher robot_status_pub_;
        ros::Publisher trajectory_executed_pub_;

        // ---- ROS - SUBSCRIBERS & CALLBACKS ---- //
        ros::Subscriber joint_velocity_command_sub_;
        ros::Subscriber cartesian_velocity_command_sub_;
        ros::Subscriber trajectory_command_sub_;
        ros::Subscriber joint_goal_command_sub_;
        ros::Subscriber cartesian_goal_command_sub_;
        void jointVelocityCallback(const trajectory_msgs::JointTrajectory msg);
        void cartesianVelocityCallback(const geometry_msgs::Twist msg);
        void jointTrajectoryCallback(const trajectory_msgs::JointTrajectory msg);
        void jointGoalCallback(const trajectory_msgs::JointTrajectoryPoint msg);
        void cartesianGoalCallback(const geometry_msgs::PoseStamped msg);

        // ---- ROS - SERVICE SERVERS & CALLBACKS ---- //
        ros::ServiceServer stop_robot_server_;
        ros::ServiceServer robotiq_gripper_server_;
        bool RobotiQGripperCallback(ur_rtde_controller::RobotiQGripperControl::Request  &req, ur_rtde_controller::RobotiQGripperControl::Response &res);
        bool stopRobotCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

        // ---- UR RTDE FUNCTIONS ---- //
        void publishJointState();
        void publishTCPPose();
        void publishTrajectoryExecuted();
        void readRobotSafetyStatus();

        // ---- EIGEN FUNCTIONS ---- //
        Eigen::Matrix<double, 4, 4> pose2eigen(geometry_msgs::PoseStamped pose);
        Eigen::VectorXd computePoseError(Eigen::Matrix<double, 4, 4> T_des, Eigen::Matrix<double, 4, 4> T);
        bool isPoseReached(Eigen::VectorXd position_error, double movement_precision);

};

#endif /* VELOCITY_CONTROLLER_H */
