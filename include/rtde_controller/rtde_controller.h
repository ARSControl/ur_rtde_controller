#ifndef RTDE_CONTROLLER_H
#define RTDE_CONTROLLER_H

#include <ros/ros.h>

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_io_interface.h>
#include <ur_rtde/robotiq_gripper.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>

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

        // ---- GLOBAL VARIABLES ---- //
        trajectory_msgs::JointTrajectory desired_trajectory_;
        trajectory_msgs::JointTrajectoryPoint desired_joint_goal_;
        bool new_trajectory_received_;
        bool new_joint_goal_received_;

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

        // ---- ROS - SUBSCRIBERS & CALLBACKS ---- //
        ros::Subscriber trajectory_command_sub_;
        ros::Subscriber joint_goal_command_sub_;
        void jointTrajectoryCallback(const trajectory_msgs::JointTrajectory msg);
        void jointGoalCallback(const trajectory_msgs::JointTrajectoryPoint msg);

        // ---- ROS - SERVICE SERVERS & CALLBACKS ---- //
        ros::ServiceServer stop_robot_server_;
        ros::ServiceServer robotiq_gripper_server_;
        bool RobotiQGripperCallback(ur_rtde_controller::RobotiQGripperControl::Request  &req, ur_rtde_controller::RobotiQGripperControl::Response &res);
        bool stopRobotCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

        // ---- UR RTDE FUNCTIONS ---- //
        void publishJointState();
        void publishTCPPose();
        void readRobotSafetyStatus();

};

#endif /* RTDE_CONTROLLER_H */
