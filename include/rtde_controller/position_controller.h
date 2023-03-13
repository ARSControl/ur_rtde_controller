#ifndef POSITION_CONTROLLER_H
#define POSITION_CONTROLLER_H

#include <ros/ros.h>

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_io_interface.h>
#include <ur_rtde/robotiq_gripper.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>

#include <std_srvs/Trigger.h>

#include "ur_rtde_controller/RobotiQGripperControl.h"
#include "ur_rtde_controller/RobotStatus.h"
#include "ur_rtde_controller/CartesianPoint.h"
#include "ur_rtde_controller/GetForwardKinematic.h"
#include "ur_rtde_controller/GetInverseKinematic.h"
#include "ur_rtde_controller/StartFreedriveMode.h"

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
        std::vector<double> actual_joint_position_;
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
        ros::Publisher robot_status_pub_;
        ros::Publisher trajectory_executed_pub_;

        // ---- ROS - SUBSCRIBERS & CALLBACKS ---- //
        ros::Subscriber trajectory_command_sub_;
        ros::Subscriber joint_goal_command_sub_;
        ros::Subscriber cartesian_goal_command_sub_;
        void jointTrajectoryCallback(const trajectory_msgs::JointTrajectory msg);
        void jointGoalCallback(const trajectory_msgs::JointTrajectoryPoint msg);
        void cartesianGoalCallback(const ur_rtde_controller::CartesianPoint msg);

        // ---- ROS - SERVICE SERVERS & CALLBACKS ---- //
        ros::ServiceServer stop_robot_server_;
        ros::ServiceServer robotiq_gripper_server_;
        ros::ServiceServer zeroFT_sensor_server_;
        ros::ServiceServer get_FK_server_;
        ros::ServiceServer get_IK_server_;
        ros::ServiceServer start_FreedriveMode_server_;
        ros::ServiceServer stop_FreedriveMode_server_;
        bool RobotiQGripperCallback(ur_rtde_controller::RobotiQGripperControl::Request  &req, ur_rtde_controller::RobotiQGripperControl::Response &res);
        bool stopRobotCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
        bool zeroFTSensorCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
        bool GetForwardKinematicCallback(ur_rtde_controller::GetForwardKinematic::Request  &req, ur_rtde_controller::GetForwardKinematic::Response &res);
        bool GetInverseKinematicCallback(ur_rtde_controller::GetInverseKinematic::Request  &req, ur_rtde_controller::GetInverseKinematic::Response &res);
        bool startFreedriveModeCallback(ur_rtde_controller::StartFreedriveMode::Request  &req, ur_rtde_controller::StartFreedriveMode::Response &res);
        bool stopFreedriveModeCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

        // ---- UR RTDE FUNCTIONS ---- //
        void publishJointState();
        void publishTCPPose();
        void publishFTSensor();
        void publishTrajectoryExecuted();
        void readRobotSafetyStatus();

};

#endif /* POSITION_CONTROLLER_H */
