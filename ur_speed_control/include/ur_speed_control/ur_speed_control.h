#ifndef UR_SPEED_CONTROL_H
#define UR_SPEED_CONTROL_H

#include <signal.h>

#include <ros/ros.h>

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_io_interface.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Trigger.h>

#include "ur_speed_control/command_gripper.h"
#include "ur_speed_control/robot_status.h"

#include <Eigen/Dense>

class URSpeedControl {

    public:
    
      URSpeedControl(ros::NodeHandle &nh, ros::Rate ros_rate);

      ~URSpeedControl();

      void spinner();

    private:

        // ---- ROS - NODE HANDLE & RATE ---- //
        ros::NodeHandle nh_;
        ros::Rate ros_rate_;

        // ---- PARAMETERS ---- //
    	std::string ROBOT_IP;

        // ---- UR RTDE LIBRARY ---- //
        ur_rtde::RTDEControlInterface* rtde_control_;
        ur_rtde::RTDEReceiveInterface* rtde_receive_;
        ur_rtde::RTDEIOInterface* rtde_io_;

        // ---- GLOBAL VARIABLES ---- //
        geometry_msgs::TwistStamped desired_twist_;
        geometry_msgs::PoseStamped pose_;
    	std::vector<double> desired_twist_dbl_;
        std::vector<double> pose_dbl_;
        Eigen::Vector3d axis_;
        double angle_;
    	double max_acc_;
        bool move_ = false;

        // ---- ROS - PUBLISHERS ---- //
        ros::Publisher pose_pub_;
        ros::Publisher robot_status_pub_;

        // ---- ROS - SUBSCRIBERS & CALLBACKS ---- //
        ros::Subscriber twist_sub_;
        void twistCallback(const geometry_msgs::TwistStamped msg);

        // ---- ROS - SERVICE SERVERS & CALLBACKS ---- //
        ros::ServiceServer onrobot_gripper_service_;
        ros::ServiceServer stop_robot_service_;
        bool onRobotGripperCallback(ur_speed_control::command_gripper::Request  &req, ur_speed_control::command_gripper::Response &res);
        bool stopRobotCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

        // ---- UR RTDE FUNCTIONS ---- //
        void readRobotSafetyStatus();

};

#endif /* UR_SPEED_CONTROL_H */
