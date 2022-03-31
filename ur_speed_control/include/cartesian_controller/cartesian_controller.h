#ifndef CARTESIAN_CONTROLLER_H
#define CARTESIAN_CONTROLLER_H

#include "ros/ros.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <ur_speed_control/setDouble.h>

#include <Eigen/Dense>

using namespace Eigen;

class CartesianController {

    public:
    
      CartesianController(ros::NodeHandle &nh, ros::Rate ros_rate);

      ~CartesianController();

      void spinner();

    private:

        // ---- ROS - NODE HANDLE & RATE ---- //
        ros::NodeHandle nh_;
        ros::Rate ros_rate_;

        // ---- PARAMETERS ---- //
        double k_ = 2.0;
        double max_linear_vel_ = 0.1;
        double max_twist_vel_  = 0.1;
        double movement_precision_ = 0.0001;

        // ---- GLOBAL VARIABLES ---- //
        std_msgs::Bool trajectory_completed_;
        geometry_msgs::PoseStamped actual_pose_;
        geometry_msgs::PoseStamped desired_pose_;
        geometry_msgs::TwistStamped twist_cmd_;
        bool actual_pose_flag_  = false;
        bool desired_pose_flag_ = false;
        Eigen::Matrix<double, 4, 4> T_des_, T_;
        Eigen::Matrix<double, 6, 1> error_, velocity_;

        // ---- ROS - PUBLISHERS ---- //
        ros::Publisher twist_cmd_pub_;
        ros::Publisher trajectory_completed_pub_;

        // ---- ROS - SUBSCRIBERS & CALLBACKS ---- //
        ros::Subscriber actual_pose_sub_;
        ros::Subscriber desired_pose_sub_;
        void actualPoseCallback(const geometry_msgs::PoseStamped msg);
        void desiredPoseCallback(const geometry_msgs::PoseStamped msg);
        
        // ---- ROS - SERVICE CLIENTS ---- //
        ros::ServiceClient stop_robot_client_;

        // ---- ROS - SERVICE SERVERS & CALLBACKS ---- //
        ros::ServiceServer set_movement_precision_server_;
        bool setMovementPrecisionCallback(ur_speed_control::setDouble::Request  &req, ur_speed_control::setDouble::Response &res);

        // ---- EIGEN FUNCTIONS ---- //
        Eigen::Matrix<double, 6, 1> compute_pose_error(Eigen::Matrix<double, 4, 4> T_des, Eigen::Matrix<double, 4, 4> T);
        Eigen::Matrix<double, 4, 4> pose2eigen(geometry_msgs::PoseStamped pose);

        // ---- OTHER FUNCTIONS ---- //
        bool movementCompletedCheck (Eigen::Matrix<double, 6, 1> position_error);


};

#endif /* CARTESIAN_CONTROLLER_H */
