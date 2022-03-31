#include "ros/ros.h"
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <std_msgs/Bool.h>
#include <ur_speed_control/setDouble.h>

using namespace Eigen;

geometry_msgs::PoseStamped actual_pose;
bool actual_pose_flag;

geometry_msgs::PoseStamped desired_pose;
bool desired_pose_flag;

std_msgs::Bool trajectory_completed;
double movement_precision = 0.0001;

void actual_pose_callback(const geometry_msgs::PoseStamped msg)
{
  actual_pose = msg;
  actual_pose_flag = true;
}

void desired_pose_callback(const geometry_msgs::PoseStamped msg)
{
  desired_pose = msg;
  desired_pose_flag = true;
}

bool setMovementPrecisionCallback(ur_speed_control::setDouble::Request  &req, ur_speed_control::setDouble::Response &res) {

  ROS_WARN_STREAM("Movement Precision Changed To: " << req.data);
  movement_precision = req.data;
	res.success = true;
	return res.success;

}

Eigen::Matrix<double, 6, 1> compute_pose_error(Eigen::Matrix<double, 4, 4> T_des, Eigen::Matrix<double, 4, 4> T) {

  Eigen::Matrix<double, 6, 1> err;

  err.block<3, 1>(0, 0) = T.block<3, 1>(0, 3) - T_des.block<3, 1>(0, 3);

  Eigen::Quaterniond orientation_quat_des = Eigen::Quaterniond(T_des.block<3, 3>(0, 0));
  Eigen::Quaterniond orientation_quat = Eigen::Quaterniond(T.block<3, 3>(0, 0));

  if (orientation_quat_des.coeffs().dot(orientation_quat.coeffs()) < 0.0) {orientation_quat.coeffs() << -orientation_quat.coeffs();}

  Eigen::Quaterniond orientation_quat_error(orientation_quat.inverse() * orientation_quat_des);

  err.block<3, 1>(3, 0) << orientation_quat_error.x(), orientation_quat_error.y(), orientation_quat_error.z();
  err.block<3, 1>(3, 0) << -T.block<3, 3>(0, 0) * err.block<3, 1>(3, 0);

  return err;

}

Eigen::Matrix<double, 4, 4> pose2eigen(geometry_msgs::PoseStamped pose) {

  Eigen::Matrix<double, 4, 4> T = Eigen::Matrix<double, 4, 4>::Identity();

  T(0, 3) = pose.pose.position.x;
  T(1, 3) = pose.pose.position.y;
  T(2, 3) = pose.pose.position.z;

  Eigen::Quaterniond q(pose.pose.orientation.w, pose.pose.orientation.x,
                       pose.pose.orientation.y, pose.pose.orientation.z);

  T.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();

  return T;

}

bool movementCompletedCheck (Eigen::Matrix<double, 6, 1> position_error)
{


  if ((Eigen::abs(position_error.array()) < movement_precision).all())
  {

    std::cout << std::endl;
    ROS_INFO_STREAM("Position Reached - Movement Completed | Position Error: ");
    std::cout << Eigen::abs(position_error.array()) << std::endl << std::endl;

    // Trajectory Completed
    trajectory_completed.data = true;

    return true;

  }

  // Not Trajectory Completed
  trajectory_completed.data = false;

  return false;

}

int main (int argc, char **argv) {

  ros::init(argc, argv, "cartesian_controller");
  ros::NodeHandle n;
  ros::Rate loop_rate(500);

  ros::Publisher twist_cmd_pub = n.advertise<geometry_msgs::TwistStamped>("/twist_cmd", 1);
  ros::Publisher trajectory_completed_pub = n.advertise<std_msgs::Bool>("/cartesian_controller/trajectory_completed", 1);

  ros::Subscriber actual_pose_sub = n.subscribe("/act_pose", 1, &actual_pose_callback);
  ros::Subscriber desired_pose_sub = n.subscribe("/des_pose", 1, &desired_pose_callback);

  ros::ServiceServer set_movement_precision_server = n.advertiseService("/cartesian_controller/set_movement_precision", &setMovementPrecisionCallback);

  Eigen::Matrix<double, 4, 4> T_des, T;
  Eigen::Matrix<double, 6, 1> error, velocity;

  geometry_msgs::TwistStamped twist_cmd;

  actual_pose_flag = false;
  desired_pose_flag = false;

  ros::Duration(2).sleep();

  while (!actual_pose_flag)
  {

    ROS_WARN_DELAYED_THROTTLE(2, "Waiting For Actual Robot Pose...");
    ros::spinOnce();
  
  };

	// Get Params
  double k, max_linear_vel, max_twist_vel;
	if(!n.param<double>("/cartesian_controller/k", k, 2.0)) {ROS_ERROR_STREAM("Failed To Get \"position_gain\" Param. Usinge Default: " << k);}
	if(!n.param<double>("/cartesian_controller/max_linear_vel", max_linear_vel, 0.1)) {ROS_ERROR_STREAM("Failed To Get \"max_linear_vel\" Param. Usinge Default: " << max_linear_vel);}
	if(!n.param<double>("/cartesian_controller/max_twist_vel", max_twist_vel, 0.1)) {ROS_ERROR_STREAM("Failed To Get \"max_twist_vel\" Param. Usinge Default: " << max_twist_vel);}
	if(!n.param<double>("/cartesian_controller/movement_precision", movement_precision, 0.0001)) {ROS_ERROR_STREAM("Failed To Get \"movement_precision\" Param. Usinge Default: " << movement_precision);}

  std::cout << std::endl;
  ROS_INFO_STREAM("Position Gain k: " << k);
  ROS_INFO_STREAM("Max Linear Velocity: " << max_linear_vel);
  ROS_INFO_STREAM("Max Angular Velocity: " << max_twist_vel);
  ROS_INFO_STREAM("Movement Precision: " << movement_precision);

  std::cout << std::endl;
  ROS_INFO("Cartesian Controller - Started\n");

  while (ros::ok()) {

    while (ros::ok() && !desired_pose_flag)
    {
     
      ROS_WARN_DELAYED_THROTTLE(2, "Waiting For Desired Robot Pose...");
      ros::spinOnce();
     
      if (desired_pose_flag) {std::cout << std::endl;}

    }

    T_des = pose2eigen(desired_pose);
    T = pose2eigen(actual_pose);
    error = -compute_pose_error(T_des, T);

    // Velocity Setpoint
    velocity = k * error;

    // Set Velocity Limits
    for (int i = 0; i < 3; i++) {
      if (fabs(velocity(i, 0)) > max_linear_vel) {velocity(i, 0) = velocity(i, 0) / fabs(velocity(i, 0) + 1e-12) * max_linear_vel;}
      if (fabs(velocity(i + 3, 0)) > max_twist_vel) {velocity(i + 3, 0) = velocity(i + 3, 0) / fabs(velocity(i + 3, 0) + 1e-12) * max_twist_vel;}
    }

    // Convert Eigen Matrix to Vector for Printing
    std::vector<double> error_vec(error.data(), error.data() + error.rows() * error.cols());
    std::vector<double> velocity_vec(velocity.data(), velocity.data() + velocity.rows() * velocity.cols());

    ROS_INFO_STREAM_THROTTLE(5, "Position Error: " << error_vec[0] << " " << error_vec[1] << " " << error_vec[2]
                                            << " " << error_vec[3] << " " << error_vec[4] << " " << error_vec[5]);

    ROS_INFO_STREAM_THROTTLE(5, "Velocity: " << velocity_vec[0] << " " << velocity_vec[1] << " " << velocity_vec[2]
                                      << " " << velocity_vec[3] << " " << velocity_vec[4] << " " << velocity_vec[5] << "\n");

    twist_cmd.twist.linear.x = velocity(0, 0);
    twist_cmd.twist.linear.y = velocity(1, 0);
    twist_cmd.twist.linear.z = velocity(2, 0);

    twist_cmd.twist.angular.x = velocity(3, 0);
    twist_cmd.twist.angular.y = velocity(4, 0);
    twist_cmd.twist.angular.z = velocity(5, 0);

    twist_cmd_pub.publish(twist_cmd);

    ros::spinOnce();

    desired_pose_flag = !movementCompletedCheck(error);
    trajectory_completed_pub.publish(trajectory_completed);

    loop_rate.sleep();

  }

  return 0;

}
