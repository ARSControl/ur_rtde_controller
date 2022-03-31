#include "cartesian_controller/cartesian_controller.h"

CartesianController::CartesianController(ros::NodeHandle &nh, ros::Rate ros_rate): nh_(nh), ros_rate_(ros_rate)
{

	// ---- LOAD PARAMETERS ---- //
	if(!nh_.param<double>("/cartesian_controller/k", k_, 2.0)) {ROS_ERROR_STREAM("Failed To Get \"position_gain\" Param. Usinge Default: " << k_);}
	if(!nh_.param<double>("/cartesian_controller/max_linear_vel", max_linear_vel_, 0.1)) {ROS_ERROR_STREAM("Failed To Get \"max_linear_vel\" Param. Usinge Default: " << max_linear_vel_);}
	if(!nh_.param<double>("/cartesian_controller/max_twist_vel", max_twist_vel_, 0.1)) {ROS_ERROR_STREAM("Failed To Get \"max_twist_vel\" Param. Usinge Default: " << max_twist_vel_);}
	if(!nh_.param<double>("/cartesian_controller/movement_precision", movement_precision_, 0.0001)) {ROS_ERROR_STREAM("Failed To Get \"movement_precision\" Param. Usinge Default: " << movement_precision_);}

  // ---- ROS - PUBLISHERS ---- //
  twist_cmd_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/twist_cmd", 1);
  trajectory_completed_pub_ = nh_.advertise<std_msgs::Bool>("/cartesian_controller/trajectory_completed", 1);

  // ---- ROS - SUBSCRIBERS ---- //
  actual_pose_sub_ = nh_.subscribe("/act_pose", 1, &CartesianController::actualPoseCallback, this);
  desired_pose_sub_ = nh_.subscribe("/des_pose", 1, &CartesianController::desiredPoseCallback, this);

	// ---- ROS - SERVICE CLIENTS ---- //
  stop_robot_client_ = nh_.serviceClient<std_srvs::Trigger>("/ur_rtde/stop_robot");

	// ---- ROS - SERVICE SERVERS ---- //
  set_movement_precision_server_ = nh_.advertiseService("/cartesian_controller/set_movement_precision", &CartesianController::setMovementPrecisionCallback, this);

  ros::Duration(2).sleep();

  // Wait Actual Position
  while (!actual_pose_flag_)
  {
    ROS_WARN_DELAYED_THROTTLE(2, "Waiting For Actual Robot Pose...");
    ros::spinOnce();
  }

  // Print Parameters
  std::cout << std::endl;
  ROS_INFO_STREAM("Position Gain k: " << k_);
  ROS_INFO_STREAM("Max Linear Velocity: " << max_linear_vel_);
  ROS_INFO_STREAM("Max Angular Velocity: " << max_twist_vel_);
  ROS_INFO_STREAM("Movement Precision: " << movement_precision_);

  std::cout << std::endl;
  ROS_INFO("Cartesian Controller - Started\n");

}

CartesianController::~CartesianController() {}

void CartesianController::actualPoseCallback(const geometry_msgs::PoseStamped msg)
{

  actual_pose_ = msg;
  actual_pose_flag_ = true;

}

void CartesianController::desiredPoseCallback(const geometry_msgs::PoseStamped msg)
{

  desired_pose_ = msg;
  desired_pose_flag_ = true;

}

bool CartesianController::setMovementPrecisionCallback(ur_speed_control::setDouble::Request  &req, ur_speed_control::setDouble::Response &res)
{

  ROS_WARN_STREAM("Movement Precision Changed To: " << req.data);
  movement_precision_ = req.data;
	res.success = true;
	return res.success;

}

Eigen::Matrix<double, 6, 1> CartesianController::compute_pose_error(Eigen::Matrix<double, 4, 4> T_des, Eigen::Matrix<double, 4, 4> T)
{

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

Eigen::Matrix<double, 4, 4> CartesianController::pose2eigen(geometry_msgs::PoseStamped pose)
{

  Eigen::Matrix<double, 4, 4> T = Eigen::Matrix<double, 4, 4>::Identity();

  T(0, 3) = pose.pose.position.x;
  T(1, 3) = pose.pose.position.y;
  T(2, 3) = pose.pose.position.z;

  Eigen::Quaterniond q(pose.pose.orientation.w, pose.pose.orientation.x,
                       pose.pose.orientation.y, pose.pose.orientation.z);

  T.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();

  return T;

}

bool CartesianController::movementCompletedCheck (Eigen::Matrix<double, 6, 1> position_error)
{

  if ((Eigen::abs(position_error.array()) < movement_precision_).all())
  {

    std::cout << std::endl;
    ROS_INFO_STREAM("Position Reached - Movement Completed | Position Error: ");
    std::cout << Eigen::abs(position_error.array()) << std::endl << std::endl;

    std_srvs::Trigger stop_robot_srv;
    if (!stop_robot_client_.call(stop_robot_srv)) {ROS_ERROR("Failed to Call Service: \"/ur_rtde/stop_robot/\"");}
    if (!stop_robot_srv.response.success) {ROS_ERROR("Failed to Stop Robot");}

    // Trajectory Completed
    trajectory_completed_.data = true;

    return true;

  }

  // Not Trajectory Completed
  trajectory_completed_.data = false;

  return false;

}

void CartesianController::spinner()
{

  while (ros::ok() && !desired_pose_flag_)
  {
    
    ROS_WARN_DELAYED_THROTTLE(2, "Waiting For Desired Robot Pose...");
    ros::spinOnce();
    
    if (desired_pose_flag_) {std::cout << std::endl;}

  }

  T_des_ = pose2eigen(desired_pose_);
  T_ = pose2eigen(actual_pose_);
  error_ = -compute_pose_error(T_des_, T_);

  // Velocity Setpoint
  velocity_ = k_ * error_;

  // Set Velocity Limits
  for (int i = 0; i < 3; i++) {
    if (fabs(velocity_(i, 0)) > max_linear_vel_) {velocity_(i, 0) = velocity_(i, 0) / fabs(velocity_(i, 0) + 1e-12) * max_linear_vel_;}
    if (fabs(velocity_(i + 3, 0)) > max_twist_vel_) {velocity_(i + 3, 0) = velocity_(i + 3, 0) / fabs(velocity_(i + 3, 0) + 1e-12) * max_twist_vel_;}
  }

  // Convert Eigen Matrix to Vector for Printing
  std::vector<double> error_vec_(error_.data(), error_.data() + error_.rows() * error_.cols());
  std::vector<double> velocity_vec_(velocity_.data(), velocity_.data() + velocity_.rows() * velocity_.cols());

  ROS_INFO_STREAM_THROTTLE(5, "Position Error: " << error_vec_[0] << " " << error_vec_[1] << " " << error_vec_[2]
                                          << " " << error_vec_[3] << " " << error_vec_[4] << " " << error_vec_[5]);

  ROS_INFO_STREAM_THROTTLE(5, "Velocity: " << velocity_vec_[0] << " " << velocity_vec_[1] << " " << velocity_vec_[2]
                                    << " " << velocity_vec_[3] << " " << velocity_vec_[4] << " " << velocity_vec_[5] << "\n");

  twist_cmd_.twist.linear.x = velocity_(0, 0);
  twist_cmd_.twist.linear.y = velocity_(1, 0);
  twist_cmd_.twist.linear.z = velocity_(2, 0);

  twist_cmd_.twist.angular.x = velocity_(3, 0);
  twist_cmd_.twist.angular.y = velocity_(4, 0);
  twist_cmd_.twist.angular.z = velocity_(5, 0);

  twist_cmd_pub_.publish(twist_cmd_);

  ros::spinOnce();

  desired_pose_flag_ = !movementCompletedCheck(error_);
  trajectory_completed_pub_.publish(trajectory_completed_);

  ros_rate_.sleep();

}
