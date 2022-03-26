#include "ros/ros.h"
#include "Ur10e_controller.h"
#include "compute_jacobian/compute_whole_body_jacobian.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/WrenchStamped.h"
#include <tf/tf.h>

double low_pass_filter (double new_value, double prev_value, double dt, double cut_frequency){

	double ret_value;
	ret_value = prev_value + (new_value - prev_value) * (1 - exp(- 2 * M_PI * cut_frequency * dt));
	return ret_value;

}

double sign(double value){
	if(value>0.0)
		return 1.0;
	else
		return -1.0;
}

//Eigen::Matrix<double, 2, 2> T_osfl;
Eigen::Matrix<double, 4, 4> T_mir_wrt_odom;
Eigen::Matrix<double, 6, 6> F_mir_to_odom;
double yaw;

double b = 0.312;

bool mir_callback_flag;

void odom_callback(const nav_msgs::Odometry msg){

	Eigen::Quaterniond q;
	q.x() = msg.pose.pose.orientation.x;
	q.y() = msg.pose.pose.orientation.y;
	q.z() = msg.pose.pose.orientation.z;
	q.w() = msg.pose.pose.orientation.w;

	T_mir_wrt_odom = Eigen::Matrix<double, 4, 4>::Identity(4,4);
	T_mir_wrt_odom.block<3,3>(0,0) = q.normalized().toRotationMatrix();
	T_mir_wrt_odom(0,3) = msg.pose.pose.position.x;
	T_mir_wrt_odom(1,3) = msg.pose.pose.position.y;
	T_mir_wrt_odom(2,3) = msg.pose.pose.position.z;

	F_mir_to_odom.block<3,3>(0,0) = T_mir_wrt_odom.block<3,3>(0,0);
	F_mir_to_odom.block<3,3>(0,3) = Eigen::Matrix<double, 3, 3>::Zero(3,3);
	F_mir_to_odom.block<3,3>(3,0) = Eigen::Matrix<double, 3, 3>::Zero(3,3);
	F_mir_to_odom.block<3,3>(3,3) = T_mir_wrt_odom.block<3,3>(0,0);

	Eigen::Matrix<double, 3, 1> euler = T_mir_wrt_odom.block<3,3>(0,0).eulerAngles(0,1,2);
	yaw = euler(2,0);
//	tf::Quaternion q(
//		msg.pose.pose.orientation.x,
//		msg.pose.pose.orientation.y,
//		msg.pose.pose.orientation.z,
//		msg.pose.pose.orientation.w);
//	tf::Matrix3x3 m(q);
//	double roll, pitch, yaw;
//	m.getRPY(roll, pitch, yaw);

//	T_osfl(0,0) = cos(yaw);
//	T_osfl(0,1) = sin(yaw);
//	T_osfl(1,0) = - 1.0/ b * sin(yaw);
//	T_osfl(1,1) = 1.0 / b * cos(yaw);

//	mir_joint_position << msg.pose.pose.position.x, msg.pose.pose.position.y, yaw;
	mir_callback_flag = true;
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "whole_body_admittance");
	ros::NodeHandle n;
	ros::Rate loop_rate(500);

	Ur10e_controller ur_controller("192.168.2.30", 0.002);

	ros::Subscriber odom_sub = n.subscribe("odom", 1, odom_callback);
	ros::Publisher mir_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

	ros::Publisher wrench_pub = n.advertise<geometry_msgs::WrenchStamped>("debug_wrech", 1);
	geometry_msgs::WrenchStamped debug_wrench;

	mir_callback_flag = false;

	while(mir_callback_flag == false){ros::spinOnce();};

	Eigen::Matrix<double, 6, 1> external_force;

	Eigen::Matrix<double, 6, 6> model_M_cart = Eigen::Matrix<double, 6, 6>::Zero(6,6);
	model_M_cart(0,0) = 5.0;
	model_M_cart(1,1) = 5.0;
	model_M_cart(2,2) = 5.0;
	model_M_cart(3,3) = 0.05;
	model_M_cart(4,4) = 0.05;
	model_M_cart(5,5) = 0.05;

	Eigen::Matrix<double, 6, 6> model_D_cart = Eigen::Matrix<double, 6, 6>::Zero(6,6);
	model_D_cart(0,0) = 15.0;
	model_D_cart(1,1) = 15.0;
	model_D_cart(2,2) = 15.0;
	model_D_cart(3,3) = 0.3;
	model_D_cart(4,4) = 0.3;
	model_D_cart(5,5) = 0.3;

	Eigen::Matrix<double, 6, 1> ur_joint_position;
	ur_joint_position = ur_controller.get_joint_position();

	Eigen::Matrix<double, 7, 1> whole_body_joint_position;
	Eigen::Matrix<double, 8, 1> whole_body_joint_velocity;
	Eigen::Matrix<double, 6, 1> whole_body_cartesian_velocity = Eigen::Matrix<double, 6, 1>::Zero(6,1);
	Eigen::Matrix<double, 6, 1> whole_body_cartesian_acceleration = Eigen::Matrix<double, 6, 1>::Zero(6,1);
	Eigen::Matrix<double, 6, 1> whole_body_cartesian_acceleration_old = Eigen::Matrix<double, 6, 1>::Zero(6,1);
	Eigen::Matrix<double, 6, 8> whole_body_jacobian;

	whole_body_joint_position(0,0) = 0.0;
	whole_body_joint_position.block<6,1>(1,0) = ur_joint_position;

	Eigen::Matrix<double, 4, 4> T_ur_base_wrt_mir;
	T_ur_base_wrt_mir.block<3,3>(0,0) = Eigen::Matrix<double, 3, 3> (Eigen::AngleAxisd(M_PI/4.0, Eigen::Vector3d::UnitZ()));
	T_ur_base_wrt_mir(0,3) = 0.312;
	T_ur_base_wrt_mir(1,3) = 0.0;
	T_ur_base_wrt_mir(2,3) = 0.85;

	Eigen::Matrix<double, 6, 6> F_ur_base_to_mir;
	F_ur_base_to_mir.block<3,3>(0,0) = T_ur_base_wrt_mir.block<3,3>(0,0);
	F_ur_base_to_mir.block<3,3>(0,3) = Eigen::Matrix<double, 3, 3>::Zero(3,3);
	F_ur_base_to_mir.block<3,3>(3,0) = Eigen::Matrix<double, 3, 3>::Zero(3,3);
	F_ur_base_to_mir.block<3,3>(3,3) = T_ur_base_wrt_mir.block<3,3>(0,0);

	Eigen::Matrix<double, 2, 1> v_mir;
	geometry_msgs::Twist mir_cmd;

	while (ros::ok()){

//		external_force = F_mir_to_odom * F_ur_base_to_mir * ur_controller.get_external_force();
		external_force = F_ur_base_to_mir * ur_controller.get_external_force();
		external_force(3,0) = 0.0;
		external_force(4,0) = 0.0;
		external_force(5,0) = 0.0;

		for(int i=0; i<6; i++)
			if(fabs(external_force(i,0)) > 5.0)
				external_force(i,0) = sign(external_force(i,0)) * 5.0;
//		external_force(0,0) = 1.0;
//		external_force(1,0) = 0.0;
//		external_force(2,0) = 0.0;

		whole_body_joint_position(0,0) = 0.0;
		whole_body_joint_position.block<6,1>(1,0) = ur_controller.get_joint_position();
		whole_body_jacobian = compute_whole_body_jacobian(whole_body_joint_position);

		whole_body_cartesian_acceleration_old = whole_body_cartesian_acceleration;
		whole_body_cartesian_acceleration = model_M_cart.inverse() * (external_force - model_D_cart * whole_body_cartesian_velocity);
		for(int i=0; i<6; i++)
			whole_body_cartesian_acceleration(i,0) = low_pass_filter(whole_body_cartesian_acceleration(i,0), whole_body_cartesian_acceleration_old(i,0), 0.002, 100.0);

		whole_body_cartesian_velocity = whole_body_cartesian_velocity + whole_body_cartesian_acceleration * 0.002;
		whole_body_joint_velocity = whole_body_jacobian.transpose() * (whole_body_jacobian * whole_body_jacobian.transpose()).inverse() * whole_body_cartesian_velocity;
//		whole_body_joint_position = whole_body_joint_position + whole_body_joint_velocity * 0.002;

		v_mir << whole_body_joint_velocity(0,0), whole_body_joint_velocity(1,0);
//		v_mir << 0.0,0.05;
//		v_mir = T_osfl * v_mir;
		mir_cmd.linear.x = v_mir(0,0);
		mir_cmd.angular.z = -v_mir(1,0);

		mir_pub.publish(mir_cmd);
		ur_controller.set_joint_velocity(whole_body_joint_velocity.block<6,1>(2,0));

		std::cout << std::endl << std::endl << std::endl << "mir speed:" << std::endl << v_mir << std::endl;
		std::cout << std::endl << "ur speed:" << std::endl << whole_body_joint_velocity << std::endl;

//		std::cout << 0.0 << std::endl;

		debug_wrench.header.stamp = ros::Time::now();
		debug_wrench.header.frame_id = "mir/base_footprint";
		debug_wrench.wrench.force.x = external_force(0,0);
		debug_wrench.wrench.force.y = external_force(1,0);
		debug_wrench.wrench.force.z = external_force(2,0);
		debug_wrench.wrench.torque.x = external_force(3,0);
		debug_wrench.wrench.torque.y = external_force(4,0);
		debug_wrench.wrench.torque.z = external_force(5,0);
		wrench_pub.publish(debug_wrench);

//		desired_twist_dbl[0] = desired_twist.twist.linear.x;
//		desired_twist_dbl[1] = desired_twist.twist.linear.y;
//		desired_twist_dbl[2] = desired_twist.twist.linear.z;
//		desired_twist_dbl[3] = desired_twist.twist.angular.x;
//		desired_twist_dbl[4] = desired_twist.twist.angular.y;
//		desired_twist_dbl[5] = desired_twist.twist.angular.z;

//		rtde_control -> speedL(desired_twist_dbl, 0.25, 0.0);

//		pose_dbl = rtde_receive->getTargetTCPPose();
//		angle = sqrt(pow(pose_dbl[3],2) + pow(pose_dbl[4],2) + pow(pose_dbl[5],2));
//		axis << pose_dbl[3], pose_dbl[4], pose_dbl[5]; axis = axis.normalized(); 

//		pose.pose.position.x = pose_dbl[0];
//		pose.pose.position.y = pose_dbl[1];
//		pose.pose.position.z = pose_dbl[2];

//		pose.pose.orientation.x = Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis)).x();
//		pose.pose.orientation.y = Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis)).y();
//		pose.pose.orientation.z = Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis)).z();
//		pose.pose.orientation.w = Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis)).w();

//		pose_pub.publish(pose);

		ros::spinOnce();
		loop_rate.sleep();

	}

//	rtde_control -> forceModeStop();
//	rtde_control->disconnect();
	return 0;
}


