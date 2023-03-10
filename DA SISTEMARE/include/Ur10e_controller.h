#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

#include <Eigen/Dense>

#include <compute_kinematic/compute_ur_direct_kinematic.h>
#include <compute_jacobian/compute_ur_jacobian.h>
#include <compute_jacobian/compute_ur_jacobian_dot.h>

//Joints velocities low pass filter default cut frequencies
#define UR10E_CONTROLLER_J1_VELOCITY_FILTER_DEFAULT_CUT_FREQUENCY 5.0
#define UR10E_CONTROLLER_J2_VELOCITY_FILTER_DEFAULT_CUT_FREQUENCY 5.0
#define UR10E_CONTROLLER_J3_VELOCITY_FILTER_DEFAULT_CUT_FREQUENCY 1.0
#define UR10E_CONTROLLER_J4_VELOCITY_FILTER_DEFAULT_CUT_FREQUENCY 5.0
#define UR10E_CONTROLLER_J5_VELOCITY_FILTER_DEFAULT_CUT_FREQUENCY 20.0
#define UR10E_CONTROLLER_J6_VELOCITY_FILTER_DEFAULT_CUT_FREQUENCY 10.0
#define UR10E_CONTROLLER_J7_VELOCITY_FILTER_DEFAULT_CUT_FREQUENCY 20.0

#define UR10E_CONTROLLER_EXT_FORCE_FILTER_DEFAULT_CUT_FREQUENCY 100.0
#define UR10E_CONTROLLER_EXT_FORCE_LINEAR_FILTER_DEFAULT_DEAD_ZONE 3.0
#define UR10E_CONTROLLER_EXT_FORCE_ANGULAR_FILTER_DEFAULT_DEAD_ZONE 0.05

#include <atomic>
#include <thread>
#include <chrono>

//https://sdurobotics.gitlab.io/ur_rtde/installation/installation.html

class Ur10e_controller {

	public:

		Ur10e_controller(std::string address, double step_time);
		~Ur10e_controller();

		Eigen::Matrix<double, 6, 1> get_joint_position(void);
		Eigen::Matrix<double, 6, 1> get_joint_velocity(void);
		Eigen::Matrix<double, 4, 4> get_pose(void);
		Eigen::Matrix<double, 6, 6> get_jacobian(void);
		Eigen::Matrix<double, 6, 1> get_jacobian_dot(void);
		Eigen::Matrix<double, 6, 1> get_external_joint_torque(void);
		Eigen::Matrix<double, 6, 1> get_external_force(void);

		Eigen::Matrix<double, 4, 4> get_pose_at(Eigen::Matrix<double, 6, 1> joint_position);
		Eigen::Matrix<double, 6, 6> get_jacobian_at(Eigen::Matrix<double, 6, 1> joint_position);
		Eigen::Matrix<double, 6, 1> get_jacobian_dot_at(Eigen::Matrix<double, 6, 1> joint_position, Eigen::Matrix<double, 6, 1> joint_velocity);

		void set_joint_position(Eigen::Matrix<double, 6, 1> desired_joint_position, double velocity, double acceleration);
		void set_joint_velocity(Eigen::Matrix<double, 6, 1> desired_joint_velocity);

		void zero_force_sensor(void);

	private:

		ur_rtde::RTDEControlInterface *_rtde_control;
		ur_rtde::RTDEReceiveInterface *_rtde_receive;

		double _step_time;

		std::atomic<bool> _is_running;
		std::thread _update_thread;

		std::vector<double> _measured_joint_position_dbl, _desired_joint_position_dbl;
		float _old_measured_joint_position_dbl[6];
		Eigen::Matrix<double, 6, 1> _measured_joint_position;

		std::vector<double> _measured_joint_velocity_dbl, _desired_joint_velocity_dbl;
		Eigen::Matrix<double, 6, 1> _measured_joint_velocity;

		float _estimated_joint_velocity_dbl[6], _estimated_joint_velocity_filtered_dbl[6], _old_estimated_joint_velocity_filtered_dbl[6], _joint_velocity_cut_frequencies_dbl[6];
		Eigen::Matrix<double, 4, 4> _measured_pose;

		Eigen::Matrix<double, 6, 6> _measured_jacobian;

		Eigen::Matrix<double, 6, 1> _measured_jacobian_dot;

		std::vector<double> _measured_external_force_dbl;
		double _measured_external_force_filtered_dbl[6], _old_measured_external_force_filtered_dbl[6], _measured_external_force_offset_dbl[6];
		double _external_force_cut_frequencies_dbl, _external_force_l_dead_zone_dbl, _external_force_a_dead_zone_dbl;
		Eigen::Matrix<double, 6, 1> _measured_external_force, _measured_external_joint_torque;


		double low_pass_filter (double new_value, double prev_value, double dt, double cut_frequency);

		void update();
		void update_joint_position(void);
		void update_joint_velocity(void);
		void update_pose(void);
		void update_jacobian(void);
		void update_jacobian_dot(void);
		void update_external_force(void);

		template <typename T> int sgn(T val) {
			return (T(0) < val) - (val < T(0));
		}

};

Ur10e_controller::Ur10e_controller(std::string address, double step_time){

	_rtde_control = new ur_rtde::RTDEControlInterface(address);
	_rtde_receive = new ur_rtde::RTDEReceiveInterface(address);

	_step_time = step_time;

	_measured_joint_position_dbl.resize(6);
	_desired_joint_position_dbl.resize(6);
//	_measured_joint_position_dbl = _rtde_receive -> getActualQ();
//	std::copy(std::begin(_measured_joint_position_dbl), std::end(_measured_joint_position_dbl), std::begin(_old_measured_joint_position_dbl));

	_measured_joint_velocity_dbl.resize(6);
	_desired_joint_velocity_dbl.resize(6);

//	std::fill(std::begin(_estimated_joint_velocity_dbl), std::end(_estimated_joint_velocity_dbl), 0.0);
//	std::fill(std::begin(_estimated_joint_velocity_filtered_dbl), std::end(_estimated_joint_velocity_filtered_dbl), 0.0);
//	std::fill(std::begin(_old_estimated_joint_velocity_filtered_dbl), std::end(_old_estimated_joint_velocity_filtered_dbl), 0.0);

//	_joint_velocity_cut_frequencies_dbl[0] = J1_VELOCITY_FILTER_DEFAULT_CUT_FREQUENCY;
//	_joint_velocity_cut_frequencies_dbl[1] = J2_VELOCITY_FILTER_DEFAULT_CUT_FREQUENCY;
//	_joint_velocity_cut_frequencies_dbl[2] = J3_VELOCITY_FILTER_DEFAULT_CUT_FREQUENCY;
//	_joint_velocity_cut_frequencies_dbl[3] = J4_VELOCITY_FILTER_DEFAULT_CUT_FREQUENCY;
//	_joint_velocity_cut_frequencies_dbl[4] = J5_VELOCITY_FILTER_DEFAULT_CUT_FREQUENCY;
//	_joint_velocity_cut_frequencies_dbl[5] = J6_VELOCITY_FILTER_DEFAULT_CUT_FREQUENCY;
//	_joint_velocity_cut_frequencies_dbl[6] = J7_VELOCITY_FILTER_DEFAULT_CUT_FREQUENCY;

	_measured_external_force_dbl.resize(6);
	_measured_external_force_dbl = _rtde_receive -> getActualTCPForce();

	std::copy(std::begin(_measured_external_force_dbl), std::end(_measured_external_force_dbl), std::begin(_measured_external_force_offset_dbl));
	std::fill(std::begin(_measured_external_force_filtered_dbl), std::end(_measured_external_force_filtered_dbl), 0.0);
	std::fill(std::begin(_old_measured_external_force_filtered_dbl), std::end(_old_measured_external_force_filtered_dbl), 0.0);

	_external_force_cut_frequencies_dbl = UR10E_CONTROLLER_EXT_FORCE_FILTER_DEFAULT_CUT_FREQUENCY;

	_external_force_l_dead_zone_dbl = UR10E_CONTROLLER_EXT_FORCE_LINEAR_FILTER_DEFAULT_DEAD_ZONE;
	_external_force_a_dead_zone_dbl = UR10E_CONTROLLER_EXT_FORCE_ANGULAR_FILTER_DEFAULT_DEAD_ZONE;

	_is_running = true;
	_update_thread = std::thread(&Ur10e_controller::update, this);

}

void Ur10e_controller::update_joint_position(void){

	std::copy(std::begin(_measured_joint_position_dbl), std::end(_measured_joint_position_dbl), std::begin(_old_measured_joint_position_dbl));
	_measured_joint_position_dbl = _rtde_receive -> getActualQ();

	for (unsigned int i = 0; i < 6; i++){
		_measured_joint_position(i,0) = _measured_joint_position_dbl[i];
	}

}

void Ur10e_controller::update_joint_velocity(void){

	_measured_joint_velocity_dbl = _rtde_receive -> getActualQd();

	for (unsigned int i = 0; i < 6; i++){
		_measured_joint_velocity(i,0) = _measured_joint_velocity_dbl[i];
	}

}


//void Ur10e_controller::update_joint_velocity(void){

//	for (unsigned int i = 0; i < 7; i++){
//		_estimated_joint_velocity_dbl[i] = (_measured_joint_position_dbl[i] - _old_measured_joint_position_dbl[i]) / _step_time; 
//		_estimated_joint_velocity_filtered_dbl[i] = low_pass_filter(_estimated_joint_velocity_dbl[i], _old_estimated_joint_velocity_filtered_dbl[i],
//			_step_time, _joint_velocity_cut_frequencies_dbl[i]);
//	}

//	std::copy(std::begin(_estimated_joint_velocity_filtered_dbl), std::end(_estimated_joint_velocity_filtered_dbl), std::begin(_old_estimated_joint_velocity_filtered_dbl));

//	for (unsigned int i = 0; i < 7; i++){
//		_measured_joint_velocity(i,0) = _estimated_joint_velocity_filtered_dbl[i];
//	}

//}

void Ur10e_controller::update_pose(void){

	_measured_pose = compute_ur_direct_kinematic(_measured_joint_position);

}

void Ur10e_controller::update_jacobian(void){

	_measured_jacobian = compute_ur_jacobian(_measured_joint_position);

}

void Ur10e_controller::update_jacobian_dot(void){

	_measured_jacobian_dot = compute_ur_jacobian_dot(_measured_joint_position, _measured_joint_velocity);

}

void Ur10e_controller::update_external_force(void){

	_measured_external_force_dbl = _rtde_receive -> getActualTCPForce();

	for(int i=0; i<3; i++){
		_measured_external_force_dbl[i] -= _measured_external_force_offset_dbl[i];
		if(fabs(_measured_external_force_dbl[i]) < _external_force_l_dead_zone_dbl){
			_measured_external_force_dbl[i] = 0.0;
		}
		else{
			_measured_external_force_dbl[i] = sgn(_measured_external_force_dbl[i]) * (fabs(_measured_external_force_dbl[i]) - _external_force_l_dead_zone_dbl);
		}
	}

	for(int i=3; i<6; i++){
		_measured_external_force_dbl[i] -= _measured_external_force_offset_dbl[i];
		if(fabs(_measured_external_force_dbl[i]) < _external_force_a_dead_zone_dbl){
			_measured_external_force_dbl[i] = 0.0;
		}
		else{
			_measured_external_force_dbl[i] = sgn(_measured_external_force_dbl[i]) * (fabs(_measured_external_force_dbl[i]) - _external_force_a_dead_zone_dbl);
		}
	}

	for(int i=0; i<6; i++){
		_measured_external_force_filtered_dbl[i] = low_pass_filter (_measured_external_force_dbl[i], _old_measured_external_force_filtered_dbl[i],
																	_step_time, _external_force_cut_frequencies_dbl);
	}
	std::copy(std::begin(_measured_external_force_filtered_dbl), std::end(_measured_external_force_filtered_dbl), std::begin(_old_measured_external_force_filtered_dbl));

	for (unsigned int i = 0; i < 6; i++){
		_measured_external_force(i,0) = _measured_external_force_filtered_dbl[i];
	}

	_measured_external_joint_torque = _measured_jacobian.transpose() * _measured_external_force;

}

void Ur10e_controller::update(){

	while(_is_running)
	{
		this->update_joint_position();
		this->update_joint_velocity();
		this->update_pose();
		this->update_jacobian();
		this->update_jacobian_dot();
		this->update_external_force();
		std::this_thread::sleep_for(std::chrono::milliseconds((int)(1000.0 * this->_step_time))); 
	}

}

Eigen::Matrix<double, 6, 1> Ur10e_controller::get_joint_position(void){return _measured_joint_position;}
Eigen::Matrix<double, 6, 1> Ur10e_controller::get_joint_velocity(void){return _measured_joint_velocity;}
Eigen::Matrix<double, 4, 4> Ur10e_controller::get_pose(void){return _measured_pose;}
Eigen::Matrix<double, 6, 6> Ur10e_controller::get_jacobian(void){return _measured_jacobian;}
Eigen::Matrix<double, 6, 1> Ur10e_controller::get_jacobian_dot(void){return _measured_jacobian_dot;}
Eigen::Matrix<double, 6, 1> Ur10e_controller::get_external_force(void){return _measured_external_force;}
Eigen::Matrix<double, 6, 1> Ur10e_controller::get_external_joint_torque(void){return _measured_external_joint_torque;}

Eigen::Matrix<double, 4, 4> Ur10e_controller::get_pose_at(Eigen::Matrix<double, 6, 1> joint_position){

	return 	compute_ur_direct_kinematic(joint_position);

}

Eigen::Matrix<double, 6, 6> Ur10e_controller::get_jacobian_at(Eigen::Matrix<double, 6, 1> joint_position){

	return compute_ur_jacobian(joint_position);

}

Eigen::Matrix<double, 6, 1> Ur10e_controller::get_jacobian_dot_at(Eigen::Matrix<double, 6, 1> joint_position, Eigen::Matrix<double, 6, 1> joint_velocity){

	return compute_ur_jacobian_dot(joint_position, joint_velocity);

}

void Ur10e_controller::set_joint_position(Eigen::Matrix<double, 6, 1> desired_joint_position, double velocity, double acceleration){

	for (unsigned int i = 0; i < 6; i++){
		_desired_joint_position_dbl[i] = desired_joint_position(i,0);
	}

	_rtde_control -> moveJ(_desired_joint_position_dbl, velocity, acceleration, false);

}


void Ur10e_controller::set_joint_velocity(Eigen::Matrix<double, 6, 1> desired_joint_velocity){

	for (unsigned int i = 0; i < 6; i++){
		_desired_joint_velocity_dbl[i] = desired_joint_velocity(i,0);
	}

	_rtde_control -> speedJ(_desired_joint_velocity_dbl, 40.0, 0.002);

}

void Ur10e_controller::zero_force_sensor(void){

	_measured_external_force_dbl = _rtde_receive -> getActualTCPForce();

	std::copy(std::begin(_measured_external_force_dbl), std::end(_measured_external_force_dbl), std::begin(_measured_external_force_offset_dbl));
	std::fill(std::begin(_measured_external_force_filtered_dbl), std::end(_measured_external_force_filtered_dbl), 0.0);
	std::fill(std::begin(_old_measured_external_force_filtered_dbl), std::end(_old_measured_external_force_filtered_dbl), 0.0);

}

double Ur10e_controller::low_pass_filter(double new_value, double prev_value, double dt, double cut_frequency) {

	double ret_value;
	ret_value = prev_value + (new_value - prev_value) * (1 - exp(- 2 * M_PI * cut_frequency * dt));
	return ret_value;

}


Ur10e_controller::~Ur10e_controller(){ 

	_rtde_control -> stopScript();
	_rtde_control -> disconnect();
	_rtde_receive -> disconnect();

	delete _rtde_control;
	delete _rtde_receive;

	_is_running = false;
	_update_thread.join();

}



















