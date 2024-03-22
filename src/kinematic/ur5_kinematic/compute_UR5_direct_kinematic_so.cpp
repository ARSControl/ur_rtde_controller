#include <Eigen/Dense>

// Eigen::Matrix<double, 4, 4> compute_UR5_direct_kinematic(Eigen::Matrix<double, 6, 1> q){
extern "C" void compute_UR5_direct_kinematic(double* result, double *q) {

Eigen::Matrix<double, 4, 4> T;

double q0 = q[0];
double q1 = q[1];
double q2 = q[2];
double q3 = q[3];
double q4 = q[4];
double q5 = q[5];

T(0,0) = cos(q5)*(sin(q0)*sin(q4)+cos(q1+q2+q3)*cos(q0)*cos(q4))-sin(q1+q2+q3)*cos(q0)*sin(q5)*1.0;

T(0,1) = sin(q5)*(sin(q0)*sin(q4)+cos(q1+q2+q3)*cos(q0)*cos(q4))*-1.0-sin(q1+q2+q3)*cos(q0)*cos(q5)*1.0;

T(0,2) = cos(q4)*sin(q0)-cos(q1+q2+q3)*cos(q0)*sin(q4);

T(0,3) = sin(q0)*1.0915E-1+cos(q4)*sin(q0)*8.23E-2+sin(q1+q2+q3)*cos(q0)*9.465E-2-cos(q0)*(cos(q1+q2)*1.569E+3+cos(q1)*1.7E+3)*2.5E-4-cos(q1+q2+q3)*cos(q0)*sin(q4)*8.23E-2;

T(1,0) = cos(q5)*(cos(q0)*sin(q4)*1.0-cos(q1+q2+q3)*cos(q4)*sin(q0)*1.0)*-1.0-sin(q1+q2+q3)*sin(q0)*sin(q5)*1.0;

T(1,1) = sin(q5)*(cos(q0)*sin(q4)*1.0-cos(q1+q2+q3)*cos(q4)*sin(q0)*1.0)-sin(q1+q2+q3)*cos(q5)*sin(q0)*1.0;

T(1,2) = -cos(q0)*cos(q4)-cos(q1+q2+q3)*sin(q0)*sin(q4);

T(1,3) = cos(q0)*-1.0915E-1-cos(q0)*cos(q4)*8.23E-2+sin(q1+q2+q3)*sin(q0)*9.465E-2-sin(q0)*(cos(q1+q2)*1.569E+3+cos(q1)*1.7E+3)*2.5E-4-cos(q1+q2+q3)*sin(q0)*sin(q4)*8.23E-2;

T(2,0) = cos(q1+q2+q3)*sin(q5)+sin(q1+q2+q3)*cos(q4)*cos(q5);

T(2,1) = cos(q1+q2+q3)*cos(q5)-sin(q1+q2+q3)*cos(q4)*sin(q5)*1.0;

T(2,2) = sin(q1+q2+q3)*sin(q4)*-1.0;

T(2,3) = cos(q1+q2+q3)*-9.465E-2-sin(q1+q2)*3.9225E-1-sin(q1)*4.25E-1-sin(q1+q2+q3)*sin(q4)*8.23E-2+8.9159E-2;

T(3,0) = 0.0;

T(3,1) = 0.0;

T(3,2) = 0.0;

T(3,3) = 1.0;

// return T;

int index = 0;
for (int row = 0; row < T.rows(); ++row) {
	for (int col = 0; col < T.cols(); ++col) {
		result[index++] = T(row, col);
	}
}

}
