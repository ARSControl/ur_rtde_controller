#include <Eigen/Dense>

// Eigen::Matrix<double, 4, 4> compute_UR10_direct_kinematic(Eigen::Matrix<double, 6, 1> q){
extern "C" void compute_UR10_direct_kinematic(double* result, double *q) {

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

T(0,3) = sin(q0)*1.63941E-1+cos(q4)*sin(q0)*9.22E-2+sin(q1+q2+q3)*cos(q0)*1.157E-1-cos(q0)*(cos(q1+q2)*5.723E+3+cos(q1)*6.12E+3)*1.0E-4-cos(q1+q2+q3)*cos(q0)*sin(q4)*9.22E-2;

T(1,0) = cos(q5)*(cos(q0)*sin(q4)*1.0-cos(q1+q2+q3)*cos(q4)*sin(q0)*1.0)*-1.0-sin(q1+q2+q3)*sin(q0)*sin(q5)*1.0;

T(1,1) = sin(q5)*(cos(q0)*sin(q4)*1.0-cos(q1+q2+q3)*cos(q4)*sin(q0)*1.0)-sin(q1+q2+q3)*cos(q5)*sin(q0)*1.0;

T(1,2) = -cos(q0)*cos(q4)-cos(q1+q2+q3)*sin(q0)*sin(q4);

T(1,3) = cos(q0)*-1.63941E-1-cos(q0)*cos(q4)*9.22E-2+sin(q1+q2+q3)*sin(q0)*1.157E-1-sin(q0)*(cos(q1+q2)*5.723E+3+cos(q1)*6.12E+3)*1.0E-4-cos(q1+q2+q3)*sin(q0)*sin(q4)*9.22E-2;

T(2,0) = cos(q1+q2+q3)*sin(q5)+sin(q1+q2+q3)*cos(q4)*cos(q5);

T(2,1) = cos(q1+q2+q3)*cos(q5)-sin(q1+q2+q3)*cos(q4)*sin(q5)*1.0;

T(2,2) = sin(q1+q2+q3)*sin(q4)*-1.0;

T(2,3) = cos(q1+q2+q3)*-1.157E-1-sin(q1+q2)*5.723E-1-sin(q1)*6.12E-1-sin(q1+q2+q3)*sin(q4)*9.22E-2+1.273E-1;

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
