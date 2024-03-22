#include <Eigen/Dense>

// Eigen::Matrix<double, 6, 6> compute_UR10e_jacobian(Eigen::Matrix<double, 6, 1> q){
extern "C" void compute_UR10e_jacobian(double* result, double *q) {

Eigen::Matrix<double, 6, 6> J;

J.setZero();

double q0 = q[0];
double q1 = q[1];
double q2 = q[2];
double q3 = q[3];
double q4 = q[4];
double q5 = q[5];

J(0,0) = cos(q0)*1.7415E-1+cos(q0)*cos(q4)*1.1655E-1-sin(q1+q2+q3)*sin(q0)*1.1985E-1+sin(q0)*(cos(q1+q2)*1.1431E+4+cos(q1)*1.2254E+4)*5.0E-5+cos(q1+q2+q3)*sin(q0)*sin(q4)*1.1655E-1;

J(0,1) = (cos(q0)*(cos(q1+q2+q3)*2.397E+3+sin(q1+q2)*1.1431E+4+sin(q1)*1.2254E+4+sin(q1+q2+q3)*sin(q4)*2.331E+3))/2.0E+4;

J(0,2) = (cos(q0)*(cos(q1+q2+q3)*2.397E+3+sin(q1+q2)*1.1431E+4+sin(q1+q2+q3)*sin(q4)*2.331E+3))/2.0E+4;

J(0,3) = cos(q0)*(cos(q1+q2+q3)*7.99E+2+sin(q1+q2+q3)*sin(q4)*7.77E+2)*1.5E-4;

J(0,4) = sin(q0)*sin(q4)*-1.1655E-1-cos(q1+q2+q3)*cos(q0)*cos(q4)*1.1655E-1;

J(0,5) = 0.0;

J(1,0) = sin(q0)*1.7415E-1+cos(q4)*sin(q0)*1.1655E-1+sin(q1+q2+q3)*cos(q0)*1.1985E-1-cos(q0)*(cos(q1+q2)*1.1431E+4+cos(q1)*1.2254E+4)*5.0E-5-cos(q1+q2+q3)*cos(q0)*sin(q4)*1.1655E-1;

J(1,1) = (sin(q0)*(cos(q1+q2+q3)*2.397E+3+sin(q1+q2)*1.1431E+4+sin(q1)*1.2254E+4+sin(q1+q2+q3)*sin(q4)*2.331E+3))/2.0E+4;

J(1,2) = (sin(q0)*(cos(q1+q2+q3)*2.397E+3+sin(q1+q2)*1.1431E+4+sin(q1+q2+q3)*sin(q4)*2.331E+3))/2.0E+4;

J(1,3) = sin(q0)*(cos(q1+q2+q3)*7.99E+2+sin(q1+q2+q3)*sin(q4)*7.77E+2)*1.5E-4;

J(1,4) = cos(q0)*sin(q4)*1.1655E-1-cos(q1+q2+q3)*cos(q4)*sin(q0)*1.1655E-1;

J(1,5) = 0.0;

J(2,0) = 0.0;

J(2,1) = sin(q1+q2+q3)*1.1985E-1-cos(q1+q2)*5.7155E-1-cos(q1)*6.127E-1-cos(q1+q2+q3)*sin(q4)*1.1655E-1;

J(2,2) = sin(q1+q2+q3)*1.1985E-1-cos(q1+q2)*5.7155E-1-cos(q1+q2+q3)*sin(q4)*1.1655E-1;

J(2,3) = sin(q1+q2+q3)*1.1985E-1-cos(q1+q2+q3)*sin(q4)*1.1655E-1;

J(2,4) = sin(q1+q2+q3)*cos(q4)*-1.1655E-1;

J(2,5) = 0.0;

J(3,0) = 0.0;

J(3,1) = sin(q0);

J(3,2) = sin(q0);

J(3,3) = sin(q0);

J(3,4) = sin(q1+q2+q3)*cos(q0);

J(3,5) = cos(q4)*sin(q0)-cos(q1+q2+q3)*cos(q0)*sin(q4);

J(4,0) = 0.0;

J(4,1) = cos(q0)*-1.0;

J(4,2) = cos(q0)*-1.0;

J(4,3) = cos(q0)*-1.0;

J(4,4) = sin(q1+q2+q3)*sin(q0);

J(4,5) = -cos(q0)*cos(q4)-cos(q1+q2+q3)*sin(q0)*sin(q4);

J(5,0) = pow(cos(q4)*sin(q0)-cos(q1+q2+q3)*cos(q0)*sin(q4),2.0)+pow(sin(q1+q2+q3)*cos(q0)*sin(q5)*-1.0+cos(q5)*sin(q0)*sin(q4)+cos(q1+q2+q3)*cos(q0)*cos(q4)*cos(q5),2.0)+pow(sin(q0)*sin(q4)*sin(q5)+sin(q1+q2+q3)*cos(q0)*cos(q5)+cos(q1+q2+q3)*cos(q0)*cos(q4)*sin(q5),2.0);

J(5,1) = 0.0;

J(5,2) = 0.0;

J(5,3) = 0.0;

J(5,4) = (cos(q0)*sin(q4)-cos(q1+q2+q3)*cos(q4)*sin(q0))*(cos(q4)*sin(q0)-cos(q1+q2+q3)*cos(q0)*sin(q4))-cos(q5)*(cos(q0)*cos(q4)*1.0+cos(q1+q2+q3)*sin(q0)*sin(q4)*1.0)*(cos(q5)*(sin(q0)*sin(q4)+cos(q1+q2+q3)*cos(q0)*cos(q4))-sin(q1+q2+q3)*cos(q0)*sin(q5)*1.0)*1.0-sin(q5)*(cos(q0)*cos(q4)*1.0+cos(q1+q2+q3)*sin(q0)*sin(q4)*1.0)*(sin(q5)*(sin(q0)*sin(q4)+cos(q1+q2+q3)*cos(q0)*cos(q4))*1.0+sin(q1+q2+q3)*cos(q0)*cos(q5)*1.0);

J(5,5) = -sin(q1+q2+q3)*sin(q4);

// return J;

int index = 0;
for (int row = 0; row < J.rows(); ++row) {
	for (int col = 0; col < J.cols(); ++col) {
		result[index++] = J(row, col);
	}
}

}
