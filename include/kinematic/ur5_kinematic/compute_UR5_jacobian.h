#include <Eigen/Dense>

Eigen::Matrix<double, 6, 6> compute_UR5_jacobian(Eigen::Matrix<double, 6, 1> q){

Eigen::Matrix<double, 6, 6> J;

J.setZero();

double q0 = q(0,0);
double q1 = q(1,0);
double q2 = q(2,0);
double q3 = q(3,0);
double q4 = q(4,0);
double q5 = q(5,0);

J(0,0) = cos(q0)*1.0915E-1+cos(q0)*cos(q4)*8.23E-2-sin(q1+q2+q3)*sin(q0)*9.465E-2+sin(q0)*(cos(q1+q2)*1.569E+3+cos(q1)*1.7E+3)*2.5E-4+cos(q1+q2+q3)*sin(q0)*sin(q4)*8.23E-2;

J(0,1) = (cos(q0)*(cos(q1+q2+q3)*1.893E+3+sin(q1+q2)*7.845E+3+sin(q1)*8.5E+3+sin(q1+q2+q3)*sin(q4)*1.646E+3))/2.0E+4;

J(0,2) = (cos(q0)*(cos(q1+q2+q3)*1.893E+3+sin(q1+q2)*7.845E+3+sin(q1+q2+q3)*sin(q4)*1.646E+3))/2.0E+4;

J(0,3) = (cos(q0)*(cos(q1+q2+q3)*1.893E+3+sin(q1+q2+q3)*sin(q4)*1.646E+3))/2.0E+4;

J(0,4) = sin(q0)*sin(q4)*-8.23E-2-cos(q1+q2+q3)*cos(q0)*cos(q4)*8.23E-2;

J(0,5) = 0.0;

J(1,0) = sin(q0)*1.0915E-1+cos(q4)*sin(q0)*8.23E-2+sin(q1+q2+q3)*cos(q0)*9.465E-2-cos(q0)*(cos(q1+q2)*1.569E+3+cos(q1)*1.7E+3)*2.5E-4-cos(q1+q2+q3)*cos(q0)*sin(q4)*8.23E-2;

J(1,1) = (sin(q0)*(cos(q1+q2+q3)*1.893E+3+sin(q1+q2)*7.845E+3+sin(q1)*8.5E+3+sin(q1+q2+q3)*sin(q4)*1.646E+3))/2.0E+4;

J(1,2) = (sin(q0)*(cos(q1+q2+q3)*1.893E+3+sin(q1+q2)*7.845E+3+sin(q1+q2+q3)*sin(q4)*1.646E+3))/2.0E+4;

J(1,3) = (sin(q0)*(cos(q1+q2+q3)*1.893E+3+sin(q1+q2+q3)*sin(q4)*1.646E+3))/2.0E+4;

J(1,4) = cos(q0)*sin(q4)*8.23E-2-cos(q1+q2+q3)*cos(q4)*sin(q0)*8.23E-2;

J(1,5) = 0.0;

J(2,0) = 0.0;

J(2,1) = sin(q1+q2+q3)*9.465E-2-cos(q1+q2)*3.9225E-1-cos(q1)*4.25E-1-cos(q1+q2+q3)*sin(q4)*8.23E-2;

J(2,2) = sin(q1+q2+q3)*9.465E-2-cos(q1+q2)*3.9225E-1-cos(q1+q2+q3)*sin(q4)*8.23E-2;

J(2,3) = sin(q1+q2+q3)*9.465E-2-cos(q1+q2+q3)*sin(q4)*8.23E-2;

J(2,4) = sin(q1+q2+q3)*cos(q4)*-8.23E-2;

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

return J;

}
