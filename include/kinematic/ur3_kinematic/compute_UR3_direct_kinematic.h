#include <Eigen/Dense>

Eigen::Matrix<double, 4, 4> compute_UR3_direct_kinematic(Eigen::Matrix<double, 6, 1> q){

Eigen::Matrix<double, 4, 4> T;

double q0 = q(0,0);
double q1 = q(1,0);
double q2 = q(2,0);
double q3 = q(3,0);
double q4 = q(4,0);
double q5 = q(5,0);

T(0,0) = cos(q5)*(sin(q0)*sin(q4)+cos(q1+q2+q3)*cos(q0)*cos(q4))-sin(q1+q2+q3)*cos(q0)*sin(q5)*1.0;

T(0,1) = sin(q5)*(sin(q0)*sin(q4)+cos(q1+q2+q3)*cos(q0)*cos(q4))*-1.0-sin(q1+q2+q3)*cos(q0)*cos(q5)*1.0;

T(0,2) = cos(q4)*sin(q0)-cos(q1+q2+q3)*cos(q0)*sin(q4);

T(0,3) = sin(q0)*1.1235E-1+cos(q4)*sin(q0)*8.19E-2+sin(q1+q2+q3)*cos(q0)*8.535E-2-cos(q0)*(cos(q1+q2)*4.265E+3+cos(q1)*4.873E+3)*5.0E-5-cos(q1+q2+q3)*cos(q0)*sin(q4)*8.19E-2;

T(1,0) = cos(q5)*(cos(q0)*sin(q4)*1.0-cos(q1+q2+q3)*cos(q4)*sin(q0)*1.0)*-1.0-sin(q1+q2+q3)*sin(q0)*sin(q5)*1.0;

T(1,1) = sin(q5)*(cos(q0)*sin(q4)*1.0-cos(q1+q2+q3)*cos(q4)*sin(q0)*1.0)-sin(q1+q2+q3)*cos(q5)*sin(q0)*1.0;

T(1,2) = -cos(q0)*cos(q4)-cos(q1+q2+q3)*sin(q0)*sin(q4);

T(1,3) = cos(q0)*-1.1235E-1-cos(q0)*cos(q4)*8.19E-2+sin(q1+q2+q3)*sin(q0)*8.535E-2-sin(q0)*(cos(q1+q2)*4.265E+3+cos(q1)*4.873E+3)*5.0E-5-cos(q1+q2+q3)*sin(q0)*sin(q4)*8.19E-2;

T(2,0) = cos(q1+q2+q3)*sin(q5)+sin(q1+q2+q3)*cos(q4)*cos(q5);

T(2,1) = cos(q1+q2+q3)*cos(q5)-sin(q1+q2+q3)*cos(q4)*sin(q5)*1.0;

T(2,2) = sin(q1+q2+q3)*sin(q4)*-1.0;

T(2,3) = cos(q1+q2+q3)*-8.535E-2-sin(q1+q2)*2.1325E-1-sin(q1)*2.4365E-1-sin(q1+q2+q3)*sin(q4)*8.19E-2+1.519E-1;

T(3,0) = 0.0;

T(3,1) = 0.0;

T(3,2) = 0.0;

T(3,3) = 1.0;

return T;

}
