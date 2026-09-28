# Robot State and Dynamics -> Published at Every Torque Control Cycle
# All Quantities Refer to the Same Joint Configuration (position, velocity)
# Matrices are 6x6, Row-Major

std_msgs/Header header

float64[6] position         # Joint Position [rad]
float64[6] velocity         # Joint Velocity [rad/s]
float64[6] effort           # Measured Joint Torque (from Motor Currents) [Nm]

float64[36] mass_matrix     # Joint Space Inertia Matrix M(q)
float64[6] coriolis         # Coriolis and Centrifugal Torques C(q, dq) * dq [Nm]
float64[36] jacobian        # TCP Jacobian J(q) (Active TCP, Base Frame)
float64[36] jacobian_dot    # TCP Jacobian Time Derivative dJ(q, dq)/dt

geometry_msgs/Pose tcp_pose
geometry_msgs/Wrench tcp_wrench
