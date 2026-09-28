#ifndef JOINT_TRAJECTORY_SAMPLER_H
#define JOINT_TRAJECTORY_SAMPLER_H

#include <string>
#include <vector>

#include <Eigen/Dense>

namespace trajectory {

struct Waypoint
{
    double time;
    std::vector<double> position;
    std::vector<double> velocity;       // Optional -> Empty if Not Provided
    std::vector<double> acceleration;   // Optional -> Empty if Not Provided
};

struct SampledTrajectory
{
    double dt = 0.0;
    std::vector<Eigen::VectorXd> position;
    std::vector<Eigen::VectorXd> velocity;
    std::vector<Eigen::VectorXd> acceleration;

    size_t size() const {return position.size();}
    bool empty() const {return position.empty();}
};

// Check Waypoints Consistency (At Least 2 Points, Sizes, Strictly Increasing Time, Finite Values)
// Returns an Empty String if Valid, the Error Description Otherwise
std::string validate(const std::vector<Waypoint> &waypoints, size_t n_joints);

// True if the Waypoints Times (Relative to the First One) are Exactly k * dt
bool isUniformlySampled(const std::vector<Waypoint> &waypoints, double dt, double tolerance = 1e-6);

// Sample the Trajectory Every dt Seconds (Times are Relative to the First Waypoint)
// Already Uniformly Sampled Waypoints with Velocities are Used As-Is, Otherwise they are Interpolated with:
//   - Position + Velocity + Acceleration -> Quintic Hermite Segments
//   - Position + Velocity                -> Cubic Hermite Segments
//   - Position Only                      -> Clamped Cubic Spline (Zero Initial and Final Velocity)
// Waypoints Must be Valid (See validate)
SampledTrajectory sample(const std::vector<Waypoint> &waypoints, double dt, bool *resampled = nullptr);

}  // namespace trajectory

#endif /* JOINT_TRAJECTORY_SAMPLER_H */
