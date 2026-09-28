#include "trajectory/joint_trajectory_sampler.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <sstream>

namespace trajectory {

namespace {

using Eigen::VectorXd;

// Segment Polynomial p(s) = c[0] + c[1] s + ... + c[5] s^5, s in [0, h] (One Coefficient Vector per Joint)
struct Segment
{
    double t_start;
    double h;
    std::array<VectorXd, 6> c;
};

bool hasAll(const std::vector<Waypoint> &waypoints, size_t n_joints, std::vector<double> Waypoint::*field)
{
    return std::all_of(waypoints.begin(), waypoints.end(), [&](const Waypoint &wp) {return (wp.*field).size() == n_joints;});
}

VectorXd toEigen(const std::vector<double> &v)
{
    return Eigen::Map<const VectorXd>(v.data(), v.size());
}

// Knot Velocities of the Clamped Cubic Spline (Zero Initial and Final Velocity) -> Tridiagonal System Solved with Thomas Algorithm
std::vector<VectorXd> clampedSplineVelocities(const std::vector<double> &t, const std::vector<VectorXd> &p)
{
    const size_t n = p.size();
    std::vector<VectorXd> v(n, VectorXd::Zero(p.front().size()));
    if (n < 3) return v;

    // Interior Unknowns: v[1] ... v[n-2]
    // h[i] v[i-1] + 2 (h[i-1] + h[i]) v[i] + h[i-1] v[i+1] = 3 (h[i] d[i-1] / h[i-1] + h[i-1] d[i] / h[i])
    const size_t m = n - 2;
    std::vector<double> c_prime(m);
    std::vector<VectorXd> r_prime(m);

    for (size_t k = 0; k < m; k++)
    {
        const size_t i = k + 1;
        const double h_prev = t[i] - t[i-1], h_next = t[i+1] - t[i];
        const double a = h_next, b = 2.0 * (h_prev + h_next), c = h_prev;
        const VectorXd r = 3.0 * (h_next * (p[i] - p[i-1]) / h_prev + h_prev * (p[i+1] - p[i]) / h_next);

        // Forward Sweep (v[0] = 0 -> No Contribution from the First Sub-Diagonal Term)
        if (k == 0) {c_prime[k] = c / b; r_prime[k] = r / b;}
        else
        {
            const double denominator = b - a * c_prime[k-1];
            c_prime[k] = c / denominator;
            r_prime[k] = (r - a * r_prime[k-1]) / denominator;
        }
    }

    // Back Substitution (v[n-1] = 0)
    v[m] = r_prime[m-1];
    for (size_t k = m - 1; k-- > 0;) v[k+1] = r_prime[k] - c_prime[k] * v[k+2];

    return v;
}

Segment computeSegment(double t_start, double h, const VectorXd &p0, const VectorXd &v0, const VectorXd &a0,
                       const VectorXd &p1, const VectorXd &v1, const VectorXd &a1, bool quintic)
{
    Segment s;
    s.t_start = t_start;
    s.h = h;

    const VectorXd d = p1 - p0;
    const VectorXd zero = VectorXd::Zero(p0.size());
    s.c[0] = p0;
    s.c[1] = v0;

    if (quintic)
    {
        s.c[2] = a0 / 2.0;
        s.c[3] = (20.0 * d - (8.0 * v1 + 12.0 * v0) * h - (3.0 * a0 - a1) * h * h) / (2.0 * std::pow(h, 3));
        s.c[4] = (-30.0 * d + (14.0 * v1 + 16.0 * v0) * h + (3.0 * a0 - 2.0 * a1) * h * h) / (2.0 * std::pow(h, 4));
        s.c[5] = (12.0 * d - 6.0 * (v1 + v0) * h + (a1 - a0) * h * h) / (2.0 * std::pow(h, 5));
    }
    else
    {
        s.c[2] = (3.0 * d / h - 2.0 * v0 - v1) / h;
        s.c[3] = (-2.0 * d / h + v0 + v1) / (h * h);
        s.c[4] = zero;
        s.c[5] = zero;
    }

    return s;
}

void evaluateSegment(const Segment &seg, double t, VectorXd &p, VectorXd &v, VectorXd &a)
{
    const double s = std::clamp(t - seg.t_start, 0.0, seg.h);
    const auto &c = seg.c;

    // Horner Evaluation
    p = c[0] + s * (c[1] + s * (c[2] + s * (c[3] + s * (c[4] + s * c[5]))));
    v = c[1] + s * (2.0 * c[2] + s * (3.0 * c[3] + s * (4.0 * c[4] + s * 5.0 * c[5])));
    a = 2.0 * c[2] + s * (6.0 * c[3] + s * (12.0 * c[4] + s * 20.0 * c[5]));
}

}  // namespace

std::string validate(const std::vector<Waypoint> &waypoints, size_t n_joints)
{
    std::ostringstream error;

    if (waypoints.size() < 2) {error << "Trajectory Must Have at Least 2 Points, Given: " << waypoints.size(); return error.str();}

    for (size_t i = 0; i < waypoints.size(); i++)
    {
        const Waypoint &wp = waypoints[i];

        if (wp.position.size() != n_joints) {error << "Point " << i << ": Position Size != " << n_joints; return error.str();}
        if (!wp.velocity.empty() && wp.velocity.size() != n_joints) {error << "Point " << i << ": Velocity Size != " << n_joints; return error.str();}
        if (!wp.acceleration.empty() && wp.acceleration.size() != n_joints) {error << "Point " << i << ": Acceleration Size != " << n_joints; return error.str();}

        for (const auto *field : {&wp.position, &wp.velocity, &wp.acceleration})
            if (!std::all_of(field->begin(), field->end(), [](double x) {return std::isfinite(x);})) {error << "Point " << i << ": Non-Finite Value"; return error.str();}

        if (!std::isfinite(wp.time) || wp.time < 0.0) {error << "Point " << i << ": Invalid Time " << wp.time; return error.str();}
        if (i > 0 && wp.time <= waypoints[i-1].time) {error << "Point " << i << ": Time Not Strictly Increasing"; return error.str();}
    }

    return "";
}

bool isUniformlySampled(const std::vector<Waypoint> &waypoints, double dt, double tolerance)
{
    const double t0 = waypoints.front().time;
    for (size_t i = 0; i < waypoints.size(); i++)
        if (std::fabs((waypoints[i].time - t0) - i * dt) > tolerance) return false;

    return true;
}

SampledTrajectory sample(const std::vector<Waypoint> &waypoints, double dt, bool *resampled)
{
    const size_t n_points = waypoints.size();
    const size_t n_joints = waypoints.front().position.size();
    const bool has_velocity = hasAll(waypoints, n_joints, &Waypoint::velocity);
    const bool has_acceleration = has_velocity && hasAll(waypoints, n_joints, &Waypoint::acceleration);

    SampledTrajectory out;
    out.dt = dt;

    // Already Sampled at dt -> Use Waypoints As-Is
    if (has_velocity && isUniformlySampled(waypoints, dt))
    {
        for (const Waypoint &wp : waypoints)
        {
            out.position.push_back(toEigen(wp.position));
            out.velocity.push_back(toEigen(wp.velocity));
        }

        // Accelerations from Central Finite Differences if Not Provided
        for (size_t i = 0; i < n_points; i++)
        {
            if (has_acceleration) {out.acceleration.push_back(toEigen(waypoints[i].acceleration)); continue;}
            const size_t prev = (i == 0) ? 0 : i - 1, next = std::min(i + 1, n_points - 1);
            out.acceleration.push_back((out.velocity[next] - out.velocity[prev]) / ((next - prev) * dt));
        }

        if (resampled) *resampled = false;
        return out;
    }

    // Knots (Times Relative to the First Waypoint)
    std::vector<double> t(n_points);
    std::vector<VectorXd> p(n_points), v, a;
    for (size_t i = 0; i < n_points; i++) {t[i] = waypoints[i].time - waypoints.front().time; p[i] = toEigen(waypoints[i].position);}

    if (has_velocity) for (const Waypoint &wp : waypoints) v.push_back(toEigen(wp.velocity));
    else v = clampedSplineVelocities(t, p);

    if (has_acceleration) for (const Waypoint &wp : waypoints) a.push_back(toEigen(wp.acceleration));
    else a.assign(n_points, VectorXd::Zero(n_joints));

    // Segment Polynomials
    std::vector<Segment> segments;
    for (size_t i = 0; i + 1 < n_points; i++)
        segments.push_back(computeSegment(t[i], t[i+1] - t[i], p[i], v[i], a[i], p[i+1], v[i+1], a[i+1], has_acceleration));

    // Sample Every dt -> Last Sample Exactly on the Final Waypoint
    const double duration = t.back();
    const size_t n_samples = static_cast<size_t>(std::ceil(duration / dt - 1e-9)) + 1;
    out.position.resize(n_samples);
    out.velocity.resize(n_samples);
    out.acceleration.resize(n_samples);

    size_t seg = 0;
    for (size_t k = 0; k < n_samples; k++)
    {
        const double time = std::min(k * dt, duration);
        while (seg + 1 < segments.size() && time > segments[seg + 1].t_start) seg++;
        evaluateSegment(segments[seg], time, out.position[k], out.velocity[k], out.acceleration[k]);
    }

    if (resampled) *resampled = true;
    return out;
}

}  // namespace trajectory
