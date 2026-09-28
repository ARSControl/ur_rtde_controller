#include <gtest/gtest.h>

#include <cmath>

#include "trajectory/joint_trajectory_sampler.h"

using trajectory::Waypoint;

namespace {

constexpr double DT = 0.002;

std::vector<Waypoint> positionOnly(const std::vector<double> &times, const std::vector<double> &positions)
{
    std::vector<Waypoint> wps;
    for (size_t i = 0; i < times.size(); i++) wps.push_back({times[i], {positions[i], -positions[i]}, {}, {}});
    return wps;
}

size_t indexAt(double t) {return static_cast<size_t>(std::round(t / DT));}

}  // namespace

TEST(Validate, RejectsInvalidTrajectories)
{
    EXPECT_FALSE(trajectory::validate(positionOnly({0.0}, {0.0}), 2).empty());
    EXPECT_FALSE(trajectory::validate(positionOnly({0.0, 0.0}, {0.0, 1.0}), 2).empty());
    EXPECT_FALSE(trajectory::validate(positionOnly({0.0, 1.0}, {0.0, NAN}), 2).empty());
    EXPECT_FALSE(trajectory::validate(positionOnly({0.0, 1.0}, {0.0, 1.0}), 3).empty());

    auto wps = positionOnly({0.0, 1.0}, {0.0, 1.0});
    wps[1].velocity = {0.0};
    EXPECT_FALSE(trajectory::validate(wps, 2).empty());

    EXPECT_TRUE(trajectory::validate(positionOnly({0.0, 0.5, 1.0}, {0.0, 0.3, 1.0}), 2).empty());
}

TEST(Sample, PositionOnlyPassesThroughWaypointsWithZeroEndVelocity)
{
    const std::vector<double> times = {0.0, 0.4, 1.0, 1.6}, positions = {0.0, 0.5, 0.2, 1.0};
    bool resampled = false;
    auto traj = trajectory::sample(positionOnly(times, positions), DT, &resampled);

    EXPECT_TRUE(resampled);
    ASSERT_EQ(traj.size(), indexAt(1.6) + 1);

    for (size_t i = 0; i < times.size(); i++)
    {
        EXPECT_NEAR(traj.position[indexAt(times[i])](0), positions[i], 1e-9);
        EXPECT_NEAR(traj.position[indexAt(times[i])](1), -positions[i], 1e-9);
    }

    EXPECT_NEAR(traj.velocity.front().norm(), 0.0, 1e-9);
    EXPECT_NEAR(traj.velocity.back().norm(), 0.0, 1e-9);

    // C2 Continuity at Interior Knots -> No Acceleration Jumps
    for (double knot : {0.4, 1.0})
    {
        const size_t k = indexAt(knot);
        EXPECT_NEAR(traj.acceleration[k - 1](0), traj.acceleration[k + 1](0), 0.1);
    }
}

TEST(Sample, VelocityIsDerivativeOfPosition)
{
    auto traj = trajectory::sample(positionOnly({0.0, 0.3, 0.7, 1.0}, {0.0, 0.4, -0.2, 0.5}), DT);

    for (size_t k = 1; k + 1 < traj.size(); k++)
    {
        // Skip Knots -> Jerk is Discontinuous there, Biasing the Central Difference
        if (k == indexAt(0.3) || k == indexAt(0.7)) continue;

        const double fd_velocity = (traj.position[k + 1](0) - traj.position[k - 1](0)) / (2 * DT);
        const double fd_acceleration = (traj.velocity[k + 1](0) - traj.velocity[k - 1](0)) / (2 * DT);
        EXPECT_NEAR(traj.velocity[k](0), fd_velocity, 1e-3);
        EXPECT_NEAR(traj.acceleration[k](0), fd_acceleration, 1e-2);
    }
}

TEST(Sample, QuinticReproducesBoundaryConditions)
{
    std::vector<Waypoint> wps = {
        {0.0, {0.0}, {0.0}, {0.0}},
        {0.5, {0.3}, {0.8}, {-1.0}},
        {1.0, {1.0}, {0.0}, {0.0}},
    };

    auto traj = trajectory::sample(wps, DT);

    for (const auto &wp : wps)
    {
        const size_t k = indexAt(wp.time);
        EXPECT_NEAR(traj.position[k](0), wp.position[0], 1e-9);
        EXPECT_NEAR(traj.velocity[k](0), wp.velocity[0], 1e-9);
        EXPECT_NEAR(traj.acceleration[k](0), wp.acceleration[0], 1e-9);
    }
}

TEST(Sample, CubicHermiteMatchesPolynomial)
{
    // p(t) = t^3 -> Exactly Represented by a Cubic Hermite Segment
    std::vector<Waypoint> wps = {{0.0, {0.0}, {0.0}, {}}, {1.0, {1.0}, {3.0}, {}}};
    auto traj = trajectory::sample(wps, DT);

    for (size_t k = 0; k < traj.size(); k++)
    {
        const double t = k * DT;
        EXPECT_NEAR(traj.position[k](0), t * t * t, 1e-9);
        EXPECT_NEAR(traj.velocity[k](0), 3 * t * t, 1e-9);
        EXPECT_NEAR(traj.acceleration[k](0), 6 * t, 1e-9);
    }
}

TEST(Sample, UniformlySampledInputIsUsedAsIs)
{
    std::vector<Waypoint> wps;
    for (int i = 0; i <= 100; i++)
    {
        const double t = 1.0 + i * DT;  // Non-Zero Start Time
        wps.push_back({t, {std::sin(t)}, {std::cos(t)}, {}});
    }

    bool resampled = true;
    auto traj = trajectory::sample(wps, DT, &resampled);

    EXPECT_FALSE(resampled);
    ASSERT_EQ(traj.size(), wps.size());
    for (size_t i = 0; i < wps.size(); i++)
    {
        EXPECT_DOUBLE_EQ(traj.position[i](0), wps[i].position[0]);
        EXPECT_DOUBLE_EQ(traj.velocity[i](0), wps[i].velocity[0]);
    }
    EXPECT_NEAR(traj.acceleration[50](0), -std::sin(wps[50].time), 1e-4);
}

TEST(Sample, UniformPositionsWithoutVelocitiesAreResampled)
{
    std::vector<Waypoint> wps;
    for (int i = 0; i <= 10; i++) wps.push_back({i * DT, {i * 0.001}, {}, {}});

    bool resampled = false;
    trajectory::sample(wps, DT, &resampled);
    EXPECT_TRUE(resampled);
}

TEST(Sample, DurationNotMultipleOfDtEndsOnFinalWaypoint)
{
    auto traj = trajectory::sample(positionOnly({0.0, 0.0105}, {0.0, 1.0}), DT);

    ASSERT_EQ(traj.size(), 7u);
    EXPECT_NEAR(traj.position.back()(0), 1.0, 1e-12);
    EXPECT_NEAR(traj.position[5](0), traj.position[6](0), 0.05);
}
