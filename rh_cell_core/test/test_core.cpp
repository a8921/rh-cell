// Unit tests for rh_cell_core. No Gazebo, no ROS. They FAIL until TODO 1 and TODO 2 are done.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "rh_cell_core/admittance_1d.hpp"
#include "rh_cell_core/force_guard.hpp"

using rh_cell_core::Admittance1D;
using rh_cell_core::ForceGuard;

TEST(Admittance, FollowsTargetInFreeSpace)
{
  Admittance1D ctrl({2.0, 60.0, 500.0}, 0.0);
  for (int i = 0; i < 1500; ++i) ctrl.step(0.05, 0.0, 0.002);
  EXPECT_NEAR(ctrl.position(), 0.05, 1e-3) << "Without contact the tool must reach x_des.";
}

TEST(Admittance, YieldsUnderConstantPush)
{
  Admittance1D ctrl({2.0, 60.0, 500.0}, 0.0);
  for (int i = 0; i < 2500; ++i) ctrl.step(0.0, -5.0, 0.002);
  EXPECT_NEAR(ctrl.position(), -0.01, 5e-4) << "-5 N on a 500 N/m spring = 10 mm back.";
}

TEST(Admittance, ContactForceStaysLowAgainstStiffWall)
{
  const double dt = 0.002, k_env = 50000.0, start = -0.05, target = 0.01;
  Admittance1D ctrl({2.0, 60.0, 500.0}, start);
  double f = 0.0, peak = 0.0;
  for (int i = 0; i <= 4000; ++i)
  {
    const double x_des = std::min(target, start + 0.01 * i * dt);
    const double x = ctrl.step(x_des, f, dt);
    f = (x > 0.0) ? -k_env * x : 0.0;
    peak = std::max(peak, std::abs(f));
  }
  EXPECT_NEAR(std::abs(f), 5.0, 0.5) << "Steady contact force should be about K x 10 mm = 5 N.";
  EXPECT_LT(peak, 20.0) << "Peak contact force must stay under the 20 N guard limit.";
}

TEST(ForceGuard, SingleSpikeDoesNotTrip)
{
  ForceGuard g(20.0, 5);
  for (int i = 0; i < 4; ++i) g.update(25.0);
  g.update(10.0);
  for (int i = 0; i < 4; ++i) g.update(25.0);
  EXPECT_FALSE(g.tripped());
}

TEST(ForceGuard, SustainedOverloadTrips)
{
  ForceGuard g(20.0, 5);
  for (int i = 0; i < 5; ++i) g.update(-25.0);
  EXPECT_TRUE(g.tripped()) << "5 samples above the limit (either sign) must trip.";
}

TEST(ForceGuard, StaysTrippedUntilReset)
{
  ForceGuard g(20.0, 5);
  for (int i = 0; i < 5; ++i) g.update(25.0);
  g.update(0.0);
  EXPECT_TRUE(g.tripped()) << "An interlock must latch.";
  g.reset();
  EXPECT_FALSE(g.tripped());
}
