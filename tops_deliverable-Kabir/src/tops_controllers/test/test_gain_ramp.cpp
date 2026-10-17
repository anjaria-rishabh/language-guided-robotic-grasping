// Copyright 2026 Kabir Dhingra. MIT licence.

#include <gtest/gtest.h>

#include "tops_controllers/gain_ramp.hpp"

using tops_controllers::GainRamp;
using tops_controllers::HysteresisTrigger;
using Vec6 = Eigen::Matrix<double, 6, 1>;

TEST(GainRamp, RespectsTheSlewRate)
{
  GainRamp<6> ramp;
  ramp.reset(Vec6::Constant(800.0));
  ramp.setMaxRate(1500.0);              // N/m per second
  ramp.setTarget(Vec6::Constant(150.0));

  // One millisecond may change a gain by at most 1.5 N/m.
  ramp.update(0.001);
  EXPECT_NEAR(ramp.value()(0), 798.5, 1e-9);
}

TEST(GainRamp, ArrivesInTheAdvertisedTime)
{
  GainRamp<6> ramp;
  ramp.reset(Vec6::Constant(800.0));
  ramp.setMaxRate(1500.0);
  ramp.setTarget(Vec6::Constant(150.0));

  // 650 N/m at 1500 N/m/s is 0.4333 s. The service reports this number to
  // the supervisor, which waits on it before declaring the switch done.
  EXPECT_NEAR(ramp.timeToTarget(), 650.0 / 1500.0, 1e-9);

  for (int i = 0; i < 434; ++i) {ramp.update(0.001);}
  EXPECT_TRUE(ramp.settled());
  EXPECT_NEAR(ramp.value()(0), 150.0, 1e-6);
}

TEST(GainRamp, NeverOvershootsTheTarget)
{
  // An overshooting ramp would momentarily apply a stiffness nobody asked
  // for, which is exactly the energy injection this class exists to avoid.
  GainRamp<6> ramp;
  ramp.reset(Vec6::Constant(0.0));
  ramp.setMaxRate(1000.0);
  ramp.setTarget(Vec6::Constant(10.0));
  for (int i = 0; i < 100; ++i) {
    ramp.update(0.1);                   // a step far larger than the gap
    EXPECT_LE(ramp.value()(0), 10.0 + 1e-12);
  }
  EXPECT_NEAR(ramp.value()(0), 10.0, 1e-12);
}

TEST(HysteresisTrigger, DoesNotChatterOnANoisyThreshold)
{
  // Enter above 1.5 N, leave below 1.0 N. A signal hovering at 1.2 N with
  // noise crosses a single 1.5 N threshold repeatedly; with hysteresis it
  // cannot re-trigger without first falling below 1.0 N.
  HysteresisTrigger trigger(1.5, 1.0, true);

  EXPECT_FALSE(trigger.update(1.2, 0.001));
  EXPECT_TRUE(trigger.update(1.6, 0.001));
  EXPECT_TRUE(trigger.update(1.2, 0.001)) << "must stay latched above the exit threshold";
  EXPECT_TRUE(trigger.update(1.05, 0.001));
  EXPECT_FALSE(trigger.update(0.9, 0.001));
  EXPECT_FALSE(trigger.update(1.2, 0.001)) << "must not re-enter below the enter threshold";
}

TEST(HysteresisTrigger, FallingSenseForTheDistanceTrigger)
{
  // The compliant mode also arms on proximity: assert BELOW 15 mm, release
  // above 25 mm.
  HysteresisTrigger trigger(0.015, 0.025, false);
  EXPECT_FALSE(trigger.update(0.050, 0.001));
  EXPECT_FALSE(trigger.update(0.020, 0.001));
  EXPECT_TRUE(trigger.update(0.014, 0.001));
  EXPECT_TRUE(trigger.update(0.022, 0.001));
  EXPECT_FALSE(trigger.update(0.030, 0.001));
}

TEST(HysteresisTrigger, MinimumDwellBlocksRapidToggling)
{
  HysteresisTrigger trigger(1.5, 1.0, true);
  trigger.setMinDwell(0.010);           // 10 ms

  // Signal goes high immediately, but the trigger has not yet dwelt long
  // enough in the low state.
  EXPECT_FALSE(trigger.update(5.0, 0.001));
  for (int i = 0; i < 9; ++i) {trigger.update(5.0, 0.001);}
  EXPECT_TRUE(trigger.update(5.0, 0.001));
}
