// Copyright 2026 Kabir Dhingra. MIT licence.
//
// The passivity monitor is the one component whose correctness is a claim
// about energy rather than about a number, so the tests below are written
// as energy budgets that can be checked by hand.

#include <gtest/gtest.h>

#include "tops_controllers/energy_tank.hpp"

using tops_controllers::EnergyTank;
using tops_controllers::EnergyTankConfig;

namespace
{
EnergyTankConfig defaultConfig()
{
  EnergyTankConfig cfg;
  cfg.initial_energy = 0.5;
  cfg.min_energy = 0.05;
  cfg.max_energy = 2.0;
  cfg.max_damping_boost = 3.0;
  return cfg;
}
}  // namespace

TEST(EnergyTank, StartsAtTheConfiguredEnergy)
{
  EnergyTank tank(defaultConfig());
  EXPECT_NEAR(tank.energy(), 0.5, 1e-9);
  EXPECT_NEAR(tank.state(), std::sqrt(1.0), 1e-9);
  EXPECT_DOUBLE_EQ(tank.valve(), 1.0);
  EXPECT_FALSE(tank.throttling());
}

TEST(EnergyTank, DissipationFillsTheTank)
{
  EnergyTank tank(defaultConfig());
  const double dt = 0.001;
  // 10 W dissipated for 100 ms is 1.0 J, so 0.5 J in should reach the 1.5 J
  // the budget predicts, well under the 2.0 J cap.
  for (int i = 0; i < 100; ++i) {
    tank.update(10.0, 0.0, dt);
  }
  EXPECT_NEAR(tank.energy(), 1.5, 1e-2);
  EXPECT_DOUBLE_EQ(tank.valve(), 1.0);
}

TEST(EnergyTank, StopsFillingAtTheCap)
{
  EnergyTank tank(defaultConfig());
  for (int i = 0; i < 5000; ++i) {
    tank.update(50.0, 0.0, 0.001);
  }
  // Without the sigma term this would have accumulated 250 J.
  EXPECT_LE(tank.energy(), 2.0 + 1e-6);
}

TEST(EnergyTank, InjectionDrainsTheTank)
{
  EnergyTank tank(defaultConfig());
  const double dt = 0.001;
  // 5 W out for 50 ms is 0.25 J, leaving 0.25 J of the initial 0.5 J.
  for (int i = 0; i < 50; ++i) {
    tank.update(0.0, 5.0, dt);
  }
  EXPECT_NEAR(tank.energy(), 0.25, 1e-2);
  EXPECT_DOUBLE_EQ(tank.valve(), 1.0) << "the tank could still afford this";
}

TEST(EnergyTank, ValveClosesWhenTheTankCannotAfford)
{
  EnergyTank tank(defaultConfig());
  const double dt = 0.001;
  // Draw hard and for long enough that the affordable energy runs out.
  for (int i = 0; i < 2000; ++i) {
    tank.update(0.0, 100.0, dt);
  }
  EXPECT_TRUE(tank.throttling());
  EXPECT_LT(tank.valve(), 0.5);
  EXPECT_GE(tank.energy(), defaultConfig().min_energy - 1e-9)
    << "the tank must never drain below its floor";
}

TEST(EnergyTank, TotalInjectedEnergyIsBounded)
{
  // The passivity claim, stated as a test: with NO dissipation, the energy
  // the controller is permitted to deliver over all time is bounded by the
  // initial budget above the floor. gamma is what enforces it.
  EnergyTank tank(defaultConfig());
  const double dt = 0.001;
  const double requested_power = 20.0;
  double delivered = 0.0;

  for (int i = 0; i < 20000; ++i) {          // 20 s of continuous demand
    const double gamma = tank.update(0.0, requested_power, dt);
    delivered += gamma * requested_power * dt;
  }

  const double budget = defaultConfig().initial_energy - defaultConfig().min_energy;
  EXPECT_LE(delivered, budget + 1e-3)
    << "delivered " << delivered << " J against a budget of " << budget << " J";
  EXPECT_GT(delivered, 0.8 * budget)
    << "the tank should actually spend its budget, not sit on it";
}

TEST(EnergyTank, NegativeInjectionRefillsAndIsNeverThrottled)
{
  EnergyTank tank(defaultConfig());
  // The environment doing work on the spring is always allowed: it makes
  // the system more passive, not less.
  for (int i = 0; i < 100; ++i) {
    const double gamma = tank.update(0.0, -5.0, 0.001);
    EXPECT_DOUBLE_EQ(gamma, 1.0);
  }
  EXPECT_GT(tank.energy(), 0.5);
}

TEST(EnergyTank, DampingBoostRisesAsTheTankEmpties)
{
  EnergyTank tank(defaultConfig());
  EXPECT_NEAR(tank.dampingBoost(), 1.0, 1e-9) << "a healthy tank must not alter damping";

  for (int i = 0; i < 5000; ++i) {
    tank.update(0.0, 100.0, 0.001);
  }
  EXPECT_GT(tank.dampingBoost(), 1.5);
  EXPECT_LE(tank.dampingBoost(), defaultConfig().max_damping_boost + 1e-9);
}

TEST(EnergyTank, NegativeDissipationIsClampedNotTrusted)
{
  // vT D v cannot be negative for a positive-definite D. If it arrives
  // negative, D is wrong -- the tank must not treat it as a withdrawal.
  EnergyTank tank(defaultConfig());
  const double before = tank.energy();
  tank.update(-100.0, 0.0, 0.001);
  EXPECT_NEAR(tank.energy(), before, 1e-9);
}

TEST(EnergyTank, ZeroTimestepIsANoOp)
{
  EnergyTank tank(defaultConfig());
  const double before = tank.energy();
  tank.update(10.0, 10.0, 0.0);
  EXPECT_DOUBLE_EQ(tank.energy(), before);
}
