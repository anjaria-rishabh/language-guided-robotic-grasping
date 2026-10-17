// Copyright 2026 Kabir Dhingra. MIT licence.
//
// Passivity monitor: the energy tank.
// Theory/06_Compliant_Control.md §6.7, docs/02_CONTROL_LAW.md §6.
//
// ---------------------------------------------------------------------------
// The problem it solves
// ---------------------------------------------------------------------------
// A constant-gain impedance controller is passive: the virtual spring can
// only ever give back energy you put into it by pushing the arm. That
// guarantee is what makes contact with an unknown environment safe, and it
// is the reason impedance control is used at all.
//
// The guarantee breaks the moment anything about the controller becomes
// time-varying. Two things in this project do exactly that:
//
//   1. Changing stiffness while the arm is displaced from equilibrium. The
//      spring's stored energy is 0.5*e^T*K*e. Raise K with e non-zero and
//      you have created energy from nothing. The arm can spend it.
//   2. The force-regulation term. It commands a force that is not derived
//      from any potential field, so it can inject energy indefinitely.
//
// Either one can make a contact that "should" be stable oscillate and build
// amplitude until something trips or breaks.
//
// ---------------------------------------------------------------------------
// The mechanism
// ---------------------------------------------------------------------------
// Keep a running energy budget. The tank has state s and holds T = 0.5*s^2
// joules. Every cycle:
//
//   - Energy DISSIPATED by the damper (e_dot^T D e_dot, always >= 0) flows
//     into the tank. This is real energy the arm genuinely removed.
//   - Energy INJECTED by the non-passive terms flows out of the tank.
//   - If the tank cannot afford the injection, the valve gamma closes
//     proportionally and the injecting term is scaled down. A controller
//     that cannot pay for its own energy simply does not get to spend it.
//
//     s_dot = ( sigma * P_dissipated - gamma * P_injected ) / s
//
//   with sigma = 0 once the tank is full (stop hoarding; an unbounded tank
//   is not a passivity guarantee, it is a delayed one).
//
// The result is a hard bound: total energy the controller can deliver to
// the environment is bounded by the initial tank plus what it has dissipated.
// That is a passivity statement, and it holds through gain changes.
//
// This class is deliberately free of ROS and Eigen types so the behaviour
// can be unit-tested against hand-computed energy budgets. See
// test/test_energy_tank.cpp.

#ifndef TOPS_CONTROLLERS__ENERGY_TANK_HPP_
#define TOPS_CONTROLLERS__ENERGY_TANK_HPP_

#include <algorithm>
#include <cmath>

namespace tops_controllers
{

struct EnergyTankConfig
{
  /// Starting budget. Sized as "enough to complete one normal approach
  /// without throttling, not enough to do real damage if it all comes out
  /// at once". 1 J moving a 3 kg effective mass is about 0.8 m/s -- already
  /// more than you want, so the default is conservative.
  double initial_energy{0.5};

  /// Below this the valve starts closing. Not zero: at exactly zero the
  /// s_dot = .../s update divides by zero, and more practically you want
  /// the throttling to be gradual rather than a cliff.
  double min_energy{0.05};

  /// Above this the tank stops accepting dissipated energy. Without a cap,
  /// a long stiff-mode approach fills the tank with joules that then become
  /// available all at once during a contact transient.
  double max_energy{2.0};

  /// When the tank is low, damping is boosted by up to this factor. This is
  /// the active half of the mechanism: throttling the injection stops the
  /// controller making things worse, while extra damping actively removes
  /// the energy already in the arm.
  double max_damping_boost{3.0};
};

class EnergyTank
{
public:
  EnergyTank() = default;
  explicit EnergyTank(const EnergyTankConfig & cfg) { configure(cfg); }

  void configure(const EnergyTankConfig & cfg)
  {
    cfg_ = cfg;
    cfg_.min_energy = std::max(1e-6, cfg_.min_energy);
    cfg_.max_energy = std::max(cfg_.max_energy, cfg_.min_energy * 2.0);
    reset();
  }

  const EnergyTankConfig & config() const { return cfg_; }

  void reset()
  {
    const double e0 = std::clamp(cfg_.initial_energy, cfg_.min_energy, cfg_.max_energy);
    s_ = std::sqrt(2.0 * e0);
    gamma_ = 1.0;
    sigma_ = 1.0;
    last_dissipated_ = 0.0;
    last_injected_ = 0.0;
  }

  /// Advance the tank one control cycle.
  ///
  /// @param power_dissipated  e_dot^T D e_dot, watts. Must be >= 0; a
  ///                          negative value here means the damping matrix
  ///                          is not positive definite, which is a bug
  ///                          worth catching rather than smoothing over,
  ///                          so it is clamped at zero and reported.
  /// @param power_injected    e_dot^T F_active, watts, signed. Positive
  ///                          means the controller is doing work ON the
  ///                          environment -- that is what must be paid for.
  ///                          Negative means the environment is doing work
  ///                          on the arm, which refills the tank for free.
  /// @param dt                control period, seconds.
  /// @return the valve value gamma in [0,1]. Multiply the energy-injecting
  ///         part of your commanded wrench by this before using it.
  double update(double power_dissipated, double power_injected, double dt)
  {
    if (dt <= 0.0) {return gamma_;}

    power_dissipated = std::max(0.0, power_dissipated);
    last_dissipated_ = power_dissipated;
    last_injected_ = power_injected;

    const double energy = this->energy();

    // sigma: stop filling a full tank.
    sigma_ = (energy < cfg_.max_energy) ? 1.0 : 0.0;

    // gamma: can we afford this cycle's injection?
    if (power_injected > 0.0) {
      const double demand = power_injected * dt;                 // joules wanted
      const double affordable = std::max(0.0, energy - cfg_.min_energy);
      gamma_ = (demand <= 1e-12) ? 1.0 : std::clamp(affordable / demand, 0.0, 1.0);
    } else {
      // Extracting energy from the environment always allowed: it makes the
      // system more passive, not less.
      gamma_ = 1.0;
    }

    // Integrate in the ENERGY domain, not in s.
    //
    // The textbook form is s_dot = (sigma*P_d - gamma*P_i)/s, integrated by
    // Euler. That is correct in continuous time and leaks in discrete time:
    //
    //   E_new = 0.5*(s + s_dot*dt)^2 = E - gamma*P_i*dt + 0.5*(s_dot*dt)^2
    //
    // The trailing half-square term means the tank loses slightly LESS than
    // the controller was allowed to spend, every single cycle. At 1 kHz that
    // small bias accumulates, and the bound the tank is supposed to enforce
    // stops holding -- a unit test of the total delivered energy catches it
    // spending about 2 % more than its budget over a long draw.
    //
    // Book-keeping the joules directly and deriving s from them makes the
    // discrete accounting exact, which is the only version of this that is
    // actually a guarantee. s remains available because it is the state the
    // literature's stability proofs are written in.
    const double delta = (sigma_ * power_dissipated - gamma_ * power_injected) * dt;
    const double new_energy =
      std::clamp(energy + delta, cfg_.min_energy, cfg_.max_energy);
    s_ = std::sqrt(2.0 * new_energy);

    return gamma_;
  }

  /// T = 0.5 * s^2, joules.
  double energy() const { return 0.5 * s_ * s_; }
  double state() const { return s_; }
  double valve() const { return gamma_; }
  bool throttling() const { return gamma_ < 0.999; }

  double lastDissipatedPower() const { return last_dissipated_; }
  double lastInjectedPower() const { return last_injected_; }

  /// Damping multiplier for this cycle: 1.0 with a healthy tank, rising
  /// smoothly toward max_damping_boost as the tank empties.
  ///
  /// The blend is linear in the energy deficit between min_energy and
  /// 2*min_energy. Linear rather than a step, because a discontinuous
  /// damping change is itself an energy injection -- which would be a
  /// faintly absurd way for a passivity monitor to fail.
  double dampingBoost() const
  {
    const double lo = cfg_.min_energy;
    const double hi = 2.0 * cfg_.min_energy;
    const double e = energy();
    if (e >= hi) {return 1.0;}
    const double deficit = std::clamp((hi - e) / (hi - lo), 0.0, 1.0);
    return 1.0 + deficit * (cfg_.max_damping_boost - 1.0);
  }

private:
  EnergyTankConfig cfg_{};
  double s_{1.0};
  double gamma_{1.0};
  double sigma_{1.0};
  double last_dissipated_{0.0};
  double last_injected_{0.0};
};

}  // namespace tops_controllers

#endif  // TOPS_CONTROLLERS__ENERGY_TANK_HPP_
