// Copyright 2026 Kabir Dhingra. MIT licence.
//
// Slew-rate-limited gain scheduling, plus hysteresis on the mode trigger.
// Theory/06_Compliant_Control.md §6.9, docs/02_CONTROL_LAW.md §5.
//
// Two small mechanisms, both of which exist because of a specific failure:
//
// 1. RAMPING. Stepping stiffness from 800 N/m to 150 N/m while the arm is
//    20 mm from equilibrium instantaneously changes the stored potential
//    energy 0.5*e^T*K*e by about 0.13 J. Going the other way CREATES that
//    energy. The arm spends it as motion, which is the "kick on mode
//    switch" that looks like a controller bug and is actually physics.
//    Ramping spreads the change over enough time that the energy tank can
//    account for it and the damper can absorb it.
//
// 2. HYSTERESIS. The compliant mode triggers on a noisy quantity (contact
//    force, or distance from a vision-estimated target). A single threshold
//    on a noisy signal chatters: stiff, compliant, stiff, compliant, many
//    times a second, each switch injecting a little energy. Separate
//    enter/exit thresholds make that impossible.

#ifndef TOPS_CONTROLLERS__GAIN_RAMP_HPP_
#define TOPS_CONTROLLERS__GAIN_RAMP_HPP_

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

namespace tops_controllers
{

/// Moves a vector of gains toward a target at a bounded rate.
template<int N>
class GainRamp
{
public:
  using Vector = Eigen::Matrix<double, N, 1>;

  GainRamp()
  {
    current_.setZero();
    target_.setZero();
    max_rate_.setConstant(1e9);  // effectively unlimited until configured
  }

  /// @param max_rate per-element maximum change per second (units/s).
  void setMaxRate(const Vector & max_rate) { max_rate_ = max_rate.cwiseAbs(); }
  void setMaxRate(double max_rate) { max_rate_.setConstant(std::abs(max_rate)); }

  /// Jump straight to a value. Only legitimate on activation, when there is
  /// no stored spring energy to disturb because there is no command yet.
  void reset(const Vector & value)
  {
    current_ = value;
    target_ = value;
  }

  void setTarget(const Vector & target) { target_ = target; }
  const Vector & target() const { return target_; }

  const Vector & update(double dt)
  {
    if (dt <= 0.0) {return current_;}
    const Vector step = max_rate_ * dt;
    const Vector delta = target_ - current_;
    current_ += delta.cwiseMax(-step).cwiseMin(step);
    return current_;
  }

  const Vector & value() const { return current_; }

  /// True once the ramp has arrived, within a relative tolerance. The
  /// supervisor waits on this before declaring the mode switch complete,
  /// so "we are in compliant mode" means the gains are actually applied,
  /// not merely requested.
  bool settled(double rel_tol = 1e-3) const
  {
    const double scale = std::max(1.0, target_.cwiseAbs().maxCoeff());
    return (target_ - current_).cwiseAbs().maxCoeff() <= rel_tol * scale;
  }

  /// Worst-case seconds to reach the current target from where we are.
  double timeToTarget() const
  {
    double worst = 0.0;
    for (int i = 0; i < N; ++i) {
      if (max_rate_(i) > 1e-12) {
        worst = std::max(worst, std::abs(target_(i) - current_(i)) / max_rate_(i));
      }
    }
    return worst;
  }

private:
  Vector current_;
  Vector target_;
  Vector max_rate_;
};

/// Schmitt trigger: separate enter and exit thresholds on a noisy scalar.
///
/// Convention here is "rises to enter": the trigger asserts when the signal
/// goes ABOVE enter_threshold and de-asserts when it falls BELOW
/// exit_threshold. Construct with `rising = false` for the opposite sense,
/// which is what the distance-to-target trigger needs (it asserts as the
/// distance SHRINKS below a radius).
class HysteresisTrigger
{
public:
  HysteresisTrigger() = default;

  HysteresisTrigger(double enter_threshold, double exit_threshold, bool rising = true)
  {
    configure(enter_threshold, exit_threshold, rising);
  }

  void configure(double enter_threshold, double exit_threshold, bool rising = true)
  {
    enter_ = enter_threshold;
    exit_ = exit_threshold;
    rising_ = rising;
    active_ = false;
  }

  /// A minimum dwell time in each state, on top of the threshold gap. The
  /// gap handles amplitude noise; the dwell handles a signal that crosses
  /// cleanly but repeatedly, such as a genuine light tapping contact.
  void setMinDwell(double seconds) { min_dwell_ = std::max(0.0, seconds); }

  bool update(double signal, double dt)
  {
    time_in_state_ += dt;
    const bool want = rising_
      ? (active_ ? (signal > exit_) : (signal > enter_))
      : (active_ ? (signal < exit_) : (signal < enter_));

    if (want != active_ && time_in_state_ >= min_dwell_) {
      active_ = want;
      time_in_state_ = 0.0;
    }
    return active_;
  }

  bool active() const { return active_; }
  double timeInState() const { return time_in_state_; }
  void reset(bool active = false)
  {
    active_ = active;
    time_in_state_ = 0.0;
  }

private:
  double enter_{0.0};
  double exit_{0.0};
  double min_dwell_{0.0};
  double time_in_state_{0.0};
  bool rising_{true};
  bool active_{false};
};

}  // namespace tops_controllers

#endif  // TOPS_CONTROLLERS__GAIN_RAMP_HPP_
