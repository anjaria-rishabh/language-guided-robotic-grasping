// Copyright 2026 Kabir Dhingra. MIT licence.
//
// First-order IIR low-pass filter with an optional deadband and a hard
// output clamp, sized at compile time so it never touches the heap.
//
// Theory/05_Control_Fundamentals.md §5.6 for why this shape, and
// docs/02_CONTROL_LAW.md §4 for how it is used on the wrist wrench.
//
// The one thing worth internalising: filtering is not free. Every bit of
// smoothing you buy is paid for in phase lag, and lag inside a feedback
// loop eats stability margin. Worse, a heavily filtered wrench DELAYS the
// safety cutoff, and delay is distance travelled into whatever you hit.
// That is why this project runs two filters off the same sensor -- a light
// one feeding the interlock, a heavier one feeding the control law.

#ifndef TOPS_CONTROLLERS__LOW_PASS_FILTER_HPP_
#define TOPS_CONTROLLERS__LOW_PASS_FILTER_HPP_

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>

namespace tops_controllers
{

/// First-order low-pass (exponential moving average) over an N-vector.
///
///   y[k] = a * x[k] + (1 - a) * y[k-1]
///
/// with `a` derived from a cutoff frequency rather than picked by feel:
///
///   tau = 1 / (2*pi*f_c)          the time constant
///   a   = dt / (tau + dt)
///
/// Stating a filter as a frequency instead of a magic constant is worth the
/// two lines of algebra; it is also the only form a reviewer can check.
template<int N>
class LowPassFilter
{
public:
  using Vector = Eigen::Matrix<double, N, 1>;

  LowPassFilter()
  {
    y_.setZero();
    clamp_min_ = Vector::Constant(-std::numeric_limits<double>::infinity());
    clamp_max_ = Vector::Constant(std::numeric_limits<double>::infinity());
  }

  /// Set the cutoff in hertz for a known sample period.
  /// A cutoff at or above the Nyquist rate is meaningless, so it is capped
  /// there rather than silently producing alpha > 1 (which would be an
  /// amplifier, not a filter).
  void setCutoff(double cutoff_hz, double dt)
  {
    if (cutoff_hz <= 0.0 || dt <= 0.0) {
      alpha_ = 1.0;  // pass-through
      return;
    }
    const double nyquist = 0.5 / dt;
    cutoff_hz = std::min(cutoff_hz, 0.99 * nyquist);
    const double tau = 1.0 / (2.0 * M_PI * cutoff_hz);
    alpha_ = dt / (tau + dt);
  }

  void setAlpha(double alpha) { alpha_ = std::clamp(alpha, 0.0, 1.0); }
  double alpha() const { return alpha_; }

  /// Effective cutoff, for the report and for logging what actually ran.
  double cutoffHz(double dt) const
  {
    if (alpha_ >= 1.0 || dt <= 0.0) {return std::numeric_limits<double>::infinity();}
    const double tau = dt * (1.0 - alpha_) / alpha_;
    return 1.0 / (2.0 * M_PI * tau);
  }

  /// Values with magnitude below the deadband are treated as zero AFTER
  /// filtering. On an F/T sensor this suppresses the residual bias that
  /// would otherwise make the controller chase a phantom 0.2 N contact
  /// forever. Set to 0 to disable.
  void setDeadband(double deadband) { deadband_ = std::max(0.0, deadband); }

  void setClamp(const Vector & lo, const Vector & hi)
  {
    clamp_min_ = lo;
    clamp_max_ = hi;
  }

  void setSymmetricClamp(double magnitude)
  {
    clamp_min_ = Vector::Constant(-std::abs(magnitude));
    clamp_max_ = Vector::Constant(std::abs(magnitude));
  }

  /// Seed the filter with the first raw sample instead of ramping up from
  /// zero. Without this, activation produces a transient that looks exactly
  /// like a real contact event and can trip the interlock on start-up.
  void reset(const Vector & seed)
  {
    y_ = seed;
    initialised_ = true;
  }

  void reset()
  {
    y_.setZero();
    initialised_ = false;
  }

  bool initialised() const { return initialised_; }

  const Vector & update(const Vector & x)
  {
    if (!initialised_) {
      y_ = x;
      initialised_ = true;
    } else {
      y_ = alpha_ * x + (1.0 - alpha_) * y_;
    }

    if (deadband_ > 0.0) {
      for (int i = 0; i < N; ++i) {
        if (std::abs(y_(i)) < deadband_) {y_(i) = 0.0;}
      }
    }

    y_ = y_.cwiseMax(clamp_min_).cwiseMin(clamp_max_);
    return y_;
  }

  const Vector & value() const { return y_; }

private:
  Vector y_;
  Vector clamp_min_;
  Vector clamp_max_;
  double alpha_{1.0};
  double deadband_{0.0};
  bool initialised_{false};
};

}  // namespace tops_controllers

#endif  // TOPS_CONTROLLERS__LOW_PASS_FILTER_HPP_
