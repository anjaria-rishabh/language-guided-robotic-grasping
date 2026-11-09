// Copyright 2026 Kabir Dhingra. MIT licence.

#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "tops_controllers/low_pass_filter.hpp"

using tops_controllers::LowPassFilter;
using Vec6 = Eigen::Matrix<double, 6, 1>;

TEST(LowPassFilter, CutoffAndAlphaAreConsistent)
{
  // alpha = dt / (tau + dt), tau = 1 / (2*pi*fc). At dt = 1 ms and
  // fc = 30 Hz: tau = 5.305 ms, alpha = 0.1587.
  LowPassFilter<6> f;
  f.setCutoff(30.0, 0.001);
  EXPECT_NEAR(f.alpha(), 0.15866, 1e-4);

  // And the cutoff must round-trip back out, so the number quoted in the
  // report is the number the filter actually ran at.
  EXPECT_NEAR(f.cutoffHz(0.001), 30.0, 1e-6);
}

TEST(LowPassFilter, CutoffIsCappedAtNyquist)
{
  // Asking for a cutoff above Nyquist is meaningless. Silently producing
  // alpha > 1 would make this an amplifier, not a filter.
  LowPassFilter<6> f;
  f.setCutoff(10000.0, 0.001);   // Nyquist is 500 Hz
  EXPECT_LE(f.alpha(), 1.0);
  // Capped at 0.99 * Nyquist = 495 Hz, which is alpha = 0.757. Note that
  // alpha does NOT approach 1 at Nyquist: a first-order IIR still smooths
  // appreciably there, which is worth knowing before quoting a cutoff near
  // the sample rate as though it were pass-through.
  EXPECT_NEAR(f.cutoffHz(0.001), 495.0, 1.0);
  EXPECT_NEAR(f.alpha(), 0.7567, 1e-3);
}

TEST(LowPassFilter, SeedsOnTheFirstSampleWithNoTransient)
{
  // Without this, activation produces a ramp from zero that looks exactly
  // like a real contact event and can trip the interlock on start-up.
  LowPassFilter<6> f;
  f.setAlpha(0.1);
  Vec6 x = Vec6::Constant(7.0);
  EXPECT_NEAR(f.update(x)(0), 7.0, 1e-12);
}

TEST(LowPassFilter, ConvergesToAConstantInput)
{
  LowPassFilter<6> f;
  f.setAlpha(0.2);
  f.reset(Vec6::Zero());
  Vec6 x = Vec6::Constant(5.0);
  for (int i = 0; i < 200; ++i) {f.update(x);}
  EXPECT_NEAR(f.value()(0), 5.0, 1e-6);
}

TEST(LowPassFilter, AttenuatesAtTheExpectedRatio)
{
  // A first-order low-pass should attenuate a sinusoid at the cutoff by
  // -3 dB, i.e. a gain of 1/sqrt(2) = 0.707. This is the test that says the
  // filter is the filter the docstring claims.
  const double dt = 0.001;
  const double fc = 20.0;
  LowPassFilter<6> f;
  f.setCutoff(fc, dt);
  f.reset(Vec6::Zero());

  double peak = 0.0;
  const int n = 4000;
  for (int k = 0; k < n; ++k) {
    Vec6 x = Vec6::Zero();
    x(0) = std::sin(2.0 * M_PI * fc * k * dt);
    const double y = f.update(x)(0);
    if (k > n / 2) {peak = std::max(peak, std::abs(y));}   // after the transient
  }
  EXPECT_NEAR(peak, 0.707, 0.03);
}

TEST(LowPassFilter, TheSafetyPathLagsLessThanTheControlPath)
{
  // The reason two filters exist. Step both, and the lightly filtered one
  // must reach a contact threshold measurably sooner -- that difference is
  // how much further the arm travels into an object before the interlock
  // fires.
  const double dt = 0.001;
  LowPassFilter<6> control;
  LowPassFilter<6> safety;
  control.setCutoff(30.0, dt);
  safety.setCutoff(120.0, dt);
  control.reset(Vec6::Zero());
  safety.reset(Vec6::Zero());

  const Vec6 step = Vec6::Constant(20.0);
  const double threshold = 15.0;
  int k_control = -1;
  int k_safety = -1;
  for (int k = 0; k < 500; ++k) {
    if (k_control < 0 && control.update(step)(0) >= threshold) {k_control = k;}
    if (k_safety < 0 && safety.update(step)(0) >= threshold) {k_safety = k;}
  }
  ASSERT_GE(k_control, 0);
  ASSERT_GE(k_safety, 0);
  EXPECT_LT(k_safety, k_control);
}

TEST(LowPassFilter, ClampBoundsTheOutputUnconditionally)
{
  LowPassFilter<6> f;
  f.setAlpha(1.0);                 // pass-through, so only the clamp acts
  f.setSymmetricClamp(10.0);
  f.reset(Vec6::Zero());
  EXPECT_NEAR(f.update(Vec6::Constant(1000.0))(0), 10.0, 1e-12);
  EXPECT_NEAR(f.update(Vec6::Constant(-1000.0))(0), -10.0, 1e-12);
}

TEST(LowPassFilter, DeadbandSuppressesResidualBias)
{
  LowPassFilter<6> f;
  f.setAlpha(1.0);
  f.setDeadband(0.5);
  f.reset(Vec6::Zero());
  EXPECT_DOUBLE_EQ(f.update(Vec6::Constant(0.3))(0), 0.0);
  EXPECT_NEAR(f.update(Vec6::Constant(2.0))(0), 2.0, 1e-12);
}

TEST(LowPassFilter, ReducesNoiseVarianceOnAConstantSignal)
{
  std::mt19937 rng(42);
  std::normal_distribution<double> noise(0.0, 2.0);

  LowPassFilter<6> f;
  f.setAlpha(0.1);
  f.reset(Vec6::Constant(6.0));

  double sum_sq = 0.0;
  const int n = 5000;
  for (int i = 0; i < n; ++i) {
    Vec6 x = Vec6::Constant(6.0 + noise(rng));
    const double err = f.update(x)(0) - 6.0;
    sum_sq += err * err;
  }
  const double rms = std::sqrt(sum_sq / n);
  // Theory for a first-order IIR: output variance is alpha/(2-alpha) times
  // input variance, so sigma 2.0 at alpha 0.1 gives about 0.46.
  EXPECT_LT(rms, 0.7);
  EXPECT_GT(rms, 0.2);
}
