# Copyright 2026 Kabir Dhingra. MIT licence.
"""
Tests for the statistics backend.

The pure-Python fallbacks exist so the pipeline runs without scipy. That is
only defensible if they are actually right, so they are checked here
against published table values rather than against themselves.
"""
import math

import pytest

from tops_experiments import statistics_backend as st


# ---------------------------------------------------------------------------
#  Distribution functions, against standard tables
# ---------------------------------------------------------------------------

@pytest.mark.parametrize('t, df, expected', [
    (2.086, 20, 0.050),
    (2.228, 10, 0.050),
    (2.750, 30, 0.010),
    (12.706, 1, 0.050),
    (1.960, 100000, 0.050),   # converges to the normal
])
def test_t_distribution_matches_published_tables(t, df, expected):
    assert st._t_sf(t, df) == pytest.approx(expected, abs=1e-3)


@pytest.mark.parametrize('chi2, expected', [
    (3.841, 0.050),
    (6.635, 0.010),
    (10.828, 0.001),
])
def test_chi_squared_df1_matches_published_tables(chi2, expected):
    assert st._chi2_sf(chi2, 1) == pytest.approx(expected, abs=1e-3)


def test_normal_cdf():
    assert st._normal_cdf(0.0) == pytest.approx(0.5)
    assert st._normal_cdf(1.96) == pytest.approx(0.975, abs=1e-4)
    assert st._normal_cdf(-1.96) == pytest.approx(0.025, abs=1e-4)


# ---------------------------------------------------------------------------
#  Descriptive statistics
# ---------------------------------------------------------------------------

def test_sample_standard_deviation_uses_n_minus_one():
    # Population sd of [2,4,4,4,5,5,7,9] is 2.0; the SAMPLE sd is 2.138.
    values = [2, 4, 4, 4, 5, 5, 7, 9]
    assert st.stdev(values) == pytest.approx(2.13809, abs=1e-4)


def test_median_handles_both_parities():
    assert st.median([3, 1, 2]) == 2
    assert st.median([4, 1, 3, 2]) == 2.5


def test_cohens_d_sign_and_magnitude():
    a = [10.0] * 10
    b = [12.0] * 10
    # Zero within-group variance makes the pooled sd zero and d undefined.
    assert math.isnan(st.cohens_d(a, b))

    a = [9.0, 10.0, 11.0] * 4
    b = [11.0, 12.0, 13.0] * 4
    d = st.cohens_d(a, b)
    assert d < 0, 'a has the lower mean, so d must be negative'
    assert abs(d) > 0.8, 'a two-unit shift on a one-unit sd is a large effect'


def test_rank_biserial_spans_minus_one_to_one():
    assert st.rank_biserial([1, 2, 3], [4, 5, 6]) == pytest.approx(-1.0)
    assert st.rank_biserial([4, 5, 6], [1, 2, 3]) == pytest.approx(1.0)
    assert st.rank_biserial([1, 2, 3], [1, 2, 3]) == pytest.approx(0.0)


# ---------------------------------------------------------------------------
#  Tests
# ---------------------------------------------------------------------------

def test_welch_on_identical_samples_finds_nothing():
    t, p, _ = st.welch_ttest([1, 2, 3, 4, 5], [1, 2, 3, 4, 5])
    assert t == pytest.approx(0.0)
    assert p == pytest.approx(1.0)


def test_welch_detects_a_clear_separation():
    a = [2.0, 2.2, 2.4, 2.1, 2.3] * 4
    b = [9.0, 9.4, 9.1, 9.3, 9.2] * 4
    _, p, _ = st.welch_ttest(a, b)
    assert p < 0.001


def test_mann_whitney_on_complete_separation():
    u, p, _ = st.mann_whitney([1, 2, 3, 4, 5], [6, 7, 8, 9, 10])
    assert u == pytest.approx(0.0), 'complete separation gives U = 0'
    assert p < 0.05


def test_mann_whitney_handles_ties_without_dividing_by_zero():
    u, p, method = st.mann_whitney([5.0] * 6, [5.0] * 6)
    assert math.isnan(p) or p > 0.5
    assert math.isfinite(u)


def test_paired_test_is_more_powerful_than_unpaired():
    """The reason the trial runner bothers to pair seeds.

    Two conditions with a small consistent difference, on top of large
    trial-to-trial variation that affects both equally. Unpaired, the
    variation swamps the effect; paired, it cancels.
    """
    common = [1.0, 5.0, 9.0, 3.0, 7.0, 2.0, 8.0, 4.0, 6.0, 10.0]
    a = [c + 0.0 for c in common]
    b = [c + 0.6 for c in common]

    _, p_unpaired, _ = st.welch_ttest(a, b)
    _, p_paired, _ = st.paired_ttest(a, b)

    assert p_unpaired > 0.05, 'unpaired should miss this'
    assert p_paired < 0.001, 'paired should find it easily'


def test_paired_test_rejects_mismatched_lengths():
    _, p, method = st.paired_ttest([1, 2, 3], [1, 2])
    assert math.isnan(p)
    assert 'unpaired' in method


def test_wilcoxon_needs_enough_pairs():
    _, p, method = st.wilcoxon_signed_rank([1, 2, 3], [2, 3, 4])
    assert math.isnan(p)


def test_wilcoxon_detects_a_consistent_shift():
    a = list(range(1, 15))
    b = [x + 1.5 for x in a]
    _, p, _ = st.wilcoxon_signed_rank(a, b)
    assert p < 0.01


# ---------------------------------------------------------------------------
#  Proportions
# ---------------------------------------------------------------------------

def test_proportion_test_detects_a_real_difference():
    _, p, _ = st.proportion_test(38, 40, 22, 40)
    assert p < 0.001


def test_proportion_test_on_equal_rates_finds_nothing():
    _, p, _ = st.proportion_test(20, 40, 20, 40)
    assert p > 0.5


def test_proportion_test_survives_a_degenerate_table():
    # Every trial succeeded in both conditions: no variation to test.
    _, p, method = st.proportion_test(40, 40, 40, 40)
    assert p == pytest.approx(1.0) or math.isnan(p)


def test_wilson_interval_stays_inside_zero_one():
    """The reason it is Wilson's and not the textbook normal interval.

    At 40/40 the normal interval's upper bound goes above 1.0, which is not
    a meaningful statement about a success rate. This experiment may well
    hit that case in the clean regime.
    """
    lo, hi = st.wilson_interval(40, 40)
    assert 0.0 <= lo <= hi <= 1.0
    assert lo > 0.9, 'forty out of forty should have a tight lower bound'

    lo, hi = st.wilson_interval(0, 40)
    assert 0.0 <= lo <= hi <= 1.0


def test_wilson_interval_brackets_the_point_estimate():
    lo, hi = st.wilson_interval(34, 40)
    assert lo < 34 / 40 < hi
