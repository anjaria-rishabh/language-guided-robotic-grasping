# Copyright 2026 Kabir Dhingra. MIT licence.
"""
The statistical tests, with a pure-Python fallback.

---------------------------------------------------------------------------
Why a fallback exists at all
---------------------------------------------------------------------------
scipy is the right tool and the analysis uses it whenever it is installed.
But scipy is not in the ROS 2 apt distribution, and a results pipeline that
cannot run until someone remembers `pip install --break-system-packages` is
a pipeline that will not be run on the evening the results are due.

So: scipy when available, and a documented normal-approximation fallback
otherwise. The report says which was used, because it matters -- the
fallbacks are approximations, accurate to a few thousandths of a p-value at
n = 30 per group and less so below n = 10.

Every function returns (statistic, p_value, method_name) so the caller can
print what actually ran rather than implying an exactness it does not have.
"""
import math
from typing import List, Sequence, Tuple

try:
    from scipy import stats as _scipy_stats
    HAVE_SCIPY = True
except ImportError:  # pragma: no cover - depends on the environment
    _scipy_stats = None
    HAVE_SCIPY = False


# ---------------------------------------------------------------------------
#  Distribution functions, so the fallback is not hand-waving
# ---------------------------------------------------------------------------

def _normal_cdf(z: float) -> float:
    return 0.5 * (1.0 + math.erf(z / math.sqrt(2.0)))


def _betacf(a: float, b: float, x: float) -> float:
    """Continued-fraction expansion for the incomplete beta function.

    Lentz's method, as in Numerical Recipes §6.4. Needed for the Student t
    distribution, which has no closed form in the standard library.
    """
    MAXIT, EPS, FPMIN = 200, 3.0e-16, 1.0e-300
    qab, qap, qam = a + b, a + 1.0, a - 1.0
    c = 1.0
    d = 1.0 - qab * x / qap
    if abs(d) < FPMIN:
        d = FPMIN
    d = 1.0 / d
    h = d
    for m in range(1, MAXIT + 1):
        m2 = 2 * m
        aa = m * (b - m) * x / ((qam + m2) * (a + m2))
        d = 1.0 + aa * d
        if abs(d) < FPMIN:
            d = FPMIN
        c = 1.0 + aa / c
        if abs(c) < FPMIN:
            c = FPMIN
        d = 1.0 / d
        h *= d * c
        aa = -(a + m) * (qab + m) * x / ((a + m2) * (qap + m2))
        d = 1.0 + aa * d
        if abs(d) < FPMIN:
            d = FPMIN
        c = 1.0 + aa / c
        if abs(c) < FPMIN:
            c = FPMIN
        d = 1.0 / d
        delta = d * c
        h *= delta
        if abs(delta - 1.0) < EPS:
            break
    return h


def _betai(a: float, b: float, x: float) -> float:
    """Regularised incomplete beta function I_x(a, b)."""
    if x <= 0.0:
        return 0.0
    if x >= 1.0:
        return 1.0
    lbeta = (math.lgamma(a + b) - math.lgamma(a) - math.lgamma(b)
             + a * math.log(x) + b * math.log(1.0 - x))
    if x < (a + 1.0) / (a + b + 2.0):
        return math.exp(lbeta) * _betacf(a, b, x) / a
    return 1.0 - math.exp(lbeta) * _betacf(b, a, 1.0 - x) / b


def _t_sf(t: float, df: float) -> float:
    """Two-sided survival function for Student's t."""
    if df <= 0:
        return float('nan')
    x = df / (df + t * t)
    return _betai(0.5 * df, 0.5, x)


def _chi2_sf(chi2: float, df: int) -> float:
    """Upper tail of the chi-squared distribution.

    Only df = 1 and df = 2 are needed here (a 2x2 contingency table is
    df = 1), and both have closed forms, so the general incomplete gamma is
    not worth carrying.
    """
    if chi2 <= 0:
        return 1.0
    if df == 1:
        return 2.0 * (1.0 - _normal_cdf(math.sqrt(chi2)))
    if df == 2:
        return math.exp(-chi2 / 2.0)
    # Wilson-Hilferty approximation for anything else.
    z = ((chi2 / df) ** (1.0 / 3.0) - (1.0 - 2.0 / (9.0 * df))) / \
        math.sqrt(2.0 / (9.0 * df))
    return 1.0 - _normal_cdf(z)


# ---------------------------------------------------------------------------
#  Descriptive
# ---------------------------------------------------------------------------

def mean(values: Sequence[float]) -> float:
    return sum(values) / len(values) if values else float('nan')


def stdev(values: Sequence[float]) -> float:
    """Sample standard deviation, n-1 in the denominator."""
    n = len(values)
    if n < 2:
        return float('nan')
    m = mean(values)
    return math.sqrt(sum((v - m) ** 2 for v in values) / (n - 1))


def median(values: Sequence[float]) -> float:
    if not values:
        return float('nan')
    s = sorted(values)
    n = len(s)
    mid = n // 2
    return s[mid] if n % 2 else 0.5 * (s[mid - 1] + s[mid])


def cohens_d(a: Sequence[float], b: Sequence[float]) -> float:
    """Standardised mean difference, pooled standard deviation.

    A p-value says a difference is real. It says nothing about whether it
    matters. Cohen's d is the size of the difference in units of the
    variation already present, and it is the number a reader actually needs:
    conventionally 0.2 is small, 0.5 medium, 0.8 large.
    """
    na, nb = len(a), len(b)
    if na < 2 or nb < 2:
        return float('nan')
    va, vb = stdev(a) ** 2, stdev(b) ** 2
    pooled = math.sqrt(((na - 1) * va + (nb - 1) * vb) / (na + nb - 2))
    if pooled == 0.0:
        return float('nan')
    return (mean(a) - mean(b)) / pooled


def rank_biserial(a: Sequence[float], b: Sequence[float]) -> float:
    """Effect size for Mann-Whitney: the probability that a random draw
    from `a` exceeds one from `b`, rescaled to [-1, 1].

    The non-parametric counterpart to Cohen's d, and the right one to quote
    alongside a Mann-Whitney result on a skewed metric like peak force.
    """
    na, nb = len(a), len(b)
    if na == 0 or nb == 0:
        return float('nan')
    wins = sum(1 for x in a for y in b if x > y)
    ties = sum(1 for x in a for y in b if x == y)
    return (2.0 * (wins + 0.5 * ties) / (na * nb)) - 1.0


# ---------------------------------------------------------------------------
#  Tests
# ---------------------------------------------------------------------------

def welch_ttest(a: Sequence[float], b: Sequence[float]) -> Tuple[float, float, str]:
    """Two-sample t-test, Welch's version (unequal variances not assumed).

    Welch rather than Student, always. Student's t assumes the two groups
    have the same variance, and that assumption is specifically wrong here:
    the whole hypothesis is that the compliant controller produces forces
    that are both lower AND less variable. Using the test that assumes
    equal variance to test a claim about unequal variance would be an
    unforced error.
    """
    if len(a) < 2 or len(b) < 2:
        return float('nan'), float('nan'), 'insufficient data'
    if HAVE_SCIPY:
        t, p = _scipy_stats.ttest_ind(a, b, equal_var=False)
        return float(t), float(p), "Welch's t (scipy)"

    na, nb = len(a), len(b)
    va, vb = stdev(a) ** 2, stdev(b) ** 2
    se = math.sqrt(va / na + vb / nb)
    if se == 0.0:
        return float('nan'), float('nan'), 'zero variance'
    t = (mean(a) - mean(b)) / se
    # Welch-Satterthwaite degrees of freedom
    df = (va / na + vb / nb) ** 2 / (
        (va / na) ** 2 / (na - 1) + (vb / nb) ** 2 / (nb - 1))
    return t, _t_sf(t, df), "Welch's t (fallback)"


def paired_ttest(a: Sequence[float], b: Sequence[float]) -> Tuple[float, float, str]:
    """Paired t-test on matched trials."""
    if len(a) != len(b) or len(a) < 2:
        return float('nan'), float('nan'), 'unpaired or insufficient data'
    if HAVE_SCIPY:
        t, p = _scipy_stats.ttest_rel(a, b)
        return float(t), float(p), 'paired t (scipy)'

    diffs = [x - y for x, y in zip(a, b)]
    n = len(diffs)
    sd = stdev(diffs)
    if sd == 0.0:
        return float('nan'), float('nan'), 'zero variance in differences'
    t = mean(diffs) / (sd / math.sqrt(n))
    return t, _t_sf(t, n - 1), 'paired t (fallback)'


def mann_whitney(a: Sequence[float], b: Sequence[float]) -> Tuple[float, float, str]:
    """Mann-Whitney U: compares by rank, assumes nothing about the shape.

    The safer choice for a skewed metric like peak force, where a handful of
    large values carry most of the physical interest and would dominate a
    mean. Run alongside the t-test on purpose: if the two agree the result
    is robust to the distributional assumption; if they disagree, outliers
    are driving the t-test and the right response is to look at the data,
    not to pick the answer you prefer.
    """
    na, nb = len(a), len(b)
    if na == 0 or nb == 0:
        return float('nan'), float('nan'), 'insufficient data'
    if HAVE_SCIPY:
        u, p = _scipy_stats.mannwhitneyu(a, b, alternative='two-sided')
        return float(u), float(p), 'Mann-Whitney U (scipy, exact/tie-corrected)'

    combined = sorted([(v, 0) for v in a] + [(v, 1) for v in b])
    ranks = _average_ranks([v for v, _ in combined])
    rank_a = sum(r for r, (_, g) in zip(ranks, combined) if g == 0)
    u_a = rank_a - na * (na + 1) / 2.0
    u = min(u_a, na * nb - u_a)

    mu = na * nb / 2.0
    # Tie-corrected variance. Ignoring ties inflates the variance and makes
    # the test conservative, which on success-rate-adjacent data with many
    # repeated values is a real effect rather than a technicality.
    tie_term = _tie_correction([v for v, _ in combined])
    n = na + nb
    sigma_sq = (na * nb / 12.0) * ((n + 1) - tie_term / (n * (n - 1)))
    if sigma_sq <= 0:
        return u, float('nan'), 'degenerate (all values identical)'
    z = (u - mu + 0.5) / math.sqrt(sigma_sq)   # continuity correction
    return u, 2.0 * _normal_cdf(-abs(z)), 'Mann-Whitney U (normal approximation)'


def wilcoxon_signed_rank(a: Sequence[float],
                         b: Sequence[float]) -> Tuple[float, float, str]:
    """Paired non-parametric test on matched trials."""
    if len(a) != len(b) or len(a) < 6:
        return float('nan'), float('nan'), 'unpaired or too few pairs'
    if HAVE_SCIPY:
        w, p = _scipy_stats.wilcoxon(a, b)
        return float(w), float(p), 'Wilcoxon signed-rank (scipy)'

    diffs = [x - y for x, y in zip(a, b) if x != y]
    n = len(diffs)
    if n < 6:
        return float('nan'), float('nan'), 'too few non-zero differences'
    ranks = _average_ranks([abs(d) for d in diffs])
    w_plus = sum(r for r, d in zip(ranks, diffs) if d > 0)
    w_minus = sum(r for r, d in zip(ranks, diffs) if d < 0)
    w = min(w_plus, w_minus)
    mu = n * (n + 1) / 4.0
    sigma = math.sqrt(n * (n + 1) * (2 * n + 1) / 24.0)
    if sigma == 0:
        return w, float('nan'), 'degenerate'
    z = (w - mu + 0.5) / sigma
    return w, 2.0 * _normal_cdf(-abs(z)), 'Wilcoxon signed-rank (normal approximation)'


def proportion_test(successes_a: int, n_a: int,
                    successes_b: int, n_b: int) -> Tuple[float, float, str]:
    """Compare two success rates.

    Success is a proportion, so neither a t-test nor Mann-Whitney is
    appropriate for it -- a point worth making explicitly because printing
    a success rate next to a t-test p-value invites exactly that confusion.
    Fisher's exact test is used when scipy is present (correct at any n);
    otherwise a chi-squared test with Yates' continuity correction.
    """
    if n_a == 0 or n_b == 0:
        return float('nan'), float('nan'), 'insufficient data'

    table = [[successes_a, n_a - successes_a],
             [successes_b, n_b - successes_b]]

    if HAVE_SCIPY:
        odds, p = _scipy_stats.fisher_exact(table)
        return float(odds), float(p), "Fisher's exact"

    total = n_a + n_b
    successes = successes_a + successes_b
    failures = total - successes
    if successes == 0 or failures == 0:
        return float('nan'), 1.0, 'degenerate (no variation in outcome)'

    chi2 = 0.0
    for i, row_total in enumerate((n_a, n_b)):
        for j, col_total in enumerate((successes, failures)):
            expected = row_total * col_total / total
            observed = table[i][j]
            chi2 += (abs(observed - expected) - 0.5) ** 2 / expected
    return chi2, _chi2_sf(chi2, 1), "chi-squared with Yates' correction"


def wilson_interval(successes: int, n: int, z: float = 1.96) -> Tuple[float, float]:
    """95 % confidence interval for a proportion, Wilson's method.

    Not the textbook normal interval. At the rates this project targets --
    85 % and above, with 30 trials per cell -- the normal interval's upper
    bound goes above 1.0, which is not a meaningful statement about a
    success rate. Wilson's stays inside [0, 1] and behaves sensibly when
    every trial succeeded, which is a case this experiment may well hit.
    """
    if n == 0:
        return float('nan'), float('nan')
    p = successes / n
    denom = 1.0 + z * z / n
    centre = (p + z * z / (2 * n)) / denom
    spread = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / denom
    return max(0.0, centre - spread), min(1.0, centre + spread)


# ---------------------------------------------------------------------------
#  helpers
# ---------------------------------------------------------------------------

def _average_ranks(values: Sequence[float]) -> List[float]:
    """Ranks with ties averaged, which is what both rank tests require."""
    indexed = sorted(range(len(values)), key=lambda i: values[i])
    ranks = [0.0] * len(values)
    i = 0
    while i < len(indexed):
        j = i
        while j + 1 < len(indexed) and values[indexed[j + 1]] == values[indexed[i]]:
            j += 1
        average = (i + j + 2) / 2.0     # ranks are 1-based
        for k in range(i, j + 1):
            ranks[indexed[k]] = average
        i = j + 1
    return ranks


def _tie_correction(values: Sequence[float]) -> float:
    """sum(t^3 - t) over tie groups, for the Mann-Whitney variance."""
    total = 0.0
    run = 1
    for i in range(1, len(values)):
        if values[i] == values[i - 1]:
            run += 1
        else:
            total += run ** 3 - run
            run = 1
    total += run ** 3 - run
    return total
