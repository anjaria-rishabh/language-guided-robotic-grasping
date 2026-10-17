#!/usr/bin/env python3
# Copyright 2026 Kabir Dhingra. MIT licence.
"""
The results pipeline: reads the trial CSV, produces the report tables.

Theory/10_Experiments_and_Statistics.md.

    ros2 run tops_experiments analyze_trials -- --demo
    ros2 run tops_experiments analyze_trials -- --csv results/trials.csv
    ros2 run tops_experiments analyze_trials -- --csv trials.csv --markdown results.md

---------------------------------------------------------------------------
Run this before you have any data
---------------------------------------------------------------------------
`--demo` generates a synthetic CSV with a known effect and analyses it. The
point is to have this code working and trusted in week 2, not week 10.
Debugging a statistics pipeline and generating the results you are about to
submit, at the same time, on the same evening, is how deadlines are missed.

---------------------------------------------------------------------------
What it reports, and why each piece is there
---------------------------------------------------------------------------
  success rate       with a Wilson interval, tested with Fisher's exact --
                     because success is a proportion and neither a t-test
                     nor Mann-Whitney is the right tool for one.
  peak force         Welch's t AND Mann-Whitney, plus Cohen's d and the
                     rank-biserial correlation. Two tests because their
                     agreement is itself informative.
  paired tests       used automatically wherever the same (regime, seed)
                     appears under both controllers, which the trial runner
                     arranges deliberately.
  per regime         because the headline number averages over four very
                     different situations, and the interesting result is
                     almost certainly that the gap widens under calibration
                     error.
"""
import argparse
import csv
import math
import os
import random
import sys
from collections import defaultdict
from typing import Dict, List, Optional, Sequence

from tops_experiments import statistics_backend as st

#: The plan's targets. Printed against every measured value, so a result
#: that misses is visible rather than buried.
TARGETS = {
    'success_rate_overall': 0.85,
    'success_rate_clean': 0.95,
    'peak_force_n': 10.0,
    'targeting_error_mm': 3.0,
    'exec_time_s': 15.0,
}

CONTINUOUS_METRICS = (
    ('peak_force_n', 'Peak contact force', 'N', 'lower', TARGETS['peak_force_n']),
    ('targeting_error_mm', 'Terminal targeting error', 'mm', 'lower',
     TARGETS['targeting_error_mm']),
    ('exec_time_s', 'Trial execution time', 's', 'lower', TARGETS['exec_time_s']),
    ('mean_contact_force_n', 'Mean force while in contact', 'N', 'lower', None),
)


# ---------------------------------------------------------------------------
#  loading
# ---------------------------------------------------------------------------

def load(path: str) -> List[dict]:
    with open(path, newline='') as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise SystemExit(f'{path} contains no data rows')

    numeric = ('peak_force_n', 'mean_contact_force_n', 'exec_time_s',
               'targeting_error_mm', 'tank_min_energy_j', 'max_update_period_ms')
    integer = ('trial_id', 'seed', 'success', 'force_limit_tripped',
               'watchdog_tripped', 'passivity_throttled', 'missed_deadlines')

    for row in rows:
        for key in numeric:
            row[key] = _to_float(row.get(key))
        for key in integer:
            row[key] = _to_int(row.get(key))
    return rows


def _to_float(value) -> float:
    try:
        f = float(value)
        return f if math.isfinite(f) else float('nan')
    except (TypeError, ValueError):
        return float('nan')


def _to_int(value) -> int:
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return 0


def _clean(values: Sequence[float]) -> List[float]:
    """Drop NaNs, and say how many were dropped where it matters.

    targeting_error_mm is NaN for any trial that never reached GRASP, which
    is correct -- there was no terminal error because there was no terminal
    moment. Silently treating those as zero would flatter the failures.
    """
    return [v for v in values if isinstance(v, float) and math.isfinite(v)]


# ---------------------------------------------------------------------------
#  reporting
# ---------------------------------------------------------------------------

class Report:
    """Accumulates lines for stdout and, optionally, a Markdown file."""

    def __init__(self):
        self.lines: List[str] = []

    def __call__(self, text: str = ''):
        print(text)
        self.lines.append(text)

    def rule(self, char: str = '='):
        self(char * 78)

    def heading(self, text: str):
        self()
        self.rule()
        self(f'  {text}')
        self.rule()

    def save(self, path: str):
        os.makedirs(os.path.dirname(os.path.abspath(path)) or '.', exist_ok=True)
        with open(path, 'w') as handle:
            handle.write('```\n')
            handle.write('\n'.join(self.lines))
            handle.write('\n```\n')
        print(f'\nwritten: {path}')


def summarise_modes(rows: List[dict], report: Report):
    modes = sorted({row['mode'] for row in rows})

    report.heading('1.  OVERVIEW')
    report(f'{len(rows)} trials, {len(modes)} modes, '
           f'{len(set(row["regime"] for row in rows))} regimes')
    report(f'statistics backend: '
           f'{"scipy" if st.HAVE_SCIPY else "pure-Python fallback (approximate)"}')
    if not st.HAVE_SCIPY:
        report('  NOTE: p-values below come from normal approximations. They are')
        report('  accurate to a few thousandths at n >= 30 per group and less so')
        report('  below n = 10. Install scipy before quoting them in the report.')

    report()
    report(f'{"mode":<13} {"n":>4} {"success":>9} {"95% CI":>16} '
           f'{"peak F (N)":>12} {"error (mm)":>12} {"time (s)":>10}')
    report('-' * 78)

    for mode in modes:
        subset = [r for r in rows if r['mode'] == mode]
        n = len(subset)
        successes = sum(r['success'] for r in subset)
        rate = successes / n if n else float('nan')
        lo, hi = st.wilson_interval(successes, n)
        peak = _clean([r['peak_force_n'] for r in subset])
        err = _clean([r['targeting_error_mm'] for r in subset])
        t = _clean([r['exec_time_s'] for r in subset])
        report(
            f'{mode:<13} {n:>4} {rate:>8.1%} '
            f'{f"[{lo:.2f}, {hi:.2f}]":>16} '
            f'{st.mean(peak):>7.2f}+-{st.stdev(peak):<4.2f} '
            f'{st.mean(err):>7.2f}+-{st.stdev(err):<4.2f} '
            f'{st.mean(t):>9.2f}')

    report()
    report('Targets: success >= 85 % overall and >= 95 % clean; peak force')
    report('<= 10.0 N; terminal error <= 3.0 mm; execution time <= 15.0 s.')


def targets_table(rows: List[dict], report: Report):
    report.heading('2.  AGAINST THE PLAN\'S TARGETS')
    modes = sorted({row['mode'] for row in rows})

    report(f'{"mode":<13} {"metric":<26} {"measured":>12} {"target":>10}  verdict')
    report('-' * 78)
    for mode in modes:
        subset = [r for r in rows if r['mode'] == mode]
        n = len(subset)
        successes = sum(r['success'] for r in subset)
        rate = successes / n if n else 0.0
        _verdict_row(report, mode, 'grasp success rate',
                     rate, TARGETS['success_rate_overall'], 'higher', '{:.1%}')

        clean = [r for r in subset if r['regime'] == 'clean']
        if clean:
            clean_rate = sum(r['success'] for r in clean) / len(clean)
            _verdict_row(report, '', 'grasp success (clean only)',
                         clean_rate, TARGETS['success_rate_clean'], 'higher', '{:.1%}')

        for key, label, unit, direction, target in CONTINUOUS_METRICS:
            if target is None:
                continue
            values = _clean([r[key] for r in subset])
            if not values:
                continue
            # The worst case, not the mean. A peak-force LIMIT that holds on
            # average is not a limit.
            worst = max(values)
            _verdict_row(report, '', f'{label.lower()} (worst)',
                         worst, target, direction, '{:.2f} ' + unit)
        report('-' * 78)


def _verdict_row(report, mode, label, value, target, direction, fmt):
    ok = (value >= target) if direction == 'higher' else (value <= target)
    verdict = 'PASS' if ok else 'MISS'
    report(f'{mode:<13} {label:<26} {fmt.format(value):>12} '
           f'{fmt.format(target):>10}  {verdict}')


def compare(rows: List[dict], baseline: str, treatment: str, report: Report):
    a = [r for r in rows if r['mode'] == treatment]
    b = [r for r in rows if r['mode'] == baseline]
    if not a or not b:
        return

    report.heading(f'3.  {treatment.upper()}  vs  {baseline.upper()}')

    # --- success rate ---------------------------------------------------
    sa, na = sum(r['success'] for r in a), len(a)
    sb, nb = sum(r['success'] for r in b), len(b)
    stat, p, method = st.proportion_test(sa, na, sb, nb)
    report(f'Grasp success:  {treatment} {sa}/{na} ({sa/na:.1%})   '
           f'{baseline} {sb}/{nb} ({sb/nb:.1%})')
    report(f'  {method}: p = {_fmt_p(p)}   {_significance(p)}')
    report('  (success is a proportion, so this is deliberately NOT a t-test)')

    # --- continuous metrics ---------------------------------------------
    pairs = _matched_pairs(a, b)
    if pairs:
        report()
        report(f'{len(pairs)} matched (regime, seed) pairs found -- paired tests '
               'are used where possible.')
        report('Pairing cancels trial-to-trial variation and is substantially')
        report('more powerful than the unpaired equivalent for the same n.')

    for key, label, unit, direction, target in CONTINUOUS_METRICS:
        va = _clean([r[key] for r in a])
        vb = _clean([r[key] for r in b])
        if len(va) < 2 or len(vb) < 2:
            continue

        report()
        report(f'--- {label} ({unit}) ---')
        report(f'  {treatment:<12} mean {st.mean(va):7.3f}  '
               f'sd {st.stdev(va):6.3f}  median {st.median(va):7.3f}  '
               f'max {max(va):7.3f}  n={len(va)}')
        report(f'  {baseline:<12} mean {st.mean(vb):7.3f}  '
               f'sd {st.stdev(vb):6.3f}  median {st.median(vb):7.3f}  '
               f'max {max(vb):7.3f}  n={len(vb)}')

        delta = st.mean(va) - st.mean(vb)
        pct = 100.0 * delta / st.mean(vb) if st.mean(vb) else float('nan')
        report(f'  difference   {delta:+7.3f} {unit}  ({pct:+.1f} %)')

        t_stat, t_p, t_method = st.welch_ttest(va, vb)
        u_stat, u_p, u_method = st.mann_whitney(va, vb)
        report(f'  {t_method:<38} t = {t_stat:7.3f}  p = {_fmt_p(t_p)}  '
               f'{_significance(t_p)}')
        report(f'  {u_method:<38} U = {u_stat:7.1f}  p = {_fmt_p(u_p)}  '
               f'{_significance(u_p)}')

        if math.isfinite(t_p) and math.isfinite(u_p):
            agree = (t_p < 0.05) == (u_p < 0.05)
            report('  the two tests ' + (
                'AGREE, so the result is robust to the distributional assumption'
                if agree else
                'DISAGREE -- outliers or skew are driving the t-test. Look at '
                'the data rather than picking the answer you prefer.'))

        d = st.cohens_d(va, vb)
        r_rb = st.rank_biserial(va, vb)
        report(f"  Cohen's d = {d:+.3f} ({_effect_label(abs(d))}), "
               f'rank-biserial r = {r_rb:+.3f}')
        report('  (a p-value says the difference is real; the effect size says '
               'whether it matters)')

        # --- paired, where available ------------------------------------
        pa, pb = _paired_values(pairs, key)
        if len(pa) >= 6:
            pt, pp, pmethod = st.paired_ttest(pa, pb)
            wt, wp, wmethod = st.wilcoxon_signed_rank(pa, pb)
            report(f'  {pmethod:<38} t = {pt:7.3f}  p = {_fmt_p(pp)}  '
                   f'{_significance(pp)}   (n={len(pa)} pairs)')
            report(f'  {wmethod:<38} W = {wt:7.1f}  p = {_fmt_p(wp)}  '
                   f'{_significance(wp)}')


def per_regime(rows: List[dict], report: Report):
    report.heading('4.  BY DISTURBANCE REGIME')
    report('The headline numbers average over four very different situations.')
    report('The result worth reporting is how the gap CHANGES across them: the')
    report('clean baseline should show little difference, and the calibration')
    report('regime -- a fixed offset that does not average out -- is where the')
    report('compliance is supposed to earn its place.')

    modes = sorted({row['mode'] for row in rows})
    regimes = ['clean', 'sensor_noise', 'calibration', 'displacement']
    regimes = [r for r in regimes if any(row['regime'] == r for row in rows)]

    for metric, label, unit in (('success', 'success rate', ''),
                                ('peak_force_n', 'mean peak force', 'N'),
                                ('targeting_error_mm', 'mean terminal error', 'mm')):
        report()
        report(f'{label}{" (" + unit + ")" if unit else ""}')
        header = f'{"regime":<15}' + ''.join(f'{m:>14}' for m in modes)
        report(header)
        report('-' * len(header))
        for regime in regimes:
            cells = []
            for mode in modes:
                subset = [r for r in rows
                          if r['regime'] == regime and r['mode'] == mode]
                if not subset:
                    cells.append(f'{"--":>14}')
                elif metric == 'success':
                    rate = sum(r['success'] for r in subset) / len(subset)
                    cells.append(f'{rate:>13.1%} ')
                else:
                    values = _clean([r[metric] for r in subset])
                    cells.append(f'{st.mean(values):>14.2f}' if values
                                 else f'{"--":>14}')
            report(f'{regime:<15}' + ''.join(cells))


def diagnostics(rows: List[dict], report: Report):
    report.heading('5.  CONTROLLER HEALTH AND FAULTS')
    report('Not results -- evidence that the results are trustworthy. A run')
    report('with hundreds of missed deadlines is not a 1 kHz experiment, and')
    report('saying so is better than having a reviewer notice.')
    report()

    modes = sorted({row['mode'] for row in rows})
    report(f'{"mode":<13} {"force trips":>12} {"watchdog":>10} '
           f'{"throttled":>11} {"max dt (ms)":>12} {"missed":>9}')
    report('-' * 78)
    for mode in modes:
        subset = [r for r in rows if r['mode'] == mode]
        dt = _clean([r['max_update_period_ms'] for r in subset])
        report(f'{mode:<13} '
               f'{sum(r["force_limit_tripped"] for r in subset):>12} '
               f'{sum(r["watchdog_tripped"] for r in subset):>10} '
               f'{sum(r["passivity_throttled"] for r in subset):>11} '
               f'{max(dt) if dt else float("nan"):>12.2f} '
               f'{max((r["missed_deadlines"] for r in subset), default=0):>9}')

    faults = defaultdict(int)
    for row in rows:
        for fault in (row.get('fault') or '').split(';'):
            if fault:
                faults[fault] += 1
    if faults:
        report()
        report('fault counts across all trials:')
        for fault, count in sorted(faults.items(), key=lambda kv: -kv[1]):
            report(f'  {fault:<28} {count:>4}')

    tank = _clean([r['tank_min_energy_j'] for r in rows
                   if r['mode'] == 'impedance'])
    if tank:
        report()
        report(f'energy tank, impedance mode: lowest observed '
               f'{min(tank):.3f} J across {len(tank)} trials')
        report('(the floor is 0.05 J; reaching it means the passivity monitor')
        report(' actively throttled the controller, which is the mechanism')
        report(' working, not a fault)')


# ---------------------------------------------------------------------------
#  pairing
# ---------------------------------------------------------------------------

def _matched_pairs(a: List[dict], b: List[dict]) -> List[tuple]:
    """Trials from the two modes that saw the identical disturbance."""
    index_b = {(r['regime'], r['seed']): r for r in b}
    pairs = []
    for row in a:
        key = (row['regime'], row['seed'])
        if key in index_b:
            pairs.append((row, index_b[key]))
    return pairs


def _paired_values(pairs, key):
    va, vb = [], []
    for ra, rb in pairs:
        x, y = ra[key], rb[key]
        if math.isfinite(x) and math.isfinite(y):
            va.append(x)
            vb.append(y)
    return va, vb


def _fmt_p(p: float) -> str:
    if not math.isfinite(p):
        return '   n/a'
    if p < 1e-4:
        return '<0.0001'
    return f'{p:.4f}'


def _significance(p: float) -> str:
    if not math.isfinite(p):
        return ''
    if p < 0.001:
        return '***'
    if p < 0.01:
        return '**'
    if p < 0.05:
        return '*'
    return '(not significant at 0.05)'


def _effect_label(d: float) -> str:
    if not math.isfinite(d):
        return 'undefined'
    if d < 0.2:
        return 'negligible'
    if d < 0.5:
        return 'small'
    if d < 0.8:
        return 'medium'
    return 'large'


# ---------------------------------------------------------------------------
#  demo data
# ---------------------------------------------------------------------------

def write_demo(path: str, per_cell: int = 10, seed: int = 0) -> str:
    """Synthetic trials with a KNOWN effect, for validating this pipeline.

    The generated numbers are plausible, not predictions. Their only job is
    to exercise every branch of the analysis -- including NaN terminal
    errors on failed trials and the paired-test path -- before real data
    exists. Do not quote them.
    """
    from tops_experiments.trial_schema import CSV_COLUMNS

    rng = random.Random(seed)
    # (peak force mean, sd, success probability, terminal error mean)
    profile = {
        ('impedance', 'clean'): (2.8, 0.5, 0.97, 1.6),
        ('impedance', 'sensor_noise'): (3.4, 0.7, 0.93, 2.2),
        ('impedance', 'calibration'): (4.6, 1.0, 0.90, 2.8),
        ('impedance', 'displacement'): (5.1, 1.2, 0.87, 3.0),
        ('admittance', 'clean'): (3.6, 0.9, 0.93, 1.9),
        ('admittance', 'sensor_noise'): (4.5, 1.3, 0.88, 2.5),
        ('admittance', 'calibration'): (6.2, 1.8, 0.83, 3.3),
        ('admittance', 'displacement'): (6.8, 2.0, 0.80, 3.5),
        ('baseline', 'clean'): (6.1, 1.4, 0.90, 1.4),
        ('baseline', 'sensor_noise'): (8.7, 2.4, 0.78, 2.0),
        ('baseline', 'calibration'): (13.2, 3.6, 0.58, 2.6),
        ('baseline', 'displacement'): (14.1, 4.0, 0.52, 2.9),
    }

    os.makedirs(os.path.dirname(os.path.abspath(path)) or '.', exist_ok=True)
    with open(path, 'w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_COLUMNS)
        writer.writeheader()
        trial_id = 0
        for repeat in range(per_cell):
            for regime_index, regime in enumerate(
                    ('clean', 'sensor_noise', 'calibration', 'displacement')):
                trial_seed = regime_index * 1000 + repeat
                for mode in ('impedance', 'admittance', 'baseline'):
                    mu, sd, p_success, err_mu = profile[(mode, regime)]
                    success = rng.random() < p_success
                    peak = max(0.2, rng.gauss(mu, sd))
                    tripped = peak > 15.0
                    if tripped:
                        success = False
                    writer.writerow({
                        'trial_id': trial_id,
                        'mode': mode,
                        'regime': regime,
                        'seed': trial_seed,
                        'object_label': 'red block',
                        'success': int(success),
                        'peak_force_n': round(peak, 3),
                        'mean_contact_force_n': round(peak * 0.55, 3),
                        'exec_time_s': round(rng.gauss(11.0, 1.8), 3),
                        # NaN when the trial never reached GRASP, which is
                        # the honest representation: there was no terminal
                        # error because there was no terminal moment.
                        'targeting_error_mm': (round(max(0.1, rng.gauss(err_mu, 0.7)), 3)
                                               if success else ''),
                        'final_state': 'HOLD' if success else 'ABORT',
                        'fault': '' if success else
                                 ('wrench_cutoff' if tripped else 'object_dropped'),
                        'force_limit_tripped': int(tripped),
                        'watchdog_tripped': 0,
                        'tank_min_energy_j': (round(rng.uniform(0.05, 0.5), 3)
                                              if mode == 'impedance' else ''),
                        'passivity_throttled': int(mode == 'impedance'
                                                   and rng.random() < 0.2),
                        'max_update_period_ms': round(rng.uniform(1.0, 1.4), 3),
                        'missed_deadlines': rng.randint(0, 40),
                        'contact_duration_s': round(rng.uniform(0.5, 2.5), 3),
                    })
                    trial_id += 1
    return path


# ---------------------------------------------------------------------------
#  entry point
# ---------------------------------------------------------------------------

def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Analyse TOPS trial results.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    parser.add_argument('--csv', default=None, help='Trial CSV to analyse.')
    parser.add_argument('--demo', action='store_true',
                        help='Generate synthetic data with a known effect and '
                             'analyse it. Use this to validate the pipeline '
                             'long before real trials exist.')
    parser.add_argument('--demo-path', default='results/demo_trials.csv')
    parser.add_argument('--baseline', default='baseline',
                        help='Mode to compare the others against.')
    parser.add_argument('--markdown', default=None,
                        help='Also write the report to this file.')
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    if args.demo:
        path = write_demo(args.demo_path)
        print(f'synthetic data written to {path}\n')
    elif args.csv:
        path = args.csv
    else:
        parser.error('give --csv PATH, or --demo to generate synthetic data')
        return 2

    if not os.path.exists(path):
        raise SystemExit(f'no such file: {path}')

    rows = load(path)
    report = Report()

    report(f'TOPS 2026 -- trial analysis of {os.path.abspath(path)}')
    if args.demo:
        report()
        report('*** SYNTHETIC DATA. These numbers exercise the pipeline and')
        report('*** are not results. Do not quote them.')

    summarise_modes(rows, report)
    targets_table(rows, report)

    modes = sorted({row['mode'] for row in rows})
    for mode in modes:
        if mode != args.baseline:
            compare(rows, args.baseline, mode, report)

    per_regime(rows, report)
    diagnostics(rows, report)

    report()
    report.rule()
    if args.markdown:
        report.save(args.markdown)
    return 0


if __name__ == '__main__':
    sys.exit(main())
