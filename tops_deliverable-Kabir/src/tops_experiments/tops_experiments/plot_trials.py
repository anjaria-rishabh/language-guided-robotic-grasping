# Copyright 2026 Kabir Dhingra. MIT licence.
"""
Turn a trial CSV into the report's figures.

    make plots                       # full battery AND quick test, whichever exist
    make plots CSV=results/x.csv     # just one particular CSV
    python3 -m tops_experiments.plot_trials --csv results/trials.csv results/quick.csv

`make trials` and `make trials-quick` call this automatically when they finish.

Writes one PNG (for reading) and one PDF (vector, for the LaTeX report) per
figure into results/figures/<csv name>/ -- so the full battery lands in
results/figures/trials/ and the smoke test in results/figures/quick/, and
the two never overwrite each other. Every
figure answers one question; the question is its title.

    1_success_by_regime     Does the controller complete the grasp? Success
                            rate per disturbance regime, with 95 % Wilson
                            intervals and the 85 % target.
    2_peak_force            How hard does it hit? Every trial's peak contact
                            force, with the 10 N target and 15 N interlock.
    3_terminal_error        How close does it get? Terminal targeting error
                            for trials that reached GRASP, with the 3 mm target.
    4_exec_time             How long does it take? Trial execution time,
                            with the 15 s target.
    5_outcomes              How does each controller end? Counts of final
                            outcome per controller (success, force interlock,
                            timeout, other).
    6_loop_timing           Is the 1 kHz claim true? Worst control-loop
                            period per trial, against the 1 ms the loop is
                            configured for.

Like analyze_trials, this needs no ROS: only the CSV, numpy and matplotlib
(both from apt: python3-numpy, python3-matplotlib).

Design rules the figures follow (so a reader can trust what they see):

* each controller keeps ONE colour in every figure, assigned by name, never
  by position: impedance blue, admittance orange, baseline aqua. A CSV with
  only two controllers does not repaint the survivors.
* one y-axis per figure, a legend whenever there are two or more series,
  text in neutral ink (never in a series colour), recessive grid.
* raw points rather than averages wherever n is small: with ten trials per
  cell a box plot invents a distribution that is not there.
* targets and limits drawn as labelled reference lines, so a miss is visible.
"""
import argparse
import math
import os
import sys
from collections import Counter
from typing import Dict, List, Optional, Sequence

import numpy as np

import matplotlib
matplotlib.use('Agg')  # files only, never a window: this runs in batch and over SSH
import matplotlib.pyplot as plt  # noqa: E402  (must follow the backend choice)
import matplotlib.ticker  # noqa: E402

from tops_experiments import statistics_backend as st  # noqa: E402
from tops_experiments.analyze_trials import TARGETS, load  # noqa: E402

# ---------------------------------------------------------------------------
#  palette and ink
# ---------------------------------------------------------------------------
# The first three slots of a categorical palette validated for colour-vision
# deficiency: every PAIR of the three stays distinguishable (CVD delta-E >= 9
# in OKLab x 100), which matters because strip plots put all three side by
# side. Fixed per controller so colour always means the same thing.
CONTROLLER_COLOURS: Dict[str, str] = {
    'impedance': '#2a78d6',   # blue   -- the deliverable
    'admittance': '#eb6834',  # orange -- the rival philosophy
    'baseline': '#1baf7a',    # aqua   -- rigid, no compliance
}
FALLBACK_COLOUR = '#8a8984'   # a controller name this script has never seen
CONTROLLER_ORDER = ('impedance', 'admittance', 'baseline')
REGIME_ORDER = ('clean', 'sensor_noise', 'calibration', 'displacement')
REGIME_LABELS = {
    'clean': 'clean',
    'sensor_noise': 'sensor noise',
    'calibration': 'calibration\noffset',
    'displacement': 'object\ndisplaced',
}

SURFACE = '#fcfcfb'
INK = '#0b0b0b'          # titles, values
INK_SECONDARY = '#52514e'  # axis labels, ticks, legend
INK_MUTED = '#8a8984'    # reference-line labels
GRID = '#e4e3df'         # one step off the surface: present, not loud
REFERENCE = '#52514e'    # target / limit lines

# Sequential blue, light -> dark, for the outcome counts heatmap.
SEQUENTIAL_BLUE = ['#f4f8fd', '#cde2fb', '#9ec5f4', '#6da7ec', '#3987e5',
                   '#2a78d6', '#1c5cab', '#104281']

INTERLOCK_N = 15.0       # the hard force cutoff (tops_controllers config)
NOMINAL_PERIOD_MS = 1.0  # the controller_manager update_rate is 1 kHz


def _style() -> None:
    """One place for every typographic and axis decision."""
    plt.rcParams.update({
        'figure.facecolor': SURFACE,
        'axes.facecolor': SURFACE,
        'savefig.facecolor': SURFACE,
        'font.family': 'DejaVu Sans',
        'font.size': 10,
        'text.color': INK,
        'axes.labelcolor': INK_SECONDARY,
        'axes.titlecolor': INK,
        'axes.titlesize': 12,
        'axes.titleweight': 'bold',
        'axes.titlelocation': 'left',
        'axes.edgecolor': GRID,
        'axes.linewidth': 1.0,
        'axes.spines.top': False,
        'axes.spines.right': False,
        'axes.grid': True,
        'axes.grid.axis': 'y',
        'axes.axisbelow': True,
        'grid.color': GRID,
        'grid.linewidth': 1.0,
        'grid.linestyle': '-',
        'xtick.color': INK_SECONDARY,
        'ytick.color': INK_SECONDARY,
        'xtick.major.size': 0,
        'ytick.major.size': 0,
        'legend.frameon': False,
        'legend.fontsize': 9,
        'legend.labelcolor': INK_SECONDARY,
    })


def _colour(mode: str) -> str:
    return CONTROLLER_COLOURS.get(mode, FALLBACK_COLOUR)


def _ordered(present: Sequence[str], preferred: Sequence[str]) -> List[str]:
    """Known names in their fixed order, then anything unexpected, sorted."""
    known = [p for p in preferred if p in present]
    return known + sorted(set(present) - set(preferred))


def _finite(values) -> List[float]:
    return [v for v in values if isinstance(v, float) and math.isfinite(v)]


# ---------------------------------------------------------------------------
#  figure helpers
# ---------------------------------------------------------------------------

def _subtitle(ax, text: str) -> None:
    """A one-line explanation under the title, in secondary ink."""
    ax.text(0.0, 1.015, text, transform=ax.transAxes, fontsize=9,
            color=INK_SECONDARY, va='bottom', ha='left')


def _reference_line(ax, y: float, label: str) -> None:
    """A target or limit: thin solid line, labelled at the right end."""
    ax.axhline(y, color=REFERENCE, linewidth=1.0, zorder=1.5)
    ax.annotate(label, xy=(1.0, y), xycoords=('axes fraction', 'data'),
                xytext=(-2, 3), textcoords='offset points',
                ha='right', va='bottom', fontsize=8, color=INK_MUTED)


def _legend(ax, modes: Sequence[str], marker: str = 'o') -> None:
    handles = [plt.Line2D([], [], linestyle='none', marker=marker, markersize=8,
                          markerfacecolor=_colour(m), markeredgecolor=SURFACE,
                          markeredgewidth=1.5, label=m) for m in modes]
    ax.legend(handles=handles, loc='upper left', bbox_to_anchor=(0.0, -0.13),
              ncol=len(modes), handletextpad=0.3, columnspacing=1.4)


def _save(fig, out_dir: str, name: str, written: List[str]) -> None:
    for ext in ('png', 'pdf'):
        path = os.path.join(out_dir, f'{name}.{ext}')
        fig.savefig(path, dpi=200 if ext == 'png' else None, bbox_inches='tight')
        written.append(path)
    plt.close(fig)


def _strip(ax, rows: List[dict], key: str, modes: Sequence[str],
           regimes: Sequence[str], rng: np.random.Generator) -> int:
    """Every trial as a dot, grouped by regime, offset by controller.

    A short horizontal tick marks each group's median. Returns how many
    points were drawn, so the caller can skip an empty figure.
    """
    width = 0.8 / max(1, len(modes))
    drawn = 0
    for gi, regime in enumerate(regimes):
        for mi, mode in enumerate(modes):
            vals = _finite([r[key] for r in rows
                            if r['regime'] == regime and r['mode'] == mode])
            if not vals:
                continue
            centre = gi - 0.4 + width * (mi + 0.5)
            jitter = rng.uniform(-width * 0.28, width * 0.28, size=len(vals))
            ax.scatter(centre + jitter, vals, s=40, color=_colour(mode),
                       edgecolors=SURFACE, linewidths=1.2, zorder=3, alpha=0.9)
            med = float(np.median(vals))
            ax.plot([centre - width * 0.38, centre + width * 0.38], [med, med],
                    color=INK, linewidth=2.0, solid_capstyle='round', zorder=4)
            drawn += len(vals)
    ax.set_xticks(range(len(regimes)))
    ax.set_xticklabels([REGIME_LABELS.get(r, r) for r in regimes])
    ax.set_xlim(-0.6, len(regimes) - 0.4)
    return drawn


# ---------------------------------------------------------------------------
#  the six figures
# ---------------------------------------------------------------------------

def fig_success(rows, modes, regimes, out_dir, written):
    groups = list(regimes) + ['all']
    fig, ax = plt.subplots(figsize=(8.0, 4.4))
    width = min(0.8 / max(1, len(modes)), 0.22)
    for mi, mode in enumerate(modes):
        xs, rates, lo_err, hi_err, labels = [], [], [], [], []
        for gi, group in enumerate(groups):
            sel = [r for r in rows if r['mode'] == mode and (group == 'all' or r['regime'] == group)]
            if not sel:
                continue
            k, n = sum(r['success'] for r in sel), len(sel)
            lo, hi = st.wilson_interval(k, n)
            rate = k / n
            xs.append(gi + (mi - (len(modes) - 1) / 2) * (width + 0.02))
            rates.append(rate * 100)
            lo_err.append((rate - lo) * 100)
            hi_err.append((hi - rate) * 100)
            labels.append(f'{k}/{n}')
        ax.bar(xs, rates, width=width, color=_colour(mode), label=mode, zorder=3)
        ax.errorbar(xs, rates, yerr=[lo_err, hi_err], fmt='none',
                    ecolor=INK_SECONDARY, elinewidth=1.0, capsize=0, zorder=4)
        for x, rate, hi, lab in zip(xs, rates, hi_err, labels):
            # Counts sit just above the interval's upper end, rotated so
            # three per group fit, and never on top of a whisker.
            ax.annotate(lab, (x, rate + hi), xytext=(0, 3), textcoords='offset points',
                        ha='center', va='bottom', fontsize=7, rotation=90,
                        color=INK_SECONDARY)
    _reference_line(ax, TARGETS['success_rate_overall'] * 100, 'target 85 %')
    ax.set_xticks(range(len(groups)))
    ax.set_xticklabels([REGIME_LABELS.get(g, g) if g != 'all' else 'all\nregimes' for g in groups])
    ax.axvline(len(regimes) - 0.5, color=GRID, linewidth=1.0)
    ax.set_ylim(0, 118)               # headroom for the counts above 100 %
    ax.set_yticks(range(0, 101, 20))
    ax.set_ylabel('grasp success (%)')
    ax.set_title('Does the controller complete the grasp?', pad=18)
    _subtitle(ax, 'Success rate per disturbance regime; whiskers are 95 % Wilson '
                  'intervals; counts are successes / trials.')
    _legend(ax, modes, marker='s')
    _save(fig, out_dir, '1_success_by_regime', written)


def fig_metric(rows, modes, regimes, out_dir, written, *, key, name, title,
               subtitle, ylabel, references, rng, only_reached=False):
    fig, ax = plt.subplots(figsize=(8.0, 4.4))
    drawn = _strip(ax, rows, key, modes, regimes, rng)
    if drawn == 0:
        plt.close(fig)
        print(f'  skipped {name}: no finite {key} values')
        return
    for y, label in references:
        _reference_line(ax, y, label)
    ax.set_ylim(bottom=0)
    ax.set_ylabel(ylabel)
    ax.set_title(title, pad=18)
    _subtitle(ax, subtitle)
    _legend(ax, [m for m in modes if any(
        r['mode'] == m and math.isfinite(r[key]) for r in rows)])
    if only_reached:
        missing = sum(1 for r in rows if not math.isfinite(r[key]))
        if missing:
            ax.text(1.0, -0.13, f'{missing} trial(s) never reached GRASP: no terminal error',
                    transform=ax.transAxes, ha='right', va='top', fontsize=8,
                    color=INK_MUTED)
    _save(fig, out_dir, name, written)


def _outcome(row: dict) -> str:
    """Collapse final state + fault into four outcomes a reader can compare."""
    if row['success']:
        return 'success'
    fault = (row.get('fault') or '').lower()
    if 'wrench' in fault or row['force_limit_tripped']:
        return 'force interlock'
    if 'timeout' in fault or row.get('final_state') not in ('ABORT', 'RETREAT', 'HOLD'):
        return 'timeout'
    return 'other abort'


def fig_outcomes(rows, modes, out_dir, written):
    outcomes = ['success', 'force interlock', 'timeout', 'other abort']
    counts = np.array([[Counter(_outcome(r) for r in rows if r['mode'] == m)[o]
                        for o in outcomes] for m in modes], dtype=float)
    fig, ax = plt.subplots(figsize=(7.2, 1.2 + 0.62 * len(modes)))
    cmap = matplotlib.colors.LinearSegmentedColormap.from_list('seq', SEQUENTIAL_BLUE)
    vmax = max(1.0, counts.max())
    ax.imshow(counts, cmap=cmap, vmin=0, vmax=vmax, aspect='auto')
    # A 2 px surface gap between cells, drawn as a white grid on the edges.
    ax.set_xticks(np.arange(-0.5, len(outcomes)), minor=True)
    ax.set_yticks(np.arange(-0.5, len(modes)), minor=True)
    ax.grid(which='minor', color=SURFACE, linewidth=3)
    ax.grid(which='major', visible=False)
    ax.tick_params(which='minor', length=0)
    for i in range(len(modes)):
        for j in range(len(outcomes)):
            v = counts[i, j]
            dark = v / vmax > 0.55
            ax.text(j, i, f'{int(v)}', ha='center', va='center', fontsize=11,
                    color=SURFACE if dark else INK, fontweight='bold' if v else 'normal')
    ax.set_xticks(range(len(outcomes)))
    ax.set_xticklabels(outcomes)
    ax.set_yticks(range(len(modes)))
    ax.set_yticklabels(modes)
    for side in ('left', 'bottom', 'top', 'right'):
        ax.spines[side].set_visible(False)
    ax.set_title('How does each controller end?', pad=18)
    _subtitle(ax, 'Trials per final outcome. Timeout = a phase ran out of time '
                  '(servo, homing, ...).')
    _save(fig, out_dir, '5_outcomes', written)


def fig_timing(rows, modes, out_dir, written, rng):
    # The rigid baseline publishes no controller diagnostics, so its rows
    # read 0 ms; leaving them in would claim a perfect loop it never ran.
    have = [m for m in modes if any(r['mode'] == m and r['max_update_period_ms'] > 0 for r in rows)]
    if not have:
        print('  skipped 6_loop_timing: no controller diagnostics in this CSV')
        return
    fig, ax = plt.subplots(figsize=(6.4, 4.2))
    for i, mode in enumerate(have):
        vals = [r['max_update_period_ms'] for r in rows
                if r['mode'] == mode and r['max_update_period_ms'] > 0]
        jitter = rng.uniform(-0.18, 0.18, size=len(vals))
        ax.scatter(i + jitter, vals, s=40, color=_colour(mode), edgecolors=SURFACE,
                   linewidths=1.2, zorder=3, alpha=0.9)
        med = float(np.median(vals))
        ax.plot([i - 0.28, i + 0.28], [med, med], color=INK, linewidth=2.0,
                solid_capstyle='round', zorder=4)
    ax.set_yscale('log')
    # Plain numbers (1, 10, 100) rather than 10^0, 10^1: easier to read.
    ax.yaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(lambda v, _: f'{v:g}'))
    _reference_line(ax, NOMINAL_PERIOD_MS, 'configured period 1 ms')
    ax.set_xticks(range(len(have)))
    ax.set_xticklabels(have)
    ax.set_xlim(-0.6, len(have) - 0.4)
    ax.set_ylabel('worst loop period in the trial (ms, log scale)')
    ax.set_title('Is the 1 kHz loop actually 1 kHz?', pad=18)
    _subtitle(ax, 'One dot per trial; anything above the line is a late cycle. '
                  'The baseline reports no diagnostics.')
    _save(fig, out_dir, '6_loop_timing', written)


# ---------------------------------------------------------------------------
#  entry point
# ---------------------------------------------------------------------------

def plot(csv_path: str, out_root: str, seed: int = 0) -> List[str]:
    rows = load(csv_path)
    stem = os.path.splitext(os.path.basename(csv_path))[0]
    out_dir = os.path.join(out_root, stem)
    os.makedirs(out_dir, exist_ok=True)

    modes = _ordered({r['mode'] for r in rows}, CONTROLLER_ORDER)
    regimes = _ordered({r['regime'] for r in rows}, REGIME_ORDER)
    # Seeded jitter: re-plotting the same CSV gives the same picture.
    rng = np.random.default_rng(seed)
    _style()
    written: List[str] = []

    print(f'{len(rows)} trials, controllers: {", ".join(modes)}; '
          f'regimes: {", ".join(regimes)}')
    fig_success(rows, modes, regimes, out_dir, written)
    fig_metric(rows, modes, regimes, out_dir, written, rng=rng,
               key='peak_force_n', name='2_peak_force',
               title='How hard does it hit?',
               subtitle='Peak contact force over the whole trial; one dot per '
                        'trial, bar = median.',
               ylabel='peak contact force (N)',
               references=[(TARGETS['peak_force_n'], 'target 10 N'),
                           (INTERLOCK_N, 'interlock 15 N')])
    fig_metric(rows, modes, regimes, out_dir, written, rng=rng,
               key='targeting_error_mm', name='3_terminal_error',
               title='How close does it get?',
               subtitle='TCP error at the moment GRASP begins; trials that never '
                        'got there have no value.',
               ylabel='terminal targeting error (mm)',
               references=[(TARGETS['targeting_error_mm'], 'target 3 mm')],
               only_reached=True)
    fig_metric(rows, modes, regimes, out_dir, written, rng=rng,
               key='exec_time_s', name='4_exec_time',
               title='How long does it take?',
               subtitle='Trial start to HOLD, or to the abort; one dot per trial, '
                        'bar = median.',
               ylabel='execution time (s)',
               references=[(TARGETS['exec_time_s'], 'target 15 s')])
    fig_outcomes(rows, modes, out_dir, written)
    fig_timing(rows, modes, out_dir, written, rng)
    return written


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description='Plot TOPS trial results.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    parser.add_argument('--csv', required=True, nargs='+',
                        help='One or more trial CSVs (e.g. the full battery and '
                             'the quick smoke test). Each gets its own folder; '
                             'a missing or empty file is skipped, not fatal.')
    parser.add_argument('--out', default='results/figures',
                        help='Folder to write into; a sub-folder named after '
                             'each CSV is created inside it.')
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    plotted = 0
    for csv_path in args.csv:
        print(f'--- {csv_path}')
        if not os.path.exists(csv_path) or os.path.getsize(csv_path) == 0:
            print('  skipped: file does not exist yet (run the trials first)')
            continue
        try:
            written = plot(csv_path, args.out)
        except SystemExit as exc:          # load() refuses a header-only CSV
            print(f'  skipped: {exc}')
            continue
        plotted += 1
        print(f'  wrote {len(written)} files into '
              f'{os.path.dirname(written[0]) if written else args.out}')
    if plotted == 0:
        print('nothing plotted: none of the CSVs has any trials in it')
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
