#!/usr/bin/env python3
# Copyright 2026 Kabir Dhingra. MIT licence.
"""
The batch trial runner: launches, runs and tears down N trials unattended.

Theory/10_Experiments_and_Statistics.md.

    ros2 run tops_experiments run_trials -- --help
    ros2 run tops_experiments run_trials -- --modes impedance baseline --per-cell 15
    ros2 run tops_experiments run_trials -- --dry-run          # print the plan only

---------------------------------------------------------------------------
Design: one process per trial
---------------------------------------------------------------------------
Every trial gets a completely fresh Gazebo. That is slow -- roughly 20 s of
start-up per trial, so a 120-trial run is about an hour -- and it is worth
every second of it.

The alternative, resetting the world in place between trials, leaks state:
contact residuals, a physics solver that has been running for twenty
minutes, an F/T bias captured under different conditions, objects that
drifted a millimetre during the previous trial's retreat. Each leak is
individually negligible and collectively they mean trial 90 is not drawn
from the same distribution as trial 1. That destroys the statistics quietly
and in a way no plot will show you.

A fresh process per trial costs an hour. Discovering in week 11 that the
results drift with trial index costs the project.

---------------------------------------------------------------------------
Design: paired seeds
---------------------------------------------------------------------------
Trial k uses seed k in EVERY condition. The impedance controller and the
baseline see the identical calibration offset, the identical noise
sequence, the identical everything.

This is the single best design choice available and it costs nothing but
bookkeeping. Paired tests (ttest_rel, wilcoxon) on matched trials are
substantially more powerful than unpaired ones for the same number of runs,
because the trial-to-trial variation cancels. analyze_trials.py detects the
pairing automatically and uses it.
"""
import argparse
import itertools
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import List, Optional

DEFAULT_MODES = ('impedance', 'admittance', 'baseline')
DEFAULT_REGIMES = ('clean', 'sensor_noise', 'calibration', 'displacement')

#: Objects cycled through so the results are not about one object's shape.
# Positions are in the ARM BASE frame (panda_link0). The arm base is at
# world (-0.05, 0, 0.75), so each entry is the world position of the block
# in tabletop.sdf shifted by (+0.05, 0, -0.75).
OBJECTS = [
    ('red block', [0.50, -0.15, 0.026]),
    ('green block', [0.55, 0.10, 0.026]),
    ('blue block', [0.43, 0.22, 0.026]),
    ('yellow block', [0.60, -0.05, 0.026]),
    ('orange block', [0.45, -0.28, 0.0235]),
    ('purple block', [0.65, 0.18, 0.0285]),
]


@dataclass
class Trial:
    trial_id: int
    mode: str
    regime: str
    seed: int
    object_label: str
    object_xyz: List[float]


def build_plan(modes, regimes, per_cell, start_id=0) -> List[Trial]:
    """Full factorial: every mode crossed with every regime, per_cell times.

    The seed depends ONLY on the regime and the repeat index, never on the
    mode. That is what makes the trials paired across controllers.
    """
    trials = []
    trial_id = start_id
    for repeat in range(per_cell):
        for regime_index, regime in enumerate(regimes):
            seed = regime_index * 1000 + repeat
            label, xyz = OBJECTS[repeat % len(OBJECTS)]
            for mode in modes:
                trials.append(Trial(
                    trial_id=trial_id,
                    mode=mode,
                    regime=regime,
                    seed=seed,
                    object_label=label,
                    object_xyz=xyz,
                ))
                trial_id += 1
    return trials


class TrialRunner:

    def __init__(self, args):
        self.args = args
        self.csv_path = os.path.abspath(args.output)
        self.interrupted = False
        signal.signal(signal.SIGINT, self._on_sigint)

    def _on_sigint(self, signum, frame):
        # Finish the trial in flight, then stop. Killing mid-trial would
        # leave a half-written row, and a CSV you have to hand-repair is a
        # CSV you cannot trust.
        print('\n[runner] interrupt received; stopping after this trial', flush=True)
        self.interrupted = True

    # ------------------------------------------------------------------
    def run(self, trials: List[Trial]) -> int:
        os.makedirs(os.path.dirname(self.csv_path) or '.', exist_ok=True)
        completed = 0
        failed_to_run = 0
        started = time.time()

        for index, trial in enumerate(trials, start=1):
            if self.interrupted:
                break

            elapsed = time.time() - started
            remaining = (elapsed / max(1, index - 1)) * (len(trials) - index + 1) \
                if index > 1 else 0.0
            print(
                f'[runner] {index}/{len(trials)}  id={trial.trial_id} '
                f'{trial.mode}/{trial.regime} seed={trial.seed} '
                f'obj="{trial.object_label}"'
                + (f'  eta {remaining / 60:.0f} min' if remaining else ''),
                flush=True)

            ok = self._run_one(trial)
            if ok:
                completed += 1
            else:
                failed_to_run += 1
                print(f'[runner] trial {trial.trial_id} did not produce a row',
                      file=sys.stderr, flush=True)

        print(f'\n[runner] {completed} trials recorded, '
              f'{failed_to_run} failed to run, '
              f'{time.time() - started:.0f} s total')
        print(f'[runner] results: {self.csv_path}')
        print(f'[runner] analyse with: '
              f'ros2 run tops_experiments analyze_trials --csv {self.csv_path}')
        return 0 if failed_to_run == 0 else 1

    def _run_one(self, trial: Trial) -> bool:
        """Launch the stack, run one trial, tear everything down."""
        env = dict(os.environ)
        # A per-trial ROS domain keeps a straggling node from a previous
        # trial from ever being discovered by this one. Without it, a
        # Gazebo that is slow to die can still be publishing /clock when
        # the next trial starts, and the symptom -- trials that
        # occasionally use the wrong simulation time -- is close to
        # impossible to diagnose from the CSV.
        env['ROS_DOMAIN_ID'] = str(60 + (trial.trial_id % 40))

        launch_cmd = [
            'ros2', 'launch', 'tops_experiments', 'trial.launch.py',
            f'controller:={trial.mode}',
            f'regime:={trial.regime}',
            f'seed:={trial.seed}',
            f'trial_id:={trial.trial_id}',
            f'object_label:={trial.object_label}',
            f'object_xyz:=[{",".join(str(v) for v in trial.object_xyz)}]',
            f'output_csv:={self.csv_path}',
            f'timeout:={self.args.trial_timeout}',
            'gui:=false',
        ]

        # Reap before as well as after: a trial must never inherit a
        # simulator, whatever left it behind.
        self._reap_simulator()

        if self.args.verbose:
            print('[runner] ' + ' '.join(launch_cmd), flush=True)

        try:
            process = subprocess.Popen(
                launch_cmd, env=env,
                stdout=None if self.args.verbose else subprocess.DEVNULL,
                stderr=None if self.args.verbose else subprocess.DEVNULL,
                start_new_session=True)
        except FileNotFoundError:
            print('[runner] ros2 not found; source the workspace first',
                  file=sys.stderr)
            return False

        # Hard wall-clock cap. The launch file has its own internal timeout
        # and should shut itself down; this is the backstop for the case
        # where Gazebo itself hangs, which it occasionally does on a cold
        # GPU. Without a backstop, one hung trial stalls the whole batch
        # overnight.
        deadline = self.args.trial_timeout + self.args.startup_allowance
        try:
            process.wait(timeout=deadline)
        except subprocess.TimeoutExpired:
            print(f'[runner] trial {trial.trial_id} exceeded {deadline:.0f} s; '
                  'killing the process group', file=sys.stderr, flush=True)
            self._kill(process)
            return False

        # Reap Gazebo before the next trial starts. This is not belt and
        # braces, it is required.
        #
        # Gazebo routinely does not exit within the 5 s launch allows after
        # SIGINT, so launch escalates to SIGTERM and moves on while the
        # server is still alive. The next trial then starts a second server,
        # which reports
        #
        #     Error: Another world of the same name is running
        #
        # and -- much worse -- the surviving controller_manager from the
        # previous trial answers the new trial's spawners, which report
        # "Controller already loaded, skipping load_controller" and then fail
        # to activate. Every trial after the first produces a row full of
        # zeros that looks like a controller defect.
        #
        # A per-trial ROS_DOMAIN_ID does not help: gz-transport is not
        # namespaced by it, so the orphaned server is still discoverable.
        # The only reliable fix is to make sure it is gone.
        self._reap_simulator()
        time.sleep(self.args.settle_time)
        return True

    @staticmethod
    def _reap_simulator():
        """Terminate any surviving Gazebo server or bridge from this trial.

        Matched narrowly on purpose: only this user's gz server, gui and
        ros_gz bridge processes. Nothing broader, because a careless pattern
        here would kill the operator's own editor or shell.
        """
        victims = list(TrialRunner._simulator_pids())

        for pid in victims:
            try:
                os.kill(pid, signal.SIGTERM)
            except OSError:
                pass
        if not victims:
            return
        time.sleep(3)
        for pid in victims:
            try:
                os.kill(pid, signal.SIGKILL)
            except OSError:
                pass

        # WAIT until they are actually gone, rather than assuming.
        #
        # A fixed sleep is not enough: Gazebo can outlive SIGKILL delivery
        # long enough for the next trial's nodes to discover it, and a
        # surviving joint_state_broadcaster from the previous trial feeds the
        # new supervisor stale joint positions. The new trial then sits in
        # HOMING waiting for an arm that -- according to the only state it
        # can see -- is not at home, and times out. Every trial after the
        # first failed this way while each one passed in isolation, which is
        # the signature of leaked state rather than a bad condition.
        for _ in range(40):                       # up to ~20 s
            if not TrialRunner._simulator_pids():
                return
            time.sleep(0.5)

    @staticmethod
    def _simulator_pids():
        # 'gz-sim' matches the process that actually matters.
        #
        # `ros2 launch` starts a ruby wrapper whose command line reads
        # "gz sim -r -s <world>", but that wrapper immediately hands off to
        #     .../libexec/gz/sim10/gz-sim-main -r -s <world>
        # which is the real server. Matching only "gz sim" killed the
        # wrapper and left the server running, so every subsequent trial hit
        #     Error: Another world of the same name is running
        # and no controller_manager ever appeared. The arm then sat in
        # HOMING with no joint states at all until it timed out — and
        # because the FIRST trial of a batch has nothing to inherit, it
        # always passed, which made this look like a flaky experiment
        # rather than a leak.
        patterns = ('gz-sim', 'gz sim', 'parameter_bridge')
        try:
            listing = subprocess.run(
                ['ps', '-u', str(os.getuid()), '-o', 'pid=,args='],
                capture_output=True, text=True, timeout=10).stdout
        except (subprocess.SubprocessError, OSError):
            return []
        out = []
        for line in listing.splitlines():
            line = line.strip()
            pid_text, _, args = line.partition(' ')
            if pid_text.isdigit() and int(pid_text) != os.getpid() \
                    and any(q in args for q in patterns):
                out.append(int(pid_text))
        return out

    @staticmethod
    def _kill(process):
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
            process.wait(timeout=10)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Run the TOPS trial battery.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    parser.add_argument('--modes', nargs='+', default=list(DEFAULT_MODES),
                        choices=list(DEFAULT_MODES))
    parser.add_argument('--regimes', nargs='+', default=list(DEFAULT_REGIMES),
                        choices=list(DEFAULT_REGIMES))
    parser.add_argument(
        '--per-cell', type=int, default=10,
        help='Repeats per (mode, regime) cell. The plan calls for at least '
             '120 trials total; 10 per cell across 3 modes and 4 regimes is '
             'exactly 120. (default: 10)')
    parser.add_argument('--output', default='results/trials.csv')
    parser.add_argument('--start-id', type=int, default=0)
    parser.add_argument(
        '--trial-timeout', type=float, default=45.0,
        help='Seconds a single trial may take once the stack is up. The '
             'plan budgets 15 s per grasp; the rest is margin for a slow '
             'real-time factor. (default: 45)')
    parser.add_argument(
        '--startup-allowance', type=float, default=90.0,
        help='Extra seconds allowed for Gazebo to start before the hard '
             'kill. (default: 90)')
    parser.add_argument('--settle-time', type=float, default=5.0,
                    help='Seconds to wait after the simulator is confirmed\n'
                         'gone, before the next trial starts (default: 5).')
    parser.add_argument('--verbose', action='store_true')
    parser.add_argument('--dry-run', action='store_true',
                        help='Print the trial plan and exit.')

    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    trials = build_plan(args.modes, args.regimes, args.per_cell, args.start_id)

    print(f'[runner] {len(trials)} trials: {len(args.modes)} modes x '
          f'{len(args.regimes)} regimes x {args.per_cell} repeats')
    est = len(trials) * (args.trial_timeout * 0.5 + 25) / 60.0
    print(f'[runner] rough estimate: {est:.0f} minutes\n')

    if args.dry_run:
        print(f'{"id":>5}  {"mode":<11} {"regime":<13} {"seed":>5}  object')
        for t in trials:
            print(f'{t.trial_id:>5}  {t.mode:<11} {t.regime:<13} '
                  f'{t.seed:>5}  {t.object_label}')
        # Prove the pairing holds, since that is what the paired statistics
        # depend on and it is cheap to check.
        by_key = {}
        for t in trials:
            by_key.setdefault((t.regime, t.seed), set()).add(t.mode)
        unpaired = [k for k, v in by_key.items() if len(v) != len(args.modes)]
        if unpaired:
            print(f'\n[runner] WARNING: {len(unpaired)} (regime, seed) cells '
                  'are not present in every mode; paired analysis will fall '
                  'back to unpaired for those.')
        else:
            print(f'\n[runner] pairing verified: every (regime, seed) cell '
                  f'appears in all {len(args.modes)} modes')
        return 0

    return TrialRunner(args).run(trials)


if __name__ == '__main__':
    sys.exit(main())
