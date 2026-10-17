# Copyright 2026 Kabir Dhingra. MIT licence.
"""The plotting script, run on synthetic data and on awkward real-world CSVs.

These check that files appear and that edge cases do not crash -- whether a
figure LOOKS right is a judgement for a human with the PNG open.
"""
import csv
import os

from tops_experiments import analyze_trials as az
from tops_experiments import plot_trials as pt
from tops_experiments.trial_schema import CSV_COLUMNS, TrialRecord


def _figure_names(paths):
    return sorted({os.path.splitext(os.path.basename(p))[0] for p in paths})


def test_full_battery_produces_all_six_figures_as_png_and_pdf(tmp_path):
    csv_path = az.write_demo(str(tmp_path / 'demo.csv'), per_cell=4, seed=3)
    written = pt.plot(csv_path, str(tmp_path / 'figures'))
    assert _figure_names(written) == [
        '1_success_by_regime', '2_peak_force', '3_terminal_error',
        '4_exec_time', '5_outcomes', '6_loop_timing']
    assert len(written) == 12                        # one PNG + one PDF each
    for path in written:
        assert os.path.getsize(path) > 1000, path
    # Figures land in a folder named after the CSV, so two CSVs never collide.
    assert all(os.path.basename(os.path.dirname(p)) == 'demo' for p in written)


def test_a_one_row_baseline_csv_skips_what_it_cannot_draw(tmp_path):
    # The shape of an aborted smoke test: one baseline trial, no terminal
    # error (never reached GRASP) and no controller diagnostics.
    path = tmp_path / 'quick.csv'
    record = TrialRecord(trial_id=0, mode='baseline', regime='clean', seed=0,
                         success=0, final_state='ABORT', fault='servo_timeout')
    with open(path, 'w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_COLUMNS)
        writer.writeheader()
        writer.writerow(record.as_row())
    written = pt.plot(str(path), str(tmp_path / 'figures'))
    names = _figure_names(written)
    assert '3_terminal_error' not in names
    assert '6_loop_timing' not in names
    assert '1_success_by_regime' in names


def test_main_plots_several_csvs_and_skips_missing_ones(tmp_path, capsys):
    full = az.write_demo(str(tmp_path / 'trials.csv'), per_cell=2, seed=4)
    missing = str(tmp_path / 'quick.csv')            # not run yet
    rc = pt.main(['--csv', full, missing, '--out', str(tmp_path / 'figures')])
    assert rc == 0
    assert os.path.isdir(tmp_path / 'figures' / 'trials')
    assert not os.path.exists(tmp_path / 'figures' / 'quick')
    assert 'does not exist yet' in capsys.readouterr().out


def test_main_fails_when_there_is_nothing_to_plot(tmp_path):
    rc = pt.main(['--csv', str(tmp_path / 'nope.csv'), '--out', str(tmp_path)])
    assert rc == 1


def test_colour_belongs_to_the_controller_not_its_position():
    # A CSV holding only admittance must still draw admittance orange.
    assert pt._colour('admittance') == pt.CONTROLLER_COLOURS['admittance']
    assert pt._ordered({'baseline', 'admittance'}, pt.CONTROLLER_ORDER) == \
        ['admittance', 'baseline']
