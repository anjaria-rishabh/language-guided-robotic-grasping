# Copyright 2026 Kabir Dhingra. MIT licence.
"""End-to-end test of the results pipeline on synthetic data.

The point is that this runs in week 2, not week 10. Debugging a statistics
pipeline on the same evening you generate the results you are about to
submit is how deadlines are missed.
"""
import csv
import math

from tops_experiments import analyze_trials as az
from tops_experiments.trial_schema import CSV_COLUMNS, TrialRecord, validate_header


def test_schema_round_trips(tmp_path):
    path = tmp_path / 'one.csv'
    record = TrialRecord(trial_id=7, mode='impedance', regime='calibration',
                         seed=2003, success=1, peak_force_n=4.2)
    with open(path, 'w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_COLUMNS)
        writer.writeheader()
        writer.writerow(record.as_row())

    rows = az.load(str(path))
    assert len(rows) == 1
    assert rows[0]['trial_id'] == 7
    assert rows[0]['peak_force_n'] == 4.2
    assert rows[0]['mode'] == 'impedance'


def test_nan_is_written_blank_and_read_back_as_nan(tmp_path):
    """A trial that never reached GRASP has no terminal error.

    Writing it blank rather than the string 'nan' keeps every reader
    agreeing it is missing, instead of one of them parsing it as a category.
    """
    path = tmp_path / 'nan.csv'
    record = TrialRecord(trial_id=1, mode='baseline', success=0)
    with open(path, 'w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_COLUMNS)
        writer.writeheader()
        writer.writerow(record.as_row())

    text = path.read_text()
    assert 'nan' not in text.lower()

    rows = az.load(str(path))
    assert math.isnan(rows[0]['targeting_error_mm'])


def test_validate_header_rejects_a_drifted_schema():
    validate_header(CSV_COLUMNS)              # the current schema is fine
    try:
        validate_header(['trial_id', 'mode'])
    except ValueError as exc:
        assert 'missing' in str(exc)
    else:
        raise AssertionError('a truncated header must be rejected')


def test_demo_generates_a_readable_full_factorial(tmp_path):
    path = az.write_demo(str(tmp_path / 'demo.csv'), per_cell=5, seed=1)
    rows = az.load(path)

    assert len(rows) == 5 * 4 * 3, '5 repeats x 4 regimes x 3 modes'
    assert {r['mode'] for r in rows} == {'impedance', 'admittance', 'baseline'}
    assert {r['regime'] for r in rows} == {
        'clean', 'sensor_noise', 'calibration', 'displacement'}


def test_demo_data_is_paired_across_modes(tmp_path):
    """The property the paired statistics depend on."""
    path = az.write_demo(str(tmp_path / 'demo.csv'), per_cell=5, seed=1)
    rows = az.load(path)

    by_cell = {}
    for row in rows:
        by_cell.setdefault((row['regime'], row['seed']), set()).add(row['mode'])
    assert all(len(modes) == 3 for modes in by_cell.values())


def test_full_report_runs_without_error(tmp_path, capsys):
    csv_path = az.write_demo(str(tmp_path / 'demo.csv'), per_cell=6, seed=2)
    out = tmp_path / 'report.md'
    assert az.main(['--csv', csv_path, '--markdown', str(out)]) == 0

    captured = capsys.readouterr().out
    for section in ('OVERVIEW', 'TARGETS', 'BY DISTURBANCE REGIME',
                    'CONTROLLER HEALTH'):
        assert section in captured
    assert out.exists()
    assert 'Peak contact force' in out.read_text()


def test_matched_pairs_only_matches_identical_conditions():
    a = [{'regime': 'clean', 'seed': 1}, {'regime': 'clean', 'seed': 2}]
    b = [{'regime': 'clean', 'seed': 1}, {'regime': 'calibration', 'seed': 2}]
    pairs = az._matched_pairs(a, b)
    assert len(pairs) == 1
    assert pairs[0][0]['seed'] == 1


def test_clean_drops_non_finite_values():
    assert az._clean([1.0, float('nan'), 2.0, float('inf')]) == [1.0, 2.0]
