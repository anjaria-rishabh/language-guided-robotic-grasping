import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import analyze_trials as a


def test_zero():
    assert a.evaluate([])['trials']==0


def test_wilson():
    lo,hi=a.wilson_interval(85,100)
    assert lo<.85<hi


def test_real_values():
    rows=a.parse([{'trial_id':'1','condition':'ideal','success':'true',
                   'localization_error_mm':'1.0','final_error_mm':'2.0',
                   'peak_force_n':'3.0','execution_time_s':'8.0'}])
    report=a.evaluate(rows)
    assert report['success_rate']==1
    assert report['threshold_counts']['peak_force_le_10n']==1
