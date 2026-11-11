#!/usr/bin/env python3
"""Analyze ACTUAL, externally recorded TOPS trial metrics; never invent runs.
Usage: python3 benchmark/analyze_trials.py benchmark/trials.csv
"""
import csv
import json
import math
import statistics
import sys
from collections import defaultdict

COLUMNS = ['trial_id', 'condition', 'success', 'localization_error_mm',
           'final_error_mm', 'peak_force_n', 'execution_time_s']


def wilson_interval(successes, n, z=1.959963984540054):
    if n <= 0: return (None,None)
    p=successes/n
    den=1+z*z/n
    center=(p+z*z/(2*n))/den
    margin=z*math.sqrt((p*(1-p)+z*z/(4*n))/n)/den
    return (max(0.0,center-margin),min(1.0,center+margin))


def yes(value):
    t=str(value).strip().lower()
    if t in ('true','1','yes','y'): return True
    if t in ('false','0','no','n'): return False
    raise ValueError('success must be true/false, 1/0, yes/no: '+str(value))


def parse(rows):
    out=[]
    for row in rows:
        if not row.get('trial_id') or not row.get('condition'):
            raise ValueError('Every trial must have ID and condition')
        trial={'trial_id':row['trial_id'],'condition':row['condition'],
               'success':yes(row['success'])}
        for name in COLUMNS[3:]:
            value=float(row[name])
            if not math.isfinite(value) or value<0:
                raise ValueError('Invalid '+name+' in '+row['trial_id'])
            trial[name]=value
        out.append(trial)
    if len({x['trial_id'] for x in out})!=len(out):
        raise ValueError('Duplicate trial IDs')
    return out


def evaluate(trials):
    if not trials: return {'trials':0,'message':'No measured trials provided'}
    n=len(trials)
    successes=sum(t['success'] for t in trials)
    lo,hi=wilson_interval(successes,n)
    means = {col:statistics.mean(t[col] for t in trials) for col in COLUMNS[3:]}
    medians={col:statistics.median(t[col] for t in trials) for col in COLUMNS[3:]}
    thresholds={'localization_precision_le_2mm':sum(t['localization_error_mm']<=2 for t in trials),
                'target_error_le_3mm':sum(t['final_error_mm']<=3 for t in trials),
                'peak_force_le_10n':sum(t['peak_force_n']<=10 for t in trials),
                'execution_time_le_15s':sum(t['execution_time_s']<=15 for t in trials)}
    return {'trials':n,'successes':successes,'success_rate':successes/n,
            'success_95pct_wilson_ci':[lo,hi],
            'means':means,'medians':medians,
            'max_peak_force_n':max(t['peak_force_n'] for t in trials),
            'threshold_counts':thresholds,
            'target_grasp_success_ge_85pct_met':successes/n>=.85,
            'note':'Observed descriptive statistics, not a confidence guarantee or proof of control safety'}


def main():
    if len(sys.argv)!=2:
        raise SystemExit('Usage: analyze_trials.py measured_trials.csv')
    with open(sys.argv[1],newline='') as stream:
        data=parse(csv.DictReader(stream))
    groups=defaultdict(list)
    for trial in data: groups[trial['condition']].append(trial)
    result={'overall':evaluate(data),
            'by_condition':{c:evaluate(v) for c,v in sorted(groups.items())}}
    print(json.dumps(result,indent=2))
    if len(data)!=120:
        print(f'NOTE: {len(data)} measured trials, proposal target is 120',file=sys.stderr)

if __name__=='__main__': main()
