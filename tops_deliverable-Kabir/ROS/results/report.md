```
TOPS 2026 -- trial analysis of /home/kabir_09/Desktop/Kabir/Jai_Shri_Maha_Mangal/GitHub/Resources/Projects/TOPS/tops_deliverable/results/trials.csv

==============================================================================
  1.  OVERVIEW
==============================================================================
120 trials, 3 modes, 4 regimes
statistics backend: pure-Python fallback (approximate)
  NOTE: p-values below come from normal approximations. They are
  accurate to a few thousandths at n >= 30 per group and less so
  below n = 10. Install scipy before quoting them in the report.

mode             n   success           95% CI   peak F (N)   error (mm)   time (s)
------------------------------------------------------------------------------
admittance      40     0.0%     [0.00, 0.09]   14.07+-2.17     nan+-nan      11.68
baseline        40     0.0%     [0.00, 0.09]    0.00+-0.00     nan+-nan      14.11
impedance       40    85.0%     [0.71, 0.93]    4.22+-5.73   14.72+-1.78     13.14

Targets: success >= 85 % overall and >= 95 % clean; peak force
<= 10.0 N; terminal error <= 3.0 mm; execution time <= 15.0 s.

==============================================================================
  2.  AGAINST THE PLAN'S TARGETS
==============================================================================
mode          metric                         measured     target  verdict
------------------------------------------------------------------------------
admittance    grasp success rate                 0.0%      85.0%  MISS
              grasp success (clean only)         0.0%      95.0%  MISS
              peak contact force (worst)      18.13 N    10.00 N  MISS
              trial execution time (worst)      15.57 s    15.00 s  MISS
------------------------------------------------------------------------------
baseline      grasp success rate                 0.0%      85.0%  MISS
              grasp success (clean only)         0.0%      95.0%  MISS
              peak contact force (worst)       0.00 N    10.00 N  PASS
              trial execution time (worst)      15.24 s    15.00 s  MISS
------------------------------------------------------------------------------
impedance     grasp success rate                85.0%      85.0%  PASS
              grasp success (clean only)        90.0%      95.0%  MISS
              peak contact force (worst)      32.14 N    10.00 N  MISS
              terminal targeting error (worst)     17.79 mm    3.00 mm  MISS
              trial execution time (worst)      33.14 s    15.00 s  MISS
------------------------------------------------------------------------------

==============================================================================
  3.  ADMITTANCE  vs  BASELINE
==============================================================================
Grasp success:  admittance 0/40 (0.0%)   baseline 0/40 (0.0%)
  degenerate (no variation in outcome): p = 1.0000   (not significant at 0.05)
  (success is a proportion, so this is deliberately NOT a t-test)

40 matched (regime, seed) pairs found -- paired tests are used where possible.
Pairing cancels trial-to-trial variation and is substantially
more powerful than the unpaired equivalent for the same n.

--- Peak contact force (N) ---
  admittance   mean  14.074  sd  2.166  median  14.527  max  18.129  n=40
  baseline     mean   0.000  sd  0.000  median   0.000  max   0.000  n=40
  difference   +14.074 N  (+nan %)
  Welch's t (fallback)                   t =  41.095  p = <0.0001  ***
  Mann-Whitney U (normal approximation)  U =     0.0  p = <0.0001  ***
  the two tests AGREE, so the result is robust to the distributional assumption
  Cohen's d = +9.189 (large), rank-biserial r = +1.000
  (a p-value says the difference is real; the effect size says whether it matters)
  paired t (fallback)                    t =  41.095  p = <0.0001  ***   (n=40 pairs)
  Wilcoxon signed-rank (normal approximation) W =     0.0  p = <0.0001  ***

--- Trial execution time (s) ---
  admittance   mean  11.680  sd  4.978  median  13.902  max  15.571  n=40
  baseline     mean  14.105  sd  0.559  median  13.790  max  15.239  n=40
  difference    -2.425 s  (-17.2 %)
  Welch's t (fallback)                   t =  -3.062  p = 0.0039  **
  Mann-Whitney U (normal approximation)  U =   722.5  p = 0.4587  (not significant at 0.05)
  the two tests DISAGREE -- outliers or skew are driving the t-test. Look at the data rather than picking the answer you prefer.
  Cohen's d = -0.685 (medium), rank-biserial r = -0.097
  (a p-value says the difference is real; the effect size says whether it matters)
  paired t (fallback)                    t =  -3.006  p = 0.0046  **   (n=40 pairs)
  Wilcoxon signed-rank (normal approximation) W =   308.0  p = 0.1725  (not significant at 0.05)

--- Mean force while in contact (N) ---
  admittance   mean   6.957  sd  1.332  median   7.004  max   9.608  n=40
  baseline     mean   0.000  sd  0.000  median   0.000  max   0.000  n=40
  difference    +6.957 N  (+nan %)
  Welch's t (fallback)                   t =  33.027  p = <0.0001  ***
  Mann-Whitney U (normal approximation)  U =     0.0  p = <0.0001  ***
  the two tests AGREE, so the result is robust to the distributional assumption
  Cohen's d = +7.385 (large), rank-biserial r = +1.000
  (a p-value says the difference is real; the effect size says whether it matters)
  paired t (fallback)                    t =  33.027  p = <0.0001  ***   (n=40 pairs)
  Wilcoxon signed-rank (normal approximation) W =     0.0  p = <0.0001  ***

==============================================================================
  3.  IMPEDANCE  vs  BASELINE
==============================================================================
Grasp success:  impedance 34/40 (85.0%)   baseline 0/40 (0.0%)
  chi-squared with Yates' correction: p = <0.0001   ***
  (success is a proportion, so this is deliberately NOT a t-test)

40 matched (regime, seed) pairs found -- paired tests are used where possible.
Pairing cancels trial-to-trial variation and is substantially
more powerful than the unpaired equivalent for the same n.

--- Peak contact force (N) ---
  impedance    mean   4.223  sd  5.726  median   2.933  max  32.136  n=40
  baseline     mean   0.000  sd  0.000  median   0.000  max   0.000  n=40
  difference    +4.223 N  (+nan %)
  Welch's t (fallback)                   t =   4.664  p = <0.0001  ***
  Mann-Whitney U (normal approximation)  U =     0.0  p = <0.0001  ***
  the two tests AGREE, so the result is robust to the distributional assumption
  Cohen's d = +1.043 (large), rank-biserial r = +1.000
  (a p-value says the difference is real; the effect size says whether it matters)
  paired t (fallback)                    t =   4.664  p = <0.0001  ***   (n=40 pairs)
  Wilcoxon signed-rank (normal approximation) W =     0.0  p = <0.0001  ***

--- Trial execution time (s) ---
  impedance    mean  13.137  sd  3.738  median  12.906  max  33.137  n=40
  baseline     mean  14.105  sd  0.559  median  13.790  max  15.239  n=40
  difference    -0.968 s  (-6.9 %)
  Welch's t (fallback)                   t =  -1.619  p = 0.1131  (not significant at 0.05)
  Mann-Whitney U (normal approximation)  U =   217.0  p = <0.0001  ***
  the two tests DISAGREE -- outliers or skew are driving the t-test. Look at the data rather than picking the answer you prefer.
  Cohen's d = -0.362 (small), rank-biserial r = -0.729
  (a p-value says the difference is real; the effect size says whether it matters)
  paired t (fallback)                    t =  -1.670  p = 0.1029  (not significant at 0.05)   (n=40 pairs)
  Wilcoxon signed-rank (normal approximation) W =    94.0  p = <0.0001  ***

--- Mean force while in contact (N) ---
  impedance    mean   1.778  sd  1.606  median   1.489  max  10.981  n=40
  baseline     mean   0.000  sd  0.000  median   0.000  max   0.000  n=40
  difference    +1.778 N  (+nan %)
  Welch's t (fallback)                   t =   7.005  p = <0.0001  ***
  Mann-Whitney U (normal approximation)  U =     0.0  p = <0.0001  ***
  the two tests AGREE, so the result is robust to the distributional assumption
  Cohen's d = +1.566 (large), rank-biserial r = +1.000
  (a p-value says the difference is real; the effect size says whether it matters)
  paired t (fallback)                    t =   7.005  p = <0.0001  ***   (n=40 pairs)
  Wilcoxon signed-rank (normal approximation) W =     0.0  p = <0.0001  ***

==============================================================================
  4.  BY DISTURBANCE REGIME
==============================================================================
The headline numbers average over four very different situations.
The result worth reporting is how the gap CHANGES across them: the
clean baseline should show little difference, and the calibration
regime -- a fixed offset that does not average out -- is where the
compliance is supposed to earn its place.

success rate
regime             admittance      baseline     impedance
---------------------------------------------------------
clean                   0.0%          0.0%         90.0% 
sensor_noise            0.0%          0.0%        100.0% 
calibration             0.0%          0.0%         90.0% 
displacement            0.0%          0.0%         60.0% 

mean peak force (N)
regime             admittance      baseline     impedance
---------------------------------------------------------
clean                   13.35          0.00          3.13
sensor_noise            15.77          0.00          3.02
calibration             13.52          0.00          2.67
displacement            13.66          0.00          8.08

mean terminal error (mm)
regime             admittance      baseline     impedance
---------------------------------------------------------
clean                      --            --         15.37
sensor_noise               --            --         13.38
calibration                --            --         15.16
displacement               --            --         15.31

==============================================================================
  5.  CONTROLLER HEALTH AND FAULTS
==============================================================================
Not results -- evidence that the results are trustworthy. A run
with hundreds of missed deadlines is not a 1 kHz experiment, and
saying so is better than having a reviewer notice.

mode           force trips   watchdog   throttled  max dt (ms)    missed
------------------------------------------------------------------------------
admittance               8          2           0        92.00      2908
baseline                 0          0           0         0.00         0
impedance                1          5           1       133.00      2644

fault counts across all trials:
  servo_timeout                  70
  wrench_cutoff                   9
  watchdog                        7

energy tank, impedance mode: lowest observed 0.051 J across 40 trials
(the floor is 0.05 J; reaching it means the passivity monitor
 actively throttled the controller, which is the mechanism
 working, not a fault)

==============================================================================
```
