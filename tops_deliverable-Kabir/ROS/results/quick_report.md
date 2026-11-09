```
TOPS 2026 -- trial analysis of /home/kabir_09/Desktop/Kabir/Jai_Shri_Maha_Mangal/GitHub/Resources/Projects/TOPS/tops_deliverable/results/quick.csv

==============================================================================
  1.  OVERVIEW
==============================================================================
24 trials, 3 modes, 4 regimes
statistics backend: pure-Python fallback (approximate)
  NOTE: p-values below come from normal approximations. They are
  accurate to a few thousandths at n >= 30 per group and less so
  below n = 10. Install scipy before quoting them in the report.

mode             n   success           95% CI   peak F (N)   error (mm)   time (s)
------------------------------------------------------------------------------
admittance       8     0.0%     [0.00, 0.32]   14.60+-3.01     nan+-nan      10.99
baseline         8     0.0%     [0.00, 0.32]    0.00+-0.00     nan+-nan      12.84
impedance        8   100.0%     [0.68, 1.00]    2.77+-0.35   13.98+-1.36     12.67

Targets: success >= 85 % overall and >= 95 % clean; peak force
<= 10.0 N; terminal error <= 3.0 mm; execution time <= 15.0 s.

==============================================================================
  2.  AGAINST THE PLAN'S TARGETS
==============================================================================
mode          metric                         measured     target  verdict
------------------------------------------------------------------------------
admittance    grasp success rate                 0.0%      85.0%  MISS
              grasp success (clean only)         0.0%      95.0%  MISS
              peak contact force (worst)      19.24 N    10.00 N  MISS
              trial execution time (worst)      14.70 s    15.00 s  PASS
------------------------------------------------------------------------------
baseline      grasp success rate                 0.0%      85.0%  MISS
              grasp success (clean only)         0.0%      95.0%  MISS
              peak contact force (worst)       0.00 N    10.00 N  PASS
              trial execution time (worst)      14.56 s    15.00 s  PASS
------------------------------------------------------------------------------
impedance     grasp success rate               100.0%      85.0%  PASS
              grasp success (clean only)       100.0%      95.0%  PASS
              peak contact force (worst)       3.24 N    10.00 N  PASS
              terminal targeting error (worst)     15.26 mm    3.00 mm  MISS
              trial execution time (worst)      13.30 s    15.00 s  PASS
------------------------------------------------------------------------------

==============================================================================
  3.  ADMITTANCE  vs  BASELINE
==============================================================================
Grasp success:  admittance 0/8 (0.0%)   baseline 0/8 (0.0%)
  degenerate (no variation in outcome): p = 1.0000   (not significant at 0.05)
  (success is a proportion, so this is deliberately NOT a t-test)

8 matched (regime, seed) pairs found -- paired tests are used where possible.
Pairing cancels trial-to-trial variation and is substantially
more powerful than the unpaired equivalent for the same n.

--- Peak contact force (N) ---
  admittance   mean  14.595  sd  3.014  median  14.547  max  19.244  n=8
  baseline     mean   0.000  sd  0.000  median   0.000  max   0.000  n=8
  difference   +14.595 N  (+nan %)
  Welch's t (fallback)                   t =  13.698  p = <0.0001  ***
  Mann-Whitney U (normal approximation)  U =     0.0  p = 0.0004  ***
  the two tests AGREE, so the result is robust to the distributional assumption
  Cohen's d = +6.849 (large), rank-biserial r = +1.000
  (a p-value says the difference is real; the effect size says whether it matters)
  paired t (fallback)                    t =  13.698  p = <0.0001  ***   (n=8 pairs)
  Wilcoxon signed-rank (normal approximation) W =     0.0  p = 0.0143  *

--- Trial execution time (s) ---
  admittance   mean  10.993  sd  5.477  median  13.713  max  14.696  n=8
  baseline     mean  12.843  sd  3.497  median  13.836  max  14.562  n=8
  difference    -1.850 s  (-14.4 %)
  Welch's t (fallback)                   t =  -0.805  p = 0.4364  (not significant at 0.05)
  Mann-Whitney U (normal approximation)  U =    27.0  p = 0.6365  (not significant at 0.05)
  the two tests AGREE, so the result is robust to the distributional assumption
  Cohen's d = -0.403 (small), rank-biserial r = -0.156
  (a p-value says the difference is real; the effect size says whether it matters)
  paired t (fallback)                    t =  -1.261  p = 0.2479  (not significant at 0.05)   (n=8 pairs)
  Wilcoxon signed-rank (normal approximation) W =     7.0  p = 0.1415  (not significant at 0.05)

--- Mean force while in contact (N) ---
  admittance   mean   6.433  sd  1.395  median   6.001  max   8.170  n=8
  baseline     mean   0.000  sd  0.000  median   0.000  max   0.000  n=8
  difference    +6.433 N  (+nan %)
  Welch's t (fallback)                   t =  13.048  p = <0.0001  ***
  Mann-Whitney U (normal approximation)  U =     0.0  p = 0.0004  ***
  the two tests AGREE, so the result is robust to the distributional assumption
  Cohen's d = +6.524 (large), rank-biserial r = +1.000
  (a p-value says the difference is real; the effect size says whether it matters)
  paired t (fallback)                    t =  13.048  p = <0.0001  ***   (n=8 pairs)
  Wilcoxon signed-rank (normal approximation) W =     0.0  p = 0.0143  *

==============================================================================
  3.  IMPEDANCE  vs  BASELINE
==============================================================================
Grasp success:  impedance 8/8 (100.0%)   baseline 0/8 (0.0%)
  chi-squared with Yates' correction: p = 0.0005   ***
  (success is a proportion, so this is deliberately NOT a t-test)

8 matched (regime, seed) pairs found -- paired tests are used where possible.
Pairing cancels trial-to-trial variation and is substantially
more powerful than the unpaired equivalent for the same n.

--- Peak contact force (N) ---
  impedance    mean   2.773  sd  0.353  median   2.677  max   3.243  n=8
  baseline     mean   0.000  sd  0.000  median   0.000  max   0.000  n=8
  difference    +2.773 N  (+nan %)
  Welch's t (fallback)                   t =  22.205  p = <0.0001  ***
  Mann-Whitney U (normal approximation)  U =     0.0  p = 0.0004  ***
  the two tests AGREE, so the result is robust to the distributional assumption
  Cohen's d = +11.103 (large), rank-biserial r = +1.000
  (a p-value says the difference is real; the effect size says whether it matters)
  paired t (fallback)                    t =  22.205  p = <0.0001  ***   (n=8 pairs)
  Wilcoxon signed-rank (normal approximation) W =     0.0  p = 0.0143  *

--- Trial execution time (s) ---
  impedance    mean  12.675  sd  0.364  median  12.685  max  13.295  n=8
  baseline     mean  12.843  sd  3.497  median  13.836  max  14.562  n=8
  difference    -0.168 s  (-1.3 %)
  Welch's t (fallback)                   t =  -0.135  p = 0.8961  (not significant at 0.05)
  Mann-Whitney U (normal approximation)  U =     8.0  p = 0.0136  *
  the two tests DISAGREE -- outliers or skew are driving the t-test. Look at the data rather than picking the answer you prefer.
  Cohen's d = -0.068 (negligible), rank-biserial r = -0.750
  (a p-value says the difference is real; the effect size says whether it matters)
  paired t (fallback)                    t =  -0.140  p = 0.8929  (not significant at 0.05)   (n=8 pairs)
  Wilcoxon signed-rank (normal approximation) W =     8.0  p = 0.1834  (not significant at 0.05)

--- Mean force while in contact (N) ---
  impedance    mean   1.489  sd  0.009  median   1.487  max   1.503  n=8
  baseline     mean   0.000  sd  0.000  median   0.000  max   0.000  n=8
  difference    +1.489 N  (+nan %)
  Welch's t (fallback)                   t = 445.110  p = <0.0001  ***
  Mann-Whitney U (normal approximation)  U =     0.0  p = 0.0004  ***
  the two tests AGREE, so the result is robust to the distributional assumption
  Cohen's d = +222.555 (large), rank-biserial r = +1.000
  (a p-value says the difference is real; the effect size says whether it matters)
  paired t (fallback)                    t = 445.110  p = <0.0001  ***   (n=8 pairs)
  Wilcoxon signed-rank (normal approximation) W =     0.0  p = 0.0143  *

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
clean                   0.0%          0.0%        100.0% 
sensor_noise            0.0%          0.0%        100.0% 
calibration             0.0%          0.0%        100.0% 
displacement            0.0%          0.0%        100.0% 

mean peak force (N)
regime             admittance      baseline     impedance
---------------------------------------------------------
clean                   15.26          0.00          2.40
sensor_noise            11.60          0.00          2.89
calibration             16.18          0.00          2.61
displacement            15.34          0.00          3.20

mean terminal error (mm)
regime             admittance      baseline     impedance
---------------------------------------------------------
clean                      --            --         14.53
sensor_noise               --            --         12.07
calibration                --            --         14.68
displacement               --            --         14.65

==============================================================================
  5.  CONTROLLER HEALTH AND FAULTS
==============================================================================
Not results -- evidence that the results are trustworthy. A run
with hundreds of missed deadlines is not a 1 kHz experiment, and
saying so is better than having a reviewer notice.

mode           force trips   watchdog   throttled  max dt (ms)    missed
------------------------------------------------------------------------------
admittance               2          0           0        69.00      2963
baseline                 0          1           0         0.00         0
impedance                0          0           0        61.00      2691

fault counts across all trials:
  servo_timeout                  13
  wrench_cutoff                   2
  watchdog                        1

energy tank, impedance mode: lowest observed 1.021 J across 8 trials
(the floor is 0.05 J; reaching it means the passivity monitor
 actively throttled the controller, which is the mechanism
 working, not a fault)

==============================================================================
```
