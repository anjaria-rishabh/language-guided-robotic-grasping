# ===========================================================================
#  TOPS 2026 deliverable -- the single interface to this workspace
# ===========================================================================
#
#  Run `make` on its own for the command list.
#
#  Every target sources ROS 2 itself, so none of them need a pre-prepared
#  shell. That is deliberate: "did you source it?" is the most common reason
#  a ROS 2 command fails, and a Makefile can simply delete the question.
#
#  Every target also runs from the workspace root regardless of where you
#  invoked it, and writes results to absolute paths, so `make analyse` cannot
#  quietly produce a report in a different directory than `make trials` wrote.

ROS_DISTRO ?= lyrical
ROS_SETUP  := /opt/ros/$(ROS_DISTRO)/setup.bash
WS_SETUP   := $(CURDIR)/install/setup.bash
VENV       := $(CURDIR)/.venv
SHELL      := /bin/bash

# Release, not CMake's default of no optimisation. This is not a style
# preference: at -O0 the Eigen expression templates in the controller's
# update() are not inlined, the 7x7 and 6x6 products run far slower, and the
# 1 ms deadline is genuinely at risk on a laptop that is also running Gazebo.
BUILD_TYPE ?= Release
COLCON_ARGS := --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=$(BUILD_TYPE)

# --- knobs you can override on the command line ---------------------------
# (Comments sit on their own lines: a comment after a value leaves trailing
#  spaces IN the value, which then show up inside paths and launch args.)
# impedance | admittance | baseline
CONTROLLER ?= impedance
# clean | sensor_noise | calibration | displacement
REGIME     ?= clean
SEED       ?= 0
# repeats per (controller, regime) cell
PER_CELL   ?= 10
RESULTS    ?= $(CURDIR)/results
CSV        ?= $(RESULTS)/trials.csv
QUICK_CSV  := $(RESULTS)/quick.csv

# The analysis report that belongs to $(CSV): the full battery keeps the
# name results/report.md; any other CSV gets <name>_report.md beside it, so a
# quick smoke test can never overwrite the real report.
REPORT = $(if $(filter $(RESULTS)/trials.csv,$(CSV)),$(RESULTS)/report.md,$(basename $(CSV))_report.md)

# What `make plots` draws: the CSV you named on the command line, or else
# both the full battery and the quick test (whichever of them exist).
PLOT_CSVS = $(if $(filter command line,$(origin CSV)),$(CSV),$(RESULTS)/trials.csv $(QUICK_CSV))

# matplotlib keeps a font cache; point it into build/ so nothing is written
# into the home directory (build/ is disposable and git-ignored).
MPL_ENV := MPLCONFIGDIR=$(CURDIR)/build/matplotlib

SOURCE_ROS := source $(ROS_SETUP)
SOURCE_WS  := source $(ROS_SETUP) && source $(WS_SETUP)

.DEFAULT_GOAL := help

.PHONY: help doctor deps build build-verbose rebuild clean clean-results \
        test test-cpp test-py lint verify view run watch trials trials-quick \
        analyse report plots urdf topics venv

# ===========================================================================
#  0.  Orientation
# ===========================================================================
help:
	@echo ''
	@echo '  TOPS 2026 -- language-guided grasping with compliant control'
	@echo '  ========================================================='
	@echo ''
	@echo '  FIRST TIME, in this order:'
	@echo '    make doctor        check what system packages are missing'
	@echo '    make deps          install the missing ones (asks for sudo)'
	@echo '    make build         compile the workspace            (~1-2 min)'
	@echo '    make test          run every unit test              (~1 min)'
	@echo '    make verify        prove the statistics pipeline works, no robot'
	@echo ''
	@echo '  SEE IT WORK:'
	@echo '    make view          the arm in RViz, sliders, no physics'
	@echo '    make watch         ONE full grasp in Gazebo, start to finish'
	@echo '    make run           bring the system up and leave it up'
	@echo ''
	@echo '  PRODUCE THE RESULTS:'
	@echo '    make trials        the full 120-trial battery         (~1-2 h)'
	@echo '    make trials-quick  a 24-trial smoke test              (~15 min)'
	@echo '    make analyse       turn the trial CSV into the report tables'
	@echo '    make plots         graphs from the full AND the quick CSV -> results/figures/'
	@echo '                       (trials / trials-quick also plot automatically)'
	@echo ''
	@echo '  HOUSEKEEPING:'
	@echo '    make urdf          print the expanded robot model'
	@echo '    make topics        list what is running (system must be up)'
	@echo '    make clean         delete build output'
	@echo '    make clean-results delete recorded trial data, reports and figures'
	@echo ''
	@echo '  Options (append to any run/trial target):'
	@echo '    CONTROLLER=impedance|admittance|baseline   (now: $(CONTROLLER))'
	@echo '    REGIME=clean|sensor_noise|calibration|displacement  (now: $(REGIME))'
	@echo '    SEED=<int>  PER_CELL=<int>  CSV=<path>'
	@echo '    Trial CSVs are APPENDED to, never overwritten: clear stale rows yourself.'
	@echo ''
	@echo '  Example:  make watch CONTROLLER=baseline REGIME=calibration'
	@echo '            (the rigid controller against a miscalibrated camera --'
	@echo '             this is the failure the project exists to fix)'
	@echo ''

# ===========================================================================
#  1.  Dependencies
# ===========================================================================
doctor:
	@./setup.sh --check

deps:
	@./setup.sh

# scipy is optional. The project rule in ../.agents/rules is that Python
# packages are never installed globally, so it goes in a local virtualenv
# created with --system-site-packages (so ROS 2's own Python modules stay
# visible from inside it).
venv:
	@if [ ! -d "$(VENV)" ]; then \
		echo "creating $(VENV) ..."; \
		python3 -m venv --system-site-packages "$(VENV)"; \
	fi
	@"$(VENV)/bin/pip" install --quiet --upgrade pip
	@"$(VENV)/bin/pip" install --quiet scipy
	@echo "scipy installed in $(VENV)."
	@echo "The analysis uses it automatically when you run it through this venv:"
	@echo "    $(VENV)/bin/python -m tops_experiments.analyze_trials --csv $(CSV)"

# ===========================================================================
#  2.  Build
# ===========================================================================
build:
	@$(SOURCE_ROS) && cd $(CURDIR) && colcon build $(COLCON_ARGS)
	@echo ''
	@echo 'Built. Nothing else to source -- every make target does it for you.'

build-verbose:
	@$(SOURCE_ROS) && cd $(CURDIR) && \
		colcon build $(COLCON_ARGS) --event-handlers console_direct+

clean:
	@rm -rf $(CURDIR)/build $(CURDIR)/install $(CURDIR)/log
	@find $(CURDIR)/src -name __pycache__ -type d -prune -exec rm -rf {} + 2>/dev/null || true
	@find $(CURDIR)/src -name .pytest_cache -type d -prune -exec rm -rf {} + 2>/dev/null || true
	@echo 'removed build/ install/ log/ and Python caches'

# A clean rebuild resolves a surprising share of "impossible" problems: a
# deleted file still being found, a stale parameter, a renamed launch file.
# Try this before debugging anything else.
rebuild: clean build

clean-results:
	@rm -f $(RESULTS)/*.csv $(RESULTS)/*.md
	@rm -rf $(RESULTS)/figures
	@echo 'cleared $(RESULTS)'

# ===========================================================================
#  3.  Test
# ===========================================================================
#
#  `make test` runs the C++ and Python suites SEPARATELY and on purpose.
#
#  colcon decides how to test a Python package based on which plugins happen
#  to be installed on the host. On a host without its pytest plugin it falls
#  back to setuptools' unittest runner, collects zero tests, and reports
#  "NO TESTS RAN" -- which is a pass as far as a CI script is concerned. A
#  test suite that silently does not run is worse than no test suite, so
#  pytest is invoked directly here rather than trusted to colcon.
test: build
	@echo ''
	@echo '--- C++ controller + model tests (via colcon) --------------------'
	@$(SOURCE_WS) && cd $(CURDIR) && \
		colcon test --packages-select tops_controllers tops_description \
			--event-handlers console_cohesion- --return-code-on-test-failure
	@$(SOURCE_ROS) && cd $(CURDIR) && colcon test-result --verbose
	@$(MAKE) --no-print-directory test-py
	@echo ''
	@echo '================================================================'
	@echo '  all suites passed'
	@echo '================================================================'

test-cpp: build
	@$(SOURCE_WS) && cd $(CURDIR) && \
		colcon test --packages-select tops_controllers \
			--event-handlers console_direct+ --return-code-on-test-failure
	@$(SOURCE_ROS) && cd $(CURDIR) && colcon test-result --verbose

# The Python suites need neither a build nor ROS 2, which makes them the
# fastest feedback loop available while working on the state machine or the
# statistics. Run this one on its own, constantly.
test-py:
	@echo ''
	@echo '--- supervisor state machine (pytest) ----------------------------'
	@cd $(CURDIR)/src/tops_supervisor && python3 -m pytest test/ -q
	@echo ''
	@echo '--- statistics + analysis pipeline (pytest) ----------------------'
	@cd $(CURDIR)/src/tops_experiments && $(MPL_ENV) python3 -m pytest test/ -q

lint: build
	@$(SOURCE_WS) && cd $(CURDIR) && \
		colcon test --packages-select tops_controllers tops_description \
			--ctest-args -R "lint|xmllint|flake8|pep257|copyright" || true
	@$(SOURCE_ROS) && cd $(CURDIR) && colcon test-result --verbose

# Exercise the entire analysis route on synthetic data with a KNOWN effect.
# Worth running in week 2 rather than week 10: debugging a statistics
# pipeline on the same evening you generate the numbers you are submitting is
# how deadlines get missed.
verify:
	@mkdir -p $(RESULTS)
	@cd $(CURDIR)/src/tops_experiments && python3 -m tops_experiments.analyze_trials \
		--demo --demo-path $(RESULTS)/demo_trials.csv \
		--markdown $(RESULTS)/demo_report.md
	@$(MAKE) --no-print-directory plots CSV=$(RESULTS)/demo_trials.csv
	@echo ''
	@echo "synthetic data : $(RESULTS)/demo_trials.csv"
	@echo "report         : $(RESULTS)/demo_report.md"
	@echo "figures        : $(RESULTS)/figures/demo_trials/"
	@echo 'Those numbers are synthetic. They prove the pipeline runs; they are not results.'

# ===========================================================================
#  4.  Run
# ===========================================================================
# Kinematics only: RViz plus a slider per joint, no physics and no
# controllers. Never skip this after editing the robot model -- a wrong joint
# axis takes ten seconds to spot here, and in Gazebo the same mistake looks
# like an arm exploding for no reason.
view: $(WS_SETUP)
	@$(SOURCE_WS) && cd $(CURDIR) && \
		ros2 launch tops_description view_robot.launch.py

# ONE grasp, GUI on, fully automatic: brings the stack up, performs the
# grasp, records a row, shuts everything down. This is the demonstration.
watch: $(WS_SETUP)
	@mkdir -p $(RESULTS)
	@echo 'One grasp: $(CONTROLLER) controller, $(REGIME) perception.'
	@echo 'Gazebo takes ~25 s to start before the arm moves. Watch the terminal'
	@echo 'for the state machine: SERVO -> COMPLIANT -> GRASP -> LIFT -> HOLD.'
	@$(SOURCE_WS) && cd $(CURDIR) && \
		ros2 launch tops_experiments trial.launch.py \
			controller:=$(CONTROLLER) regime:=$(REGIME) seed:=$(SEED) \
			output_csv:=$(RESULTS)/watch.csv gui:=true
	@echo ''
	@echo "recorded: $(RESULTS)/watch.csv"

# Bring the system up and leave it up, for poking at it by hand.
run: $(WS_SETUP)
	@echo 'Starting the full system. It will come up and then WAIT.'
	@echo 'To make it attempt a grasp, open a second terminal and run:'
	@echo ''
	@echo '    cd $(CURDIR) && make grasp'
	@echo ''
	@$(SOURCE_WS) && cd $(CURDIR) && \
		ros2 launch tops_bringup tops_system.launch.py \
			controller:=$(CONTROLLER) regime:=$(REGIME) seed:=$(SEED) rviz:=false

.PHONY: grasp
grasp: $(WS_SETUP)
	@$(SOURCE_WS) && ros2 service call \
		/tops_supervisor/start_trial std_srvs/srv/Trigger "{}"

# ===========================================================================
#  5.  Experiments
# ===========================================================================
# New rows are APPENDED to $(CSV). Clear stale rows yourself (or run
# `make clean-results`) before a run whose numbers must stand on their own.
trials: $(WS_SETUP)
	@mkdir -p $(RESULTS)
	@$(SOURCE_WS) && cd $(CURDIR) && \
		ros2 run tops_experiments run_trials -- \
			--per-cell $(PER_CELL) --output $(CSV)
	@$(MAKE) --no-print-directory analyse CSV=$(CSV)
	@$(MAKE) --no-print-directory plots CSV=$(CSV)

# The same battery at 2 repeats per cell, into its own CSV, report and
# figures folder (results/quick.csv, quick_report.md, figures/quick/).
trials-quick:
	@$(MAKE) --no-print-directory trials PER_CELL=2 CSV=$(QUICK_CSV)

analyse: $(WS_SETUP)
	@mkdir -p $(RESULTS)
	@$(SOURCE_WS) && cd $(CURDIR) && \
		ros2 run tops_experiments analyze_trials -- \
			--csv $(CSV) --markdown $(REPORT)

report: analyse

# Graphs. Needs no build and no ROS: plain Python + matplotlib, like verify.
# Each CSV gets its own folder: results/figures/trials/, results/figures/quick/.
plots:
	@mkdir -p $(RESULTS)/figures
	@cd $(CURDIR)/src/tops_experiments && $(MPL_ENV) python3 -m tops_experiments.plot_trials \
		--csv $(PLOT_CSVS) --out $(RESULTS)/figures

# ===========================================================================
#  6.  Inspection
# ===========================================================================
# Print the robot model with every macro expanded. Reading the real output is
# always faster than guessing what the macros produced.
urdf: $(WS_SETUP)
	@$(SOURCE_WS) && xacro \
		$(CURDIR)/src/tops_description/urdf/tops_robot.urdf.xacro \
		controllers_config:=/dev/null

topics: $(WS_SETUP)
	@$(SOURCE_WS) && echo '--- nodes ---'       && ros2 node list \
		&& echo '--- topics ---'                && ros2 topic list \
		&& echo '--- controllers ---'           && ros2 control list_controllers \
		&& echo '--- hardware interfaces ---'   && ros2 control list_hardware_interfaces

# A readable error instead of a wall of shell noise when the workspace has
# not been compiled yet.
$(WS_SETUP):
	@echo ''
	@echo 'The workspace is not built yet. Run:    make build'
	@echo ''
	@exit 1
