#!/usr/bin/env bash
# ===========================================================================
#  Dependency bootstrap for the TOPS 2026 deliverable
# ===========================================================================
#
#   ./setup.sh --check     report what is missing and change nothing
#   ./setup.sh             install whatever is missing (asks for sudo)
#
# Prefer `make doctor` and `make deps`, which call this.
#
# Target platform: Ubuntu 26.04 with ROS 2 Lyrical and Gazebo. The versions
# are not incidental -- the gz_ros2_control plugin that lets the controller
# drive a simulated robot is compiled against both, so mixing releases
# produces load-time plugin errors rather than anything helpful.
#
# NOTE ON PYTHON: this script installs system (apt) packages only. It never
# runs a global `pip install`, per the project rule in ../.agents/rules.
# The one optional Python dependency, scipy, goes into a local virtualenv
# via `make venv`.

set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-lyrical}"
CHECK_ONLY=0
[[ "${1:-}" == "--check" ]] && CHECK_ONLY=1

bold() { printf '\033[1m%s\033[0m\n' "$*"; }
ok()   { printf '  \033[32mok\033[0m       %s\n' "$*"; }
bad()  { printf '  \033[31mMISSING\033[0m  %s\n' "$*"; }
warn() { printf '\033[33m%s\033[0m\n' "$*"; }

# Every package, with the reason it is here. "Install this list of fourteen
# things" is not something anybody should do on trust.
declare -a PKGS=(
  "ros-${ROS_DISTRO}-desktop|core ROS 2, RViz, standard message types"
  "ros-${ROS_DISTRO}-ros-gz|the bridge between Gazebo and ROS 2"
  "ros-${ROS_DISTRO}-gz-ros2-control|lets the controller drive a SIMULATED robot"
  "ros-${ROS_DISTRO}-ros2-control|the controller framework itself"
  "ros-${ROS_DISTRO}-ros2-controllers|stock controllers: joint state, trajectory, gripper"
  "ros-${ROS_DISTRO}-kdl-parser|turns the robot model into a solvable kinematic chain"
  "ros-${ROS_DISTRO}-joint-state-publisher-gui|the slider window used by 'make view'"
  "ros-${ROS_DISTRO}-xacro|expands the robot model macros into plain XML"
  "ros-${ROS_DISTRO}-tf2-eigen|coordinate-frame maths conversions"
  "liborocos-kdl-dev|the rigid-body dynamics solvers the controller calls"
  "libeigen3-dev|the linear algebra library every matrix in the controller uses"
  "python3-colcon-common-extensions|the build tool"
  "libgtest-dev|the C++ unit-test framework"
  "python3-pytest|the Python unit-test framework"
  "python3-numpy|array maths for the figures"
  "python3-matplotlib|draws the result figures (make plots)"
)

bold ""
bold "TOPS 2026 dependency check  (ROS 2 ${ROS_DISTRO})"
echo

if ! command -v apt-get >/dev/null 2>&1; then
  warn "apt-get was not found. This workspace targets Ubuntu 26.04 and"
  warn "cannot run on Windows or macOS."
  exit 1
fi

if [[ ! -d "/opt/ros/${ROS_DISTRO}" ]]; then
  warn "ROS 2 '${ROS_DISTRO}' is not installed: /opt/ros/${ROS_DISTRO} does not exist."
  echo
  echo "Install it first:  https://docs.ros.org/en/${ROS_DISTRO}/Installation.html"
  echo "Or, if you have a different release:  ROS_DISTRO=<yours> ./setup.sh --check"
  exit 1
fi
ok "ROS 2 found at /opt/ros/${ROS_DISTRO}"
echo

missing=()
for entry in "${PKGS[@]}"; do
  pkg="${entry%%|*}"
  why="${entry##*|}"
  if dpkg -s "$pkg" >/dev/null 2>&1; then
    ok "$(printf '%-44s' "$pkg")$why"
  else
    bad "$(printf '%-44s' "$pkg")$why"
    missing+=("$pkg")
  fi
done

echo
if python3 -c 'import scipy' >/dev/null 2>&1; then
  ok "scipy present: the analysis will use exact p-values"
else
  echo "  --       scipy absent (optional)"
  echo "           The analysis still runs: it falls back to pure-Python"
  echo "           approximations and says so in its own output. They match"
  echo "           published statistical tables to five decimal places, so"
  echo "           they are fine for development. Install scipy before"
  echo "           quoting final p-values in the report:"
  echo "               make venv"
fi

echo
if [[ ${#missing[@]} -eq 0 ]]; then
  bold "Everything required is installed."
  echo
  echo "Next:    make build && make test"
  echo
  exit 0
fi

bold "${#missing[@]} required package(s) missing."
echo
if [[ $CHECK_ONLY -eq 1 ]]; then
  echo "Install them with either of:"
  echo "    make deps"
  echo "    sudo apt-get install ${missing[*]}"
  echo
  exit 1
fi

echo "Installing now. This needs sudo."
echo
sudo apt-get update
sudo apt-get install -y "${missing[@]}"

echo
bold "Done."
echo
echo "Next:    make build && make test"
echo
