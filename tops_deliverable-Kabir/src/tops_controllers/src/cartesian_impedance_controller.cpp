// Copyright 2026 Kabir Dhingra. MIT licence.
//
// Implementation of the Cartesian impedance controller. See the header for
// the control law and docs/02_CONTROL_LAW.md for the derivation.

#include "tops_controllers/cartesian_impedance_controller.hpp"

#include <algorithm>
#include <cmath>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace
{

/// Convert an Eigen quaternion to the (x, y, z, w) array layout used in the
/// snapshot structs. Kept local so the ordering convention is written down
/// exactly once; getting it backwards is a bug that produces a plausible
/// but wrong orientation error, which is very hard to see in a plot.
inline void quatToArray(const Eigen::Quaterniond & q, double (&out)[4])
{
  out[0] = q.x();
  out[1] = q.y();
  out[2] = q.z();
  out[3] = q.w();
}

inline void poseToArray(const Eigen::Isometry3d & T, double (&out)[7])
{
  out[0] = T.translation().x();
  out[1] = T.translation().y();
  out[2] = T.translation().z();
  const Eigen::Quaterniond q(T.rotation());
  out[3] = q.x();
  out[4] = q.y();
  out[5] = q.z();
  out[6] = q.w();
}

template<typename VecT>
inline void copyToArray(const VecT & v, double * out, int n)
{
  for (int i = 0; i < n; ++i) {out[i] = v(i);}
}

}  // namespace

namespace tops_controllers
{

// ===========================================================================
//  Lifecycle: on_init
// ===========================================================================
//
// Parameters are declared, not read. At on_init time the controller has a
// node but the parameter server may not yet have been populated from YAML,
// so reading here would silently capture defaults. Reading happens in
// on_configure. This is a standard ros2_control trap and the source of a
// lot of "my YAML is being ignored" confusion.

controller_interface::CallbackReturn CartesianImpedanceController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::string>("base_frame", "panda_link0");
    auto_declare<std::string>("tip_frame", "panda_hand_tcp");
    auto_declare<std::string>("ft_sensor_name", "panda_ft_sensor");
    auto_declare<std::vector<double>>("gravity", {0.0, 0.0, -9.80665});

    auto_declare<std::vector<double>>("stiffness.stiff",
      {800.0, 800.0, 800.0, 60.0, 60.0, 60.0});
    auto_declare<std::vector<double>>("stiffness.compliant",
      {150.0, 150.0, 150.0, 12.0, 12.0, 12.0});
    auto_declare<std::vector<double>>("damping_ratio",
      {0.9, 0.9, 0.9, 0.9, 0.9, 0.9});
    auto_declare<double>("stiffness_ramp_rate", 1500.0);

    auto_declare<double>("nullspace.stiffness", 20.0);
    auto_declare<double>("nullspace.damping_ratio", 1.0);
    auto_declare<std::vector<double>>("nullspace.q_desired",
      {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785});

    auto_declare<std::vector<double>>("torque.limits",
      {87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0});
    auto_declare<double>("torque.rate_limit", 1000.0);
    auto_declare<double>("singularity_damping", 0.05);
    auto_declare<bool>("compensate_coriolis", true);

    auto_declare<std::vector<double>>("joint_limits.lower",
      {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973});
    auto_declare<std::vector<double>>("joint_limits.upper",
      {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973});
    auto_declare<double>("joint_limits.margin", 0.15);
    auto_declare<double>("joint_limits.stiffness", 40.0);

    auto_declare<double>("wrench.cutoff_hz_control", 30.0);
    auto_declare<double>("wrench.cutoff_hz_safety", 120.0);
    auto_declare<double>("wrench.deadband_n", 0.3);
    auto_declare<double>("wrench.clamp_n", 50.0);
    auto_declare<int>("wrench.bias_samples", 500);
    auto_declare<std::string>("wrench.sensor_frame", "panda_ft_sensor_link");
    auto_declare<double>("wrench.tool_mass", 1.07);
    auto_declare<std::vector<double>>("wrench.tool_com", {0.0, 0.0073, 0.1065});

    auto_declare<bool>("force_control.enabled", true);
    auto_declare<std::vector<double>>("force_control.axis_tip", {0.0, 0.0, 1.0});
    auto_declare<double>("force_control.desired_n", 5.0);
    auto_declare<double>("force_control.contact_threshold_n", 1.0);
    auto_declare<double>("force_control.kp", 0.5);
    auto_declare<double>("force_control.ki", 2.0);
    auto_declare<double>("force_control.integral_clamp_n", 3.0);
    auto_declare<double>("force_control.axis_stiffness_scale", 0.2);

    auto_declare<double>("tank.initial_energy", 0.5);
    auto_declare<double>("tank.min_energy", 0.05);
    auto_declare<double>("tank.max_energy", 2.0);
    auto_declare<double>("tank.max_damping_boost", 3.0);

    auto_declare<double>("safety.hard_force_limit_n", 15.0);
    auto_declare<double>("safety.command_timeout_s", 0.1);
    auto_declare<double>("safety.max_command_step_m", 0.25);

    auto_declare<double>("publish_rate_hz", 100.0);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init failed: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

// ===========================================================================
//  Which interfaces this controller claims
// ===========================================================================

controller_interface::InterfaceConfiguration
CartesianImpedanceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.reserve(joint_names_.size());
  for (const auto & joint : joint_names_) {
    // Effort, and only effort. Claiming a position interface as well would
    // be rejected by the resource manager anyway -- two controllers cannot
    // both own a joint -- but more to the point, impedance control has no
    // use for one.
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return config;
}

controller_interface::InterfaceConfiguration
CartesianImpedanceController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.reserve(joint_names_.size() * 2 + 6);

  for (const auto & joint : joint_names_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }

  // The wrist wrench, read in-process rather than off a topic. A topic hop
  // would add a copy, a serialisation, and non-deterministic latency to the
  // signal the safety interlock depends on.
  for (const auto & axis : {"force.x", "force.y", "force.z",
                            "torque.x", "torque.y", "torque.z"})
  {
    config.names.push_back(ft_sensor_name_ + "/" + axis);
  }

  return config;
}

// ===========================================================================
//  Lifecycle: on_configure
// ===========================================================================
//
// Everything expensive happens here: parameter reads, URDF parsing, solver
// construction, filter sizing, publisher and service creation. After this
// returns SUCCESS nothing in the real-time path will ever allocate again.

controller_interface::CallbackReturn CartesianImpedanceController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();
  auto logger = node->get_logger();

  joint_names_ = node->get_parameter("joints").as_string_array();
  if (static_cast<int>(joint_names_.size()) != kNumJoints) {
    RCLCPP_ERROR(
      logger,
      "'joints' has %zu entries; this controller is compiled for exactly %d. "
      "See kinematics_model.hpp for why that is fixed at compile time.",
      joint_names_.size(), kNumJoints);
    return controller_interface::CallbackReturn::ERROR;
  }

  base_frame_ = node->get_parameter("base_frame").as_string();
  tip_frame_ = node->get_parameter("tip_frame").as_string();
  ft_sensor_name_ = node->get_parameter("ft_sensor_name").as_string();

  const auto read_vec = [&](const std::string & name, int expected,
      Eigen::Ref<Eigen::VectorXd> out) -> bool {
      const auto v = node->get_parameter(name).as_double_array();
      if (static_cast<int>(v.size()) != expected) {
        RCLCPP_ERROR(
          logger, "parameter '%s' has %zu entries, expected %d",
          name.c_str(), v.size(), expected);
        return false;
      }
      for (int i = 0; i < expected; ++i) {out(i) = v[i];}
      return true;
    };

  if (!read_vec("stiffness.stiff", 6, stiffness_stiff_)) {
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!read_vec("stiffness.compliant", 6, stiffness_compliant_)) {
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!read_vec("damping_ratio", 6, damping_ratio_)) {
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!read_vec("nullspace.q_desired", kNumJoints, q_nullspace_)) {
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!read_vec("torque.limits", kNumJoints, torque_limits_)) {
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!read_vec("joint_limits.lower", kNumJoints, q_lower_)) {
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!read_vec("joint_limits.upper", kNumJoints, q_upper_)) {
    return controller_interface::CallbackReturn::ERROR;
  }

  {
    const auto g = node->get_parameter("gravity").as_double_array();
    if (g.size() != 3) {
      RCLCPP_ERROR(logger, "parameter 'gravity' must have 3 entries");
      return controller_interface::CallbackReturn::ERROR;
    }
    gravity_ = Eigen::Vector3d(g[0], g[1], g[2]);
  }

  stiffness_ramp_rate_ = node->get_parameter("stiffness_ramp_rate").as_double();
  nullspace_stiffness_ = node->get_parameter("nullspace.stiffness").as_double();
  nullspace_damping_ratio_ = node->get_parameter("nullspace.damping_ratio").as_double();
  torque_rate_limit_ = node->get_parameter("torque.rate_limit").as_double();
  singularity_damping_ = node->get_parameter("singularity_damping").as_double();
  compensate_coriolis_ = node->get_parameter("compensate_coriolis").as_bool();
  joint_limit_margin_ = node->get_parameter("joint_limits.margin").as_double();
  joint_limit_stiffness_ = node->get_parameter("joint_limits.stiffness").as_double();

  wrench_clamp_n_ = node->get_parameter("wrench.clamp_n").as_double();
  tool_mass_ = node->get_parameter("wrench.tool_mass").as_double();
  {
    const auto c = node->get_parameter("wrench.tool_com").as_double_array();
    if (c.size() != 3) {
      RCLCPP_ERROR(logger, "parameter 'wrench.tool_com' must have 3 entries");
      return controller_interface::CallbackReturn::ERROR;
    }
    tool_com_ = Eigen::Vector3d(c[0], c[1], c[2]);
  }
  sensor_frame_ = node->get_parameter("wrench.sensor_frame").as_string();
  bias_samples_total_ =
    static_cast<int>(node->get_parameter("wrench.bias_samples").as_int());
  bias_samples_remaining_ = bias_samples_total_;

  force_control_enabled_ = node->get_parameter("force_control.enabled").as_bool();
  {
    const auto a = node->get_parameter("force_control.axis_tip").as_double_array();
    if (a.size() != 3) {
      RCLCPP_ERROR(logger, "parameter 'force_control.axis_tip' must have 3 entries");
      return controller_interface::CallbackReturn::ERROR;
    }
    force_axis_tip_ = Eigen::Vector3d(a[0], a[1], a[2]);
    const double n = force_axis_tip_.norm();
    if (n < 1e-9) {
      RCLCPP_ERROR(logger, "'force_control.axis_tip' must be a non-zero direction");
      return controller_interface::CallbackReturn::ERROR;
    }
    force_axis_tip_ /= n;
  }
  force_desired_n_ = node->get_parameter("force_control.desired_n").as_double();
  force_contact_threshold_n_ =
    node->get_parameter("force_control.contact_threshold_n").as_double();
  force_kp_ = node->get_parameter("force_control.kp").as_double();
  force_ki_ = node->get_parameter("force_control.ki").as_double();
  force_integral_clamp_n_ = node->get_parameter("force_control.integral_clamp_n").as_double();
  force_axis_stiffness_scale_ =
    node->get_parameter("force_control.axis_stiffness_scale").as_double();

  hard_force_limit_n_ = node->get_parameter("safety.hard_force_limit_n").as_double();
  command_timeout_s_ = node->get_parameter("safety.command_timeout_s").as_double();
  max_command_step_m_ = node->get_parameter("safety.max_command_step_m").as_double();
  publish_rate_hz_ = node->get_parameter("publish_rate_hz").as_double();

  EnergyTankConfig tank_cfg;
  tank_cfg.initial_energy = node->get_parameter("tank.initial_energy").as_double();
  tank_cfg.min_energy = node->get_parameter("tank.min_energy").as_double();
  tank_cfg.max_energy = node->get_parameter("tank.max_energy").as_double();
  tank_cfg.max_damping_boost = node->get_parameter("tank.max_damping_boost").as_double();
  tank_.configure(tank_cfg);

  // --- the rigid-body model ---------------------------------------------
  // robot_description arrives as a parameter on the controller_manager and
  // is forwarded to each controller. If it is empty the usual cause is that
  // robot_state_publisher started after the controller manager.
  std::string urdf;
  if (node->has_parameter("robot_description")) {
    urdf = node->get_parameter("robot_description").as_string();
  }
  if (urdf.empty()) {
    urdf = get_robot_description();
  }
  if (urdf.empty()) {
    RCLCPP_ERROR(
      logger,
      "robot_description is empty. The controller cannot build a dynamics "
      "model without it. Check that robot_state_publisher is running and "
      "that the controller_manager received the description.");
    return controller_interface::CallbackReturn::ERROR;
  }

  std::string model_error;
  if (!model_.initialise(urdf, base_frame_, tip_frame_, gravity_, model_error)) {
    RCLCPP_ERROR(logger, "kinematic model failed: %s", model_error.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  // --- where the F/T sensor's axes point, relative to the TCP -------------
  // The sensor reports in its own frame. On the real Panda the hand is
  // bolted to the flange at -45 degrees about z, so the sensor's x/y axes
  // and the TCP's x/y axes differ by 45 degrees. Ignoring that rotates every
  // lateral force component by 45 degrees: the magnitude and the approach
  // (z) component survive, anything reading fx or fy is wrong. The rotation
  // is read from the URDF, so it stays correct if the mount ever changes.
  {
    Eigen::Isometry3d tip_T_sensor;
    std::string tf_error;
    if (!KinematicsModel::fixedTransform(urdf, tip_frame_, sensor_frame_, tip_T_sensor, tf_error)) {
      RCLCPP_ERROR(
        logger, "cannot relate the F/T sensor frame '%s' to the tip '%s': %s",
        sensor_frame_.c_str(), tip_frame_.c_str(), tf_error.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    R_tip_sensor_ = tip_T_sensor.rotation();
  }

  // The chain's joint order is the order every vector in this controller is
  // indexed by. If the YAML lists them differently, silently reordering
  // would be worse than refusing: the gains would be applied to the wrong
  // joints and the arm would behave plausibly but wrongly.
  for (int i = 0; i < kNumJoints; ++i) {
    if (model_.jointNames()[i] != joint_names_[i]) {
      RCLCPP_ERROR(
        logger,
        "joint order mismatch at index %d: the URDF chain has '%s' but "
        "'joints' lists '%s'. List them in chain order.",
        i, model_.jointNames()[i].c_str(), joint_names_[i].c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  model_ok_ = true;

  // --- filters ------------------------------------------------------------
  // Two filters off one sensor, deliberately. The control law wants
  // smoothness; the interlock wants speed. A single filter has to pick one,
  // and either choice is wrong for the other consumer.
  const double dt_nominal = 1.0 / std::max(1.0, static_cast<double>(get_update_rate()));
  nominal_period_ = dt_nominal;
  wrench_filter_control_.setCutoff(
    node->get_parameter("wrench.cutoff_hz_control").as_double(), dt_nominal);
  wrench_filter_safety_.setCutoff(
    node->get_parameter("wrench.cutoff_hz_safety").as_double(), dt_nominal);
  const double deadband = node->get_parameter("wrench.deadband_n").as_double();
  wrench_filter_control_.setDeadband(deadband);
  wrench_filter_control_.setSymmetricClamp(wrench_clamp_n_);
  wrench_filter_safety_.setSymmetricClamp(wrench_clamp_n_);

  stiffness_ramp_.setMaxRate(stiffness_ramp_rate_);

  RCLCPP_INFO(
    logger,
    "configured: chain %s -> %s, %d joints, update rate %d Hz, "
    "control-path wrench cutoff %.1f Hz, safety-path cutoff %.1f Hz",
    base_frame_.c_str(), tip_frame_.c_str(), kNumJoints, get_update_rate(),
    wrench_filter_control_.cutoffHz(dt_nominal),
    wrench_filter_safety_.cutoffHz(dt_nominal));

  // --- ROS interfaces -----------------------------------------------------
  grasp_target_sub_ = node->create_subscription<tops_msgs::msg::GraspTarget>(
    "~/grasp_target", rclcpp::SystemDefaultsQoS(),
    std::bind(&CartesianImpedanceController::graspTargetCallback, this,
      std::placeholders::_1));

  pose_target_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "~/target_pose", rclcpp::SystemDefaultsQoS(),
    std::bind(&CartesianImpedanceController::poseTargetCallback, this,
      std::placeholders::_1));

  diagnostics_pub_ = node->create_publisher<tops_msgs::msg::ControllerDiagnostics>(
    "~/diagnostics", rclcpp::SystemDefaultsQoS());

  wrench_pub_ = node->create_publisher<geometry_msgs::msg::WrenchStamped>(
    "~/external_wrench", rclcpp::SystemDefaultsQoS());

  mode_service_ = node->create_service<tops_msgs::srv::SetControlMode>(
    "~/set_control_mode",
    std::bind(&CartesianImpedanceController::setControlMode, this,
      std::placeholders::_1, std::placeholders::_2));

  retreat_service_ = node->create_service<tops_msgs::srv::TriggerRetreat>(
    "~/trigger_retreat",
    std::bind(&CartesianImpedanceController::triggerRetreat, this,
      std::placeholders::_1, std::placeholders::_2));

  // Diagnostics leave the real-time loop through a snapshot box and are
  // turned into messages here, on the executor thread. update() therefore
  // never touches the middleware.
  const double period = 1.0 / std::max(1.0, publish_rate_hz_);
  publish_timer_ = node->create_wall_timer(
    std::chrono::duration<double>(period),
    std::bind(&CartesianImpedanceController::publishSnapshot, this));

  return controller_interface::CallbackReturn::SUCCESS;
}

// ===========================================================================
//  Lifecycle: on_activate / on_deactivate
// ===========================================================================

controller_interface::CallbackReturn CartesianImpedanceController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = get_node()->get_logger();

  // Resolve every interface to an index once. update() then does array
  // indexing, not string comparison against a vector of loaned interfaces.
  position_state_indices_.clear();
  velocity_state_indices_.clear();
  effort_command_indices_.clear();
  ft_state_indices_.clear();

  const auto find_state = [&](const std::string & name, std::size_t & out) -> bool {
      for (std::size_t i = 0; i < state_interfaces_.size(); ++i) {
        if (state_interfaces_[i].get_name() == name) {
          out = i;
          return true;
        }
      }
      RCLCPP_ERROR(logger, "state interface '%s' was not claimed", name.c_str());
      return false;
    };

  const auto find_command = [&](const std::string & name, std::size_t & out) -> bool {
      for (std::size_t i = 0; i < command_interfaces_.size(); ++i) {
        if (command_interfaces_[i].get_name() == name) {
          out = i;
          return true;
        }
      }
      RCLCPP_ERROR(logger, "command interface '%s' was not claimed", name.c_str());
      return false;
    };

  std::size_t idx = 0;
  for (const auto & joint : joint_names_) {
    if (!find_state(joint + "/" + hardware_interface::HW_IF_POSITION, idx)) {
      return controller_interface::CallbackReturn::ERROR;
    }
    position_state_indices_.push_back(idx);

    if (!find_state(joint + "/" + hardware_interface::HW_IF_VELOCITY, idx)) {
      return controller_interface::CallbackReturn::ERROR;
    }
    velocity_state_indices_.push_back(idx);

    if (!find_command(joint + "/" + hardware_interface::HW_IF_EFFORT, idx)) {
      return controller_interface::CallbackReturn::ERROR;
    }
    effort_command_indices_.push_back(idx);
  }

  for (const auto & axis : {"force.x", "force.y", "force.z",
                            "torque.x", "torque.y", "torque.z"})
  {
    if (!find_state(ft_sensor_name_ + "/" + axis, idx)) {
      RCLCPP_ERROR(
        logger,
        "The wrist F/T sensor is not available. Without it there is no "
        "contact force measurement, so force regulation and the wrench "
        "interlock cannot run. Check the <sensor> block in the URDF's "
        "ros2_control section and the Gazebo force_torque sensor on "
        "'%s'.", ft_sensor_name_.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    ft_state_indices_.push_back(idx);
  }

  // --- seed everything from the arm's actual state -----------------------
  // The equilibrium pose starts AT the current pose, so activation applies
  // zero spring force. Starting it anywhere else means the arm lunges the
  // instant the controller activates, which is both alarming and a genuine
  // hazard when someone is standing next to it.
  if (!readJointState()) {
    RCLCPP_ERROR(logger, "could not read initial joint state");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!model_.compute(q_, qdot_, rb_)) {
    RCLCPP_ERROR(
      logger,
      "the dynamics model failed on the arm's current configuration. The "
      "usual cause is a non-physical inertial block in the URDF: M(q) is "
      "not positive definite and the Cholesky factorisation failed.");
    return controller_interface::CallbackReturn::ERROR;
  }

  pose_desired_ = rb_.tip_pose;
  twist_desired_.setZero();
  tau_previous_.setZero();
  force_integral_n_ = 0.0;
  safety_latched_ = false;
  safety_reason_.clear();
  requested_mode_ = static_cast<int>(tops_msgs::srv::SetControlMode::Request::MODE_STIFF);

  // Gains jump straight to the stiff set here, which is the one moment when
  // that is legitimate: there is no stored spring energy to disturb because
  // the error is exactly zero.
  stiffness_ramp_.reset(stiffness_stiff_);
  stiffness_ramp_.setTarget(stiffness_stiff_);

  tank_.reset();
  stiffness_prev_ = stiffness_stiff_;
  have_prev_twist_ = false;
  tip_twist_prev_.setZero();
  // 25 Hz: fast enough to track a real acceleration transient, slow enough
  // that differentiated velocity noise does not reach the interlock.
  tcp_accel_filter_.setCutoff(25.0, nominal_period_);
  tcp_accel_filter_.reset();

  // Re-bias the F/T sensor on every activation. Sensor offsets drift, and
  // the arm's pose at activation is not the pose it was at last time. The
  // arm must be at rest and out of contact while this runs -- roughly half
  // a second at the default 500 samples.
  wrench_filter_control_.reset();
  wrench_filter_safety_.reset();
  wrench_bias_.setZero();
  bias_accumulator_.setZero();
  bias_samples_remaining_ = bias_samples_total_;

  CommandSnapshot seed;
  seed.position[0] = pose_desired_.translation().x();
  seed.position[1] = pose_desired_.translation().y();
  seed.position[2] = pose_desired_.translation().z();
  quatToArray(Eigen::Quaterniond(pose_desired_.rotation()), seed.orientation);
  seed.valid = true;
  seed.stamp_seconds = get_node()->now().seconds();
  command_box_.write(seed);
  command_ = seed;
  last_command_generation_ = command_box_.generation();
  seconds_since_command_ = 0.0;

  update_period_max_ = 0.0;
  missed_deadlines_ = 0;

  RCLCPP_INFO(
    logger,
    "activated. Equilibrium seeded at the current TCP pose [%.3f %.3f %.3f]. "
    "Hold the arm still and clear of contact for %.2f s while the F/T bias "
    "is captured.",
    pose_desired_.translation().x(), pose_desired_.translation().y(),
    pose_desired_.translation().z(),
    bias_samples_remaining_ * nominal_period_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianImpedanceController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Write zero torque on the way out. Leaving the last commanded torque
  // latched in the hardware would leave the arm pushing against whatever it
  // was pushing against, with nothing left running to stop it.
  for (std::size_t i = 0; i < effort_command_indices_.size(); ++i) {
    (void)command_interfaces_[effort_command_indices_[i]].set_value(0.0);
  }
  RCLCPP_INFO(get_node()->get_logger(), "deactivated; commanded torques zeroed");
  return controller_interface::CallbackReturn::SUCCESS;
}

// ===========================================================================
//  The real-time path
// ===========================================================================

bool CartesianImpedanceController::readJointState()
{
  for (int i = 0; i < kNumJoints; ++i) {
    const auto p = state_interfaces_[position_state_indices_[i]].get_optional();
    const auto v = state_interfaces_[velocity_state_indices_[i]].get_optional();
    if (!p.has_value() || !v.has_value()) {return false;}
    q_(i) = p.value();
    qdot_(i) = v.value();
    // A NaN arriving from the hardware must not be allowed to propagate: it
    // would poison M(q), then tau, and the arm would be commanded NaN
    // torque, which most drivers interpret as "whatever was in the register".
    if (!std::isfinite(q_(i)) || !std::isfinite(qdot_(i))) {return false;}
  }
  return true;
}

void CartesianImpedanceController::readAndConditionWrench(
  double dt, const Eigen::Isometry3d & tip_pose, const CartesianVector & tip_twist)
{
  CartesianVector raw = CartesianVector::Zero();
  for (int i = 0; i < 6; ++i) {
    const auto v = state_interfaces_[ft_state_indices_[i]].get_optional();
    raw(i) = (v.has_value() && std::isfinite(v.value())) ? v.value() : 0.0;
  }

  // Re-express the raw reading along the TCP axes. Only the axes change: the
  // torque is still taken about the sensor origin, which is the reference
  // point tool_com is measured from.
  raw.head<3>() = R_tip_sensor_ * raw.head<3>();
  raw.tail<3>() = R_tip_sensor_ * raw.tail<3>();

  // --- subtract the tool's own weight ------------------------------------
  // The sensor sits between the flange and the hand, so it carries the hand
  // and fingers. That load rotates with the wrist, which means a constant
  // offset captured once is NOT enough: tilting the wrist 90 degrees would
  // otherwise read as an 8 N contact. Predict the gravitational wrench in
  // the sensor frame and remove it analytically.
  const Eigen::Matrix3d R_base_tip = tip_pose.rotation();

  // Remove the tool's INERTIA as well as its weight.
  //
  // The sensor sits between the flange and the hand, so it carries whatever
  // is distal to it -- and that load is m(a - g), not just mg. Compensating
  // only gravity leaves the m·a term in the signal, and at 1.55 kg a brisk
  // free-space move reads as tens of newtons. It tripped the 15 N interlock
  // at 27 N with the gripper still 374 mm above the table, which looks
  // exactly like a crash and is nothing of the sort.
  //
  // The TCP's linear acceleration is obtained by differentiating the twist
  // the kinematics already produce. It must be filtered: differentiating a
  // velocity at 1 kHz is noisy, and an unfiltered estimate would inject that
  // noise straight into the safety path.
  Eigen::Vector3d tcp_accel = Eigen::Vector3d::Zero();
  if (have_prev_twist_ && dt > 1e-6) {
    tcp_accel = (tip_twist.head<3>() - tip_twist_prev_.head<3>()) / dt;
  }
  tip_twist_prev_ = tip_twist;
  have_prev_twist_ = true;
  const Eigen::Vector3d accel = tcp_accel_filter_.update(tcp_accel);

  // Sign: (accel - gravity), NOT (gravity - accel).
  //
  // The sensor reports the force it transmits, which for a mass m hanging
  // below it is m(a - g) resolved into the sensor frame. Writing m(g - a)
  // gives a prediction of the right magnitude and the WRONG sign, so
  // `raw - predicted` adds the tool's weight instead of removing it.
  //
  // This is worth labouring because the bug hides. The bias capture at
  // activation simply absorbs the doubled offset, so the arm reads a
  // healthy 0.2 N at rest and everything looks correct. The error only
  // appears once the wrist ROTATES, at which point the mis-signed term
  // swings by twice the tool weight — about 30 N — and trips the 15 N
  // interlock in clear air. Measured at the sensor: raw_z = -15.16,
  // predicted_z = +15.20, captured bias = -30.44.
  const Eigen::Vector3d f_tool_tip =
    R_base_tip.transpose() * (tool_mass_ * (accel - gravity_));
  const Eigen::Vector3d t_tool_tip = tool_com_.cross(f_tool_tip);

  CartesianVector predicted;
  predicted.head<3>() = f_tool_tip;
  predicted.tail<3>() = t_tool_tip;

  CartesianVector compensated = raw - predicted;

  // --- one-time bias capture ---------------------------------------------
  // Whatever is left after gravity compensation while the arm is at rest
  // and untouched is sensor offset. Averaging it out is the difference
  // between a usable 0.3 N deadband and a useless 2 N one.
  if (bias_samples_remaining_ > 0) {
    bias_accumulator_ += compensated;
    --bias_samples_remaining_;
    if (bias_samples_remaining_ == 0) {
      wrench_bias_ =
        bias_accumulator_ / std::max(1.0, static_cast<double>(bias_samples_total_));
    }
    // While biasing, report zero rather than a number that is known wrong.
    wrench_external_.setZero();
    wrench_safety_.setZero();
    return;
  }

  compensated -= wrench_bias_;

  // Express in the base frame, which is where the control law lives.
  CartesianVector in_base;
  in_base.head<3>() = R_base_tip * compensated.head<3>();
  in_base.tail<3>() = R_base_tip * compensated.tail<3>();

  (void)dt;  // cutoffs were set from the nominal period in on_configure
  wrench_external_ = wrench_filter_control_.update(in_base);
  wrench_safety_ = wrench_filter_safety_.update(in_base);
}

CartesianVector CartesianImpedanceController::poseError(
  const Eigen::Isometry3d & tip_pose) const
{
  CartesianVector e;
  e.head<3>() = pose_desired_.translation() - tip_pose.translation();

  // Orientation error as an axis-angle vector.
  //
  // Rotation matrices do not subtract and quaternions do not either. The
  // error is the rotation that takes the current orientation to the desired
  // one, expressed as axis * angle so it lives in the same space as the
  // angular part of the Jacobian.
  Eigen::Quaterniond q_current(tip_pose.rotation());
  Eigen::Quaterniond q_desired(pose_desired_.rotation());

  // q and -q are the same rotation but differ by 2*pi in the axis-angle
  // extraction below. Without this sign fix the arm occasionally decides to
  // take the long way round -- a full rotation to reach an orientation it
  // was already at.
  if (q_desired.coeffs().dot(q_current.coeffs()) < 0.0) {
    q_current.coeffs() = -q_current.coeffs();
  }

  const Eigen::Quaterniond q_error = q_current.inverse() * q_desired;
  Eigen::AngleAxisd aa(q_error);
  // Expressed in the tip frame by the multiplication above; rotate into the
  // base frame to match the Jacobian's convention.
  e.tail<3>() = tip_pose.rotation() * (aa.axis() * aa.angle());

  return e;
}

CartesianVector CartesianImpedanceController::forceRegulation(
  double dt, const Eigen::Isometry3d & tip_pose)
{
  CartesianVector f_reg = CartesianVector::Zero();
  force_loop_active_ = false;

  // Only while APPROACHING. MODE_GRASPED keeps the compliant stiffness but
  // switches this loop off, because once the object is held there is
  // nothing left to press into and the press fights the lift.
  if (!force_control_enabled_ ||
    requested_mode_ != tops_msgs::srv::SetControlMode::Request::MODE_COMPLIANT)
  {
    force_integral_n_ = 0.0;
    return f_reg;
  }

  // Only regulate force once there is actually contact.
  //
  // In free space the measured force is zero, so the error is the full
  // setpoint and the loop commands a steady push at nothing. That both
  // accelerates the arm into whatever it eventually meets and bills the
  // energy tank continuously for work done on no environment at all, which
  // emptied the budget during every approach. Regulating force before
  // contact is in any case meaningless: there is no force to regulate.
  const double contact_magnitude = wrench_external_.head<3>().norm();
  if (contact_magnitude < force_contact_threshold_n_) {
    force_integral_n_ = 0.0;
    return f_reg;
  }
  force_loop_active_ = true;

  // The approach axis, expressed in the base frame. Regulating force along
  // a fixed world axis would be wrong the moment the wrist tilts.
  const Eigen::Vector3d axis_base = tip_pose.rotation() * force_axis_tip_;

  // Signed contact force along the approach axis. Positive means the
  // environment is pushing back, i.e. we are pressing into something.
  const double f_measured = -wrench_external_.head<3>().dot(axis_base);
  const double f_error = force_desired_n_ - f_measured;

  // Conditional integration, which is the anti-windup mechanism.
  //
  // The failure it prevents: press the gripper against a rigid table, ask
  // for more force than the torque limit can deliver, and a plain integrator
  // winds up for as long as contact lasts. Lift off and the arm leaps,
  // because the integrator is still holding the accumulated demand. The
  // clamp below bounds the integrator; the conditional test stops it
  // accumulating in the direction that is already saturated.
  const double candidate = force_integral_n_ + force_ki_ * f_error * dt;
  const bool saturated_high = candidate >= force_integral_clamp_n_ && f_error > 0.0;
  const bool saturated_low = candidate <= -force_integral_clamp_n_ && f_error < 0.0;
  if (!saturated_high && !saturated_low) {
    force_integral_n_ = std::clamp(
      candidate, -force_integral_clamp_n_, force_integral_clamp_n_);
  }

  const double f_command = force_kp_ * f_error + force_integral_n_;

  // Sign: ALONG the approach axis, not against it.
  //
  // axis_base is the TCP's approach direction expressed in the base frame,
  // which for a top-down grasp points DOWNWARD. Pressing harder means
  // adding force along it. The negated form pushed the arm UP by up to
  // 5.5 N whenever it was not yet touching anything — so during the final
  // approach the force loop fought the spring, won, and drove the gripper
  // back up away from the object while the error grew.
  f_reg.head<3>() = axis_base * f_command;
  return f_reg;
}

JointVector CartesianImpedanceController::nullSpaceTorque(const RigidBodyState & rb) const
{
  // Posture control in the null space.
  //
  // Seven joints controlling six Cartesian degrees of freedom leaves one
  // degree of internal freedom: the elbow can swing through a whole arc
  // without the hand moving at all. Left alone it drifts, and it reliably
  // drifts into a joint limit or a singularity halfway through an approach,
  // at which point the task-space controller quietly stops being able to do
  // its job. This term spends that freedom on staying near a chosen posture.
  //
  // The projector is the dynamically consistent one (Khatib): torque applied
  // through N produces NO end-effector acceleration, so posture control is
  // genuinely invisible to the task. The kinematic projector I - J^+ J does
  // not have that property and lets the posture term fight the task.
  const CartesianMatrix Lambda =
    KinematicsModel::taskSpaceInertia(rb.jacobian, rb.mass_inverse, singularity_damping_);
  const Eigen::Matrix<double, kNumJoints, 6> J_bar =
    rb.mass_inverse * rb.jacobian.transpose() * Lambda;
  const JointMatrix N =
    JointMatrix::Identity() - rb.jacobian.transpose() * J_bar.transpose();

  // Critical damping against the joint-space inertia's diagonal. Using the
  // full M here would be more correct and less robust; the diagonal is the
  // standard compromise and is what makes this term stable across the
  // workspace without re-tuning.
  JointVector tau_null;
  for (int i = 0; i < kNumJoints; ++i) {
    const double m_eff = std::max(0.05, rb.mass(i, i));
    const double d = 2.0 * nullspace_damping_ratio_ *
      std::sqrt(std::max(0.0, nullspace_stiffness_ * m_eff));
    tau_null(i) = -nullspace_stiffness_ * (q_(i) - q_nullspace_(i)) - d * qdot_(i);
  }

  // Joint-limit avoidance rides in the same null space: a one-sided spring
  // that is exactly zero until the joint enters the margin, so it costs
  // nothing anywhere else in the workspace.
  for (int i = 0; i < kNumJoints; ++i) {
    const double over_upper = q_(i) - (q_upper_(i) - joint_limit_margin_);
    const double under_lower = (q_lower_(i) + joint_limit_margin_) - q_(i);
    if (over_upper > 0.0) {
      tau_null(i) -= joint_limit_stiffness_ * over_upper;
    } else if (under_lower > 0.0) {
      tau_null(i) += joint_limit_stiffness_ * under_lower;
    }
  }

  return N * tau_null;
}

void CartesianImpedanceController::applyTorqueLimits(JointVector & tau, double dt)
{
  // Rate limit first, then magnitude.
  //
  // Order matters. Rate-limiting after clamping lets a clamped value slew
  // toward the clamp; clamping after rate-limiting guarantees the final
  // value is inside the limit no matter what the rate limiter produced.
  // The magnitude bound is the one that must hold unconditionally, so it
  // goes last.
  const double max_step = torque_rate_limit_ * dt;
  for (int i = 0; i < kNumJoints; ++i) {
    if (!std::isfinite(tau(i))) {tau(i) = 0.0;}
    const double delta = std::clamp(tau(i) - tau_previous_(i), -max_step, max_step);
    tau(i) = tau_previous_(i) + delta;
    tau(i) = std::clamp(tau(i), -torque_limits_(i), torque_limits_(i));
  }
  tau_previous_ = tau;
}

// ---------------------------------------------------------------------------
//  update() -- runs at the controller_manager's update_rate, 1 kHz
// ---------------------------------------------------------------------------
//
// Reading order matters and is: state, then command, then law, then limits,
// then write. Every branch below ends in a torque being written, including
// every failure branch. A controller that returns early without writing
// leaves the previous torque latched in the hardware, which is the exact
// opposite of what you want when something has gone wrong.

controller_interface::return_type CartesianImpedanceController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const double dt = period.seconds();

  // A zero or absurd dt happens on the first cycle and after a sim pause.
  // Dividing by it would produce infinities in the damping and tank terms.
  const double dt_safe = (dt > 1e-6 && dt < 0.1) ? dt : nominal_period_;
  if (dt > 1.5 * nominal_period_) {++missed_deadlines_;}
  update_period_max_ = std::max(update_period_max_, dt);

  // ---- 1. state ---------------------------------------------------------
  if (!readJointState() || !model_ok_) {
    for (int i = 0; i < kNumJoints; ++i) {
      (void)command_interfaces_[effort_command_indices_[i]].set_value(0.0);
    }
    tau_previous_.setZero();
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "joint state unreadable or non-finite; commanding zero torque");
    return controller_interface::return_type::OK;
  }

  if (!model_.compute(q_, qdot_, rb_)) {
    // Gravity compensation alone is the right degraded mode: the arm holds
    // itself up and does nothing else. It is the only behaviour that is
    // safe without a trustworthy model.
    JointVector tau = rb_.gravity;
    applyTorqueLimits(tau, dt_safe);
    for (int i = 0; i < kNumJoints; ++i) {
      (void)command_interfaces_[effort_command_indices_[i]].set_value(tau(i));
    }
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "dynamics solver failed; holding on gravity compensation only");
    return controller_interface::return_type::OK;
  }

  readAndConditionWrench(dt_safe, rb_.tip_pose, rb_.tip_twist);

  // ---- 2. command and watchdog -----------------------------------------
  const std::uint32_t generation = command_box_.generation();
  if (generation != last_command_generation_) {
    CommandSnapshot fresh;
    if (command_box_.read(fresh)) {
      // Reject a target that jumps further than a plausible servo step.
      // A vision pipeline that briefly grounds the wrong object produces
      // exactly this, and without the guard the arm would set off across
      // the table at full stiffness.
      const Eigen::Vector3d p_new(fresh.position[0], fresh.position[1], fresh.position[2]);
      const double jump = (p_new - pose_desired_.translation()).norm();
      if (fresh.valid && jump <= max_command_step_m_) {
        command_ = fresh;
        last_command_generation_ = generation;
        seconds_since_command_ = 0.0;
      } else {
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "rejected a target %.3f m from the current equilibrium "
          "(limit %.3f m); holding position",
          jump, max_command_step_m_);
      }
    }
  } else {
    seconds_since_command_ += dt_safe;
  }

  const bool command_stale = seconds_since_command_ > command_timeout_s_;

  if (command_.valid) {
    pose_desired_.translation() =
      Eigen::Vector3d(command_.position[0], command_.position[1], command_.position[2]);
    const Eigen::Quaterniond q_cmd(
      command_.orientation[3], command_.orientation[0],
      command_.orientation[1], command_.orientation[2]);
    pose_desired_.linear() = q_cmd.normalized().toRotationMatrix();

    // The watchdog. A stale command means the commanding node died, the
    // network stalled, or the perception loop lost the object. In every one
    // of those cases the correct action is the same: stop moving, hold the
    // last equilibrium, and keep the spring active so the arm is still
    // compliant if someone is touching it. Feed-forward velocity is what
    // gets zeroed, not the whole controller.
    if (command_stale) {
      twist_desired_.setZero();
    } else {
      twist_desired_.head<3>() = Eigen::Vector3d(
        command_.linear_velocity[0], command_.linear_velocity[1],
        command_.linear_velocity[2]);
      twist_desired_.tail<3>() = Eigen::Vector3d(
        command_.angular_velocity[0], command_.angular_velocity[1],
        command_.angular_velocity[2]);
    }
  }

  // ---- 3. safety interlock ---------------------------------------------
  // Read off the LIGHTLY filtered wrench, on purpose. The control path can
  // afford 30 Hz of smoothing; the interlock cannot, because every
  // millisecond of filter lag is another millimetre travelled into whatever
  // is being crushed.
  const double contact_force = wrench_safety_.head<3>().norm();
  if (contact_force > hard_force_limit_n_ && !safety_latched_) {
    safety_latched_ = true;
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "HARD FORCE LIMIT: %.2f N exceeds %.2f N. Latching to compliant hold; "
      "call ~/trigger_retreat or deactivate to clear.",
      contact_force, hard_force_limit_n_);
  }

  // ---- 4. gains ---------------------------------------------------------
  CartesianVector target_stiffness;
  switch (requested_mode_) {
    case tops_msgs::srv::SetControlMode::Request::MODE_COMPLIANT:
    case tops_msgs::srv::SetControlMode::Request::MODE_GRASPED:
      target_stiffness = stiffness_compliant_;
      break;
    case tops_msgs::srv::SetControlMode::Request::MODE_FLOAT:
      target_stiffness = CartesianVector::Zero();
      break;
    case tops_msgs::srv::SetControlMode::Request::MODE_HOLD:
    case tops_msgs::srv::SetControlMode::Request::MODE_STIFF:
    default:
      target_stiffness = stiffness_stiff_;
      break;
  }

  // A latched force fault overrides whatever mode was requested: go limp
  // except for damping, so the arm stops pressing but still resists being
  // flung around.
  if (safety_latched_) {
    target_stiffness.setZero();
  }

  stiffness_ramp_.setTarget(target_stiffness);
  const CartesianVector K = stiffness_ramp_.update(dt_safe);

  // ---- 5. the control law ----------------------------------------------
  const CartesianVector e = poseError(rb_.tip_pose);
  const CartesianVector e_dot = twist_desired_ - rb_.tip_twist;

  // Damping derived from the stiffness actually in effect, every cycle,
  // rather than configured independently. This is the single most useful
  // thing in the whole controller: it makes it impossible to raise
  // stiffness and forget to raise damping, which is the classic way to turn
  // a well-behaved impedance controller into an oscillator.
  //
  //   d = 2 * zeta * sqrt(k * m_task)
  //
  // with m_task the task-space inertia along each axis, so the damping
  // ratio holds as the arm's effective inertia changes across the workspace.
  const CartesianMatrix Lambda =
    KinematicsModel::taskSpaceInertia(rb_.jacobian, rb_.mass_inverse, singularity_damping_);
  for (int i = 0; i < 6; ++i) {
    const double m_task = std::max(0.05, std::abs(Lambda(i, i)));
    damping_(i) = 2.0 * damping_ratio_(i) * std::sqrt(std::max(0.0, K(i) * m_task));
  }

  const CartesianVector f_force = safety_latched_
    ? CartesianVector::Zero()
    : forceRegulation(dt_safe, rb_.tip_pose);

  // Along a force-regulated axis, stiffness is scaled down. You cannot
  // control position and force along the same direction at the same time --
  // they are the same degree of freedom, and asking for both gives you a
  // fight between two loops rather than either result.
  // Give up position stiffness on the approach axis ONLY while the force
  // loop is actually regulating.
  //
  // The rule is that position and force cannot both be controlled along one
  // direction. The converse matters just as much: if the force loop is not
  // running, that direction must keep its position stiffness. Scaling it to
  // 20 % unconditionally left the vertical axis with neither — 30 N/m of
  // spring and no force loop — so the arm sagged to a standstill 90 mm above
  // the object, with a 97 mm error it had no authority to close.
  CartesianVector K_effective = K;
  if (force_loop_active_ && !safety_latched_ &&
    requested_mode_ == tops_msgs::srv::SetControlMode::Request::MODE_COMPLIANT)
  {
    const Eigen::Vector3d axis_base = rb_.tip_pose.rotation() * force_axis_tip_;
    for (int i = 0; i < 3; ++i) {
      const double along = std::abs(axis_base(i));
      K_effective(i) = K(i) * (1.0 - along * (1.0 - force_axis_stiffness_scale_));
    }
  }

  // Split the wrench into the part that can inject energy and the part that
  // can only remove it. The tank pays for the first; the second is free.

  const CartesianVector f_damping = damping_.asDiagonal() * e_dot;

  // ---- 6. passivity ------------------------------------------------------
  const CartesianVector v = rb_.tip_twist;
  const double power_dissipated = v.dot(damping_.asDiagonal() * v);

  // Rate of change of the stiffness, which is the one quantity the energy
  // tank still has to bill for (see below).
  CartesianVector stiffness_rate = CartesianVector::Zero();
  if (dt_safe > 0.0) {stiffness_rate = (K - stiffness_prev_) / dt_safe;}
  stiffness_prev_ = K;

  // Sign convention, because it is easy to get backwards and the symptom of
  // getting it backwards is a tank that fills during exactly the transients
  // it exists to catch:
  //
  //   P > 0  the controller is doing work ON the arm -- pay for it
  //   P < 0  the environment (or a person) is doing work on the spring,
  //          which refills the tank for free
  //
  // Bill the tank for the NON-PASSIVE power only.
  //
  // This is the correction that matters most in the whole file. The obvious
  // implementation charges the tank the full v.(K e + F_force), and it is
  // wrong: a constant-stiffness spring with a fixed setpoint is PASSIVE.
  // Energy pushed into it is stored as 0.5 eᵀKe and comes back out. Charging
  // for it means an ordinary 0.4 m traverse of free space bills about 10 J
  // against a 0.5 J budget, the tank bottoms out within a second, gamma
  // collapses and the arm is throttled to a crawl while doing nothing
  // whatsoever that threatens passivity. That is exactly what happened.
  //
  // Differentiating the stored energy V = 0.5 eᵀKe with e = x_d - x gives
  //
  //     F.ẋ  =  eᵀK ẋ_d  +  0.5 eᵀK̇ e  -  dV/dt
  //
  // so the power delivered to the arm has three sources: the work done by
  // MOVING the equilibrium, the energy created by CHANGING the stiffness,
  // and the spring giving back what it had stored. Only the first two are
  // external; the third is the passive exchange. Add the force-regulation
  // term, which is not derived from any potential and so is never passive.
  // Moving the equilibrium is deliberately NOT billed.
  //
  // The source term eᵀK ẋ_d is real, but almost all of it goes straight into
  // the spring's own potential 0.5 eᵀKe, where it is stored and recoverable
  // rather than spent. Charging it against a depleting budget makes the tank
  // a servo-speed limiter: at a 0.2 m tracking error and a 0.05 m/s approach
  // it bills 1.5 W continuously, empties any sane budget in a few seconds,
  // and the damping boost then stalls the arm 200 mm short of the object —
  // which is precisely what it did.
  //
  // That energy is already bounded by two other mechanisms that do not need
  // a budget to work: the supervisor slews the equilibrium at a fixed speed,
  // and the per-joint torque clamp is unconditional. What is left for the
  // tank is what nothing else bounds — energy conjured by CHANGING the
  // stiffness, and the force-regulation term, which follows from no
  // potential at all and so is never passive. Those are exactly the two
  // things this monitor was specified to catch.
  const double power_stiffness = 0.5 * e.dot(stiffness_rate.asDiagonal() * e);
  const double power_force = v.dot(f_force);
  const double power_injected = power_stiffness + power_force;

  const double gamma = tank_.update(power_dissipated, power_injected, dt_safe);

  // gamma throttles the force-regulation term, NOT the spring. Scaling the
  // spring would make the arm go limp exactly when it is holding something,
  // and would break the passive storage the budget above assumes.
  const CartesianVector f_active_throttled =
    K_effective.asDiagonal() * e + f_force * gamma;
  const CartesianVector F =
    f_active_throttled + f_damping * tank_.dampingBoost();

  // ---- 7. torques --------------------------------------------------------
  const JointVector tau_task = rb_.jacobian.transpose() * F;
  const JointVector tau_null = nullSpaceTorque(rb_);

  JointVector tau = tau_task + tau_null + rb_.gravity;
  if (compensate_coriolis_) {
    // Defensible to omit at grasp-approach speeds, since C grows with the
    // square of velocity. Included because it costs one already-computed
    // vector add and stops being negligible during the retreat, which is
    // the fastest motion in the whole trial.
    tau += rb_.coriolis;
  }

  applyTorqueLimits(tau, dt_safe);

  // ---- 8. write ----------------------------------------------------------
  for (int i = 0; i < kNumJoints; ++i) {
    (void)command_interfaces_[effort_command_indices_[i]].set_value(tau(i));
  }

  // ---- 9. diagnostics (no middleware on this path) ----------------------
  DiagnosticsSnapshot snap;
  poseToArray(rb_.tip_pose, snap.pose_current);
  poseToArray(pose_desired_, snap.pose_desired);
  copyToArray(e.head<3>(), snap.position_error, 3);
  copyToArray(e.tail<3>(), snap.orientation_error, 3);
  copyToArray(F, snap.wrench_commanded, 6);
  copyToArray(wrench_external_, snap.wrench_external, 6);
  copyToArray(K_effective, snap.stiffness, 6);
  copyToArray(damping_, snap.damping, 6);
  copyToArray(tau_task, snap.tau_task, kNumJoints);
  copyToArray(tau_null, snap.tau_nullspace, kNumJoints);
  copyToArray(rb_.gravity, snap.tau_gravity, kNumJoints);
  copyToArray(tau, snap.tau_commanded, kNumJoints);
  snap.tank_energy = tank_.energy();
  snap.tank_state = tank_.state();
  snap.tank_valve = gamma;
  snap.power_dissipated = power_dissipated;
  snap.power_injected = power_injected;
  snap.passivity_throttling = tank_.throttling();
  snap.stiffness_scale = (stiffness_stiff_(0) > 1e-9) ? K(0) / stiffness_stiff_(0) : 0.0;
  snap.update_period = dt;
  snap.update_period_max = update_period_max_;
  snap.missed_deadlines = missed_deadlines_;
  snap.command_stale = command_stale;
  snap.stamp_seconds = time.seconds();
  diagnostics_box_.write(snap);

  return controller_interface::return_type::OK;
}

// ===========================================================================
//  Non-real-time: callbacks, services, publishing
// ===========================================================================

void CartesianImpedanceController::graspTargetCallback(
  const tops_msgs::msg::GraspTarget::SharedPtr msg)
{
  // Frame checking happens here, in the callback, not in update(). A target
  // silently interpreted in the wrong frame is the most expensive kind of
  // bug in this system: everything downstream looks healthy and the arm
  // goes confidently to the wrong place.
  if (!msg->header.frame_id.empty() && msg->header.frame_id != base_frame_) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "ignoring GraspTarget in frame '%s'; this controller only accepts "
      "targets already expressed in '%s'. Transform upstream.",
      msg->header.frame_id.c_str(), base_frame_.c_str());
    return;
  }

  CommandSnapshot cmd;
  cmd.position[0] = msg->pose.position.x;
  cmd.position[1] = msg->pose.position.y;
  cmd.position[2] = msg->pose.position.z;
  cmd.orientation[0] = msg->pose.orientation.x;
  cmd.orientation[1] = msg->pose.orientation.y;
  cmd.orientation[2] = msg->pose.orientation.z;
  cmd.orientation[3] = msg->pose.orientation.w;
  cmd.linear_velocity[0] = msg->twist.linear.x;
  cmd.linear_velocity[1] = msg->twist.linear.y;
  cmd.linear_velocity[2] = msg->twist.linear.z;
  cmd.angular_velocity[0] = msg->twist.angular.x;
  cmd.angular_velocity[1] = msg->twist.angular.y;
  cmd.angular_velocity[2] = msg->twist.angular.z;
  cmd.stamp_seconds = rclcpp::Time(msg->header.stamp).seconds();

  const double qn = std::sqrt(
    cmd.orientation[0] * cmd.orientation[0] + cmd.orientation[1] * cmd.orientation[1] +
    cmd.orientation[2] * cmd.orientation[2] + cmd.orientation[3] * cmd.orientation[3]);
  cmd.valid = std::isfinite(cmd.position[0]) && std::isfinite(cmd.position[1]) &&
    std::isfinite(cmd.position[2]) && qn > 1e-6;

  if (!cmd.valid) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "rejected a GraspTarget with a non-finite position or a zero quaternion");
    return;
  }

  command_box_.write(cmd);
}

void CartesianImpedanceController::poseTargetCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // Convenience path for `ros2 topic pub` during bring-up and tuning. It is
  // the same command with no feed-forward velocity and no confidence.
  auto target = std::make_shared<tops_msgs::msg::GraspTarget>();
  target->header = msg->header;
  target->pose = msg->pose;
  target->confidence = 1.0;
  graspTargetCallback(target);
}

void CartesianImpedanceController::setControlMode(
  const std::shared_ptr<tops_msgs::srv::SetControlMode::Request> request,
  std::shared_ptr<tops_msgs::srv::SetControlMode::Response> response)
{
  using Req = tops_msgs::srv::SetControlMode::Request;

  if (request->mode > Req::MODE_GRASPED) {
    response->accepted = false;
    response->message = "unknown mode";
    return;
  }

  if (safety_latched_) {
    response->accepted = false;
    response->message =
      "a hard force fault is latched; call ~/trigger_retreat to clear before "
      "requesting a mode";
    return;
  }

  // MODE_HOLD freezes the equilibrium where the TCP is right now, which is
  // different from "keep the last commanded target": if the arm is being
  // pushed off target, hold means hold HERE, not spring back.
  if (request->mode == Req::MODE_HOLD) {
    CommandSnapshot cmd;
    cmd.position[0] = rb_.tip_pose.translation().x();
    cmd.position[1] = rb_.tip_pose.translation().y();
    cmd.position[2] = rb_.tip_pose.translation().z();
    quatToArray(Eigen::Quaterniond(rb_.tip_pose.rotation()), cmd.orientation);
    cmd.valid = true;
    cmd.stamp_seconds = get_node()->now().seconds();
    command_box_.write(cmd);
  }

  requested_mode_ = static_cast<int>(request->mode);

  CartesianVector target =
    (request->mode == Req::MODE_COMPLIANT || request->mode == Req::MODE_GRASPED)
    ? stiffness_compliant_
    : (request->mode == Req::MODE_FLOAT ? CartesianVector::Zero() : stiffness_stiff_);
  stiffness_ramp_.setTarget(target);

  response->accepted = true;
  response->expected_ramp_time = stiffness_ramp_.timeToTarget();
  response->message = "ramping at " + std::to_string(stiffness_ramp_rate_) + " N/m/s";

  RCLCPP_INFO(
    get_node()->get_logger(), "mode %u requested; gains ramping over %.3f s",
    request->mode, response->expected_ramp_time);
}

void CartesianImpedanceController::triggerRetreat(
  const std::shared_ptr<tops_msgs::srv::TriggerRetreat::Request> request,
  std::shared_ptr<tops_msgs::srv::TriggerRetreat::Response> response)
{
  const double distance = (request->distance > 0.0) ? request->distance : 0.05;

  // Retreat along the NEGATIVE approach axis, in the base frame, from where
  // the TCP actually is. Retreating from the commanded equilibrium would be
  // wrong precisely when it matters: after a force fault the arm is
  // displaced from its equilibrium, and that displacement is the thing
  // pressing on the object.
  const Eigen::Vector3d axis_base = rb_.tip_pose.rotation() * force_axis_tip_;
  const Eigen::Vector3d retreat_to = rb_.tip_pose.translation() - axis_base * distance;

  CommandSnapshot cmd;
  cmd.position[0] = retreat_to.x();
  cmd.position[1] = retreat_to.y();
  cmd.position[2] = retreat_to.z();
  quatToArray(Eigen::Quaterniond(rb_.tip_pose.rotation()), cmd.orientation);
  cmd.valid = true;
  cmd.stamp_seconds = get_node()->now().seconds();
  command_box_.write(cmd);

  // Clearing the latch is what makes retreat possible at all -- a latched
  // fault holds zero stiffness, and a limp arm cannot back itself off.
  // Compliant gains, not stiff: the object is still there.
  safety_latched_ = false;
  safety_reason_.clear();
  force_integral_n_ = 0.0;
  requested_mode_ = static_cast<int>(tops_msgs::srv::SetControlMode::Request::MODE_COMPLIANT);
  stiffness_ramp_.setTarget(stiffness_compliant_);
  tank_.reset();

  response->accepted = true;
  response->message = "retreating " + std::to_string(distance) + " m along -approach";
  RCLCPP_WARN(
    get_node()->get_logger(),
    "retreat requested: backing off %.3f m; force fault cleared, compliant gains",
    distance);
}

void CartesianImpedanceController::publishSnapshot()
{
  DiagnosticsSnapshot snap;
  if (!diagnostics_box_.read(snap)) {return;}

  const auto stamp = get_node()->now();

  {
    tops_msgs::msg::ControllerDiagnostics msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = base_frame_;

    const auto fill_pose = [](const double (&a)[7], geometry_msgs::msg::Pose & p) {
        p.position.x = a[0];
        p.position.y = a[1];
        p.position.z = a[2];
        p.orientation.x = a[3];
        p.orientation.y = a[4];
        p.orientation.z = a[5];
        p.orientation.w = a[6];
      };
    fill_pose(snap.pose_current, msg.pose_current);
    fill_pose(snap.pose_desired, msg.pose_desired);

    for (int i = 0; i < 3; ++i) {
      msg.position_error[i] = snap.position_error[i];
      msg.orientation_error[i] = snap.orientation_error[i];
    }
    msg.position_error_norm = std::sqrt(
      snap.position_error[0] * snap.position_error[0] +
      snap.position_error[1] * snap.position_error[1] +
      snap.position_error[2] * snap.position_error[2]);

    const auto fill_wrench = [](const double (&a)[6], geometry_msgs::msg::Wrench & w) {
        w.force.x = a[0];
        w.force.y = a[1];
        w.force.z = a[2];
        w.torque.x = a[3];
        w.torque.y = a[4];
        w.torque.z = a[5];
      };
    fill_wrench(snap.wrench_commanded, msg.wrench_commanded);
    fill_wrench(snap.wrench_external, msg.wrench_external);
    msg.external_force_norm = std::sqrt(
      snap.wrench_external[0] * snap.wrench_external[0] +
      snap.wrench_external[1] * snap.wrench_external[1] +
      snap.wrench_external[2] * snap.wrench_external[2]);

    for (int i = 0; i < 6; ++i) {
      msg.stiffness[i] = snap.stiffness[i];
      msg.damping[i] = snap.damping[i];
    }
    msg.stiffness_scale = snap.stiffness_scale;

    msg.tank_energy = snap.tank_energy;
    msg.tank_state = snap.tank_state;
    msg.power_dissipated = snap.power_dissipated;
    msg.power_injected = snap.power_injected;
    msg.tank_valve = snap.tank_valve;
    msg.passivity_throttling = snap.passivity_throttling;

    msg.tau_task.assign(snap.tau_task, snap.tau_task + kNumJoints);
    msg.tau_nullspace.assign(snap.tau_nullspace, snap.tau_nullspace + kNumJoints);
    msg.tau_gravity.assign(snap.tau_gravity, snap.tau_gravity + kNumJoints);
    msg.tau_commanded.assign(snap.tau_commanded, snap.tau_commanded + kNumJoints);

    msg.update_period = snap.update_period;
    msg.update_period_max = snap.update_period_max;
    msg.missed_deadlines = snap.missed_deadlines;
    msg.command_stale = snap.command_stale;

    diagnostics_pub_->publish(msg);
  }

  {
    // Also published on its own, in the standard type, so rqt_plot and
    // rviz's wrench display work without anyone writing a custom plugin.
    geometry_msgs::msg::WrenchStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = base_frame_;
    msg.wrench.force.x = snap.wrench_external[0];
    msg.wrench.force.y = snap.wrench_external[1];
    msg.wrench.force.z = snap.wrench_external[2];
    msg.wrench.torque.x = snap.wrench_external[3];
    msg.wrench.torque.y = snap.wrench_external[4];
    msg.wrench.torque.z = snap.wrench_external[5];
    wrench_pub_->publish(msg);
  }
}

}  // namespace tops_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  tops_controllers::CartesianImpedanceController,
  controller_interface::ControllerInterface)
