// Copyright 2026 Kabir Dhingra. MIT licence.

#include "tops_controllers/cartesian_admittance_controller.hpp"

#include <algorithm>
#include <cmath>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"

namespace tops_controllers
{

controller_interface::CallbackReturn CartesianAdmittanceController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::string>("base_frame", "panda_link0");
    auto_declare<std::string>("tip_frame", "panda_hand_tcp");
    auto_declare<std::string>("ft_sensor_name", "panda_ft_sensor");
    auto_declare<std::vector<double>>("gravity", {0.0, 0.0, -9.80665});

    // The virtual dynamics. Choosing these is the whole design problem.
    //
    // virtual_mass is what the operator (or the object) feels when they
    // push the arm. Too low and the arm darts away from the lightest touch
    // and amplifies sensor noise into motion; too high and it feels like
    // shoving furniture. 6 kg is roughly "a heavy but movable object".
    //
    // virtual_damping sets how quickly the induced motion dies away once
    // the push stops. With zero virtual stiffness the reference never
    // returns on its own, so damping alone decides the settling.
    auto_declare<std::vector<double>>("virtual_mass", {6.0, 6.0, 6.0, 0.25, 0.25, 0.25});
    auto_declare<std::vector<double>>("virtual_damping", {60.0, 60.0, 60.0, 4.0, 4.0, 4.0});
    auto_declare<std::vector<double>>("virtual_stiffness", {60.0, 60.0, 60.0, 4.0, 4.0, 4.0});

    auto_declare<std::vector<double>>("inner_stiffness",
      {2500.0, 2500.0, 2500.0, 150.0, 150.0, 150.0});
    auto_declare<std::vector<double>>("inner_damping_ratio", {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});

    auto_declare<std::vector<double>>("torque.limits",
      {87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0});
    auto_declare<double>("torque.rate_limit", 1000.0);
    auto_declare<double>("singularity_damping", 0.05);
    auto_declare<double>("nullspace.stiffness", 20.0);
    auto_declare<double>("nullspace.damping_ratio", 1.0);
    auto_declare<std::vector<double>>("nullspace.q_desired",
      {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785});

    auto_declare<double>("wrench.cutoff_hz", 20.0);
    auto_declare<double>("wrench.deadband_n", 0.3);
    auto_declare<double>("wrench.clamp_n", 50.0);
    auto_declare<int>("wrench.bias_samples", 500);
    auto_declare<std::string>("wrench.sensor_frame", "panda_ft_sensor_link");
    auto_declare<double>("wrench.tool_mass", 1.07);
    auto_declare<std::vector<double>>("wrench.tool_com", {0.0, 0.0073, 0.1065});

    auto_declare<double>("limits.max_reference_offset_m", 0.10);
    auto_declare<double>("limits.max_reference_speed", 0.5);
    auto_declare<double>("safety.hard_force_limit_n", 15.0);
    auto_declare<double>("safety.command_timeout_s", 0.1);
    auto_declare<double>("publish_rate_hz", 100.0);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init failed: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
CartesianAdmittanceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & j : joint_names_) {
    config.names.push_back(j + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return config;
}

controller_interface::InterfaceConfiguration
CartesianAdmittanceController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & j : joint_names_) {
    config.names.push_back(j + "/" + hardware_interface::HW_IF_POSITION);
    config.names.push_back(j + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  for (const auto & axis : {"force.x", "force.y", "force.z",
                            "torque.x", "torque.y", "torque.z"})
  {
    config.names.push_back(ft_sensor_name_ + "/" + axis);
  }
  return config;
}

controller_interface::CallbackReturn CartesianAdmittanceController::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto node = get_node();
  auto logger = node->get_logger();

  joint_names_ = node->get_parameter("joints").as_string_array();
  if (static_cast<int>(joint_names_.size()) != kNumJoints) {
    RCLCPP_ERROR(logger, "'joints' must have exactly %d entries", kNumJoints);
    return controller_interface::CallbackReturn::ERROR;
  }
  base_frame_ = node->get_parameter("base_frame").as_string();
  tip_frame_ = node->get_parameter("tip_frame").as_string();
  ft_sensor_name_ = node->get_parameter("ft_sensor_name").as_string();

  const auto read6 = [&](const std::string & name, CartesianVector & out) -> bool {
      const auto v = node->get_parameter(name).as_double_array();
      if (v.size() != 6) {
        RCLCPP_ERROR(logger, "'%s' must have 6 entries", name.c_str());
        return false;
      }
      for (int i = 0; i < 6; ++i) {out(i) = v[i];}
      return true;
    };

  if (!read6("virtual_mass", virtual_mass_) ||
    !read6("virtual_damping", virtual_damping_) ||
    !read6("virtual_stiffness", virtual_stiffness_) ||
    !read6("inner_stiffness", inner_stiffness_) ||
    !read6("inner_damping_ratio", inner_damping_ratio_))
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  // A zero or negative virtual mass makes the admittance integration
  // divide by zero. Catching it here, once, is better than producing NaN
  // torques at 1 kHz and trying to work out where they came from.
  for (int i = 0; i < 6; ++i) {
    if (virtual_mass_(i) <= 1e-6) {
      RCLCPP_ERROR(
        logger,
        "virtual_mass[%d] = %.6f. The admittance model integrates "
        "F/m, so every entry must be strictly positive.", i, virtual_mass_(i));
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  {
    const auto v = node->get_parameter("torque.limits").as_double_array();
    const auto qn = node->get_parameter("nullspace.q_desired").as_double_array();
    const auto g = node->get_parameter("gravity").as_double_array();
    const auto c = node->get_parameter("wrench.tool_com").as_double_array();
    if (static_cast<int>(v.size()) != kNumJoints ||
      static_cast<int>(qn.size()) != kNumJoints || g.size() != 3 || c.size() != 3)
    {
      RCLCPP_ERROR(logger, "torque.limits / nullspace.q_desired / gravity / tool_com size mismatch");
      return controller_interface::CallbackReturn::ERROR;
    }
    for (int i = 0; i < kNumJoints; ++i) {
      torque_limits_(i) = v[i];
      q_nullspace_(i) = qn[i];
    }
    gravity_ = Eigen::Vector3d(g[0], g[1], g[2]);
    tool_com_ = Eigen::Vector3d(c[0], c[1], c[2]);
  }

  torque_rate_limit_ = node->get_parameter("torque.rate_limit").as_double();
  singularity_damping_ = node->get_parameter("singularity_damping").as_double();
  nullspace_stiffness_ = node->get_parameter("nullspace.stiffness").as_double();
  nullspace_damping_ratio_ = node->get_parameter("nullspace.damping_ratio").as_double();
  max_reference_offset_m_ = node->get_parameter("limits.max_reference_offset_m").as_double();
  max_reference_speed_ = node->get_parameter("limits.max_reference_speed").as_double();
  hard_force_limit_n_ = node->get_parameter("safety.hard_force_limit_n").as_double();
  command_timeout_s_ = node->get_parameter("safety.command_timeout_s").as_double();
  publish_rate_hz_ = node->get_parameter("publish_rate_hz").as_double();
  tool_mass_ = node->get_parameter("wrench.tool_mass").as_double();
  sensor_frame_ = node->get_parameter("wrench.sensor_frame").as_string();
  bias_samples_total_ =
    static_cast<int>(node->get_parameter("wrench.bias_samples").as_int());
  bias_samples_remaining_ = bias_samples_total_;

  std::string urdf = get_robot_description();
  if (urdf.empty() && node->has_parameter("robot_description")) {
    urdf = node->get_parameter("robot_description").as_string();
  }
  if (urdf.empty()) {
    RCLCPP_ERROR(logger, "robot_description is empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  std::string err;
  if (!model_.initialise(urdf, base_frame_, tip_frame_, gravity_, err)) {
    RCLCPP_ERROR(logger, "kinematic model failed: %s", err.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Sensor axes -> TCP axes, read from the URDF. Same reason as in the
  // impedance controller: the Panda's hand sits at -45 degrees on the
  // flange, so the two frames' x/y axes do not coincide.
  {
    Eigen::Isometry3d tip_T_sensor;
    if (!KinematicsModel::fixedTransform(urdf, tip_frame_, sensor_frame_, tip_T_sensor, err)) {
      RCLCPP_ERROR(
        logger, "cannot relate the F/T sensor frame '%s' to the tip '%s': %s",
        sensor_frame_.c_str(), tip_frame_.c_str(), err.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    R_tip_sensor_ = tip_T_sensor.rotation();
  }
  model_ok_ = true;

  nominal_period_ = 1.0 / std::max(1.0, static_cast<double>(get_update_rate()));

  // A single filter here, unlike the impedance controller's two. The
  // admittance loop's dynamics ARE a low-pass, so the control path is
  // already smoothed; the interlock runs off the same signal, and the
  // 20 Hz cutoff is chosen to keep its lag under about 8 ms.
  wrench_filter_.setCutoff(node->get_parameter("wrench.cutoff_hz").as_double(), nominal_period_);
  wrench_filter_.setDeadband(node->get_parameter("wrench.deadband_n").as_double());
  wrench_filter_.setSymmetricClamp(node->get_parameter("wrench.clamp_n").as_double());

  grasp_target_sub_ = node->create_subscription<tops_msgs::msg::GraspTarget>(
    "~/grasp_target", rclcpp::SystemDefaultsQoS(),
    std::bind(&CartesianAdmittanceController::graspTargetCallback, this,
      std::placeholders::_1));

  diagnostics_pub_ = node->create_publisher<tops_msgs::msg::ControllerDiagnostics>(
    "~/diagnostics", rclcpp::SystemDefaultsQoS());

  publish_timer_ = node->create_wall_timer(
    std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_)),
    std::bind(&CartesianAdmittanceController::publishSnapshot, this));

  RCLCPP_INFO(
    logger, "admittance controller configured: virtual mass %.1f kg, "
    "damping %.1f N.s/m, stiffness %.1f N/m; inner loop %.0f N/m",
    virtual_mass_(0), virtual_damping_(0), virtual_stiffness_(0), inner_stiffness_(0));

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianAdmittanceController::on_activate(
  const rclcpp_lifecycle::State &)
{
  auto logger = get_node()->get_logger();
  position_state_indices_.clear();
  velocity_state_indices_.clear();
  effort_command_indices_.clear();
  ft_state_indices_.clear();

  const auto find = [](const auto & interfaces, const std::string & name,
      std::size_t & out) -> bool {
      for (std::size_t i = 0; i < interfaces.size(); ++i) {
        if (interfaces[i].get_name() == name) {
          out = i;
          return true;
        }
      }
      return false;
    };

  std::size_t idx = 0;
  for (const auto & j : joint_names_) {
    if (!find(state_interfaces_, j + "/position", idx)) {
      RCLCPP_ERROR(logger, "missing state interface %s/position", j.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    position_state_indices_.push_back(idx);
    if (!find(state_interfaces_, j + "/velocity", idx)) {
      RCLCPP_ERROR(logger, "missing state interface %s/velocity", j.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    velocity_state_indices_.push_back(idx);
    if (!find(command_interfaces_, j + "/effort", idx)) {
      RCLCPP_ERROR(logger, "missing command interface %s/effort", j.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    effort_command_indices_.push_back(idx);
  }

  for (const auto & axis : {"force.x", "force.y", "force.z",
                            "torque.x", "torque.y", "torque.z"})
  {
    if (!find(state_interfaces_, ft_sensor_name_ + "/" + axis, idx)) {
      RCLCPP_ERROR(
        logger,
        "the F/T sensor is missing. Admittance control is measure-force-"
        "command-motion: without a force measurement there is no controller "
        "at all, so this is fatal rather than degraded.");
      return controller_interface::CallbackReturn::ERROR;
    }
    ft_state_indices_.push_back(idx);
  }

  if (!readJointState() || !model_.compute(q_, qdot_, rb_)) {
    RCLCPP_ERROR(logger, "could not read initial state or solve dynamics");
    return controller_interface::CallbackReturn::ERROR;
  }

  pose_desired_ = rb_.tip_pose;
  reference_offset_.setZero();
  reference_rate_.setZero();
  tau_previous_.setZero();
  safety_latched_ = false;
  wrench_filter_.reset();
  wrench_bias_.setZero();
  bias_accumulator_.setZero();
  bias_samples_remaining_ = bias_samples_total_;

  CommandSnapshot seed;
  seed.position[0] = pose_desired_.translation().x();
  seed.position[1] = pose_desired_.translation().y();
  seed.position[2] = pose_desired_.translation().z();
  const Eigen::Quaterniond qd(pose_desired_.rotation());
  seed.orientation[0] = qd.x();
  seed.orientation[1] = qd.y();
  seed.orientation[2] = qd.z();
  seed.orientation[3] = qd.w();
  seed.valid = true;
  command_box_.write(seed);
  command_ = seed;
  last_command_generation_ = command_box_.generation();
  seconds_since_command_ = 0.0;
  update_period_max_ = 0.0;
  missed_deadlines_ = 0;

  RCLCPP_INFO(logger, "admittance controller activated at the current TCP pose");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianAdmittanceController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  for (auto i : effort_command_indices_) {
    (void)command_interfaces_[i].set_value(0.0);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

bool CartesianAdmittanceController::readJointState()
{
  for (int i = 0; i < kNumJoints; ++i) {
    const auto p = state_interfaces_[position_state_indices_[i]].get_optional();
    const auto v = state_interfaces_[velocity_state_indices_[i]].get_optional();
    if (!p.has_value() || !v.has_value()) {return false;}
    q_(i) = p.value();
    qdot_(i) = v.value();
    if (!std::isfinite(q_(i)) || !std::isfinite(qdot_(i))) {return false;}
  }
  return true;
}

void CartesianAdmittanceController::readWrench(const Eigen::Isometry3d & tip_pose)
{
  CartesianVector raw = CartesianVector::Zero();
  for (int i = 0; i < 6; ++i) {
    const auto v = state_interfaces_[ft_state_indices_[i]].get_optional();
    raw(i) = (v.has_value() && std::isfinite(v.value())) ? v.value() : 0.0;
  }
  // Along the TCP axes; torque still about the sensor origin.
  raw.head<3>() = R_tip_sensor_ * raw.head<3>();
  raw.tail<3>() = R_tip_sensor_ * raw.tail<3>();

  const Eigen::Matrix3d R = tip_pose.rotation();
  // See the impedance controller: the sensor transmits m(a - g), so the
  // prediction must be -m*g, not +m*g, or the compensation doubles the
  // tool's weight instead of removing it.
  const Eigen::Vector3d f_grav_tip = R.transpose() * (-tool_mass_ * gravity_);
  CartesianVector predicted;
  predicted.head<3>() = f_grav_tip;
  predicted.tail<3>() = tool_com_.cross(f_grav_tip);

  CartesianVector compensated = raw - predicted;

  if (bias_samples_remaining_ > 0) {
    bias_accumulator_ += compensated;
    if (--bias_samples_remaining_ == 0) {
      wrench_bias_ =
        bias_accumulator_ / std::max(1.0, static_cast<double>(bias_samples_total_));
    }
    wrench_external_.setZero();
    return;
  }

  compensated -= wrench_bias_;

  CartesianVector in_base;
  in_base.head<3>() = R * compensated.head<3>();
  in_base.tail<3>() = R * compensated.tail<3>();
  wrench_external_ = wrench_filter_.update(in_base);
}

// ---------------------------------------------------------------------------
//  update()
// ---------------------------------------------------------------------------

controller_interface::return_type CartesianAdmittanceController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const double dt = period.seconds();
  const double dt_safe = (dt > 1e-6 && dt < 0.1) ? dt : nominal_period_;
  if (dt > 1.5 * nominal_period_) {++missed_deadlines_;}
  update_period_max_ = std::max(update_period_max_, dt);

  if (!readJointState() || !model_ok_ || !model_.compute(q_, qdot_, rb_)) {
    for (auto i : effort_command_indices_) {
      (void)command_interfaces_[i].set_value(0.0);
    }
    tau_previous_.setZero();
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "state or dynamics unavailable; commanding zero torque");
    return controller_interface::return_type::OK;
  }

  readWrench(rb_.tip_pose);

  // ---- command ----------------------------------------------------------
  const std::uint32_t gen = command_box_.generation();
  if (gen != last_command_generation_) {
    CommandSnapshot fresh;
    if (command_box_.read(fresh) && fresh.valid) {
      command_ = fresh;
      last_command_generation_ = gen;
      seconds_since_command_ = 0.0;
    }
  } else {
    seconds_since_command_ += dt_safe;
  }
  const bool command_stale = seconds_since_command_ > command_timeout_s_;

  pose_desired_.translation() =
    Eigen::Vector3d(command_.position[0], command_.position[1], command_.position[2]);
  pose_desired_.linear() = Eigen::Quaterniond(
    command_.orientation[3], command_.orientation[0],
    command_.orientation[1], command_.orientation[2]).normalized().toRotationMatrix();

  const double contact_force = wrench_external_.head<3>().norm();
  if (contact_force > hard_force_limit_n_) {safety_latched_ = true;}

  // ---- integrate the admittance model -----------------------------------
  //
  //   M_a * xddot_r  =  F_ext  -  D_a * xdot_r  -  K_a * x_r
  //
  // solved for the reference OFFSET from the commanded pose. Semi-implicit
  // Euler (rate first, then position, using the NEW rate) rather than
  // explicit: for a mass-spring-damper the explicit form gains energy every
  // step and will eventually go unstable on its own, with no help from the
  // environment. That is an embarrassing way to lose a comparison.
  CartesianVector f_in = safety_latched_ ? CartesianVector::Zero() : wrench_external_;

  for (int i = 0; i < 6; ++i) {
    const double accel =
      (f_in(i) - virtual_damping_(i) * reference_rate_(i) -
      virtual_stiffness_(i) * reference_offset_(i)) / virtual_mass_(i);
    reference_rate_(i) += accel * dt_safe;
    reference_rate_(i) = std::clamp(reference_rate_(i), -max_reference_speed_, max_reference_speed_);
    reference_offset_(i) += reference_rate_(i) * dt_safe;
  }

  // Bound the offset. Unbounded, a persistent force (a jammed gripper, a
  // stuck sensor reading a constant 4 N) integrates the reference off into
  // the distance, and the stiff inner loop follows it there.
  const double offset_norm = reference_offset_.head<3>().norm();
  if (offset_norm > max_reference_offset_m_) {
    reference_offset_.head<3>() *= max_reference_offset_m_ / offset_norm;
    // Zero the outward rate too, or the clamp fights the integrator every
    // cycle and the reference sits chattering on the boundary.
    const Eigen::Vector3d n = reference_offset_.head<3>().normalized();
    const double outward = reference_rate_.head<3>().dot(n);
    if (outward > 0.0) {reference_rate_.head<3>() -= outward * n;}
  }

  // ---- inner tracking loop ----------------------------------------------
  const Eigen::Vector3d x_reference =
    pose_desired_.translation() + reference_offset_.head<3>();

  CartesianVector e;
  e.head<3>() = x_reference - rb_.tip_pose.translation();

  Eigen::Quaterniond q_cur(rb_.tip_pose.rotation());
  Eigen::Quaterniond q_ref = Eigen::Quaterniond(pose_desired_.rotation()) *
    Eigen::Quaterniond(Eigen::AngleAxisd(
      reference_offset_.tail<3>().norm(),
      reference_offset_.tail<3>().norm() > 1e-9
      ? reference_offset_.tail<3>().normalized()
      : Eigen::Vector3d::UnitZ()));
  if (q_ref.coeffs().dot(q_cur.coeffs()) < 0.0) {q_cur.coeffs() = -q_cur.coeffs();}
  const Eigen::AngleAxisd aa(q_cur.inverse() * q_ref);
  e.tail<3>() = rb_.tip_pose.rotation() * (aa.axis() * aa.angle());

  CartesianVector e_dot;
  e_dot.head<3>() = reference_rate_.head<3>() - rb_.tip_twist.head<3>();
  e_dot.tail<3>() = reference_rate_.tail<3>() - rb_.tip_twist.tail<3>();

  const CartesianMatrix Lambda =
    KinematicsModel::taskSpaceInertia(rb_.jacobian, rb_.mass_inverse, singularity_damping_);

  CartesianVector F;
  for (int i = 0; i < 6; ++i) {
    const double m_task = std::max(0.05, std::abs(Lambda(i, i)));
    const double k = safety_latched_ ? 0.0 : inner_stiffness_(i);
    const double d = 2.0 * inner_damping_ratio_(i) * std::sqrt(std::max(0.0, k * m_task));
    // With k = 0 under a latched fault the damping would also go to zero and
    // the arm would be entirely free. Keep a floor so it still resists being
    // flung about.
    const double d_floor = 2.0 * inner_damping_ratio_(i) *
      std::sqrt(std::max(0.0, 100.0 * m_task));
    F(i) = k * e(i) + std::max(d, d_floor) * e_dot(i);
  }

  // ---- null space + gravity ---------------------------------------------
  const Eigen::Matrix<double, kNumJoints, 6> J_bar =
    rb_.mass_inverse * rb_.jacobian.transpose() * Lambda;
  const JointMatrix N =
    JointMatrix::Identity() - rb_.jacobian.transpose() * J_bar.transpose();

  JointVector tau_null;
  for (int i = 0; i < kNumJoints; ++i) {
    const double m_eff = std::max(0.05, rb_.mass(i, i));
    const double d = 2.0 * nullspace_damping_ratio_ *
      std::sqrt(std::max(0.0, nullspace_stiffness_ * m_eff));
    tau_null(i) = -nullspace_stiffness_ * (q_(i) - q_nullspace_(i)) - d * qdot_(i);
  }

  JointVector tau = rb_.jacobian.transpose() * F + N * tau_null + rb_.gravity + rb_.coriolis;

  const double max_step = torque_rate_limit_ * dt_safe;
  for (int i = 0; i < kNumJoints; ++i) {
    if (!std::isfinite(tau(i))) {tau(i) = 0.0;}
    tau(i) = tau_previous_(i) +
      std::clamp(tau(i) - tau_previous_(i), -max_step, max_step);
    tau(i) = std::clamp(tau(i), -torque_limits_(i), torque_limits_(i));
  }
  tau_previous_ = tau;

  for (int i = 0; i < kNumJoints; ++i) {
    (void)command_interfaces_[effort_command_indices_[i]].set_value(tau(i));
  }

  // ---- diagnostics -------------------------------------------------------
  DiagnosticsSnapshot snap;
  const auto fill = [](const Eigen::Isometry3d & T, double (&a)[7]) {
      a[0] = T.translation().x();
      a[1] = T.translation().y();
      a[2] = T.translation().z();
      const Eigen::Quaterniond q(T.rotation());
      a[3] = q.x();
      a[4] = q.y();
      a[5] = q.z();
      a[6] = q.w();
    };
  fill(rb_.tip_pose, snap.pose_current);
  fill(pose_desired_, snap.pose_desired);
  for (int i = 0; i < 3; ++i) {
    snap.position_error[i] = e(i);
    snap.orientation_error[i] = e(i + 3);
  }
  for (int i = 0; i < 6; ++i) {
    snap.wrench_commanded[i] = F(i);
    snap.wrench_external[i] = wrench_external_(i);
    snap.stiffness[i] = safety_latched_ ? 0.0 : inner_stiffness_(i);
    snap.damping[i] = virtual_damping_(i);
  }
  for (int i = 0; i < kNumJoints; ++i) {
    snap.tau_task[i] = (rb_.jacobian.transpose() * F)(i);
    snap.tau_nullspace[i] = (N * tau_null)(i);
    snap.tau_gravity[i] = rb_.gravity(i);
    snap.tau_commanded[i] = tau(i);
  }
  // The admittance controller has no energy tank: passivity here is a
  // property of the inner loop's tracking bandwidth, not of an explicit
  // budget. These fields are reported as neutral so a plot of the two
  // controllers side by side does not imply a tank that does not exist.
  snap.tank_energy = 0.0;
  snap.tank_valve = 1.0;
  snap.passivity_throttling = false;
  snap.update_period = dt;
  snap.update_period_max = update_period_max_;
  snap.missed_deadlines = missed_deadlines_;
  snap.command_stale = command_stale;
  snap.stamp_seconds = time.seconds();
  diagnostics_box_.write(snap);

  return controller_interface::return_type::OK;
}

void CartesianAdmittanceController::graspTargetCallback(
  const tops_msgs::msg::GraspTarget::SharedPtr msg)
{
  if (!msg->header.frame_id.empty() && msg->header.frame_id != base_frame_) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "ignoring target in frame '%s'; expected '%s'",
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
  cmd.valid = std::isfinite(cmd.position[0]) && std::isfinite(cmd.position[1]) &&
    std::isfinite(cmd.position[2]);
  cmd.stamp_seconds = rclcpp::Time(msg->header.stamp).seconds();
  if (cmd.valid) {command_box_.write(cmd);}
}

void CartesianAdmittanceController::publishSnapshot()
{
  DiagnosticsSnapshot snap;
  if (!diagnostics_box_.read(snap)) {return;}

  tops_msgs::msg::ControllerDiagnostics msg;
  msg.header.stamp = get_node()->now();
  msg.header.frame_id = base_frame_;
  msg.pose_current.position.x = snap.pose_current[0];
  msg.pose_current.position.y = snap.pose_current[1];
  msg.pose_current.position.z = snap.pose_current[2];
  msg.pose_desired.position.x = snap.pose_desired[0];
  msg.pose_desired.position.y = snap.pose_desired[1];
  msg.pose_desired.position.z = snap.pose_desired[2];
  for (int i = 0; i < 3; ++i) {
    msg.position_error[i] = snap.position_error[i];
    msg.orientation_error[i] = snap.orientation_error[i];
  }
  msg.position_error_norm = std::sqrt(
    snap.position_error[0] * snap.position_error[0] +
    snap.position_error[1] * snap.position_error[1] +
    snap.position_error[2] * snap.position_error[2]);
  msg.wrench_external.force.x = snap.wrench_external[0];
  msg.wrench_external.force.y = snap.wrench_external[1];
  msg.wrench_external.force.z = snap.wrench_external[2];
  msg.external_force_norm = std::sqrt(
    snap.wrench_external[0] * snap.wrench_external[0] +
    snap.wrench_external[1] * snap.wrench_external[1] +
    snap.wrench_external[2] * snap.wrench_external[2]);
  for (int i = 0; i < 6; ++i) {
    msg.stiffness[i] = snap.stiffness[i];
    msg.damping[i] = snap.damping[i];
  }
  msg.tau_commanded.assign(snap.tau_commanded, snap.tau_commanded + kNumJoints);
  msg.tau_gravity.assign(snap.tau_gravity, snap.tau_gravity + kNumJoints);
  msg.update_period = snap.update_period;
  msg.update_period_max = snap.update_period_max;
  msg.missed_deadlines = snap.missed_deadlines;
  msg.command_stale = snap.command_stale;
  diagnostics_pub_->publish(msg);
}

}  // namespace tops_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  tops_controllers::CartesianAdmittanceController,
  controller_interface::ControllerInterface)
