// Copyright 2026 Kabir Dhingra. MIT licence.
//
// ===========================================================================
//  Cartesian impedance controller  --  the D2 deliverable
// ===========================================================================
//
// A ros2_control controller plugin that makes a 7-DOF Panda behave like a
// programmable spring-damper in Cartesian space, running in the
// controller_manager's 1 kHz update loop.
//
// The control law, in full:
//
//   e     = [ x_d - x ;  orientation error as an axis-angle vector ]
//   e_dot = [ v_d - v ;  omega_d - omega ]
//
//   F     = gamma * ( K e + F_force )  +  D e_dot          (task wrench)
//
//   tau   =  J^T F                                          task torque
//         +  N ( -K_n (q - q_n) - D_n qdot )                null-space posture
//         +  g(q)                                           gravity compensation
//         +  C(q, qdot) qdot                                Coriolis (optional)
//
//   tau  <- rate_limit( clamp( tau, +/- tau_max ) )
//
// with N = I - J^T Jbar^T the dynamically consistent null-space projector,
// and gamma in [0,1] the energy-tank valve that makes the whole thing
// passive across gain changes.
//
// Read docs/02_CONTROL_LAW.md for the derivation of every term, and
// Theory/06_Compliant_Control.md for why this is the right law at all.
//
// ---------------------------------------------------------------------------
// What is deliberately NOT here
// ---------------------------------------------------------------------------
// No planning, no inverse kinematics, no iteration, no convergence test.
// The law is a single pass of matrix arithmetic with a hard bound on its
// output. That is not a simplification -- it is the property that makes it
// safe to run in a hard real-time loop next to a person. Anything that can
// fail to converge has no business inside update().
//
// ---------------------------------------------------------------------------
// Real-time contract
// ---------------------------------------------------------------------------
// update() does not allocate, lock, log, throw, or call anything that might.
// Every matrix is fixed-size and stack-resident, every solver is built in
// on_configure(), and both boundaries to the rest of ROS 2 go through
// RealtimeBox (see realtime_box.hpp). The one exception is the throttled
// RCLCPP_*_THROTTLE on the fault path, which fires at most once a second
// and only when the controller is already refusing to move the arm.

#ifndef TOPS_CONTROLLERS__CARTESIAN_IMPEDANCE_CONTROLLER_HPP_
#define TOPS_CONTROLLERS__CARTESIAN_IMPEDANCE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "tops_msgs/msg/controller_diagnostics.hpp"
#include "tops_msgs/msg/grasp_target.hpp"
#include "tops_msgs/srv/set_control_mode.hpp"
#include "tops_msgs/srv/trigger_retreat.hpp"

#include "tops_controllers/energy_tank.hpp"
#include "tops_controllers/gain_ramp.hpp"
#include "tops_controllers/kinematics_model.hpp"
#include "tops_controllers/low_pass_filter.hpp"
#include "tops_controllers/realtime_box.hpp"

namespace tops_controllers
{

/// Fixed-size command snapshot handed from the ROS callback thread into the
/// control loop. Trivially copyable by construction; see RealtimeBox.
struct CommandSnapshot
{
  double position[3]{0.0, 0.0, 0.0};
  double orientation[4]{0.0, 0.0, 0.0, 1.0};  // x, y, z, w
  double linear_velocity[3]{0.0, 0.0, 0.0};
  double angular_velocity[3]{0.0, 0.0, 0.0};
  double stamp_seconds{0.0};
  bool valid{false};
};

/// Fixed-size diagnostic snapshot handed the other way: written by the
/// control loop, read by a plain (non-real-time) timer which does the
/// message construction and the publish.
struct DiagnosticsSnapshot
{
  double pose_current[7]{};       // x y z qx qy qz qw
  double pose_desired[7]{};
  double position_error[3]{};
  double orientation_error[3]{};
  double wrench_commanded[6]{};
  double wrench_external[6]{};
  double stiffness[6]{};
  double damping[6]{};
  double tau_task[kNumJoints]{};
  double tau_nullspace[kNumJoints]{};
  double tau_gravity[kNumJoints]{};
  double tau_commanded[kNumJoints]{};
  double tank_energy{0.0};
  double tank_state{0.0};
  double tank_valve{1.0};
  double power_dissipated{0.0};
  double power_injected{0.0};
  double stiffness_scale{1.0};
  double update_period{0.0};
  double update_period_max{0.0};
  double stamp_seconds{0.0};
  std::uint32_t missed_deadlines{0};
  bool passivity_throttling{false};
  bool command_stale{false};
};

class CartesianImpedanceController : public controller_interface::ControllerInterface
{
public:
  CartesianImpedanceController() = default;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  // ---- the pieces of update(), split so each can be read on its own ----
  bool readJointState();
  void readAndConditionWrench(
    double dt, const Eigen::Isometry3d & tip_pose,
    const CartesianVector & tip_twist);
  CartesianVector poseError(const Eigen::Isometry3d & tip_pose) const;
  CartesianVector forceRegulation(double dt, const Eigen::Isometry3d & tip_pose);
  JointVector nullSpaceTorque(const RigidBodyState & rb) const;
  void applyTorqueLimits(JointVector & tau, double dt);
  void publishSnapshot();

  // ---- ROS callbacks (non-real-time) ------------------------------------
  void graspTargetCallback(const tops_msgs::msg::GraspTarget::SharedPtr msg);
  void poseTargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void setControlMode(
    const std::shared_ptr<tops_msgs::srv::SetControlMode::Request> request,
    std::shared_ptr<tops_msgs::srv::SetControlMode::Response> response);
  void triggerRetreat(
    const std::shared_ptr<tops_msgs::srv::TriggerRetreat::Request> request,
    std::shared_ptr<tops_msgs::srv::TriggerRetreat::Response> response);

  // ---- configuration ----------------------------------------------------
  std::vector<std::string> joint_names_;
  std::string base_frame_;
  std::string tip_frame_;
  std::string ft_sensor_name_;

  CartesianVector stiffness_stiff_{CartesianVector::Zero()};
  CartesianVector stiffness_compliant_{CartesianVector::Zero()};
  CartesianVector damping_ratio_{CartesianVector::Zero()};
  double stiffness_ramp_rate_{1500.0};

  double nullspace_stiffness_{20.0};
  double nullspace_damping_ratio_{1.0};
  JointVector q_nullspace_{JointVector::Zero()};

  JointVector torque_limits_{JointVector::Zero()};
  double torque_rate_limit_{1000.0};
  double singularity_damping_{0.05};
  bool compensate_coriolis_{true};

  double joint_limit_margin_{0.15};
  double joint_limit_stiffness_{40.0};
  JointVector q_lower_{JointVector::Zero()};
  JointVector q_upper_{JointVector::Zero()};

  double wrench_clamp_n_{50.0};
  double tool_mass_{1.07};
  /// Centre of mass of everything distal to the sensor, measured from the
  /// sensor origin and expressed along the TCP axes.
  Eigen::Vector3d tool_com_{Eigen::Vector3d(0.0, 0.0073, 0.1065)};
  /// Frame the F/T sensor reports in, and its fixed rotation into the TCP
  /// frame (read from the URDF in on_configure, never typed in by hand).
  std::string sensor_frame_{"panda_ft_sensor_link"};
  Eigen::Matrix3d R_tip_sensor_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d gravity_{Eigen::Vector3d(0.0, 0.0, -9.80665)};

  bool force_control_enabled_{true};
  Eigen::Vector3d force_axis_tip_{Eigen::Vector3d(0.0, 0.0, 1.0)};
  double force_desired_n_{5.0};
  double force_contact_threshold_n_{1.0};
  //: true only while the force loop is genuinely regulating against
  //: something. The stiffness reduction below is tied to this.
  bool force_loop_active_{false};
  double force_kp_{0.5};
  double force_ki_{2.0};
  double force_integral_clamp_n_{3.0};
  double force_axis_stiffness_scale_{0.2};

  double hard_force_limit_n_{15.0};
  double command_timeout_s_{0.1};
  double max_command_step_m_{0.25};
  double publish_rate_hz_{100.0};

  // ---- runtime state ----------------------------------------------------
  KinematicsModel model_;
  RigidBodyState rb_;

  JointVector q_{JointVector::Zero()};
  JointVector qdot_{JointVector::Zero()};
  JointVector tau_previous_{JointVector::Zero()};

  Eigen::Isometry3d pose_desired_{Eigen::Isometry3d::Identity()};
  CartesianVector twist_desired_{CartesianVector::Zero()};

  GainRamp<6> stiffness_ramp_;
  CartesianVector damping_{CartesianVector::Zero()};

  LowPassFilter<6> wrench_filter_control_;
  LowPassFilter<6> wrench_filter_safety_;
  CartesianVector wrench_bias_{CartesianVector::Zero()};
  CartesianVector wrench_external_{CartesianVector::Zero()};
  CartesianVector wrench_safety_{CartesianVector::Zero()};
  int bias_samples_remaining_{0};
  // The configured sample count, cached. update() must never read a
  // parameter: get_parameter() takes a mutex on the parameter service,
  // and a 1 kHz loop that can block on a lock held by a ROS callback
  // thread is not a real-time loop, however rarely it actually blocks.
  int bias_samples_total_{0};
  CartesianVector bias_accumulator_{CartesianVector::Zero()};

  EnergyTank tank_;
  double force_integral_n_{0.0};

  // State needed to bill the tank for the NON-PASSIVE power only: the
  // equilibrium's own motion and any change of stiffness. See update().
  CartesianVector stiffness_prev_{CartesianVector::Zero()};

  // For removing the tool's INERTIAL load from the measured wrench. The
  // wrist sensor reads gravity + inertia + contact; only the third is of
  // interest, and the second is large whenever the arm accelerates.
  CartesianVector tip_twist_prev_{CartesianVector::Zero()};
  bool have_prev_twist_{false};
  LowPassFilter<3> tcp_accel_filter_;

  RealtimeBox<CommandSnapshot> command_box_;
  RealtimeBox<DiagnosticsSnapshot> diagnostics_box_;
  CommandSnapshot command_{};
  std::uint32_t last_command_generation_{0};
  double seconds_since_command_{0.0};

  double update_period_max_{0.0};
  double nominal_period_{0.001};
  std::uint32_t missed_deadlines_{0};

  bool model_ok_{false};
  bool safety_latched_{false};
  std::string safety_reason_;
  int requested_mode_{0};

  // Index of each F/T state interface within state_interfaces_, resolved
  // once in on_activate so update() does no string comparison.
  std::vector<std::size_t> ft_state_indices_;
  std::vector<std::size_t> position_state_indices_;
  std::vector<std::size_t> velocity_state_indices_;
  std::vector<std::size_t> effort_command_indices_;

  rclcpp::Subscription<tops_msgs::msg::GraspTarget>::SharedPtr grasp_target_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_target_sub_;
  rclcpp::Publisher<tops_msgs::msg::ControllerDiagnostics>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
  rclcpp::Service<tops_msgs::srv::SetControlMode>::SharedPtr mode_service_;
  rclcpp::Service<tops_msgs::srv::TriggerRetreat>::SharedPtr retreat_service_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace tops_controllers

#endif  // TOPS_CONTROLLERS__CARTESIAN_IMPEDANCE_CONTROLLER_HPP_
