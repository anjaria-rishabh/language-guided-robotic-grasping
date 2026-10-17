// Copyright 2026 Kabir Dhingra. MIT licence.
//
// ===========================================================================
//  Cartesian admittance controller  --  the comparison arm of the study
// ===========================================================================
//
// Impedance and admittance are not two implementations of one idea. They are
// opposite causalities, and the difference decides where each one works.
//
//   IMPEDANCE   measure motion  ->  command force
//               The arm is a spring. Needs torque control and a good
//               dynamic model. Naturally stable against a stiff
//               environment: push a spring into a wall and nothing bad
//               happens. Struggles against a very soft environment and
//               against its own friction, because it has no integrator and
//               cannot push through stiction.
//
//   ADMITTANCE  measure force  ->  command motion
//               The arm is a mass being pushed around. Needs a force sensor
//               and a stiff, accurate inner position loop. Excellent on
//               soft environments and on geared arms with real friction,
//               because the inner loop hides all of it. Goes unstable
//               against a STIFF environment: contact closes a loop through
//               the sensor with very high loop gain, and the classic result
//               is buzzing against a rigid table.
//
// Theory/06_Compliant_Control.md §6.5. The project's hypothesis is that for
// a grasp approach onto a rigid tabletop, impedance wins -- and the point of
// implementing both properly is that the claim can be measured instead of
// asserted.
//
// The virtual dynamics integrated here:
//
//     M_a xddot_r + D_a xdot_r + K_a (x_r - x_d) = F_ext
//
// x_r, the admittance reference, is then tracked by a deliberately stiff
// inner Cartesian PD on the same effort interfaces the impedance controller
// uses. Using the same actuation path for both is what makes the comparison
// fair: any difference in the results is the control law, not the plumbing.

#ifndef TOPS_CONTROLLERS__CARTESIAN_ADMITTANCE_CONTROLLER_HPP_
#define TOPS_CONTROLLERS__CARTESIAN_ADMITTANCE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "tops_msgs/msg/controller_diagnostics.hpp"
#include "tops_msgs/msg/grasp_target.hpp"

#include "tops_controllers/cartesian_impedance_controller.hpp"  // CommandSnapshot, DiagnosticsSnapshot
#include "tops_controllers/kinematics_model.hpp"
#include "tops_controllers/low_pass_filter.hpp"
#include "tops_controllers/realtime_box.hpp"

namespace tops_controllers
{

class CartesianAdmittanceController : public controller_interface::ControllerInterface
{
public:
  CartesianAdmittanceController() = default;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  bool readJointState();
  void readWrench(const Eigen::Isometry3d & tip_pose);
  void graspTargetCallback(const tops_msgs::msg::GraspTarget::SharedPtr msg);
  void publishSnapshot();

  std::vector<std::string> joint_names_;
  std::string base_frame_;
  std::string tip_frame_;
  std::string ft_sensor_name_;

  // Virtual dynamics of the admittance model.
  CartesianVector virtual_mass_{CartesianVector::Zero()};
  CartesianVector virtual_damping_{CartesianVector::Zero()};
  CartesianVector virtual_stiffness_{CartesianVector::Zero()};

  // Inner tracking loop. Deliberately stiff: the admittance model IS the
  // compliance, so a soft inner loop would double-count it and the arm
  // would end up softer than the model says.
  CartesianVector inner_stiffness_{CartesianVector::Zero()};
  CartesianVector inner_damping_ratio_{CartesianVector::Zero()};

  JointVector torque_limits_{JointVector::Zero()};
  double torque_rate_limit_{1000.0};
  double singularity_damping_{0.05};
  double nullspace_stiffness_{20.0};
  double nullspace_damping_ratio_{1.0};
  JointVector q_nullspace_{JointVector::Zero()};

  double max_reference_offset_m_{0.10};
  double max_reference_speed_{0.5};
  double hard_force_limit_n_{15.0};
  double command_timeout_s_{0.1};
  double publish_rate_hz_{100.0};
  double tool_mass_{1.07};
  Eigen::Vector3d tool_com_{Eigen::Vector3d(0.0, 0.0073, 0.1065)};
  std::string sensor_frame_{"panda_ft_sensor_link"};
  Eigen::Matrix3d R_tip_sensor_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d gravity_{Eigen::Vector3d(0.0, 0.0, -9.80665)};

  KinematicsModel model_;
  RigidBodyState rb_;
  JointVector q_{JointVector::Zero()};
  JointVector qdot_{JointVector::Zero()};
  JointVector tau_previous_{JointVector::Zero()};

  Eigen::Isometry3d pose_desired_{Eigen::Isometry3d::Identity()};

  // The admittance reference state: an offset from the commanded pose, and
  // its rate. Held as an offset rather than an absolute pose so that a new
  // command does not discard the deflection the environment has caused.
  CartesianVector reference_offset_{CartesianVector::Zero()};
  CartesianVector reference_rate_{CartesianVector::Zero()};

  LowPassFilter<6> wrench_filter_;
  CartesianVector wrench_bias_{CartesianVector::Zero()};
  CartesianVector bias_accumulator_{CartesianVector::Zero()};
  CartesianVector wrench_external_{CartesianVector::Zero()};
  int bias_samples_remaining_{0};
  // The configured sample count, cached. update() must never read a
  // parameter: get_parameter() takes a mutex on the parameter service,
  // and a 1 kHz loop that can block on a lock held by a ROS callback
  // thread is not a real-time loop, however rarely it actually blocks.
  int bias_samples_total_{0};

  RealtimeBox<CommandSnapshot> command_box_;
  RealtimeBox<DiagnosticsSnapshot> diagnostics_box_;
  CommandSnapshot command_{};
  std::uint32_t last_command_generation_{0};
  double seconds_since_command_{0.0};
  double nominal_period_{0.001};
  double update_period_max_{0.0};
  std::uint32_t missed_deadlines_{0};
  bool model_ok_{false};
  bool safety_latched_{false};

  std::vector<std::size_t> ft_state_indices_;
  std::vector<std::size_t> position_state_indices_;
  std::vector<std::size_t> velocity_state_indices_;
  std::vector<std::size_t> effort_command_indices_;

  rclcpp::Subscription<tops_msgs::msg::GraspTarget>::SharedPtr grasp_target_sub_;
  rclcpp::Publisher<tops_msgs::msg::ControllerDiagnostics>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace tops_controllers

#endif  // TOPS_CONTROLLERS__CARTESIAN_ADMITTANCE_CONTROLLER_HPP_
