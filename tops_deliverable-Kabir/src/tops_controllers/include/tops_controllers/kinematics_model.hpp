// Copyright 2026 Kabir Dhingra. MIT licence.
//
// Rigid-body model: forward kinematics, the Jacobian, and the three terms
// of the manipulator equation.
//
//     M(q) qddot  +  C(q, qdot) qdot  +  g(q)  =  tau  +  J^T F_ext
//
// Theory/03_Kinematics.md and Theory/04_Dynamics.md.
//
// ---------------------------------------------------------------------------
// Why KDL rather than Pinocchio
// ---------------------------------------------------------------------------
// Pinocchio is the better library and has the nicer API. This uses Orocos
// KDL for one reason: KDL ships with the ROS 2 desktop install and
// Pinocchio's C++ development package does not. A deliverable that cannot
// compile until someone runs a pip install with --break-system-packages is
// not a deliverable.
//
// Nothing is given up by the substitution. KDL's recursive solvers (CRBA
// for M, RNE for C and g) are the same algorithms, and
// test/test_kinematics_model.cpp pins the results against statics computed
// by hand rather than against another library's output.
//
// ---------------------------------------------------------------------------
// Real-time discipline
// ---------------------------------------------------------------------------
// Every KDL array and solver is constructed once in `initialise()`. Nothing
// in `compute()` allocates, resizes, throws, logs, or takes a lock. That is
// not a stylistic preference: a single unexpected malloc inside a 1 kHz
// loop is a page fault waiting to happen, and a page fault is a missed
// deadline. See Theory/05_Control_Fundamentals.md §5.8.

#ifndef TOPS_CONTROLLERS__KINEMATICS_MODEL_HPP_
#define TOPS_CONTROLLERS__KINEMATICS_MODEL_HPP_

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>

namespace tops_controllers
{

/// The arm is a Panda and the Panda has seven joints. Fixing this at
/// compile time makes every matrix below stack-allocated and every Eigen
/// expression allocation-free, which is what a hard real-time loop needs.
/// A different arm is a recompile, not a runtime reconfiguration -- and the
/// gains would all need re-deriving anyway, so nothing is really lost.
inline constexpr int kNumJoints = 7;

using JointVector = Eigen::Matrix<double, kNumJoints, 1>;
using JointMatrix = Eigen::Matrix<double, kNumJoints, kNumJoints>;
using JacobianMatrix = Eigen::Matrix<double, 6, kNumJoints>;
using CartesianVector = Eigen::Matrix<double, 6, 1>;
using CartesianMatrix = Eigen::Matrix<double, 6, 6>;

/// Everything the control law needs about the robot's current state,
/// filled in one pass so the recursive solvers walk the chain once.
struct RigidBodyState
{
  Eigen::Isometry3d tip_pose{Eigen::Isometry3d::Identity()};  ///< base -> tip
  JacobianMatrix jacobian{JacobianMatrix::Zero()};            ///< base frame, tip reference point
  JointMatrix mass{JointMatrix::Identity()};                  ///< M(q)
  JointMatrix mass_inverse{JointMatrix::Identity()};          ///< M(q)^-1
  JointVector coriolis{JointVector::Zero()};                  ///< C(q,qdot) qdot
  JointVector gravity{JointVector::Zero()};                   ///< g(q)
  CartesianVector tip_twist{CartesianVector::Zero()};         ///< J qdot
};

class KinematicsModel
{
public:
  KinematicsModel() = default;

  /// Build the chain and the solvers. Call once, off the real-time path.
  ///
  /// @param urdf_string  the robot_description, already xacro-expanded
  /// @param base_link    root of the chain; the frame all Cartesian
  ///                     quantities are expressed in
  /// @param tip_link     the controlled frame (the TCP)
  /// @param gravity      gravity vector IN THE BASE FRAME. For a robot
  ///                     bolted upright this is (0, 0, -9.80665). For one
  ///                     mounted on a wall or inverted it is not, and
  ///                     getting it wrong produces a gravity-compensation
  ///                     term that is confidently wrong in a direction.
  /// @param error        human-readable reason on failure
  /// @return false if the URDF will not parse or the chain does not have
  ///         exactly kNumJoints movable joints
  bool initialise(
    const std::string & urdf_string,
    const std::string & base_link,
    const std::string & tip_link,
    const Eigen::Vector3d & gravity,
    std::string & error);

  /// Fill `state` from the current joint position and velocity.
  /// Allocation-free and non-throwing. Returns false only if a solver
  /// reports an error, which for these recursive solvers means a malformed
  /// chain -- i.e. something that should have failed in initialise().
  bool compute(const JointVector & q, const JointVector & qdot, RigidBodyState & state);

  /// Joint names in chain order. The order state interfaces must be read in.
  const std::vector<std::string> & jointNames() const { return joint_names_; }

  bool initialised() const { return initialised_; }

  /// Damped-least-squares inverse of the task-space inertia, used by the
  /// null-space projector. lambda is the damping: 0 gives the exact
  /// inverse and blows up at a singularity; a small positive value trades
  /// a little accuracy for not blowing up. See docs/02_CONTROL_LAW.md §3.
  static CartesianMatrix taskSpaceInertia(
    const JacobianMatrix & J, const JointMatrix & M_inv, double lambda);

  /// The constant transform from `from_link` to `to_link`, read out of the
  /// URDF: the pose of `to_link` expressed in `from_link`.
  ///
  /// Only defined when every joint between the two frames is FIXED -- the
  /// answer must not depend on q. Used once, off the real-time path, to
  /// relate the frame a sensor reports in to the TCP. Reading it from the
  /// model rather than typing it into a YAML file is the point: on the real
  /// Panda the hand is bolted on at -45 degrees, and a hand-copied constant
  /// is exactly the kind of number that is right in one file and wrong in
  /// another.
  ///
  /// @return false (with `error` set) if the URDF does not parse, either
  ///         frame is missing, or a movable joint lies between them
  static bool fixedTransform(
    const std::string & urdf_string,
    const std::string & from_link,
    const std::string & to_link,
    Eigen::Isometry3d & transform,
    std::string & error);

private:
  KDL::Chain chain_;
  std::vector<std::string> joint_names_;

  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  std::unique_ptr<KDL::ChainDynParam> dyn_solver_;

  // Pre-allocated scratch. Mutable because compute() is logically const but
  // KDL's solver API writes through these.
  KDL::JntArray q_kdl_{kNumJoints};
  KDL::JntArray qdot_kdl_{kNumJoints};
  KDL::JntArray gravity_kdl_{kNumJoints};
  KDL::JntArray coriolis_kdl_{kNumJoints};
  KDL::JntSpaceInertiaMatrix mass_kdl_{kNumJoints};
  KDL::Jacobian jac_kdl_{kNumJoints};
  KDL::Frame frame_kdl_;

  bool initialised_{false};
};

}  // namespace tops_controllers

#endif  // TOPS_CONTROLLERS__KINEMATICS_MODEL_HPP_
