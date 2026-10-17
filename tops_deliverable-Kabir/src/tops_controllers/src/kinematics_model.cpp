// Copyright 2026 Kabir Dhingra. MIT licence.

#include "tops_controllers/kinematics_model.hpp"

#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <sstream>

namespace tops_controllers
{

bool KinematicsModel::initialise(
  const std::string & urdf_string,
  const std::string & base_link,
  const std::string & tip_link,
  const Eigen::Vector3d & gravity,
  std::string & error)
{
  initialised_ = false;
  joint_names_.clear();

  KDL::Tree tree;
  if (!kdl_parser::treeFromString(urdf_string, tree)) {
    error = "could not parse robot_description into a KDL tree";
    return false;
  }

  if (!tree.getChain(base_link, tip_link, chain_)) {
    std::ostringstream os;
    os << "no kinematic chain from '" << base_link << "' to '" << tip_link
       << "'. Check both frames exist and that nothing between them is a "
          "floating or planar joint.";
    error = os.str();
    return false;
  }

  // Collect the movable joints in chain order. Fixed joints are real links
  // in the chain but contribute no degree of freedom, so they are skipped
  // here and the indices below are DOF indices, not segment indices.
  for (unsigned int i = 0; i < chain_.getNrOfSegments(); ++i) {
    const KDL::Joint & joint = chain_.getSegment(i).getJoint();
    if (joint.getType() != KDL::Joint::None) {
      joint_names_.push_back(joint.getName());
    }
  }

  if (static_cast<int>(chain_.getNrOfJoints()) != kNumJoints) {
    std::ostringstream os;
    os << "chain '" << base_link << "' -> '" << tip_link << "' has "
       << chain_.getNrOfJoints() << " movable joints, but this controller is "
       << "compiled for exactly " << kNumJoints << ". Either the tip frame is "
       << "wrong (a common cause: pointing at a finger instead of the TCP, "
       << "which adds the prismatic finger joint) or the arm is not a Panda.";
    error = os.str();
    return false;
  }

  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
  jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
  dyn_solver_ = std::make_unique<KDL::ChainDynParam>(
    chain_, KDL::Vector(gravity.x(), gravity.y(), gravity.z()));

  q_kdl_.resize(kNumJoints);
  qdot_kdl_.resize(kNumJoints);
  gravity_kdl_.resize(kNumJoints);
  coriolis_kdl_.resize(kNumJoints);
  mass_kdl_.resize(kNumJoints);
  jac_kdl_.resize(kNumJoints);

  initialised_ = true;
  error.clear();
  return true;
}

bool KinematicsModel::compute(
  const JointVector & q, const JointVector & qdot, RigidBodyState & state)
{
  if (!initialised_) {return false;}

  for (int i = 0; i < kNumJoints; ++i) {
    q_kdl_(i) = q(i);
    qdot_kdl_(i) = qdot(i);
  }

  // --- forward kinematics ------------------------------------------------
  if (fk_solver_->JntToCart(q_kdl_, frame_kdl_) < 0) {return false;}

  state.tip_pose.setIdentity();
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      state.tip_pose.linear()(r, c) = frame_kdl_.M(r, c);
    }
    state.tip_pose.translation()(r) = frame_kdl_.p(r);
  }

  // --- Jacobian ----------------------------------------------------------
  // KDL expresses this in the base frame with the reference point at the
  // tip -- the same convention Pinocchio calls LOCAL_WORLD_ALIGNED. That
  // matters: it is the convention in which a target expressed in the base
  // frame produces a force in the base frame. Mixing conventions gives you
  // an arm that moves confidently in the wrong direction, which is a
  // genuinely confusing bug because nothing looks numerically wrong.
  if (jac_solver_->JntToJac(q_kdl_, jac_kdl_) < 0) {return false;}
  for (int r = 0; r < 6; ++r) {
    for (int c = 0; c < kNumJoints; ++c) {
      state.jacobian(r, c) = jac_kdl_(r, c);
    }
  }

  state.tip_twist.noalias() = state.jacobian * qdot;

  // --- dynamics ----------------------------------------------------------
  if (dyn_solver_->JntToMass(q_kdl_, mass_kdl_) < 0) {return false;}
  if (dyn_solver_->JntToCoriolis(q_kdl_, qdot_kdl_, coriolis_kdl_) < 0) {return false;}
  if (dyn_solver_->JntToGravity(q_kdl_, gravity_kdl_) < 0) {return false;}

  for (int r = 0; r < kNumJoints; ++r) {
    state.coriolis(r) = coriolis_kdl_(r);
    state.gravity(r) = gravity_kdl_(r);
    for (int c = 0; c < kNumJoints; ++c) {
      state.mass(r, c) = mass_kdl_(r, c);
    }
  }

  // M is symmetric positive definite by construction, so an LLT is both the
  // fastest inverse available and a free sanity check: if the factorisation
  // fails, the inertial parameters in the URDF are not physically valid.
  // Falling back to the identity keeps the loop deterministic; the caller
  // sees the failure and drops to gravity compensation.
  const Eigen::LLT<JointMatrix> llt(state.mass);
  if (llt.info() != Eigen::Success) {
    state.mass_inverse.setIdentity();
    return false;
  }
  state.mass_inverse = llt.solve(JointMatrix::Identity());

  return true;
}

CartesianMatrix KinematicsModel::taskSpaceInertia(
  const JacobianMatrix & J, const JointMatrix & M_inv, double lambda)
{
  // Lambda = (J M^-1 J^T + lambda^2 I)^-1
  //
  // The damping term is what stops this exploding at a singularity. At a
  // singularity J loses rank, J M^-1 J^T becomes singular, and the exact
  // inverse is unbounded -- which in a torque controller means an unbounded
  // commanded torque, which means the arm tries to destroy itself. The
  // damping bounds it at the cost of a small, well-understood error in the
  // null-space projection near singular configurations. That is an
  // unambiguously good trade.
  CartesianMatrix lambda_inv = J * M_inv * J.transpose();
  lambda_inv.diagonal().array() += lambda * lambda;
  return lambda_inv.inverse();
}

bool KinematicsModel::fixedTransform(
  const std::string & urdf_string,
  const std::string & from_link,
  const std::string & to_link,
  Eigen::Isometry3d & transform,
  std::string & error)
{
  transform.setIdentity();
  if (from_link == to_link) {
    error.clear();
    return true;
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromString(urdf_string, tree)) {
    error = "could not parse robot_description into a KDL tree";
    return false;
  }

  // getChain walks up to the common ancestor and back down, so this works
  // for sibling frames too (e.g. a sensor frame hanging off link7 and the
  // TCP hanging off the hand), not just for a frame and its descendant.
  KDL::Chain chain;
  if (!tree.getChain(from_link, to_link, chain)) {
    error = "no kinematic chain from '" + from_link + "' to '" + to_link + "'";
    return false;
  }
  if (chain.getNrOfJoints() != 0) {
    error = "the chain '" + from_link + "' -> '" + to_link + "' contains " +
      std::to_string(chain.getNrOfJoints()) + " movable joint(s); a fixed "
      "transform needs every joint between the two frames to be fixed";
    return false;
  }

  // With no movable joints the forward kinematics is just the product of
  // the fixed segment transforms; an empty joint array evaluates it.
  KDL::ChainFkSolverPos_recursive fk(chain);
  KDL::JntArray q_empty(0);
  KDL::Frame frame;
  if (fk.JntToCart(q_empty, frame) < 0) {
    error = "forward kinematics failed on the fixed chain";
    return false;
  }
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      transform.linear()(r, c) = frame.M(r, c);
    }
    transform.translation()(r) = frame.p(r);
  }
  error.clear();
  return true;
}

}  // namespace tops_controllers
