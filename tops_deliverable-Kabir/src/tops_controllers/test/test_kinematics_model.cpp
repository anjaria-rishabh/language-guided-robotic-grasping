// Copyright 2026 Kabir Dhingra. MIT licence.
//
// The dynamics model is the layer every control gain sits on top of. If
// g(q) is wrong by a sign the arm sags or lunges and it looks like a tuning
// problem; if M(q) is wrong the damping derived from it is wrong everywhere
// in the workspace. So these tests check the solvers against a chain whose
// mass matrix, gravity vector and Jacobian can all be written down by hand.
//
// The test chain: seven revolute joints about +y, spaced 0.1 m along +x,
// each carrying a 1 kg point-like mass 0.1 m distal of its own joint. At
// q = 0 the whole thing lies straight along +x, horizontal, which is the
// configuration where every quantity has a one-line closed form.

#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "tops_controllers/kinematics_model.hpp"

using tops_controllers::JointVector;
using tops_controllers::KinematicsModel;
using tops_controllers::RigidBodyState;
using tops_controllers::kNumJoints;

namespace
{

constexpr double kLink = 0.1;      // metres between consecutive joints
constexpr double kMass = 1.0;      // kg per link
constexpr double kInertia = 1e-4;  // kg.m^2, isotropic, about each link's COM
constexpr double kG = 9.80665;

std::string buildTestUrdf()
{
  std::string urdf =
    "<?xml version=\"1.0\"?><robot name=\"test_chain\"><link name=\"base\"/>";
  for (int i = 0; i < kNumJoints; ++i) {
    const std::string parent = (i == 0) ? "base" : ("l" + std::to_string(i));
    const std::string child = "l" + std::to_string(i + 1);
    const double offset = (i == 0) ? 0.0 : kLink;
    urdf +=
      "<joint name=\"j" + std::to_string(i + 1) + "\" type=\"revolute\">"
      "<parent link=\"" + parent + "\"/><child link=\"" + child + "\"/>"
      "<origin xyz=\"" + std::to_string(offset) + " 0 0\" rpy=\"0 0 0\"/>"
      "<axis xyz=\"0 1 0\"/>"
      "<limit lower=\"-3\" upper=\"3\" effort=\"100\" velocity=\"3\"/></joint>"
      "<link name=\"" + child + "\"><inertial>"
      "<origin xyz=\"" + std::to_string(kLink) + " 0 0\"/>"
      "<mass value=\"" + std::to_string(kMass) + "\"/>"
      "<inertia ixx=\"1e-4\" ixy=\"0\" ixz=\"0\" iyy=\"1e-4\" iyz=\"0\" izz=\"1e-4\"/>"
      "</inertial></link>";
  }
  urdf += "</robot>";
  return urdf;
}

/// x-coordinate of joint i's axis, at q = 0.
double jointX(int i) { return i * kLink; }

/// x-coordinate of link i's centre of mass, at q = 0.
double massX(int i) { return (i + 1) * kLink; }

KinematicsModel makeModel()
{
  KinematicsModel model;
  std::string error;
  const bool ok = model.initialise(
    buildTestUrdf(), "base", "l7", Eigen::Vector3d(0.0, 0.0, -kG), error);
  EXPECT_TRUE(ok) << error;
  return model;
}

}  // namespace

TEST(KinematicsModel, RejectsAChainWithTheWrongNumberOfJoints)
{
  KinematicsModel model;
  std::string error;
  // "l4" is only four joints down the chain.
  EXPECT_FALSE(model.initialise(
      buildTestUrdf(), "base", "l4", Eigen::Vector3d(0, 0, -kG), error));
  EXPECT_NE(error.find("movable joints"), std::string::npos)
    << "the error message must say what was actually wrong: " << error;
}

TEST(KinematicsModel, RejectsAMissingFrame)
{
  KinematicsModel model;
  std::string error;
  EXPECT_FALSE(model.initialise(
      buildTestUrdf(), "base", "no_such_link", Eigen::Vector3d(0, 0, -kG), error));
  EXPECT_FALSE(error.empty());
}

TEST(KinematicsModel, ReportsJointNamesInChainOrder)
{
  auto model = makeModel();
  ASSERT_EQ(model.jointNames().size(), static_cast<std::size_t>(kNumJoints));
  for (int i = 0; i < kNumJoints; ++i) {
    EXPECT_EQ(model.jointNames()[i], "j" + std::to_string(i + 1));
  }
}

TEST(KinematicsModel, ForwardKinematicsAtZero)
{
  auto model = makeModel();
  RigidBodyState rb;
  ASSERT_TRUE(model.compute(JointVector::Zero(), JointVector::Zero(), rb));

  // Six 0.1 m offsets from base to the seventh joint, and the tip frame is
  // that joint's frame, so the tip is at x = 0.6.
  EXPECT_NEAR(rb.tip_pose.translation().x(), 6 * kLink, 1e-9);
  EXPECT_NEAR(rb.tip_pose.translation().y(), 0.0, 1e-9);
  EXPECT_NEAR(rb.tip_pose.translation().z(), 0.0, 1e-9);
  EXPECT_TRUE(rb.tip_pose.rotation().isApprox(Eigen::Matrix3d::Identity(), 1e-9));
}

TEST(KinematicsModel, GravityMatchesTheHandComputedStatics)
{
  auto model = makeModel();
  RigidBodyState rb;
  ASSERT_TRUE(model.compute(JointVector::Zero(), JointVector::Zero(), rb));

  // g(q) is the torque the actuators must apply to hold still, so it is the
  // NEGATIVE of the torque gravity exerts. Getting this sign backwards is a
  // classic and it doubles the error rather than cancelling it: the arm
  // accelerates downward at 2g.
  for (int i = 0; i < kNumJoints; ++i) {
    double gravity_torque = 0.0;
    for (int j = i; j < kNumJoints; ++j) {
      gravity_torque += kMass * kG * (massX(j) - jointX(i));
    }
    EXPECT_NEAR(rb.gravity(i), -gravity_torque, 1e-4)
      << "joint " << i;
  }

  // The base joint carries everything: 7 kg with its centre of mass at
  // x = 0.4 m, which is 27.46 N.m.
  EXPECT_NEAR(rb.gravity(0), -7.0 * kG * 0.4, 1e-4);
}

TEST(KinematicsModel, GravityDoesNotDependOnVelocity)
{
  auto model = makeModel();
  RigidBodyState at_rest;
  RigidBodyState moving;
  ASSERT_TRUE(model.compute(JointVector::Zero(), JointVector::Zero(), at_rest));
  ASSERT_TRUE(model.compute(JointVector::Zero(), JointVector::Constant(1.5), moving));
  EXPECT_TRUE(at_rest.gravity.isApprox(moving.gravity, 1e-9));
}

TEST(KinematicsModel, CoriolisIsZeroAtRestAndGrowsWithVelocitySquared)
{
  auto model = makeModel();
  RigidBodyState rb;

  ASSERT_TRUE(model.compute(JointVector::Constant(0.3), JointVector::Zero(), rb));
  EXPECT_NEAR(rb.coriolis.norm(), 0.0, 1e-9)
    << "C(q, qdot) qdot must vanish when qdot does";

  ASSERT_TRUE(model.compute(JointVector::Constant(0.3), JointVector::Constant(1.0), rb));
  const double at_one = rb.coriolis.norm();
  ASSERT_TRUE(model.compute(JointVector::Constant(0.3), JointVector::Constant(2.0), rb));
  const double at_two = rb.coriolis.norm();

  // Doubling every joint velocity quadruples the Coriolis term. This is why
  // omitting it is defensible at grasp-approach speeds and indefensible
  // during a fast retreat.
  EXPECT_NEAR(at_two / at_one, 4.0, 0.05);
}

TEST(KinematicsModel, MassMatrixMatchesTheParallelAxisTheorem)
{
  auto model = makeModel();
  RigidBodyState rb;
  ASSERT_TRUE(model.compute(JointVector::Zero(), JointVector::Zero(), rb));

  // M(i,i) is the moment of inertia about joint i of everything distal to
  // it: sum over j >= i of m*d^2 plus each link's own Iyy.
  for (int i = 0; i < kNumJoints; ++i) {
    double expected = 0.0;
    for (int j = i; j < kNumJoints; ++j) {
      const double d = massX(j) - jointX(i);
      expected += kMass * d * d + kInertia;
    }
    EXPECT_NEAR(rb.mass(i, i), expected, 1e-6) << "diagonal entry " << i;
  }

  EXPECT_NEAR(rb.mass(0, 0), 1.4007, 1e-4);
  EXPECT_NEAR(rb.mass(6, 6), 0.0101, 1e-6);
}

TEST(KinematicsModel, MassMatrixIsSymmetricAndOffDiagonalIsNotZero)
{
  auto model = makeModel();
  RigidBodyState rb;
  ASSERT_TRUE(model.compute(JointVector::Constant(0.2), JointVector::Zero(), rb));

  EXPECT_TRUE(rb.mass.isApprox(rb.mass.transpose(), 1e-9));

  // The off-diagonal entries ARE the dynamic coupling: accelerating joint 7
  // exerts a torque on joint 1. This is why seven independent per-joint PID
  // loops fight each other, and why model-based control exists at all.
  EXPECT_GT(std::abs(rb.mass(0, 6)), 1e-3);
}

TEST(KinematicsModel, MassInverseIsActuallyTheInverse)
{
  auto model = makeModel();
  RigidBodyState rb;
  ASSERT_TRUE(model.compute(JointVector::Constant(-0.4), JointVector::Zero(), rb));
  const auto product = rb.mass * rb.mass_inverse;
  EXPECT_TRUE(product.isApprox(
      Eigen::Matrix<double, kNumJoints, kNumJoints>::Identity(), 1e-8));
}

TEST(KinematicsModel, JacobianMapsJointVelocityToTipTwist)
{
  auto model = makeModel();
  RigidBodyState rb;
  ASSERT_TRUE(model.compute(JointVector::Zero(), JointVector::Zero(), rb));

  // Rotating joint 1 about +y moves a tip at x = 0.6 in -z at 0.6 m/s per
  // rad/s. That single number pins down the frame convention: base frame,
  // tip reference point.
  EXPECT_NEAR(rb.jacobian(2, 0), -6 * kLink, 1e-9);
  EXPECT_NEAR(rb.jacobian(0, 0), 0.0, 1e-9);

  // The last joint is AT the tip, so it produces pure rotation there.
  const double last_joint_linear = rb.jacobian.col(6).head<3>().norm();
  EXPECT_NEAR(last_joint_linear, 0.0, 1e-9);
  EXPECT_NEAR(rb.jacobian(4, 6), 1.0, 1e-9);

  // And the twist really is J qdot.
  const JointVector qdot = JointVector::Constant(0.5);
  ASSERT_TRUE(model.compute(JointVector::Zero(), qdot, rb));
  EXPECT_TRUE(rb.tip_twist.isApprox(rb.jacobian * qdot, 1e-9));
}

TEST(KinematicsModel, TaskSpaceInertiaStaysFiniteAtASingularity)
{
  auto model = makeModel();
  RigidBodyState rb;
  // Straight out is singular for this chain: all seven axes are parallel,
  // so the Jacobian has rank 2 at most and J M^-1 J^T is badly rank
  // deficient. Without the damping term the inverse is unbounded, and an
  // unbounded task-space inertia becomes an unbounded commanded torque.
  ASSERT_TRUE(model.compute(JointVector::Zero(), JointVector::Zero(), rb));

  const auto damped =
    KinematicsModel::taskSpaceInertia(rb.jacobian, rb.mass_inverse, 0.05);
  EXPECT_TRUE(damped.allFinite());
  EXPECT_LT(damped.norm(), 1e4);

  // With no damping it is not merely large -- at an exact singularity the
  // inverse comes back as NaN. That is worth asserting in this form rather
  // than as "a big number", because NaN propagates silently: it would reach
  // the null-space projector, then the commanded torque, and the hardware
  // would be handed a value that is not a number at all.
  const auto undamped =
    KinematicsModel::taskSpaceInertia(rb.jacobian, rb.mass_inverse, 0.0);
  const bool unusable =
    !undamped.allFinite() || undamped.norm() > 100.0 * damped.norm();
  EXPECT_TRUE(unusable)
    << "undamped norm was " << undamped.norm()
    << "; if this ever becomes well behaved, the damping term's "
       "justification needs revisiting";
}

TEST(KinematicsModel, RejectsComputeBeforeInitialise)
{
  KinematicsModel model;
  RigidBodyState rb;
  EXPECT_FALSE(model.compute(JointVector::Zero(), JointVector::Zero(), rb));
}

// ---------------------------------------------------------------------------
//  fixedTransform: relating the F/T sensor frame to the TCP
// ---------------------------------------------------------------------------
// A miniature of the real Panda wrist: the sensor frame sits on link7, the
// hand is bolted to the flange at -45 degrees about z, and the TCP hangs off
// the hand. The controllers rotate every wrench reading by the sensor-to-TCP
// rotation this returns, so if it were transposed or the sign of the 45
// degrees flipped, every lateral force would come out rotated by 90 degrees.

namespace
{

std::string buildWristUrdf()
{
  return
    "<?xml version=\"1.0\"?><robot name=\"wrist\">"
    "<link name=\"link6\"/><link name=\"link7\"/><link name=\"sensor\"/>"
    "<link name=\"link8\"/><link name=\"hand\"/><link name=\"tcp\"/>"
    "<joint name=\"j7\" type=\"revolute\"><parent link=\"link6\"/><child link=\"link7\"/>"
    "<axis xyz=\"0 0 1\"/><limit lower=\"-3\" upper=\"3\" effort=\"10\" velocity=\"3\"/></joint>"
    "<joint name=\"s\" type=\"fixed\"><parent link=\"link7\"/><child link=\"sensor\"/></joint>"
    "<joint name=\"f\" type=\"fixed\"><parent link=\"link7\"/><child link=\"link8\"/>"
    "<origin xyz=\"0 0 0.107\"/></joint>"
    "<joint name=\"h\" type=\"fixed\"><parent link=\"link8\"/><child link=\"hand\"/>"
    "<origin rpy=\"0 0 -0.7853981633974483\"/></joint>"
    "<joint name=\"t\" type=\"fixed\"><parent link=\"hand\"/><child link=\"tcp\"/>"
    "<origin xyz=\"0 0 0.1034\"/></joint>"
    "</robot>";
}

}  // namespace

TEST(KinematicsModel, FixedTransformRecoversTheHandMountRotation)
{
  Eigen::Isometry3d tcp_T_sensor;
  std::string error;
  ASSERT_TRUE(KinematicsModel::fixedTransform(
      buildWristUrdf(), "tcp", "sensor", tcp_T_sensor, error)) << error;

  // The hand is at -45 deg relative to the sensor, so the sensor is at
  // +45 deg relative to the TCP.
  const Eigen::Matrix3d expected =
    Eigen::AngleAxisd(M_PI / 4.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  EXPECT_TRUE(tcp_T_sensor.rotation().isApprox(expected, 1e-12))
    << "got\n" << tcp_T_sensor.rotation();

  // The sensor origin is 0.107 + 0.1034 m back up the TCP's z axis.
  EXPECT_NEAR(tcp_T_sensor.translation().z(), -(0.107 + 0.1034), 1e-12);
  EXPECT_NEAR(tcp_T_sensor.translation().head<2>().norm(), 0.0, 1e-12);

  // A force pushing along the SENSOR's +x reads, along the TCP's axes, as
  // equal parts +x and +y: the 45 degree split that the old code missed.
  const Eigen::Vector3d f_tcp = tcp_T_sensor.rotation() * Eigen::Vector3d::UnitX();
  EXPECT_NEAR(f_tcp.x(), std::sqrt(0.5), 1e-12);
  EXPECT_NEAR(f_tcp.y(), std::sqrt(0.5), 1e-12);
}

TEST(KinematicsModel, FixedTransformOfAFrameToItselfIsIdentity)
{
  Eigen::Isometry3d t;
  std::string error;
  ASSERT_TRUE(KinematicsModel::fixedTransform(buildWristUrdf(), "tcp", "tcp", t, error));
  EXPECT_TRUE(t.isApprox(Eigen::Isometry3d::Identity()));
}

TEST(KinematicsModel, FixedTransformRefusesAMovableJoint)
{
  // link6 -> tcp crosses the revolute j7, so the answer would depend on q7.
  Eigen::Isometry3d t;
  std::string error;
  EXPECT_FALSE(KinematicsModel::fixedTransform(buildWristUrdf(), "link6", "tcp", t, error));
  EXPECT_NE(error.find("movable"), std::string::npos) << error;
}

TEST(KinematicsModel, FixedTransformRefusesAMissingFrame)
{
  Eigen::Isometry3d t;
  std::string error;
  EXPECT_FALSE(KinematicsModel::fixedTransform(buildWristUrdf(), "tcp", "nope", t, error));
  EXPECT_FALSE(error.empty());
}
