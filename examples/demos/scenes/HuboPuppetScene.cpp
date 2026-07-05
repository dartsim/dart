/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

// Ported from examples/hubo_puppet: a purely kinematic DRC-Hubo whole-body IK
// puppet, analogous to atlas_puppet but with six end effectors (adding
// l_peg/r_peg on the wrists) and custom closed-form analytical IK
// (HuboArmIK/HuboLegIK, derived by Matt Zucker) instead of atlas_puppet's
// pure Jacobian-based IK for the limbs.
//
// Deviations from the original: same render-rate-via-renderPanel treatment
// as atlas_puppet/wam_ikfast (see atlas_puppet's file comment -- this host
// has no per-render-frame hook, so movement + solveAndApply runs once per
// rendered frame at the top of renderPanel, exactly as customPreRefresh did
// at render rate). viewer->allowSimulation(false) applied in onActivate,
// restored on teardown; gravity zeroed so a forced step (Step button /
// --cycle-scenes / --headless, all of which call world->step() directly,
// bypassing allowSimulation) cannot make the unactuated robot collapse.
// WASDQEFZ (+ uppercase) and Left-Shift-held amplify need KEYDOWN and
// KEYUP, so -- like atlas_puppet -- they are handled by a small
// osgGA::GUIEventHandler rather than KeyActions; 1-6 target toggle, t
// reset, x/c support toggle, and p print (routed to the Diagnostics log
// instead of stdout) are ordinary KeyActions. drchubo's asset path uses the
// DART_DATA_PATH compile-time macro (not a dart://sample URI) exactly as
// the original does -- this macro is already used by wam_ikfast (B3) in
// this same catalog, so no new asset-resolution plumbing is needed. The
// skeleton clone-after-EE-setup (`cloneSkeleton("hubo_copy")`) is kept: the
// original explicitly does this to exercise IK/EndEffector cloning, per the
// parity portRisk ("keep or the demo semantics change").

#include "Scenes.hpp"

#include <dart/gui/osg/osg.hpp>

#include <dart/utils/urdf/urdf.hpp>

#include <dart/dart.hpp>

#include <complex>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace dart_demos {

namespace {

using dart::dynamics::BodyNode;
using dart::dynamics::BodyNodePtr;
using dart::dynamics::EndEffector;
using dart::dynamics::Frame;
using dart::dynamics::InverseKinematics;
using dart::dynamics::JacobianNode;
using dart::dynamics::SkeletonPtr;
using dart::dynamics::WeakBodyNodePtr;
using dart::math::constantsd;

constexpr double kDisplayElevation = 0.05;

//==============================================================================
/// Ported verbatim from the original (identical to atlas_puppet's copy).
class RelaxedPosture : public dart::optimizer::Function
{
public:
  RelaxedPosture(
      const Eigen::VectorXd& idealPosture,
      const Eigen::VectorXd& lower,
      const Eigen::VectorXd& upper,
      const Eigen::VectorXd& weights)
    : enforceIdealPosture(false),
      mIdeal(idealPosture),
      mLower(lower),
      mUpper(upper),
      mWeights(weights)
  {
    mResultVector.setZero(mIdeal.size());
  }

  double eval(const Eigen::VectorXd& x) override
  {
    computeResultVector(x);
    return 0.5 * mResultVector.dot(mResultVector);
  }

  void evalGradient(
      const Eigen::VectorXd& x, Eigen::Map<Eigen::VectorXd> grad) override
  {
    computeResultVector(x);
    grad.setZero();
    const int smaller = std::min<int>(mResultVector.size(), grad.size());
    for (int i = 0; i < smaller; ++i)
      grad[i] = mResultVector[i];
  }

  void computeResultVector(const Eigen::VectorXd& x)
  {
    mResultVector.setZero();
    if (enforceIdealPosture) {
      for (int i = 0; i < x.size() && i < mIdeal.size(); ++i)
        mResultVector[i] = mWeights[i] * (x[i] - mIdeal[i]);
    } else {
      for (int i = 0; i < x.size() && i < mIdeal.size(); ++i) {
        if (x[i] < mLower[i])
          mResultVector[i] = mWeights[i] * (x[i] - mLower[i]);
        else if (mUpper[i] < x[i])
          mResultVector[i] = mWeights[i] * (x[i] - mUpper[i]);
      }
    }
  }

  bool enforceIdealPosture;

protected:
  Eigen::VectorXd mResultVector;
  Eigen::VectorXd mIdeal;
  Eigen::VectorXd mLower;
  Eigen::VectorXd mUpper;
  Eigen::VectorXd mWeights;
};

//==============================================================================
inline bool checkDist(Eigen::Vector3d& p, double a, double b)
{
  const double d = p.norm();
  const double dmax = a + b;
  const double dmin = std::abs(a - b);

  if (d > dmax) {
    p *= dmax / d;
    return false;
  } else if (d < dmin) {
    p *= dmin / d;
    return false;
  }
  return true;
}

//==============================================================================
inline void clampSinCos(double& sincos, bool& valid)
{
  if (sincos < -1) {
    valid = false;
    sincos = -1;
  } else if (sincos > 1) {
    valid = false;
    sincos = 1;
  }
}

//==============================================================================
inline Eigen::Vector3d flipEuler3Axis(const Eigen::Vector3d& u)
{
  Eigen::Vector3d v;
  v[0] = u[0] - constantsd::pi();
  v[1] = constantsd::pi() - u[1];
  v[2] = u[2] - constantsd::pi();
  return v;
}

//==============================================================================
/// HuboArmIK is based on the derivation of Hubo's arm IK by Matt Zucker;
/// ported verbatim from the original.
class HuboArmIK : public InverseKinematics::Analytical
{
public:
  HuboArmIK(
      InverseKinematics* ik,
      const std::string& baseLinkName,
      const Analytical::Properties& properties = Analytical::Properties())
    : Analytical(ik, "HuboArmIK_" + baseLinkName, properties),
      configured(false),
      mBaseLinkName(baseLinkName)
  {
  }

  std::unique_ptr<GradientMethod> clone(InverseKinematics* newIk) const override
  {
    return std::make_unique<HuboArmIK>(
        newIk, mBaseLinkName, getAnalyticalProperties());
  }

  const std::vector<Solution>& computeSolutions(
      const Eigen::Isometry3d& desiredBodyTf) override
  {
    mSolutions.clear();
    mSolutions.reserve(8);

    if (!configured) {
      configure();
      if (!configured) {
        dtwarn << "[HuboArmIK::computeSolutions] This analytical IK was not "
               << "able to configure properly, so it will not be able to "
               << "compute solutions\n";
        return mSolutions;
      }
    }

    const BodyNodePtr& base = mBaseLink.lock();
    if (nullptr == base) {
      dterr << "[HuboArmIK::computeSolutions] Attempting to perform an IK on "
            << "a limb that no longer exists [" << getMethodName() << "]!\n";
      DART_ASSERT(false);
      return mSolutions;
    }

    if (nullptr == mWristEnd) {
      dterr << "[HuboArmIK::computeSolutions] Attempting to perform IK "
            << "without a wrist!\n";
      DART_ASSERT(false);
      return mSolutions;
    }

    const std::size_t SP = 0;
    const std::size_t SR = 1;
    const std::size_t SY = 2;
    const std::size_t EP = 3;
    const std::size_t WY = 4;
    const std::size_t WP = 5;

    const SkeletonPtr& skel = base->getSkeleton();

    Eigen::Isometry3d B
        = base->getParentBodyNode()->getWorldTransform().inverse()
          * desiredBodyTf * mWristEnd->getTransform(mIK->getNode());

    Eigen::Isometry3d shoulderFromWrist = shoulderTf.inverse() * B;
    Eigen::Vector3d p = shoulderFromWrist.inverse().translation();

    const double a2 = L5 * L5 + L4 * L4;
    const double b2 = L3 * L3 + L4 * L4;
    const double a = std::sqrt(a2);
    const double b = std::sqrt(b2);

    const double alpha = std::atan2(L5, L4);
    const double beta = std::atan2(L3, L4);

    const bool startValid = checkDist(p, a, b);

    const double c2Sq = p.dot(p);
    const double x = p.x();
    const double y = p.y();
    const double z = p.z();

    for (std::size_t i = 0; i < 8; ++i) {
      const int flipEP = alternatives(i, 0);
      const int incWY = alternatives(i, 1);
      const int flipShoulder = alternatives(i, 2);

      Eigen::Vector6d testQ;
      bool isValid = startValid;

      double cosGamma = (a2 + b2 - c2Sq) / (2 * a * b);
      clampSinCos(cosGamma, isValid);

      const double gamma = flipEP * std::acos(cosGamma);
      const double theta3 = alpha + beta + gamma - 2 * constantsd::pi();
      testQ(EP) = theta3;

      const double c3 = std::cos(theta3);
      const double s3 = std::sin(theta3);

      const double number = -y;
      const double denom = (-L4 * c3 - L3 * s3 + L4);

      double s2;
      double theta2;
      if (std::abs(denom) < zeroSize) {
        isValid = false;
        const double& prevWY = skel->getPosition(mDofs[WY]);
        theta2 = incWY ? prevWY : constantsd::pi() - prevWY;
        s2 = std::sin(theta2);
      } else {
        s2 = number / denom;
        clampSinCos(s2, isValid);
        theta2 = incWY ? constantsd::pi() - std::asin(s2) : std::asin(s2);
      }
      testQ(WY) = theta2;

      const double c2 = std::cos(theta2);
      const double r = L4 * c2 - L4 * c2 * c3 - L3 * s3 * c2;
      const double q = -L4 * s3 + L3 * c3 + L5;
      const double det = -(q * q + r * r);
      if (std::abs(det) < zeroSize)
        isValid = false;
      const double k = det < 0 ? -1 : 1;

      const double ks1 = k * (q * x - r * z);
      const double kc1 = k * (-r * x - q * z);
      const double theta1 = std::atan2(ks1, kc1);
      testQ(WP) = theta1;

      const Eigen::Quaterniond rLower
          = Eigen::Quaterniond(
                Eigen::AngleAxisd(testQ(EP), Eigen::Vector3d::UnitY()))
            * Eigen::Quaterniond(
                Eigen::AngleAxisd(testQ(WY), Eigen::Vector3d::UnitZ()))
            * Eigen::Quaterniond(
                Eigen::AngleAxisd(testQ(WP), Eigen::Vector3d::UnitY()));

      const Eigen::Matrix3d rUpper = B.rotation() * rLower.inverse().matrix();
      Eigen::Vector3d euler = rUpper.eulerAngles(1, 0, 2);
      if (flipShoulder)
        euler = flipEuler3Axis(euler);

      testQ(SP) = euler[0];
      testQ(SR) = euler[1];
      testQ(SY) = euler[2];

      for (std::size_t j = 0; j < 6; ++j) {
        testQ[j] = dart::math::wrapToPi(testQ[j]);
        if (std::abs(testQ[j]) < zeroSize)
          testQ[j] = 0.0;
      }

      const int validity = isValid ? VALID : OUT_OF_REACH;
      mSolutions.push_back(Solution(testQ, validity));
    }

    checkSolutionJointLimits();
    return mSolutions;
  }

  const std::vector<std::size_t>& getDofs() const override
  {
    if (!configured)
      configure();
    return mDofs;
  }

  const double zeroSize = 1e-8;

protected:
  void configure() const
  {
    configured = false;

    mBaseLink = mIK->getNode()->getSkeleton()->getBodyNode(mBaseLinkName);
    BodyNode* base = mBaseLink.lock();
    if (nullptr == base) {
      dterr << "[HuboArmIK::configure] base link is a nullptr\n";
      DART_ASSERT(false);
      return;
    }

    const SkeletonPtr& skel = base->getSkeleton();
    const BodyNodePtr& pelvis = skel->getBodyNode("Body_TSY");
    if (nullptr == pelvis) {
      dterr << "[HuboArmIK::configure] Could not find Hubo's pelvis "
            << "(Body_TSY)\n";
      DART_ASSERT(false);
      return;
    }

    Eigen::Vector6d savedQ;
    dart::dynamics::DegreeOfFreedom* dofs[6];
    BodyNode* bn = base;
    for (std::size_t i = 0; i < 6; ++i) {
      dart::dynamics::Joint* joint = bn->getParentJoint();
      if (joint->getNumDofs() != 1) {
        dterr << "[HuboArmIK::configure] Invalid number of DOFs ("
              << joint->getNumDofs() << ") in the Joint [" << joint->getName()
              << "]\n";
        DART_ASSERT(false);
        return;
      }
      dofs[i] = joint->getDof(0);
      savedQ[i] = dofs[i]->getPosition();
      dofs[i]->setPosition(0.0);
      bn = bn->getChildBodyNode(0);
    }

    BodyNode* elbow = dofs[3]->getChildBodyNode();
    L3 = std::abs(
        elbow->getTransform(dofs[2]->getParentBodyNode()).translation()[2]);
    L4 = std::abs(
        elbow->getTransform(dofs[3]->getParentBodyNode()).translation()[0]);

    BodyNode* wrist = dofs[5]->getChildBodyNode();
    const Eigen::Isometry3d wristTf = wrist->getTransform(elbow);
    L5 = std::abs(wristTf.translation()[2]);

    shoulderTf = Eigen::Isometry3d::Identity();
    shoulderTf.translate(
        dofs[3]->getParentBodyNode()->getTransform(pelvis).translation()[0]
        * Eigen::Vector3d::UnitX());
    shoulderTf.translate(
        dofs[2]->getParentBodyNode()->getTransform(pelvis).translation()[1]
        * Eigen::Vector3d::UnitY());
    shoulderTf.translate(
        dofs[2]->getParentBodyNode()->getTransform(pelvis).translation()[2]
        * Eigen::Vector3d::UnitZ());

    mWristEnd = dofs[5]->getChildBodyNode();

    alternatives << 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 0, 0, -1, 1, 1, -1, 1, 0, -1,
        0, 1, -1, 0, 0;

    for (std::size_t i = 0; i < 6; ++i) {
      dofs[i]->setPosition(savedQ[i]);
      mDofs.push_back(dofs[i]->getIndexInSkeleton());
    }

    configured = true;
  }

  mutable bool configured;
  mutable Eigen::Isometry3d shoulderTf;
  mutable double L3 = 0.0;
  mutable double L4 = 0.0;
  mutable double L5 = 0.0;
  mutable Eigen::Matrix<int, 8, 3> alternatives;
  mutable std::vector<std::size_t> mDofs;
  std::string mBaseLinkName;
  mutable WeakBodyNodePtr mBaseLink;
  mutable JacobianNode* mWristEnd = nullptr;
};

//==============================================================================
/// HuboLegIK; baseLink should be Body_LHY or Body_RHY. Ported verbatim from
/// the original.
class HuboLegIK : public InverseKinematics::Analytical
{
public:
  HuboLegIK(
      InverseKinematics* ik,
      const std::string& baseLinkName,
      const Analytical::Properties& properties = Analytical::Properties())
    : Analytical(ik, "HuboLegIK_" + baseLinkName, properties),
      configured(false),
      mBaseLinkName(baseLinkName)
  {
  }

  std::unique_ptr<GradientMethod> clone(InverseKinematics* newIk) const override
  {
    return std::make_unique<HuboLegIK>(
        newIk, mBaseLinkName, getAnalyticalProperties());
  }

  const std::vector<Solution>& computeSolutions(
      const Eigen::Isometry3d& desiredBodyTf) override
  {
    mSolutions.clear();
    mSolutions.reserve(8);

    if (!configured) {
      configure();
      if (!configured) {
        dtwarn << "[HuboLegIK::computeSolutions] This analytical IK was not "
               << "able to configure properly, so it will not be able to "
               << "compute solutions\n";
        return mSolutions;
      }
    }

    const BodyNodePtr& base = mBaseLink.lock();
    if (nullptr == base) {
      dterr << "[HuboLegIK::computeSolutions] Attempting to perform IK on a "
            << "limb that no longer exists!\n";
      DART_ASSERT(false);
      return mSolutions;
    }

    const Eigen::Isometry3d B
        = (base->getParentBodyNode()->getWorldTransform() * waist).inverse()
          * desiredBodyTf * footTfInv;
    const Eigen::Isometry3d Binv = B.inverse();

    const double nx = Binv(0, 0);
    const double sx = Binv(0, 1);
    const double ax = Binv(0, 2);
    const double px = Binv(0, 3);
    const double ny = Binv(1, 0);
    const double sy = Binv(1, 1);
    const double ay = Binv(1, 2);
    const double py = Binv(1, 3);
    const double az = Binv(2, 2);
    const double pz = Binv(2, 3);

    for (std::size_t i = 0; i < 8; ++i) {
      bool isValid = true;

      double q1, q2, q3, q4, q5, q6;
      double s2, s4, s6;
      double c2, c4, c5, c6;
      double c45;
      std::complex<double> radical;
      std::complex<double> sqrtRadical;

      c4 = ((px + L6) * (px + L6) - L4 * L4 - L5 * L5 + py * py + pz * pz)
           / (2 * L4 * L5);
      radical = 1 - c4 * c4;
      sqrtRadical = std::sqrt(radical);
      if (sqrtRadical.imag() != 0)
        isValid = false;
      q4 = std::atan2(alternatives(i, 0) * sqrtRadical.real(), c4);

      s4 = std::sin(q4);
      const double psi = std::atan2(s4 * L4, c4 * L4 + L5);
      radical = (px + L6) * (px + L6) + py * py;
      sqrtRadical = std::sqrt(radical);
      if (sqrtRadical.imag() != 0)
        isValid = false;

      q5 = dart::math::wrapToPi(
          std::atan2(-pz, alternatives(i, 1) * sqrtRadical.real()) - psi);

      q6 = std::atan2(py, -(px + L6));
      c45 = std::cos(q4 + q5);
      c5 = std::cos(q5);
      if (c45 * L4 + c5 * L5 < 0)
        q6 = dart::math::wrapToPi(q6 + constantsd::pi());

      s6 = std::sin(q6);
      c6 = std::cos(q6);

      s2 = c6 * ay + s6 * ax;
      radical = 1 - s2 * s2;
      sqrtRadical = std::sqrt(radical);
      if (sqrtRadical.imag() != 0)
        isValid = false;
      q2 = std::atan2(s2, alternatives(i, 2) * sqrtRadical.real());

      q1 = std::atan2(c6 * sy + s6 * sx, c6 * ny + s6 * nx);
      c2 = std::cos(q2);
      if (c2 < 0)
        q1 = dart::math::wrapToPi(q1 + constantsd::pi());

      const double q345 = std::atan2(-az / c2, -(c6 * ax - s6 * ay) / c2);
      q3 = dart::math::wrapToPi(q345 - q4 - q5);

      Eigen::Vector6d testQ;
      testQ[0] = q1;
      testQ[1] = q2;
      testQ[2] = q3;
      testQ[3] = q4;
      testQ[4] = q5;
      testQ[5] = q6;

      for (int k = 0; k < testQ.size(); ++k)
        if (std::abs(testQ[k]) < zeroSize)
          testQ[k] = 0;

      const int validity = isValid ? VALID : OUT_OF_REACH;
      mSolutions.push_back(Solution(testQ, validity));
    }

    checkSolutionJointLimits();
    return mSolutions;
  }

  const std::vector<std::size_t>& getDofs() const override
  {
    if (!configured)
      configure();
    return mDofs;
  }

  const double zeroSize = 1e-8;

protected:
  void configure() const
  {
    configured = false;

    mBaseLink = mIK->getNode()->getSkeleton()->getBodyNode(mBaseLinkName);
    BodyNode* base = mBaseLink.lock();
    if (nullptr == base) {
      dterr << "[HuboLegIK::configure] base link is a nullptr\n";
      DART_ASSERT(false);
      return;
    }

    const SkeletonPtr& skel = mIK->getNode()->getSkeleton();
    BodyNode* pelvis = skel->getBodyNode("Body_TSY");
    if (nullptr == pelvis) {
      dterr << "[HuboLegIK::configure] Could not find Hubo's pelvis "
            << "(Body_TSY)\n";
      DART_ASSERT(false);
      return;
    }

    Eigen::Vector6d savedQ;
    dart::dynamics::DegreeOfFreedom* dofs[6];
    BodyNode* bn = base;
    for (std::size_t i = 0; i < 6; ++i) {
      dart::dynamics::Joint* joint = bn->getParentJoint();
      if (joint->getNumDofs() != 1) {
        dterr << "[HuboLegIK::configure] Invalid number of DOFs ("
              << joint->getNumDofs() << ") in the Joint [" << joint->getName()
              << "]\n";
        DART_ASSERT(false);
        return;
      }
      dofs[i] = joint->getDof(0);
      savedQ[i] = dofs[i]->getPosition();
      dofs[i]->setPosition(0.0);
      if (bn->getNumChildBodyNodes() > 0)
        bn = bn->getChildBodyNode(0);
    }

    L4 = std::abs(
        dofs[3]->getChildBodyNode()->getRelativeTransform().translation()[2]);
    L5 = std::abs(
        dofs[4]->getChildBodyNode()->getRelativeTransform().translation()[2]);
    // This offset is handled by footTfInv instead.
    L6 = 0.0;

    hipRotation = Eigen::Isometry3d::Identity();
    hipRotation.rotate(Eigen::AngleAxisd(
        90.0 * constantsd::pi() / 180.0, Eigen::Vector3d::UnitZ()));

    waist = dofs[2]->getChildBodyNode()->getTransform(
                dofs[0]->getParentBodyNode())
            * hipRotation;

    footTfInv = Eigen::Isometry3d::Identity();
    footTfInv.rotate(Eigen::AngleAxisd(
        -90.0 * constantsd::pi() / 180.0, Eigen::Vector3d::UnitY()));
    footTfInv
        = footTfInv * mIK->getNode()->getTransform(dofs[5]->getChildBodyNode());
    footTfInv = footTfInv.inverse();

    alternatives << 1, 1, 1, 1, 1, -1, 1, -1, 1, 1, -1, -1, -1, 1, 1, -1, 1, -1,
        -1, -1, 1, -1, -1, -1;

    for (std::size_t i = 0; i < 6; ++i) {
      dofs[i]->setPosition(savedQ[i]);
      mDofs.push_back(dofs[i]->getIndexInSkeleton());
    }

    configured = true;
  }

  mutable double L4 = 0.0;
  mutable double L5 = 0.0;
  mutable double L6 = 0.0;
  mutable Eigen::Isometry3d waist;
  mutable Eigen::Isometry3d hipRotation;
  mutable Eigen::Isometry3d footTfInv;
  mutable Eigen::Matrix<int, 8, 3> alternatives;
  mutable std::vector<std::size_t> mDofs;
  mutable bool configured;
  std::string mBaseLinkName;
  mutable WeakBodyNodePtr mBaseLink;
};

//==============================================================================
enum MoveComponent
{
  MoveQ = 0,
  MoveW,
  MoveE,
  MoveA,
  MoveS,
  MoveD,
  MoveF,
  MoveZ,
  NumMoveComponents
};

//==============================================================================
/// Per-instance state captured by this scene's renderPanel/key-action lambdas
/// and the move handler.
struct HuboPuppetState
{
  SkeletonPtr hubo;

  Eigen::VectorXd restConfig;
  std::vector<bool> moveComponents
      = std::vector<bool>(NumMoveComponents, false);
  bool amplifyMovement = false;

  std::vector<std::size_t> endEffectorIndex;
  std::vector<std::pair<Eigen::Vector6d, Eigen::Vector6d>> defaultBounds;
  dart::common::aligned_vector<Eigen::Isometry3d> defaultTargetTf;
  std::vector<bool> constraintActive;

  std::shared_ptr<RelaxedPosture> posture;
  std::shared_ptr<dart::constraint::BalanceConstraint> balance;

  std::function<void(const std::string&)> log;
};

//==============================================================================
SkeletonPtr createGround()
{
  auto ground = dart::dynamics::Skeleton::create("ground");
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  const double thickness = 0.01;
  tf.translation() = Eigen::Vector3d(0, 0, -thickness / 2.0);
  dart::dynamics::WeldJoint::Properties joint;
  joint.mT_ParentBodyToJoint = tf;
  ground->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(nullptr, joint);
  auto groundShape = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(10, 10, thickness));
  auto* shapeNode = ground->getBodyNode(0)
                        ->createShapeNodeWith<
                            dart::dynamics::VisualAspect,
                            dart::dynamics::CollisionAspect,
                            dart::dynamics::DynamicsAspect>(groundShape);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue(0.2));
  return ground;
}

//==============================================================================
SkeletonPtr createHubo()
{
  dart::utils::DartLoader loader;
  loader.addPackageDirectory("drchubo", DART_DATA_PATH "/urdf/drchubo");
  auto hubo = loader.parseSkeleton(DART_DATA_PATH "/urdf/drchubo/drchubo.urdf");
  if (!hubo)
    return nullptr;

  for (std::size_t i = 0; i < hubo->getNumBodyNodes(); ++i) {
    auto* bn = hubo->getBodyNode(i);
    if (bn->getName().substr(0, 7) == "Body_LF"
        || bn->getName().substr(0, 7) == "Body_RF") {
      bn->remove();
      --i;
    }
  }

  return hubo;
}

//==============================================================================
double toRadian(double deg)
{
  return deg * constantsd::pi() / 180.0;
}

//==============================================================================
void setStartupConfiguration(const SkeletonPtr& hubo)
{
  hubo->getDof("LHP")->setPosition(toRadian(-45.0));
  hubo->getDof("LKP")->setPosition(toRadian(90.0));
  hubo->getDof("LAP")->setPosition(toRadian(-45.0));

  hubo->getDof("RHP")->setPosition(toRadian(-45.0));
  hubo->getDof("RKP")->setPosition(toRadian(90.0));
  hubo->getDof("RAP")->setPosition(toRadian(-45.0));

  hubo->getDof("LSP")->setPosition(toRadian(30.0));
  hubo->getDof("LEP")->setPosition(toRadian(-120.0));

  hubo->getDof("RSP")->setPosition(toRadian(30.0));
  hubo->getDof("REP")->setPosition(toRadian(-120.0));

  hubo->getDof("LSY")->setPositionLowerLimit(toRadian(-90.0));
  hubo->getDof("LSY")->setPositionUpperLimit(toRadian(90.0));
  hubo->getDof("LWY")->setPositionLowerLimit(toRadian(-90.0));
  hubo->getDof("LWY")->setPositionUpperLimit(toRadian(90.0));

  hubo->getDof("RSY")->setPositionLowerLimit(toRadian(-90.0));
  hubo->getDof("RSY")->setPositionUpperLimit(toRadian(90.0));
  hubo->getDof("RWY")->setPositionLowerLimit(toRadian(-90.0));
  hubo->getDof("RWY")->setPositionUpperLimit(toRadian(90.0));
}

//==============================================================================
void setupEndEffectors(const SkeletonPtr& hubo)
{
  Eigen::VectorXd rootJointWeights = 0.01 * Eigen::VectorXd::Ones(7);
  const double extraErrorClamp = 0.1;

  Eigen::Vector3d linearBounds
      = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d angularBounds
      = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());

  Eigen::Isometry3d tfHand(Eigen::Isometry3d::Identity());
  tfHand.translate(Eigen::Vector3d(0.0, 0.0, -0.09));

  auto* lHand = hubo->getBodyNode("Body_LWR")->createEndEffector("l_hand");
  lHand->setDefaultRelativeTransform(tfHand, true);
  auto lhTarget = std::make_shared<dart::gui::osg::InteractiveFrame>(
      Frame::World(), "lh_target");
  lHand->getIK(true)->setTarget(lhTarget);
  lHand->getIK()->useWholeBody();
  lHand->getIK()->setGradientMethod<HuboArmIK>("Body_LSP");
  lHand->getIK()->getAnalytical()->setExtraDofUtilization(
      InverseKinematics::Analytical::POST_ANALYTICAL);
  lHand->getIK()->getAnalytical()->setExtraErrorLengthClamp(extraErrorClamp);
  lHand->getIK()->getGradientMethod().setComponentWeights(rootJointWeights);
  lHand->getIK()->getErrorMethod().setLinearBounds(-linearBounds, linearBounds);
  lHand->getIK()->getErrorMethod().setAngularBounds(
      -angularBounds, angularBounds);

  auto* rHand = hubo->getBodyNode("Body_RWR")->createEndEffector("r_hand");
  rHand->setDefaultRelativeTransform(tfHand, true);
  auto rhTarget = std::make_shared<dart::gui::osg::InteractiveFrame>(
      Frame::World(), "rh_target");
  rHand->getIK(true)->setTarget(rhTarget);
  rHand->getIK()->useWholeBody();
  rHand->getIK()->setGradientMethod<HuboArmIK>("Body_RSP");
  rHand->getIK()->getAnalytical()->setExtraDofUtilization(
      InverseKinematics::Analytical::POST_ANALYTICAL);
  rHand->getIK()->getAnalytical()->setExtraErrorLengthClamp(extraErrorClamp);
  rHand->getIK()->getGradientMethod().setComponentWeights(rootJointWeights);
  rHand->getIK()->getErrorMethod().setLinearBounds(-linearBounds, linearBounds);
  rHand->getIK()->getErrorMethod().setAngularBounds(
      -angularBounds, angularBounds);

  dart::math::SupportGeometry footSupport;
  footSupport.push_back(Eigen::Vector3d(-0.08, 0.05, 0.0));
  footSupport.push_back(Eigen::Vector3d(-0.18, 0.05, 0.0));
  footSupport.push_back(Eigen::Vector3d(-0.18, -0.05, 0.0));
  footSupport.push_back(Eigen::Vector3d(-0.08, -0.05, 0.0));

  Eigen::Isometry3d tfFoot(Eigen::Isometry3d::Identity());
  const double groundDist = 0.01;
  tfFoot.translation() = Eigen::Vector3d(0.14, 0.0, -0.136 + groundDist);

  linearBounds[2] = 1e-8;
  const Eigen::Vector3d groundOffset = groundDist * Eigen::Vector3d::UnitZ();
  angularBounds[0] = 1e-8;
  angularBounds[1] = 1e-8;

  auto* lFoot = hubo->getBodyNode("Body_LAR")->createEndEffector("l_foot");
  lFoot->setDefaultRelativeTransform(tfFoot, true);
  auto lfTarget = std::make_shared<dart::gui::osg::InteractiveFrame>(
      Frame::World(), "lf_target");
  lFoot->getIK(true)->setTarget(lfTarget);
  lFoot->getIK()->setHierarchyLevel(1);
  lFoot->getIK()->getErrorMethod().setLinearBounds(
      -linearBounds + groundOffset, linearBounds + groundOffset);
  lFoot->getIK()->getErrorMethod().setAngularBounds(
      -angularBounds, angularBounds);
  lFoot->getIK()->setGradientMethod<HuboLegIK>("Body_LHY");
  lFoot->getSupport(true)->setGeometry(footSupport);
  lFoot->getSupport()->setActive();

  auto* rFoot = hubo->getBodyNode("Body_RAR")->createEndEffector("r_foot");
  rFoot->setDefaultRelativeTransform(tfFoot, true);
  auto rfTarget = std::make_shared<dart::gui::osg::InteractiveFrame>(
      Frame::World(), "rf_target");
  rFoot->getIK(true)->setTarget(rfTarget);
  rFoot->getIK()->setHierarchyLevel(1);
  rFoot->getIK()->getErrorMethod().setLinearBounds(
      -linearBounds + groundOffset, linearBounds + groundOffset);
  rFoot->getIK()->getErrorMethod().setAngularBounds(
      -angularBounds, angularBounds);
  rFoot->getIK()->setGradientMethod<HuboLegIK>("Body_RHY");
  rFoot->getSupport(true)->setGeometry(footSupport);
  rFoot->getSupport()->setActive();

  dart::math::SupportGeometry pegSupport;
  pegSupport.push_back(Eigen::Vector3d::Zero());

  linearBounds
      = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  angularBounds = linearBounds;

  Eigen::Isometry3d tfPeg(Eigen::Isometry3d::Identity());
  tfPeg.translation() = Eigen::Vector3d(0.0, 0.0, 0.09);

  auto* lPeg = hubo->getBodyNode("Body_LWP")->createEndEffector("l_peg");
  lPeg->setDefaultRelativeTransform(tfPeg, true);
  auto lpTarget = std::make_shared<dart::gui::osg::InteractiveFrame>(
      Frame::World(), "lp_target");
  lPeg->getIK(true)->setTarget(lpTarget);
  lPeg->getIK()->setGradientMethod<HuboArmIK>("Body_LSP");
  lPeg->getIK()->getErrorMethod().setLinearBounds(-linearBounds, linearBounds);
  lPeg->getIK()->getErrorMethod().setAngularBounds(
      -angularBounds, angularBounds);
  lPeg->getSupport(true)->setGeometry(pegSupport);

  auto* rPeg = hubo->getBodyNode("Body_RWP")->createEndEffector("r_peg");
  rPeg->setDefaultRelativeTransform(tfPeg, true);
  auto rpTarget = std::make_shared<dart::gui::osg::InteractiveFrame>(
      Frame::World(), "rp_target");
  rPeg->getIK(true)->setTarget(rpTarget);
  rPeg->getIK()->setGradientMethod<HuboArmIK>("Body_RSP");
  rPeg->getIK()->getErrorMethod().setLinearBounds(-linearBounds, linearBounds);
  rPeg->getIK()->getErrorMethod().setAngularBounds(
      -angularBounds, angularBounds);
  rPeg->getSupport(true)->setGeometry(pegSupport);

  const double heightChange
      = -rFoot->getWorldTransform().translation()[2] + groundDist;
  hubo->getDof("rootJoint_pos_z")->setPosition(heightChange);

  lFoot->getIK()->getTarget()->setTransform(lFoot->getTransform());
  rFoot->getIK()->getTarget()->setTransform(rFoot->getTransform());
}

//==============================================================================
void setupWholeBodySolver(const SkeletonPtr& hubo)
{
  auto solver
      = std::dynamic_pointer_cast<dart::optimizer::GradientDescentSolver>(
          hubo->getIK(true)->getSolver());

  const std::size_t nDofs = hubo->getNumDofs();
  const double defaultWeight = 0.01;
  Eigen::VectorXd weights = defaultWeight * Eigen::VectorXd::Ones(nDofs);
  weights[2] = 0.0;
  weights[3] = 0.0;
  weights[4] = 0.0;

  Eigen::VectorXd lowerPosture = Eigen::VectorXd::Constant(
      nDofs, -std::numeric_limits<double>::infinity());
  lowerPosture[0] = -0.35;
  lowerPosture[1] = -0.35;
  lowerPosture[5] = 0.55;

  Eigen::VectorXd upperPosture = Eigen::VectorXd::Constant(
      nDofs, std::numeric_limits<double>::infinity());
  upperPosture[0] = 0.35;
  upperPosture[1] = 0.50;
  upperPosture[5] = 0.95;

  auto objective = std::make_shared<RelaxedPosture>(
      hubo->getPositions(), lowerPosture, upperPosture, weights);
  hubo->getIK()->setObjective(objective);

  auto balance
      = std::make_shared<dart::constraint::BalanceConstraint>(hubo->getIK());
  hubo->getIK()->getProblem()->addEqConstraint(balance);
  balance->setErrorMethod(dart::constraint::BalanceConstraint::FROM_CENTROID);
  balance->setBalanceMethod(dart::constraint::BalanceConstraint::SHIFT_SUPPORT);

  solver->setNumMaxIterations(5);
}

//==============================================================================
/// Composes the WASDQEFZ (+ Shift-amplify) root-motion delta and re-solves
/// whole-body IK; run once per rendered frame (see the file comment).
void updatePuppet(HuboPuppetState& state)
{
  bool anyMovement = false;
  for (bool move : state.moveComponents)
    anyMovement = anyMovement || move;

  if (anyMovement) {
    const Eigen::Isometry3d oldTf
        = state.hubo->getBodyNode(0)->getWorldTransform();
    Eigen::Isometry3d newTf = Eigen::Isometry3d::Identity();

    Eigen::Vector3d forward = oldTf.linear().col(0);
    forward[2] = 0.0;
    forward = forward.norm() > 1e-10 ? forward.normalized()
                                     : Eigen::Vector3d::Zero();

    Eigen::Vector3d left = oldTf.linear().col(1);
    left[2] = 0.0;
    left = left.norm() > 1e-10 ? left.normalized() : Eigen::Vector3d::Zero();

    const Eigen::Vector3d up = Eigen::Vector3d::UnitZ();

    double linearStep = 0.01;
    double elevationStep = 0.2 * linearStep;
    double rotationalStep = 2.0 * constantsd::pi() / 180.0;
    if (state.amplifyMovement) {
      linearStep *= 2.0;
      elevationStep *= 2.0;
      rotationalStep *= 2.0;
    }

    if (state.moveComponents[MoveW])
      newTf.translate(linearStep * forward);
    if (state.moveComponents[MoveS])
      newTf.translate(-linearStep * forward);
    if (state.moveComponents[MoveA])
      newTf.translate(linearStep * left);
    if (state.moveComponents[MoveD])
      newTf.translate(-linearStep * left);
    if (state.moveComponents[MoveF])
      newTf.translate(elevationStep * up);
    if (state.moveComponents[MoveZ])
      newTf.translate(-elevationStep * up);
    if (state.moveComponents[MoveQ])
      newTf.rotate(Eigen::AngleAxisd(rotationalStep, up));
    if (state.moveComponents[MoveE])
      newTf.rotate(Eigen::AngleAxisd(-rotationalStep, up));

    newTf.pretranslate(oldTf.translation());
    newTf.rotate(oldTf.rotation());

    state.hubo->getJoint(0)->setPositions(
        dart::dynamics::FreeJoint::convertToPositions(newTf));
  }

  state.hubo->getIK(true)->solveAndApply(true);
}

//==============================================================================
/// Ported from the original's InputHandler: WASDQEFZ (upper+lowercase) set/
/// clear per-axis move flags on KEYDOWN/KEYUP; Left-Shift held doubles the
/// step sizes; 'r' toggles posture-enforcement + balance mode while held.
/// Needs KEYUP as well as KEYDOWN -- see atlas_puppet's file comment.
class HuboPuppetMoveHandler : public ::osgGA::GUIEventHandler
{
public:
  explicit HuboPuppetMoveHandler(std::shared_ptr<HuboPuppetState> state)
    : mState(std::move(state))
  {
  }

  bool handle(
      const ::osgGA::GUIEventAdapter& ea, ::osgGA::GUIActionAdapter&) override
  {
    const bool down = ea.getEventType() == ::osgGA::GUIEventAdapter::KEYDOWN;
    const bool up = ea.getEventType() == ::osgGA::GUIEventAdapter::KEYUP;
    if (!down && !up)
      return false;

    if (ea.getKey() == ::osgGA::GUIEventAdapter::KEY_Shift_L) {
      mState->amplifyMovement = down;
      return true;
    }

    if (ea.getKey() == 'r') {
      if (mState->posture)
        mState->posture->enforceIdealPosture = down;
      if (mState->balance) {
        mState->balance->setErrorMethod(
            down ? dart::constraint::BalanceConstraint::OPTIMIZE_BALANCE
                 : dart::constraint::BalanceConstraint::FROM_CENTROID);
      }
      return true;
    }

    int component = -1;
    switch (ea.getKey()) {
      case 'w':
      case 'W':
        component = MoveW;
        break;
      case 'a':
      case 'A':
        component = MoveA;
        break;
      case 's':
      case 'S':
        component = MoveS;
        break;
      case 'd':
      case 'D':
        component = MoveD;
        break;
      case 'q':
      case 'Q':
        component = MoveQ;
        break;
      case 'e':
      case 'E':
        component = MoveE;
        break;
      case 'f':
      case 'F':
        component = MoveF;
        break;
      case 'z':
      case 'Z':
        component = MoveZ;
        break;
      default:
        return false;
    }

    mState->moveComponents[component] = down;
    return true;
  }

private:
  std::shared_ptr<HuboPuppetState> mState;
};

//==============================================================================
void toggleEndEffectorTarget(
    HuboPuppetState& state,
    const dart::simulation::WorldPtr& world,
    std::size_t index)
{
  if (index >= state.constraintActive.size())
    return;

  auto* ee = state.hubo->getEndEffector(state.endEffectorIndex[index]);
  const auto& ik = ee->getIK();
  if (!ik)
    return;

  if (state.constraintActive[index]) {
    state.constraintActive[index] = false;
    ik->getErrorMethod().setBounds(state.defaultBounds[index]);
    ik->getTarget()->setRelativeTransform(state.defaultTargetTf[index]);
    world->removeSimpleFrame(ik->getTarget());
  } else {
    state.constraintActive[index] = true;
    ik->getErrorMethod().setBounds();
    ik->getTarget()->setTransform(ee->getTransform());
    world->addSimpleFrame(ik->getTarget());
  }
}

//==============================================================================
void resetToRestConfig(HuboPuppetState& state)
{
  for (std::size_t i = 0; i < state.hubo->getNumDofs(); ++i) {
    if (i < 2 || 4 < i)
      state.hubo->getDof(i)->setPosition(state.restConfig[i]);
  }
}

//==============================================================================
void printDofValues(HuboPuppetState& state)
{
  if (!state.log)
    return;
  for (std::size_t i = 0; i < state.hubo->getNumDofs(); ++i) {
    std::ostringstream ss;
    ss << state.hubo->getDof(i)->getName() << ": "
       << state.hubo->getDof(i)->getPosition();
    state.log(ss.str());
  }
}

} // namespace

//==============================================================================
DemoScene makeHuboPuppetScene()
{
  DemoScene scene;
  scene.id = "hubo_puppet";
  scene.title = "Hubo Puppet";
  scene.category = "Control & IK";
  scene.summary = "Hubo puppet driven by analytical arm and leg IK.";

  scene.factory = [] {
    auto world = dart::simulation::World::create();
    // Purely kinematic (see the file comment): zero gravity keeps a forced
    // step from making the unactuated robot collapse.
    world->setGravity(Eigen::Vector3d::Zero());

    auto hubo = createHubo();
    if (!hubo)
      throw std::runtime_error("failed to load " DART_DATA_PATH
                               "/urdf/drchubo/drchubo.urdf");

    setStartupConfiguration(hubo);
    setupEndEffectors(hubo);

    // The original clones the skeleton after EE/IK setup to exercise
    // IK/EndEffector cloning; kept for behavior parity (see the file
    // comment).
    const Eigen::VectorXd positions = hubo->getPositions();
    hubo = hubo->cloneSkeleton("hubo_copy");
    hubo->setPositions(positions);

    world->addSkeleton(hubo);
    world->addSkeleton(createGround());

    setupWholeBodySolver(hubo);

    auto state = std::make_shared<HuboPuppetState>();
    state->hubo = hubo;
    state->restConfig = hubo->getPositions();

    for (std::size_t i = 0; i < hubo->getNumEndEffectors(); ++i) {
      const auto& ik = hubo->getEndEffector(i)->getIK();
      if (ik) {
        state->defaultBounds.push_back(ik->getErrorMethod().getBounds());
        state->defaultTargetTf.push_back(
            ik->getTarget()->getRelativeTransform());
        state->constraintActive.push_back(false);
        state->endEffectorIndex.push_back(i);
      }
    }

    state->posture = std::dynamic_pointer_cast<RelaxedPosture>(
        hubo->getIK(true)->getObjective());
    state->balance
        = std::dynamic_pointer_cast<dart::constraint::BalanceConstraint>(
            hubo->getIK(true)->getProblem()->getEqConstraint(1));

    DemoSceneSetup setup;
    setup.world = world;
    setup.cameraHome = CameraHome{
        ::osg::Vec3d(5.34, 3.00, 1.91),
        ::osg::Vec3d(0.00, 0.00, 0.50),
        ::osg::Vec3d(-0.20, -0.08, 0.98)};

    setup.onActivate = [state, hubo, world](DemoHostContext& ctx) {
      auto* viewer = ctx.viewer();
      state->log = [ctx](const std::string& message) {
        ctx.log(message);
      };

      viewer->allowSimulation(false);
      ctx.addTeardown([viewer] { viewer->allowSimulation(true); });

      for (std::size_t i = 0; i < hubo->getNumBodyNodes(); ++i) {
        if (auto* dnd
            = viewer->enableDragAndDrop(hubo->getBodyNode(i), false, false))
          ctx.addTeardown([viewer, dnd] { viewer->disableDragAndDrop(dnd); });
      }
      for (std::size_t i = 0; i < hubo->getNumEndEffectors(); ++i) {
        auto* ee = hubo->getEndEffector(i);
        if (!ee->getIK())
          continue;
        if (const auto& frame
            = std::dynamic_pointer_cast<dart::gui::osg::InteractiveFrame>(
                ee->getIK()->getTarget())) {
          if (auto* dnd = viewer->enableDragAndDrop(frame.get()))
            ctx.addTeardown([viewer, dnd] { viewer->disableDragAndDrop(dnd); });
        }
      }

      ctx.addAttachment(
          new dart::gui::osg::SupportPolygonVisual(hubo, kDisplayElevation));
      ctx.addEventHandler(new HuboPuppetMoveHandler(state));
    };

    const char* const eeLabels[6]
        = {"l_hand", "r_hand", "l_foot", "r_foot", "l_peg", "r_peg"};
    for (std::size_t i = 0; i < 6; ++i) {
      setup.keyActions.push_back(KeyAction{
          static_cast<int>('1' + i),
          std::string("Toggle ") + eeLabels[i] + " target",
          [state, world, i] {
            toggleEndEffectorTarget(*state, world, i);
          }});
    }
    setup.keyActions.push_back(
        KeyAction{'x', "Toggle l_foot support", [hubo] {
                    auto* ee = hubo->getEndEffector("l_foot");
                    ee->getSupport()->setActive(!ee->getSupport()->isActive());
                  }});
    setup.keyActions.push_back(
        KeyAction{'c', "Toggle r_foot support", [hubo] {
                    auto* ee = hubo->getEndEffector("r_foot");
                    ee->getSupport()->setActive(!ee->getSupport()->isActive());
                  }});
    setup.keyActions.push_back(KeyAction{'t', "Reset to rest pose", [state] {
                                           resetToRestConfig(*state);
                                         }});
    setup.keyActions.push_back(KeyAction{'p', "Print DOF values", [state] {
                                           printDofValues(*state);
                                         }});

    setup.renderPanel = [state] {
      updatePuppet(*state);

      ImGui::TextWrapped(
          "Purely kinematic (physics stepping is disabled for this scene). "
          "Alt+drag translates a body, Ctrl+drag rotates it, Shift+drag "
          "moves it via its parent joint. W/A/S/D move the robot, Q/E "
          "rotate it, F/Z change elevation (hold Left-Shift to double the "
          "step); hold 'r' to optimize posture and balance. 1-6 toggle the "
          "l_hand/r_hand/l_foot/r_foot/l_peg/r_peg targets. Green = support "
          "polygon, blue/red ball = center of mass, green ball = centroid.");
    };

    return setup;
  };

  return scene;
}

} // namespace dart_demos
