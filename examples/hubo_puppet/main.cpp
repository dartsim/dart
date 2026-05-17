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

#include <dart/config.hpp>

#include <dart/gui/application.hpp>
#include <dart/gui/debug.hpp>
#include <dart/gui/gizmo.hpp>
#include <dart/gui/panel.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/simulation/world.hpp>

#include <dart/constraint/balance_constraint.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/end_effector.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/inverse_kinematics.hpp>
#include <dart/dynamics/line_segment_shape.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/simple_frame.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <dart/math/geometry.hpp>
#include <dart/math/optimization/function.hpp>
#include <dart/math/optimization/gradient_descent_solver.hpp>
#include <dart/math/optimization/problem.hpp>

#include <dart/common/macros.hpp>
#include <dart/common/uri.hpp>

#include <dart/io/read.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <complex>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <cmath>

namespace {

constexpr const char* kHuboSkeletonName = "hubo_copy";
constexpr const char* kGroundSkeletonName = "ground";
constexpr const char* kHuboSupportOverlayName = "hubo_support_polygon_overlay";
constexpr const char* kHuboSupportComOverlayName = "hubo_support_com_overlay";
constexpr double kDegrees = 3.14159265358979323846 / 180.0;
constexpr double kTeleopLinearStep = 0.01;
constexpr double kTeleopElevationStep = 0.2 * kTeleopLinearStep;
constexpr double kTeleopYawStep = 2.0 * 3.14159265358979323846 / 180.0;
constexpr double kSupportVisualElevation = 0.05;
constexpr double kSupportComMarkerRadius = 0.06;

enum class PuppetMotion
{
  Forward,
  Backward,
  Left,
  Right,
  Up,
  Down,
  YawLeft,
  YawRight,
};

class RelaxedPosture final : public dart::math::Function
{
public:
  RelaxedPosture(
      const Eigen::VectorXd& idealPosture,
      const Eigen::VectorXd& lower,
      const Eigen::VectorXd& upper,
      const Eigen::VectorXd& weights,
      bool enforceIdeal = false)
    : enforceIdealPosture(enforceIdeal),
      mIdeal(idealPosture),
      mLower(lower),
      mUpper(upper),
      mWeights(weights)
  {
    const Eigen::Index dofCount = mIdeal.size();
    if (mLower.size() != dofCount || mUpper.size() != dofCount
        || mWeights.size() != dofCount) {
      throw std::runtime_error(
          "Hubo relaxed-posture objective has mismatched vector sizes");
    }
    mResultVector.setZero(dofCount);
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
    const Eigen::Index count = std::min(mResultVector.size(), grad.size());
    for (Eigen::Index i = 0; i < count; ++i) {
      grad[i] = mResultVector[i];
    }
  }

  void computeResultVector(const Eigen::VectorXd& x)
  {
    mResultVector.setZero();
    const Eigen::Index count = std::min(mIdeal.size(), x.size());
    for (Eigen::Index i = 0; i < count; ++i) {
      if (enforceIdealPosture) {
        mResultVector[i] = mWeights[i] * (x[i] - mIdeal[i]);
      } else if (x[i] < mLower[i]) {
        mResultVector[i] = mWeights[i] * (x[i] - mLower[i]);
      } else if (mUpper[i] < x[i]) {
        mResultVector[i] = mWeights[i] * (x[i] - mUpper[i]);
      }
    }
  }

  bool enforceIdealPosture;

private:
  Eigen::VectorXd mResultVector;
  Eigen::VectorXd mIdeal;
  Eigen::VectorXd mLower;
  Eigen::VectorXd mUpper;
  Eigen::VectorXd mWeights;
};

struct HuboWholeBodySolverState
{
  std::shared_ptr<RelaxedPosture> posture;
  std::shared_ptr<dart::constraint::BalanceConstraint> balance;
};

using dart::dynamics::BodyNode;
using dart::dynamics::BodyNodePtr;
using dart::dynamics::DegreeOfFreedom;
using dart::dynamics::InverseKinematics;
using dart::dynamics::JacobianNode;
using dart::dynamics::Joint;
using dart::dynamics::SkeletonPtr;
using dart::dynamics::WeakBodyNodePtr;
using IK = dart::dynamics::InverseKinematics;

bool checkDist(Eigen::Vector3d& point, double a, double b)
{
  const double distance = point.norm();
  const double maxDistance = a + b;
  const double minDistance = std::abs(a - b);

  if (distance > maxDistance) {
    point *= maxDistance / distance;
    return false;
  }
  if (distance < minDistance) {
    point *= minDistance / distance;
    return false;
  }
  return true;
}

void clampSineCosine(double& value, bool& valid)
{
  if (value < -1.0) {
    valid = false;
    value = -1.0;
  } else if (value > 1.0) {
    valid = false;
    value = 1.0;
  }
}

Eigen::Vector3d flipEuler3Axis(const Eigen::Vector3d& u)
{
  Eigen::Vector3d v;
  v[0] = u[0] - dart::math::pi;
  v[1] = dart::math::pi - u[1];
  v[2] = u[2] - dart::math::pi;
  return v;
}

class HuboArmIK final : public InverseKinematics::Analytical
{
public:
  HuboArmIK(
      InverseKinematics* ik,
      const std::string& baseLinkName,
      const Analytical::Properties& properties = Analytical::Properties())
    : Analytical(ik, "HuboArmIK_" + baseLinkName, properties),
      mBaseLinkName(baseLinkName)
  {
  }

  std::unique_ptr<GradientMethod> clone(InverseKinematics* newIk) const override
  {
    return std::make_unique<HuboArmIK>(
        newIk, mBaseLinkName, getAnalyticalProperties());
  }

  std::span<const Solution> computeSolutions(
      const Eigen::Isometry3d& desiredBodyTransform) override
  {
    mSolutions.clear();
    mSolutions.reserve(8);

    if (!mConfigured) {
      configure();
      if (!mConfigured) {
        DART_WARN(
            "This analytical IK was not able to configure properly, so it will "
            "not be able to compute solutions");
        return mSolutions;
      }
    }

    const BodyNodePtr& base = mBaseLink.lock();
    if (base == nullptr) {
      DART_ERROR(
          "Attempting to perform an IK on a limb that no longer exists [{}]!",
          getMethodName());
      DART_ASSERT(false);
      return mSolutions;
    }
    if (mWristEnd == nullptr) {
      DART_ERROR("Attempting to perform IK without a wrist!");
      DART_ASSERT(false);
      return mSolutions;
    }

    constexpr std::size_t SP = 0;
    constexpr std::size_t SR = 1;
    constexpr std::size_t SY = 2;
    constexpr std::size_t EP = 3;
    constexpr std::size_t WY = 4;
    constexpr std::size_t WP = 5;

    const SkeletonPtr& skeleton = base->getSkeleton();
    const Eigen::Isometry3d targetInBase
        = base->getParentBodyNode()->getWorldTransform().inverse()
          * desiredBodyTransform * mWristEnd->getTransform(mIK->getNode());
    Eigen::Vector3d p
        = (mShoulderTransform.inverse() * targetInBase).inverse().translation();

    const double aSquared = mL5 * mL5 + mL4 * mL4;
    const double bSquared = mL3 * mL3 + mL4 * mL4;
    const double a = std::sqrt(aSquared);
    const double b = std::sqrt(bSquared);
    const double alpha = std::atan2(mL5, mL4);
    const double beta = std::atan2(mL3, mL4);
    const bool startValid = checkDist(p, a, b);

    const double cSquared = p.dot(p);
    const double x = p.x();
    const double y = p.y();
    const double z = p.z();

    for (std::size_t i = 0; i < 8; ++i) {
      const int flipEP = mAlternatives(i, 0);
      const int incWY = mAlternatives(i, 1);
      const int flipShoulder = mAlternatives(i, 2);

      Eigen::Vector6d testQ;
      bool isValid = startValid;

      double cosGamma = (aSquared + bSquared - cSquared) / (2.0 * a * b);
      clampSineCosine(cosGamma, isValid);
      const double gamma = flipEP * std::acos(cosGamma);
      const double theta3 = alpha + beta + gamma - 2.0 * dart::math::pi;
      testQ(EP) = theta3;

      const double c3 = std::cos(theta3);
      const double s3 = std::sin(theta3);
      const double denom = (-mL4 * c3 - mL3 * s3 + mL4);

      double theta2;
      if (std::abs(denom) < kZeroSize) {
        isValid = false;
        const double& prevWY = skeleton->getPosition(mDofs[WY]);
        theta2 = incWY ? prevWY : dart::math::pi - prevWY;
      } else {
        double s2 = -y / denom;
        clampSineCosine(s2, isValid);
        theta2 = incWY ? dart::math::pi - std::asin(s2) : std::asin(s2);
      }
      testQ(WY) = theta2;

      const double c2 = std::cos(theta2);
      const double r = mL4 * c2 - mL4 * c2 * c3 - mL3 * s3 * c2;
      const double q = -mL4 * s3 + mL3 * c3 + mL5;
      const double det = -(q * q + r * r);
      if (std::abs(det) < kZeroSize) {
        isValid = false;
      }
      const double k = det < 0.0 ? -1.0 : 1.0;
      testQ(WP) = std::atan2(k * (q * x - r * z), k * (-r * x - q * z));

      Eigen::Quaterniond lowerRotation
          = Eigen::Quaterniond(
                Eigen::AngleAxisd(testQ(EP), Eigen::Vector3d::UnitY()))
            * Eigen::Quaterniond(
                Eigen::AngleAxisd(testQ(WY), Eigen::Vector3d::UnitZ()))
            * Eigen::Quaterniond(
                Eigen::AngleAxisd(testQ(WP), Eigen::Vector3d::UnitY()));
      Eigen::Vector3d euler
          = (targetInBase.rotation() * lowerRotation.inverse().matrix())
                .eulerAngles(1, 0, 2);
      if (flipShoulder != 0) {
        euler = flipEuler3Axis(euler);
      }

      testQ(SP) = euler[0];
      testQ(SR) = euler[1];
      testQ(SY) = euler[2];
      for (std::size_t j = 0; j < 6; ++j) {
        testQ[j] = dart::math::wrapToPi(testQ[j]);
        if (std::abs(testQ[j]) < kZeroSize) {
          testQ[j] = 0.0;
        }
      }

      const int validity = isValid ? VALID : OUT_OF_REACH;
      mSolutions.emplace_back(testQ, validity);
    }

    checkSolutionJointLimits();
    return mSolutions;
  }

  std::span<const std::size_t> getDofs() const override
  {
    if (!mConfigured) {
      configure();
    }
    return mDofs;
  }

private:
  void configure() const
  {
    mConfigured = false;
    mBaseLink = mIK->getNode()->getSkeleton()->getBodyNode(mBaseLinkName);

    BodyNode* base = mBaseLink.lock();
    if (base == nullptr) {
      DART_ERROR("base link is a nullptr");
      DART_ASSERT(false);
      return;
    }

    const SkeletonPtr& skeleton = base->getSkeleton();
    const BodyNodePtr& pelvis = skeleton->getBodyNode("Body_TSY");
    if (pelvis == nullptr) {
      DART_ERROR("Could not find Hubo's pelvis (Body_TSY)");
      DART_ASSERT(false);
      return;
    }

    Eigen::Vector6d savedPositions;
    DegreeOfFreedom* dofs[6];
    BodyNode* body = base;
    for (std::size_t i = 0; i < 6; ++i) {
      Joint* joint = body->getParentJoint();
      if (joint->getNumDofs() != 1) {
        DART_ERROR(
            "Invalid number of DOFs ({}) in the Joint [{}]",
            joint->getNumDofs(),
            joint->getName());
        DART_ASSERT(false);
        return;
      }

      dofs[i] = joint->getDof(0);
      savedPositions[i] = dofs[i]->getPosition();
      dofs[i]->setPosition(0.0);
      body = body->getChildBodyNode(0);
    }

    BodyNode* elbow = dofs[3]->getChildBodyNode();
    mL3 = std::abs(
        elbow->getTransform(dofs[2]->getParentBodyNode()).translation()[2]);
    mL4 = std::abs(
        elbow->getTransform(dofs[3]->getParentBodyNode()).translation()[0]);

    BodyNode* wrist = dofs[5]->getChildBodyNode();
    mL5 = std::abs(wrist->getTransform(elbow).translation()[2]);

    mShoulderTransform = Eigen::Isometry3d::Identity();
    mShoulderTransform.translate(
        dofs[3]->getParentBodyNode()->getTransform(pelvis).translation()[0]
        * Eigen::Vector3d::UnitX());
    mShoulderTransform.translate(
        dofs[2]->getParentBodyNode()->getTransform(pelvis).translation()[1]
        * Eigen::Vector3d::UnitY());
    mShoulderTransform.translate(
        dofs[2]->getParentBodyNode()->getTransform(pelvis).translation()[2]
        * Eigen::Vector3d::UnitZ());

    mWristEnd = dofs[5]->getChildBodyNode();
    mAlternatives << 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 0, 0, -1, 1, 1, -1, 1, 0, -1,
        0, 1, -1, 0, 0;

    mDofs.clear();
    for (std::size_t i = 0; i < 6; ++i) {
      dofs[i]->setPosition(savedPositions[i]);
      mDofs.push_back(dofs[i]->getIndexInSkeleton());
    }
    mConfigured = true;
  }

  static constexpr double kZeroSize = 1e-8;
  mutable bool mConfigured = false;
  mutable Eigen::Isometry3d mShoulderTransform = Eigen::Isometry3d::Identity();
  mutable double mL3 = 0.0;
  mutable double mL4 = 0.0;
  mutable double mL5 = 0.0;
  mutable Eigen::Matrix<int, 8, 3> mAlternatives;
  mutable std::vector<std::size_t> mDofs;
  std::string mBaseLinkName;
  mutable WeakBodyNodePtr mBaseLink;
  mutable JacobianNode* mWristEnd = nullptr;
};

class HuboLegIK final : public InverseKinematics::Analytical
{
public:
  HuboLegIK(
      InverseKinematics* ik,
      const std::string& baseLinkName,
      const Analytical::Properties& properties = Analytical::Properties())
    : Analytical(ik, "HuboLegIK_" + baseLinkName, properties),
      mBaseLinkName(baseLinkName)
  {
  }

  std::unique_ptr<GradientMethod> clone(InverseKinematics* newIk) const override
  {
    return std::make_unique<HuboLegIK>(
        newIk, mBaseLinkName, getAnalyticalProperties());
  }

  std::span<const Solution> computeSolutions(
      const Eigen::Isometry3d& desiredBodyTransform) override
  {
    mSolutions.clear();
    mSolutions.reserve(8);

    if (!mConfigured) {
      configure();
      if (!mConfigured) {
        DART_WARN(
            "This analytical IK was not able to configure properly, so it will "
            "not be able to compute solutions");
        return mSolutions;
      }
    }

    const BodyNodePtr& base = mBaseLink.lock();
    if (base == nullptr) {
      DART_ERROR("Attempting to perform IK on a limb that no longer exists!");
      DART_ASSERT(false);
      return mSolutions;
    }

    const Eigen::Isometry3d target
        = (base->getParentBodyNode()->getWorldTransform() * mWaist).inverse()
          * desiredBodyTransform * mFootTransformInverse;
    const Eigen::Isometry3d targetInverse = target.inverse();

    const double nx = targetInverse(0, 0);
    const double sx = targetInverse(0, 1);
    const double ax = targetInverse(0, 2);
    const double px = targetInverse(0, 3);
    const double ny = targetInverse(1, 0);
    const double sy = targetInverse(1, 1);
    const double ay = targetInverse(1, 2);
    const double py = targetInverse(1, 3);
    const double az = targetInverse(2, 2);
    const double pz = targetInverse(2, 3);

    for (std::size_t i = 0; i < 8; ++i) {
      bool isValid = true;

      const double c4 = ((px + mL6) * (px + mL6) - mL4 * mL4 - mL5 * mL5
                         + py * py + pz * pz)
                        / (2.0 * mL4 * mL5);
      std::complex<double> radical = 1.0 - c4 * c4;
      std::complex<double> sqrtRadical = std::sqrt(radical);
      if (sqrtRadical.imag() != 0.0) {
        isValid = false;
      }
      const double q4
          = std::atan2(mAlternatives(i, 0) * sqrtRadical.real(), c4);

      const double s4 = std::sin(q4);
      const double psi = std::atan2(s4 * mL4, c4 * mL4 + mL5);
      radical = (px + mL6) * (px + mL6) + py * py;
      sqrtRadical = std::sqrt(radical);
      if (sqrtRadical.imag() != 0.0) {
        isValid = false;
      }

      const double q5 = dart::math::wrapToPi(
          std::atan2(-pz, mAlternatives(i, 1) * sqrtRadical.real()) - psi);

      double q6 = std::atan2(py, -(px + mL6));
      const double c45 = std::cos(q4 + q5);
      const double c5 = std::cos(q5);
      if (c45 * mL4 + c5 * mL5 < 0.0) {
        q6 = dart::math::wrapToPi(q6 + dart::math::pi);
      }

      const double s6 = std::sin(q6);
      const double c6 = std::cos(q6);
      const double s2 = c6 * ay + s6 * ax;
      radical = 1.0 - s2 * s2;
      sqrtRadical = std::sqrt(radical);
      if (sqrtRadical.imag() != 0.0) {
        isValid = false;
      }
      const double q2
          = std::atan2(s2, mAlternatives(i, 2) * sqrtRadical.real());

      double q1 = std::atan2(c6 * sy + s6 * sx, c6 * ny + s6 * nx);
      const double c2 = std::cos(q2);
      if (c2 < 0.0) {
        q1 = dart::math::wrapToPi(q1 + dart::math::pi);
      }

      const double q345 = std::atan2(-az / c2, -(c6 * ax - s6 * ay) / c2);
      const double q3 = dart::math::wrapToPi(q345 - q4 - q5);

      Eigen::Vector6d testQ;
      testQ[0] = q1;
      testQ[1] = q2;
      testQ[2] = q3;
      testQ[3] = q4;
      testQ[4] = q5;
      testQ[5] = q6;

      for (std::size_t j = 0; j < 6; ++j) {
        if (std::abs(testQ[j]) < kZeroSize) {
          testQ[j] = 0.0;
        }
      }

      const int validity = isValid ? VALID : OUT_OF_REACH;
      mSolutions.emplace_back(testQ, validity);
    }

    checkSolutionJointLimits();
    return mSolutions;
  }

  std::span<const std::size_t> getDofs() const override
  {
    if (!mConfigured) {
      configure();
    }
    return mDofs;
  }

private:
  void configure() const
  {
    mConfigured = false;
    mBaseLink = mIK->getNode()->getSkeleton()->getBodyNode(mBaseLinkName);

    BodyNode* base = mBaseLink.lock();
    if (base == nullptr) {
      DART_ERROR("base link is a nullptr");
      DART_ASSERT(false);
      return;
    }

    const SkeletonPtr& skeleton = mIK->getNode()->getSkeleton();
    BodyNode* pelvis = skeleton->getBodyNode("Body_TSY");
    if (pelvis == nullptr) {
      DART_ERROR("Could not find Hubo's pelvis (Body_TSY)");
      DART_ASSERT(false);
      return;
    }

    Eigen::Vector6d savedPositions;
    DegreeOfFreedom* dofs[6];
    BodyNode* body = base;
    for (std::size_t i = 0; i < 6; ++i) {
      Joint* joint = body->getParentJoint();
      if (joint->getNumDofs() != 1) {
        DART_ERROR(
            "Invalid number of DOFs ({}) in the Joint [{}]",
            joint->getNumDofs(),
            joint->getName());
        DART_ASSERT(false);
        return;
      }

      dofs[i] = joint->getDof(0);
      savedPositions[i] = dofs[i]->getPosition();
      dofs[i]->setPosition(0.0);
      if (body->getNumChildBodyNodes() > 0) {
        body = body->getChildBodyNode(0);
      }
    }

    mL4 = std::abs(
        dofs[3]->getChildBodyNode()->getRelativeTransform().translation()[2]);
    mL5 = std::abs(
        dofs[4]->getChildBodyNode()->getRelativeTransform().translation()[2]);
    mL6 = 0.0;

    mHipRotation = Eigen::Isometry3d::Identity();
    mHipRotation.rotate(
        Eigen::AngleAxisd(
            90.0 * dart::math::pi / 180.0, Eigen::Vector3d::UnitZ()));
    mWaist = dofs[2]->getChildBodyNode()->getTransform(
                 dofs[0]->getParentBodyNode())
             * mHipRotation;

    mFootTransformInverse = Eigen::Isometry3d::Identity();
    mFootTransformInverse.rotate(
        Eigen::AngleAxisd(
            -90.0 * dart::math::pi / 180.0, Eigen::Vector3d::UnitY()));
    mFootTransformInverse
        = mFootTransformInverse
          * mIK->getNode()->getTransform(dofs[5]->getChildBodyNode());
    mFootTransformInverse = mFootTransformInverse.inverse();

    mAlternatives << 1, 1, 1, 1, 1, -1, 1, -1, 1, 1, -1, -1, -1, 1, 1, -1, 1,
        -1, -1, -1, 1, -1, -1, -1;

    mDofs.clear();
    for (std::size_t i = 0; i < 6; ++i) {
      dofs[i]->setPosition(savedPositions[i]);
      mDofs.push_back(dofs[i]->getIndexInSkeleton());
    }
    mConfigured = true;
  }

  static constexpr double kZeroSize = 1e-8;
  mutable double mL4 = 0.0;
  mutable double mL5 = 0.0;
  mutable double mL6 = 0.0;
  mutable Eigen::Isometry3d mWaist = Eigen::Isometry3d::Identity();
  mutable Eigen::Isometry3d mHipRotation = Eigen::Isometry3d::Identity();
  mutable Eigen::Isometry3d mFootTransformInverse
      = Eigen::Isometry3d::Identity();
  mutable Eigen::Matrix<int, 8, 3> mAlternatives;
  mutable std::vector<std::size_t> mDofs;
  mutable bool mConfigured = false;
  std::string mBaseLinkName;
  mutable WeakBodyNodePtr mBaseLink;
};

void disableSkeletonCollisionAndGravity(
    const dart::dynamics::SkeletonPtr& skeleton)
{
  if (skeleton == nullptr) {
    return;
  }

  skeleton->disableSelfCollisionCheck();
  skeleton->setAdjacentBodyCheck(false);
  for (std::size_t i = 0; i < skeleton->getNumBodyNodes(); ++i) {
    auto* body = skeleton->getBodyNode(i);
    if (body == nullptr) {
      continue;
    }
    body->setCollidable(false);
    body->setGravityMode(false);
    body->eachShapeNodeWith<dart::dynamics::CollisionAspect>(
        [](dart::dynamics::ShapeNode* shapeNode) {
          shapeNode->getCollisionAspect()->setCollidable(false);
        });
  }
}

std::optional<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
computeVisualWorldBounds(const dart::dynamics::SkeletonPtr& skeleton)
{
  if (skeleton == nullptr) {
    return std::nullopt;
  }

  bool hasBounds = false;
  Eigen::Vector3d min = Eigen::Vector3d::Zero();
  Eigen::Vector3d max = Eigen::Vector3d::Zero();

  const auto includePoint = [&](const Eigen::Vector3d& point) {
    if (!hasBounds) {
      min = point;
      max = point;
      hasBounds = true;
      return;
    }
    min = min.cwiseMin(point);
    max = max.cwiseMax(point);
  };

  for (std::size_t i = 0; i < skeleton->getNumBodyNodes(); ++i) {
    auto* body = skeleton->getBodyNode(i);
    if (body == nullptr) {
      continue;
    }

    body->eachShapeNodeWith<dart::dynamics::VisualAspect>(
        [&](const dart::dynamics::ShapeNode* shapeNode) {
          if (shapeNode == nullptr || shapeNode->getShape() == nullptr
              || shapeNode->getVisualAspect()->isHidden()) {
            return;
          }

          const auto& bounds = shapeNode->getShape()->getBoundingBox();
          const Eigen::Vector3d localMin = bounds.getMin();
          const Eigen::Vector3d localMax = bounds.getMax();
          if (!localMin.allFinite() || !localMax.allFinite()) {
            return;
          }

          const Eigen::Isometry3d transform = shapeNode->getWorldTransform();
          for (int x = 0; x < 2; ++x) {
            for (int y = 0; y < 2; ++y) {
              for (int z = 0; z < 2; ++z) {
                includePoint(
                    transform
                    * Eigen::Vector3d(
                        x == 0 ? localMin.x() : localMax.x(),
                        y == 0 ? localMin.y() : localMax.y(),
                        z == 0 ? localMin.z() : localMax.z()));
              }
            }
          }
        });
  }

  if (!hasBounds) {
    return std::nullopt;
  }

  return std::make_pair(min, max);
}

void setRequiredDofPosition(
    const dart::dynamics::SkeletonPtr& hubo, const char* name, double position)
{
  auto* dof = hubo ? hubo->getDof(name) : nullptr;
  if (dof == nullptr) {
    throw std::runtime_error(
        "Hubo puppet model is missing expected DOF " + std::string(name));
  }
  dof->setPosition(position);
}

void setRequiredDofLimits(
    const dart::dynamics::SkeletonPtr& hubo,
    const char* name,
    double lower,
    double upper)
{
  auto* dof = hubo ? hubo->getDof(name) : nullptr;
  if (dof == nullptr) {
    throw std::runtime_error(
        "Hubo puppet model is missing expected DOF " + std::string(name));
  }
  dof->setPositionLowerLimit(lower);
  dof->setPositionUpperLimit(upper);
}

void setupHuboPuppetStartConfiguration(const dart::dynamics::SkeletonPtr& hubo)
{
  const std::array<std::pair<const char*, double>, 10> jointPositions{{
      {"LHP", -45.0},
      {"LKP", 90.0},
      {"LAP", -45.0},
      {"RHP", -45.0},
      {"RKP", 90.0},
      {"RAP", -45.0},
      {"LSP", 30.0},
      {"LEP", -120.0},
      {"RSP", 30.0},
      {"REP", -120.0},
  }};
  for (const auto& [name, degrees] : jointPositions) {
    setRequiredDofPosition(hubo, name, degrees * kDegrees);
  }

  const std::array<const char*, 4> limitedDofs{{"LSY", "LWY", "RSY", "RWY"}};
  for (const char* name : limitedDofs) {
    setRequiredDofLimits(hubo, name, -90.0 * kDegrees, 90.0 * kDegrees);
  }
}

void removeHuboPuppetFingerBodyNodes(const dart::dynamics::SkeletonPtr& hubo)
{
  if (hubo == nullptr) {
    return;
  }

  for (std::size_t i = 0; i < hubo->getNumBodyNodes();) {
    auto* body = hubo->getBodyNode(i);
    if (body == nullptr) {
      ++i;
      continue;
    }

    const std::string name = body->getName();
    if (name.starts_with("Body_LF") || name.starts_with("Body_RF")) {
      body->remove();
      continue;
    }
    ++i;
  }
}

dart::dynamics::SkeletonPtr loadHuboPuppetSkeleton()
{
  dart::io::ReadOptions options;
  options.addPackageDirectory(
      "drchubo", dart::config::dataPath("urdf/drchubo"));
  const dart::common::Uri huboUri = dart::common::Uri::createFromPath(
      dart::config::dataPath("urdf/drchubo/drchubo.urdf"));
  auto hubo = dart::io::readSkeleton(huboUri, options);
  if (hubo == nullptr) {
    throw std::runtime_error(
        "Failed to load Hubo puppet model from " + huboUri.toString());
  }

  hubo->setName(kHuboSkeletonName);
  removeHuboPuppetFingerBodyNodes(hubo);
  setupHuboPuppetStartConfiguration(hubo);

  auto* rootBody = hubo->getRootBodyNode();
  if (rootBody != nullptr
      && dynamic_cast<dart::dynamics::FreeJoint*>(rootBody->getParentJoint())
             != nullptr) {
    Eigen::Isometry3d transform = rootBody->getWorldTransform();
    transform.translation().x() = 0.0;
    transform.translation().y() = 0.0;
    dart::dynamics::FreeJoint::setTransformOf(rootBody, transform);
    if (const auto bounds = computeVisualWorldBounds(hubo)) {
      constexpr double groundClearance = 0.015;
      transform = rootBody->getWorldTransform();
      transform.translation().z() += groundClearance - bounds->first.z();
      dart::dynamics::FreeJoint::setTransformOf(rootBody, transform);
    }
  }

  disableSkeletonCollisionAndGravity(hubo);
  return hubo;
}

dart::math::SupportGeometry makeHuboPuppetFootSupportGeometry()
{
  dart::math::SupportGeometry support;
  support.emplace_back(-0.08, 0.05, 0.0);
  support.emplace_back(-0.18, 0.05, 0.0);
  support.emplace_back(-0.18, -0.05, 0.0);
  support.emplace_back(-0.08, -0.05, 0.0);
  return support;
}

std::shared_ptr<dart::dynamics::LineSegmentShape> createLineShape(
    const std::vector<dart::gui::DebugLineDescriptor>& lines)
{
  auto shape = std::make_shared<dart::dynamics::LineSegmentShape>(3.0f);
  for (const auto& line : lines) {
    const auto start = shape->addVertex(line.from);
    shape->addVertex(line.to, start);
  }
  return shape;
}

void appendMarkerAxisLines(
    std::vector<dart::gui::DebugLineDescriptor>& lines,
    const Eigen::Vector3d& center,
    double radius,
    const std::string& label)
{
  dart::gui::DebugLineDescriptor x;
  x.from = center - Eigen::Vector3d::UnitX() * radius;
  x.to = center + Eigen::Vector3d::UnitX() * radius;
  x.label = label + ".x";
  lines.push_back(x);

  dart::gui::DebugLineDescriptor y;
  y.from = center - Eigen::Vector3d::UnitY() * radius;
  y.to = center + Eigen::Vector3d::UnitY() * radius;
  y.label = label + ".y";
  lines.push_back(y);

  dart::gui::DebugLineDescriptor z;
  z.from = center - Eigen::Vector3d::UnitZ() * radius;
  z.to = center + Eigen::Vector3d::UnitZ() * radius;
  z.label = label + ".z";
  lines.push_back(z);
}

std::vector<dart::gui::DebugLineDescriptor> makeHuboSupportPolygonLines(
    const dart::dynamics::SkeletonPtr& hubo)
{
  if (hubo == nullptr) {
    return {};
  }

  dart::gui::DebugDrawOptions options;
  options.drawGrid = false;
  options.drawWorldFrame = false;
  options.drawSupportPolygons = true;
  options.supportPolygonElevation = kSupportVisualElevation;
  return dart::gui::makeSupportPolygonDebugLines(*hubo, options, "hubo");
}

std::optional<Eigen::Vector2d> computeHuboComSupportProjection(
    const dart::dynamics::SkeletonPtr& hubo)
{
  if (hubo == nullptr || hubo->getMass() <= 0.0
      || !std::isfinite(hubo->getMass())) {
    return std::nullopt;
  }

  const auto& axes = hubo->getSupportAxes();
  if (!axes.first.allFinite() || !axes.second.allFinite()) {
    return std::nullopt;
  }

  const Eigen::Vector3d com = hubo->getCOM();
  if (!com.allFinite()) {
    return std::nullopt;
  }

  return Eigen::Vector2d(com.dot(axes.first), com.dot(axes.second));
}

std::vector<dart::gui::DebugLineDescriptor> makeHuboSupportComLines(
    const dart::dynamics::SkeletonPtr& hubo)
{
  if (hubo == nullptr) {
    return {};
  }

  const auto projectedCom = computeHuboComSupportProjection(hubo);
  if (!projectedCom) {
    return {};
  }

  const auto& axes = hubo->getSupportAxes();
  const Eigen::Vector3d up = axes.first.cross(axes.second);
  if (!up.allFinite() || up.squaredNorm() <= 1e-18) {
    return {};
  }

  const Eigen::Vector3d center = axes.first * projectedCom->x()
                                 + axes.second * projectedCom->y()
                                 + up.normalized() * kSupportVisualElevation;
  std::vector<dart::gui::DebugLineDescriptor> lines;
  lines.reserve(3);
  appendMarkerAxisLines(
      lines, center, kSupportComMarkerRadius, "hubo.support_com");
  return lines;
}

Eigen::Vector4d huboSupportComColor(const dart::dynamics::SkeletonPtr& hubo)
{
  const auto projectedCom = computeHuboComSupportProjection(hubo);
  if (hubo == nullptr || !projectedCom) {
    return Eigen::Vector4d(1.0, 0.0, 0.0, 1.0);
  }

  return dart::math::isInsideSupportPolygon(
             *projectedCom, hubo->getSupportPolygon())
             ? Eigen::Vector4d(0.0, 0.0, 1.0, 1.0)
             : Eigen::Vector4d(1.0, 0.0, 0.0, 1.0);
}

dart::dynamics::SimpleFramePtr createHuboSupportPolygonOverlay(
    const dart::dynamics::SkeletonPtr& hubo)
{
  auto overlay = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World(), kHuboSupportOverlayName);
  overlay->setShape(createLineShape(makeHuboSupportPolygonLines(hubo)));
  overlay->getVisualAspect(true)->setRGBA(
      Eigen::Vector4d(0.22, 0.86, 0.38, 0.86));
  return overlay;
}

dart::dynamics::SimpleFramePtr createHuboSupportComOverlay(
    const dart::dynamics::SkeletonPtr& hubo)
{
  auto overlay = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World(), kHuboSupportComOverlayName);
  overlay->setShape(createLineShape(makeHuboSupportComLines(hubo)));
  overlay->getVisualAspect(true)->setRGBA(huboSupportComColor(hubo));
  return overlay;
}

void updateHuboSupportPolygonOverlay(
    const dart::dynamics::SkeletonPtr& hubo,
    const dart::dynamics::SimpleFramePtr& overlay)
{
  if (overlay == nullptr) {
    return;
  }

  overlay->setShape(createLineShape(makeHuboSupportPolygonLines(hubo)));
}

void updateHuboSupportComOverlay(
    const dart::dynamics::SkeletonPtr& hubo,
    const dart::dynamics::SimpleFramePtr& overlay)
{
  if (overlay == nullptr) {
    return;
  }

  overlay->setShape(createLineShape(makeHuboSupportComLines(hubo)));
  overlay->getVisualAspect(true)->setRGBA(huboSupportComColor(hubo));
}

void setUnconstrainedIkBounds(const dart::dynamics::InverseKinematicsPtr& ik)
{
  Eigen::Vector3d linearBounds
      = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d angularBounds
      = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  ik->getErrorMethod().setLinearBounds(-linearBounds, linearBounds);
  ik->getErrorMethod().setAngularBounds(-angularBounds, angularBounds);
}

void setVectorEntry(Eigen::VectorXd& values, Eigen::Index index, double value)
{
  if (0 <= index && index < values.size()) {
    values[index] = value;
  }
}

HuboWholeBodySolverState setupHuboWholeBodySolver(
    const dart::dynamics::SkeletonPtr& hubo)
{
  HuboWholeBodySolverState state;
  if (hubo == nullptr) {
    return state;
  }

  auto wholeBodyIk = hubo->getIK(true);
  auto solver = std::dynamic_pointer_cast<dart::math::GradientDescentSolver>(
      wholeBodyIk->getSolver());
  if (solver != nullptr) {
    solver->setNumMaxIterations(5);
  }

  const Eigen::Index dofCount = static_cast<Eigen::Index>(hubo->getNumDofs());
  constexpr double defaultWeight = 0.01;
  Eigen::VectorXd weights = defaultWeight * Eigen::VectorXd::Ones(dofCount);
  setVectorEntry(weights, 2, 0.0);
  setVectorEntry(weights, 3, 0.0);
  setVectorEntry(weights, 4, 0.0);

  Eigen::VectorXd lowerPosture = Eigen::VectorXd::Constant(
      dofCount, -std::numeric_limits<double>::infinity());
  setVectorEntry(lowerPosture, 0, -0.35);
  setVectorEntry(lowerPosture, 1, -0.35);
  setVectorEntry(lowerPosture, 5, 0.55);

  Eigen::VectorXd upperPosture = Eigen::VectorXd::Constant(
      dofCount, std::numeric_limits<double>::infinity());
  setVectorEntry(upperPosture, 0, 0.35);
  setVectorEntry(upperPosture, 1, 0.50);
  setVectorEntry(upperPosture, 5, 0.95);

  state.posture = std::make_shared<RelaxedPosture>(
      hubo->getPositions(), lowerPosture, upperPosture, weights);
  wholeBodyIk->setObjective(state.posture);

  state.balance = std::make_shared<dart::constraint::BalanceConstraint>(
      wholeBodyIk,
      dart::constraint::BalanceConstraint::SHIFT_SUPPORT,
      dart::constraint::BalanceConstraint::FROM_CENTROID);
  wholeBodyIk->getProblem()->addEqConstraint(state.balance);
  return state;
}

dart::dynamics::SkeletonPtr createGround()
{
  auto ground = dart::dynamics::Skeleton::create(kGroundSkeletonName);
  auto* body
      = ground->createJointAndBodyNodePair<dart::dynamics::WeldJoint>().second;
  constexpr double thickness = 0.01;
  auto* shapeNode = body->createShapeNodeWith<dart::dynamics::VisualAspect>(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(10.0, 10.0, thickness)));
  shapeNode->setRelativeTranslation(
      Eigen::Vector3d(0.0, 0.0, -thickness / 2.0));
  shapeNode->getVisualAspect()->setRGBA(Eigen::Vector4d(0.0, 0.0, 0.2, 1.0));
  return ground;
}

struct HuboPuppetScene
{
  dart::simulation::WorldPtr world;
  dart::dynamics::SkeletonPtr hubo;
  dart::dynamics::SimpleFramePtr supportOverlay;
  dart::dynamics::SimpleFramePtr supportComOverlay;
  std::vector<dart::gui::InverseKinematicsHandle> ikHandles;
  std::vector<dart::gui::Gizmo> gizmos;
  HuboWholeBodySolverState wholeBodySolver;
  struct TargetState
  {
    dart::simulation::WorldPtr world;
    dart::dynamics::EndEffector* effector = nullptr;
    dart::dynamics::InverseKinematicsPtr ik;
    dart::dynamics::SimpleFramePtr target;
    std::pair<Eigen::Vector6d, Eigen::Vector6d> defaultBounds;
    Eigen::Isometry3d defaultTargetTransform = Eigen::Isometry3d::Identity();
    std::string label;
    char hotkey = '\0';
    bool active = false;

    void activate()
    {
      if (active || world == nullptr || effector == nullptr || target == nullptr
          || ik == nullptr) {
        return;
      }

      ik->getErrorMethod().setBounds();
      target->setTransform(effector->getWorldTransform());
      world->addSimpleFrame(target);
      active = true;
      std::cout << "Activated IK target '" << effector->getName() << "'.\n";
    }

    void deactivate()
    {
      if (!active || world == nullptr || target == nullptr || ik == nullptr) {
        return;
      }

      ik->getErrorMethod().setBounds(defaultBounds);
      target->setTransform(defaultTargetTransform);
      world->removeSimpleFrame(target);
      active = false;
      std::cout << "Deactivated IK target '" << effector->getName() << "'.\n";
    }

    void toggle()
    {
      if (active) {
        deactivate();
      } else {
        activate();
      }
    }
  };

  std::vector<std::shared_ptr<TargetState>> targetStates;
  Eigen::VectorXd restConfiguration;
};

void addHuboPuppetIkTargets(
    HuboPuppetScene& scene, const dart::dynamics::SkeletonPtr& hubo)
{
  enum class AnalyticalIkKind
  {
    Arm,
    Leg,
  };

  struct Config
  {
    const char* bodyNode;
    const char* effectorName;
    const char* targetName;
    const char* label;
    int hotkey;
    Eigen::Isometry3d relativeTransform;
    AnalyticalIkKind analyticalKind;
    const char* analyticalBase;
    bool useWholeBody;
    bool usePostAnalyticalDofs;
    bool supportContact;
  };

  Eigen::Isometry3d hand = Eigen::Isometry3d::Identity();
  hand.translation() = Eigen::Vector3d(0.0, 0.0, -0.09);

  Eigen::Isometry3d foot = Eigen::Isometry3d::Identity();
  foot.translation() = Eigen::Vector3d(0.14, 0.0, -0.126);

  Eigen::Isometry3d peg = Eigen::Isometry3d::Identity();
  peg.translation() = Eigen::Vector3d(0.0, 0.0, 0.09);

  const std::array<Config, 6> configs{{
      {"Body_LWR",
       "l_hand",
       "hubo_puppet_ik_target_left_hand",
       "1 left hand",
       '1',
       hand,
       AnalyticalIkKind::Arm,
       "Body_LSP",
       true,
       true,
       false},
      {"Body_RWR",
       "r_hand",
       "hubo_puppet_ik_target_right_hand",
       "2 right hand",
       '2',
       hand,
       AnalyticalIkKind::Arm,
       "Body_RSP",
       true,
       true,
       false},
      {"Body_LAR",
       "l_foot",
       "hubo_puppet_ik_target_left_foot",
       "3 left foot",
       '3',
       foot,
       AnalyticalIkKind::Leg,
       "Body_LHY",
       false,
       false,
       true},
      {"Body_RAR",
       "r_foot",
       "hubo_puppet_ik_target_right_foot",
       "4 right foot",
       '4',
       foot,
       AnalyticalIkKind::Leg,
       "Body_RHY",
       false,
       false,
       true},
      {"Body_LWP",
       "l_peg",
       "hubo_puppet_ik_target_left_peg",
       "5 left peg",
       '5',
       peg,
       AnalyticalIkKind::Arm,
       "Body_LSP",
       false,
       false,
       false},
      {"Body_RWP",
       "r_peg",
       "hubo_puppet_ik_target_right_peg",
       "6 right peg",
       '6',
       peg,
       AnalyticalIkKind::Arm,
       "Body_RSP",
       false,
       false,
       false},
  }};

  const auto footSupportGeometry = makeHuboPuppetFootSupportGeometry();
  const Eigen::VectorXd rootJointWeights = 0.01 * Eigen::VectorXd::Ones(7);
  constexpr double extraErrorClamp = 0.1;
  for (const Config& config : configs) {
    auto* bodyNode = hubo->getBodyNode(config.bodyNode);
    if (bodyNode == nullptr) {
      throw std::runtime_error(
          "Hubo puppet model is missing body node "
          + std::string(config.bodyNode));
    }

    auto* endEffector = bodyNode->createEndEffector(config.effectorName);
    endEffector->setDefaultRelativeTransform(config.relativeTransform, true);
    if (config.supportContact) {
      auto* support = endEffector->getSupport(true);
      support->setGeometry(footSupportGeometry);
      support->setActive(true);
    }

    auto ik = endEffector->getIK(true);
    if (config.useWholeBody) {
      ik->useWholeBody();
    }
    if (config.analyticalKind == AnalyticalIkKind::Arm) {
      ik->setGradientMethod<HuboArmIK>(config.analyticalBase);
      if (config.usePostAnalyticalDofs) {
        ik->getAnalytical()->setExtraDofUtilization(
            IK::Analytical::POST_ANALYTICAL);
        ik->getAnalytical()->setExtraErrorLengthClamp(extraErrorClamp);
        ik->getGradientMethod().setComponentWeights(rootJointWeights);
      }
    } else {
      ik->setGradientMethod<HuboLegIK>(config.analyticalBase);
    }
    ik->getSolver()->setNumMaxIterations(30);
    if (config.supportContact) {
      ik->setHierarchyLevel(1);
      Eigen::Vector3d linearBounds
          = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
      Eigen::Vector3d angularBounds
          = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
      linearBounds.z() = 1e-8;
      angularBounds.x() = 1e-8;
      angularBounds.y() = 1e-8;
      ik->getErrorMethod().setLinearBounds(-linearBounds, linearBounds);
      ik->getErrorMethod().setAngularBounds(-angularBounds, angularBounds);
    } else {
      setUnconstrainedIkBounds(ik);
    }

    auto target = dart::dynamics::SimpleFrame::createShared(
        dart::dynamics::Frame::World(),
        config.targetName,
        endEffector->getWorldTransform());
    ik->setTarget(target);

    auto state = std::make_shared<HuboPuppetScene::TargetState>();
    state->world = scene.world;
    state->effector = endEffector;
    state->ik = ik;
    state->target = target;
    state->defaultBounds = ik->getErrorMethod().getBounds();
    state->defaultTargetTransform = target->getRelativeTransform();
    state->label = config.label;
    state->hotkey = config.hotkey;

    dart::gui::InverseKinematicsHandle handle;
    handle.label = config.label;
    handle.hotkey = config.hotkey;
    handle.target = target;
    handle.ik = ik;
    scene.ikHandles.push_back(std::move(handle));

    dart::gui::Gizmo gizmo;
    gizmo.label = config.targetName;
    gizmo.target = target;
    gizmo.size = 0.24;
    gizmo.isVisible = [state]() {
      return state->active;
    };
    scene.gizmos.push_back(std::move(gizmo));
    scene.targetStates.push_back(std::move(state));
  }
}

HuboPuppetScene createHuboPuppetScene()
{
  HuboPuppetScene scene;
  scene.world = dart::simulation::World::create("dartsim_hubo_puppet");
  scene.world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  scene.world->addSkeleton(createGround());

  auto hubo = loadHuboPuppetSkeleton();
  scene.hubo = hubo;
  scene.world->addSkeleton(hubo);
  addHuboPuppetIkTargets(scene, hubo);
  scene.wholeBodySolver = setupHuboWholeBodySolver(scene.hubo);
  scene.restConfiguration = hubo->getPositions();
  scene.supportOverlay = createHuboSupportPolygonOverlay(scene.hubo);
  scene.world->addSimpleFrame(scene.supportOverlay);
  scene.supportComOverlay = createHuboSupportComOverlay(scene.hubo);
  scene.world->addSimpleFrame(scene.supportComOverlay);
  return scene;
}

void solveHuboWholeBody(const dart::dynamics::SkeletonPtr& hubo)
{
  const auto wholeBodyIk = hubo ? hubo->getIK() : nullptr;
  if (wholeBodyIk != nullptr) {
    wholeBodyIk->solveAndApply(true);
  }
}

bool applyRootTeleoperationStep(
    const dart::dynamics::SkeletonPtr& hubo, PuppetMotion motion)
{
  auto* rootBody = hubo ? hubo->getRootBodyNode() : nullptr;
  if (rootBody == nullptr
      || dynamic_cast<dart::dynamics::FreeJoint*>(rootBody->getParentJoint())
             == nullptr) {
    return false;
  }

  const Eigen::Isometry3d current = rootBody->getWorldTransform();
  Eigen::Isometry3d next = current;

  Eigen::Vector3d forward = current.linear().col(0);
  forward.z() = 0.0;
  if (forward.norm() > 1e-10) {
    forward.normalize();
  } else {
    forward.setZero();
  }

  Eigen::Vector3d left = current.linear().col(1);
  left.z() = 0.0;
  if (left.norm() > 1e-10) {
    left.normalize();
  } else {
    left.setZero();
  }

  switch (motion) {
    case PuppetMotion::Forward:
      next.translation() += kTeleopLinearStep * forward;
      break;
    case PuppetMotion::Backward:
      next.translation() -= kTeleopLinearStep * forward;
      break;
    case PuppetMotion::Left:
      next.translation() += kTeleopLinearStep * left;
      break;
    case PuppetMotion::Right:
      next.translation() -= kTeleopLinearStep * left;
      break;
    case PuppetMotion::Up:
      next.translation() += kTeleopElevationStep * Eigen::Vector3d::UnitZ();
      break;
    case PuppetMotion::Down:
      next.translation() -= kTeleopElevationStep * Eigen::Vector3d::UnitZ();
      break;
    case PuppetMotion::YawLeft:
      next.linear()
          = Eigen::AngleAxisd(kTeleopYawStep, Eigen::Vector3d::UnitZ())
                .toRotationMatrix()
            * current.linear();
      break;
    case PuppetMotion::YawRight:
      next.linear()
          = Eigen::AngleAxisd(-kTeleopYawStep, Eigen::Vector3d::UnitZ())
                .toRotationMatrix()
            * current.linear();
      break;
  }

  dart::dynamics::FreeJoint::setTransformOf(rootBody, next);
  return true;
}

std::vector<dart::gui::KeyboardAction> createHuboPuppetKeyboardActions(
    const dart::dynamics::SkeletonPtr& hubo,
    const std::vector<std::shared_ptr<HuboPuppetScene::TargetState>>&
        targetStates,
    const Eigen::VectorXd& restConfiguration,
    const HuboWholeBodySolverState& wholeBodySolver)
{
  struct Config
  {
    char key;
    const char* label;
    PuppetMotion motion;
  };

  const std::array<Config, 8> configs{{
      {'w', "Move Hubo forward", PuppetMotion::Forward},
      {'s', "Move Hubo backward", PuppetMotion::Backward},
      {'a', "Move Hubo left", PuppetMotion::Left},
      {'d', "Move Hubo right", PuppetMotion::Right},
      {'f', "Raise Hubo root", PuppetMotion::Up},
      {'z', "Lower Hubo root", PuppetMotion::Down},
      {'q', "Yaw Hubo left", PuppetMotion::YawLeft},
      {'e', "Yaw Hubo right", PuppetMotion::YawRight},
  }};

  std::vector<dart::gui::KeyboardAction> actions;
  actions.reserve(configs.size() + targetStates.size() + 6);
  for (const Config& config : configs) {
    dart::gui::KeyboardAction action;
    action.label = config.label;
    action.shortcut = dart::gui::KeyboardShortcut::characterKey(config.key);
    action.repeat = true;
    action.callback = [hubo, motion = config.motion](
                          dart::gui::KeyboardActionContext& context) {
      if (applyRootTeleoperationStep(hubo, motion)) {
        solveHuboWholeBody(hubo);
        if (context.lifecycle != nullptr) {
          context.lifecycle->paused = true;
        }
      }
    };
    actions.push_back(std::move(action));
  }

  for (const auto& state : targetStates) {
    if (state == nullptr || state->hotkey == '\0') {
      continue;
    }

    dart::gui::KeyboardAction action;
    action.label = "Toggle Hubo target " + state->label;
    action.shortcut = dart::gui::KeyboardShortcut::characterKey(state->hotkey);
    action.callback = [hubo, state](dart::gui::KeyboardActionContext& context) {
      state->toggle();
      solveHuboWholeBody(hubo);
      if (context.lifecycle != nullptr) {
        context.lifecycle->paused = true;
      }
    };
    actions.push_back(std::move(action));
  }

  const auto addSupportToggle
      = [&](char key, const char* effectorName, const char* label) {
          dart::gui::KeyboardAction action;
          action.label = label;
          action.shortcut = dart::gui::KeyboardShortcut::characterKey(key);
          action.callback =
              [hubo, effectorName](dart::gui::KeyboardActionContext& context) {
                auto* effector
                    = hubo ? hubo->getEndEffector(effectorName) : nullptr;
                auto* support = effector ? effector->getSupport() : nullptr;
                if (support != nullptr) {
                  support->setActive(!support->isActive());
                  solveHuboWholeBody(hubo);
                  if (context.lifecycle != nullptr) {
                    context.lifecycle->paused = true;
                  }
                }
              };
          actions.push_back(std::move(action));
        };
  addSupportToggle('x', "l_foot", "Toggle left Hubo foot support");
  addSupportToggle('c', "r_foot", "Toggle right Hubo foot support");

  dart::gui::KeyboardAction printDofs;
  printDofs.label = "Print Hubo DOFs";
  printDofs.shortcut = dart::gui::KeyboardShortcut::characterKey('p');
  printDofs.callback = [hubo](dart::gui::KeyboardActionContext&) {
    if (hubo == nullptr) {
      return;
    }
    for (std::size_t i = 0; i < hubo->getNumDofs(); ++i) {
      auto* dof = hubo->getDof(i);
      std::cout << dof->getName() << ": " << dof->getPosition() << "\n";
    }
  };
  actions.push_back(std::move(printDofs));

  dart::gui::KeyboardAction resetPosture;
  resetPosture.label = "Reset Hubo relaxed posture";
  resetPosture.shortcut = dart::gui::KeyboardShortcut::characterKey('t');
  resetPosture.callback = [hubo, restConfiguration](
                              dart::gui::KeyboardActionContext& context) {
    if (hubo == nullptr
        || static_cast<std::size_t>(restConfiguration.size())
               != hubo->getNumDofs()) {
      return;
    }

    for (std::size_t i = 0; i < hubo->getNumDofs(); ++i) {
      if (i < 2 || 4 < i) {
        hubo->getDof(i)->setPosition(restConfiguration[static_cast<int>(i)]);
      }
    }
    solveHuboWholeBody(hubo);
    if (context.lifecycle != nullptr) {
      context.lifecycle->paused = true;
    }
  };
  actions.push_back(std::move(resetPosture));

  dart::gui::KeyboardAction optimizePosture;
  optimizePosture.label = "Optimize Hubo posture and balance";
  optimizePosture.shortcut = dart::gui::KeyboardShortcut::characterKey('r');
  optimizePosture.callback
      = [wholeBodySolver, hubo](dart::gui::KeyboardActionContext& context) {
          if (wholeBodySolver.posture != nullptr) {
            wholeBodySolver.posture->enforceIdealPosture = true;
          }
          if (wholeBodySolver.balance != nullptr) {
            wholeBodySolver.balance->setErrorMethod(
                dart::constraint::BalanceConstraint::OPTIMIZE_BALANCE);
          }
          solveHuboWholeBody(hubo);
          if (context.lifecycle != nullptr) {
            context.lifecycle->paused = true;
          }
        };
  actions.push_back(std::move(optimizePosture));

  dart::gui::KeyboardAction restoreBalanceMode;
  restoreBalanceMode.label = "Restore Hubo centroid balance mode";
  restoreBalanceMode.shortcut = dart::gui::KeyboardShortcut::characterKey('r');
  restoreBalanceMode.trigger = dart::gui::KeyboardActionTrigger::Release;
  restoreBalanceMode.callback
      = [wholeBodySolver, hubo](dart::gui::KeyboardActionContext& context) {
          if (wholeBodySolver.posture != nullptr) {
            wholeBodySolver.posture->enforceIdealPosture = false;
          }
          if (wholeBodySolver.balance != nullptr) {
            wholeBodySolver.balance->setErrorMethod(
                dart::constraint::BalanceConstraint::FROM_CENTROID);
          }
          solveHuboWholeBody(hubo);
          if (context.lifecycle != nullptr) {
            context.lifecycle->paused = true;
          }
        };
  actions.push_back(std::move(restoreBalanceMode));
  return actions;
}

dart::gui::Panel createHuboPuppetPanel()
{
  dart::gui::Panel panel;
  panel.title = "Hubo Puppet";
  panel.buildWithContext = [](dart::gui::PanelBuilder& builder,
                              dart::gui::PanelContext& context) {
    builder.text("Hubo whole-body IK puppet");
    builder.text("Press 1-6 to toggle/select targets.");
    builder.text("Left-drag active target gizmo handles.");
    builder.text("Arrow keys and PageUp/PageDown nudge it.");
    builder.text("Hold X/Y/Z with Ctrl-drag to constrain an axis.");
    builder.text("WASD moves the root; Q/E yaw; F/Z height.");
    builder.text("X/C toggles foot support; P prints DOFs; T resets posture.");
    builder.text("Hold R to optimize whole-body posture and balance.");
    builder.text("The support polygon overlay follows active foot support.");
    builder.text("Blue/red COM marker shows support-polygon validity.");
    builder.text("Whole-body IK solves active targets and balance each step.");
    builder.separator();
    if (context.lifecycle != nullptr) {
      if (builder.button(context.lifecycle->paused ? "Resume" : "Pause")) {
        dart::gui::togglePaused(*context.lifecycle);
      }
      builder.sameLine();
      if (builder.button("Step")) {
        dart::gui::requestSingleStep(*context.lifecycle);
      }
    }
    builder.text("time: " + std::to_string(context.simulationTime));
    builder.text("selected: " + context.selectedLabel);
  };
  return panel;
}

dart::gui::RunOptions makeHuboPuppetRunDefaults()
{
  dart::gui::RunOptions options;
  options.width = 1280;
  options.height = 960;
  return options;
}

dart::gui::OrbitCamera makeHuboPuppetCamera()
{
  dart::gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d(0.0, 0.0, 0.50);
  camera.up = Eigen::Vector3d(-0.20, -0.08, 0.98);
  camera.yaw = 0.5118558424318241;
  camera.pitch = 0.22626228031830078;
  camera.distance = 6.285196894290584;
  return camera;
}

} // namespace

int main(int argc, char* argv[])
{
  try {
    HuboPuppetScene scene = createHuboPuppetScene();

    dart::gui::ApplicationOptions options;
    options.world = scene.world;
    options.ikHandles = scene.ikHandles;
    options.gizmos = scene.gizmos;
    options.runDefaults = makeHuboPuppetRunDefaults();
    options.camera = makeHuboPuppetCamera();
    options.preStep = [hubo = scene.hubo,
                       supportOverlay = scene.supportOverlay,
                       supportComOverlay = scene.supportComOverlay]() {
      solveHuboWholeBody(hubo);
      updateHuboSupportPolygonOverlay(hubo, supportOverlay);
      updateHuboSupportComOverlay(hubo, supportComOverlay);
    };
    options.keyboardActions = createHuboPuppetKeyboardActions(
        scene.hubo,
        scene.targetStates,
        scene.restConfiguration,
        scene.wholeBodySolver);
    options.panels.push_back(createHuboPuppetPanel());
    return dart::gui::runApplication(argc, argv, options);
  } catch (const std::exception& e) {
    std::cerr << "hubo_puppet: " << e.what() << "\n";
    return 1;
  }
}
