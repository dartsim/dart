/*
 * Copyright (c) 2011-2019, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/io/io.hpp>
#include <dart/io/urdf/urdf.hpp>

using namespace dart::dynamics;
using namespace dart::simulation;

class RelaxedPosture : public dart::optimizer::Function
{
public:

  RelaxedPosture(const Eigen::VectorXd& idealPosture,
                 const Eigen::VectorXd& lower, const Eigen::VectorXd& upper,
                 const Eigen::VectorXd& weights, bool enforceIdeal = false)
    : enforceIdealPosture(enforceIdeal),
      mIdeal(idealPosture),
      mLower(lower),
      mUpper(upper),
      mWeights(weights)
  {
    int dofs = mIdeal.size();
    if(mLower.size() != dofs || mWeights.size() != dofs || mUpper.size() != dofs)
    {
      dterr << "[RelaxedPose::RelaxedPose] Dimension mismatch:\n"
            << "  ideal:   " << mIdeal.size()   << "\n"
            << "  lower:   " << mLower.size()   << "\n"
            << "  upper:   " << mUpper.size()   << "\n"
            << "  weights: " << mWeights.size() << "\n";
    }
    mResultVector.setZero(dofs);
  }

  double eval(const Eigen::VectorXd& _x) const override
  {
    computeResultVector(_x);
    return 0.5 * mResultVector.dot(mResultVector);
  }

  void evalGradient(const Eigen::VectorXd& _x,
                    Eigen::Map<Eigen::VectorXd> _grad) const override
  {
    computeResultVector(_x);

    _grad.setZero();
    int smaller = std::min(mResultVector.size(), _grad.size());
    for(int i=0; i < smaller; ++i)
      _grad[i] = mResultVector[i];
  }

  void computeResultVector(const Eigen::VectorXd& _x) const
  {
    mResultVector.setZero();

    if(enforceIdealPosture)
    {
      for(int i=0; i < _x.size(); ++i)
      {
        if(mIdeal.size() <= i)
          break;

        mResultVector[i] = mWeights[i]*(_x[i] - mIdeal[i]);
      }
    }
    else
    {
      for(int i=0; i < _x.size(); ++i)
      {
        if(mIdeal.size() <= i)
          break;

        if(_x[i] < mLower[i])
          mResultVector[i] = mWeights[i]*(_x[i] - mLower[i]);
        else if(mUpper[i] < _x[i])
          mResultVector[i] = mWeights[i]*(_x[i] - mUpper[i]);
      }
    }
  }

  bool enforceIdealPosture;

protected:

  mutable Eigen::VectorXd mResultVector;

  Eigen::VectorXd mIdeal;

  Eigen::VectorXd mLower;

  Eigen::VectorXd mUpper;

  Eigen::VectorXd mWeights;
};

static inline bool checkDist(Eigen::Vector3d& p, double a, double b)
{
  double d = p.norm();
  double dmax = a+b;
  double dmin = fabs(a-b);

  if (d > dmax)
  {
    p *= dmax/d;
    return false;
  }
  else if (d < dmin)
  {
    p *= dmin/d;
    return false;
  }
  else
  {
    return true;
  }
}

static inline void clamp_sincos(double& sincos, bool& valid)
{
  if (sincos < -1)
  {
    valid = false;
    sincos = -1;
  }
  else if (sincos > 1)
  {
    valid = false;
    sincos = 1;
  }
}

static inline Eigen::Vector3d flipEuler3Axis(const Eigen::Vector3d& u)
{
  Eigen::Vector3d v;
  v[0] = u[0] - M_PI;
  v[1] = M_PI - u[1];
  v[2] = u[2] - M_PI;
  return v;
}

/// The HuboArmIK is based on the derivation of Hubo's arm IK by Matt Zucker.
class HuboArmIK : public InverseKinematics::Analytical
{
public:

  HuboArmIK(InverseKinematics* _ik, const std::string& baseLinkName,
            const Analytical::Properties& properties = Analytical::Properties())
    : Analytical(_ik, "HuboArmIK_"+baseLinkName, properties),
      configured(false),
      mBaseLinkName(baseLinkName)
  {
    // Do nothing
  }

  std::unique_ptr<GradientMethod> clone(InverseKinematics* _newIK) const override
  {
    return dart::common::make_unique<HuboArmIK>(
        _newIK, mBaseLinkName, getAnalyticalProperties());
  }

  const std::vector<Solution>& computeSolutions(
      const Eigen::Isometry3d& _desiredBodyTf) override
  {
    mSolutions.clear();
    mSolutions.reserve(8);

    if(!configured)
    {
      configure();

      if(!configured)
      {
        dtwarn << "[HuboArmIK::computeSolutions] This analytical IK was not able "
               << "to configure properly, so it will not be able to compute "
               << "solutions\n";
        return mSolutions;
      }
    }

    const BodyNodePtr& base = mBaseLink.lock();
    if(nullptr == base)
    {
      dterr << "[HuboArmIK::computeSolutions] Attempting to perform an IK on a "
            << "limb that no longer exists [" << getMethodName() << "]!\n";
      assert(false);
      return mSolutions;
    }

    if(nullptr == mWristEnd)
    {
      dterr << "[HuboArmIK::computeSolutions] Attempting to perform IK without "
            << "a wrist!\n";
      assert(false);
      return mSolutions;
    }

    const std::size_t SP = 0;
    const std::size_t SR = 1;
    const std::size_t SY = 2;
    const std::size_t EP = 3;
    const std::size_t WY = 4;
    const std::size_t WP = 5;

    const SkeletonPtr& skel = base->getSkeleton();

    Eigen::Isometry3d B =
        base->getParentBodyNode()->getWorldTransform().inverse()
        * _desiredBodyTf * mWristEnd->getTransform(mIK->getNode());

    Eigen::Isometry3d shoulder_from_wrist = shoulderTf.inverse() * B;
    Eigen::Vector3d p = shoulder_from_wrist.inverse().translation();

    const double a2 = L5*L5 + L4*L4;
    const double b2 = L3*L3 + L4*L4;
    const double a = sqrt(a2);
    const double b = sqrt(b2);

    const double alpha = atan2(L5, L4);
    const double beta = atan2(L3, L4);

    bool startValid = checkDist(p, a, b);

    double c2 = p.dot(p);
    double x = p.x();
    double y = p.y();
    double z = p.z();

    for(std::size_t i = 0; i < 8; ++i)
    {
      const int flipEP = alterantives(i,0);
      const int incWY = alterantives(i,1);
      const int flipShoulder = alterantives(i,2);

      Eigen::Vector6d testQ;
      bool isValid = startValid;

      double cosGamma = (a2 + b2 - c2) / (2*a*b);
      clamp_sincos(cosGamma, isValid);

      double gamma = flipEP * acos( cosGamma );
      double theta3 = alpha + beta + gamma - 2*M_PI;

      testQ(EP) = theta3;

      double c3 = cos(theta3);
      double s3 = sin(theta3);

      double numer = -y;
      double denom = (-L4*c3 - L3*s3 + L4);

      double s2, theta2;

      if(std::abs(denom) < zeroSize)
      {
        isValid = false;
        const double& prevWY = skel->getPosition(mDofs[WY]);
        theta2 = incWY ? prevWY : M_PI - prevWY;
        s2 = sin(theta2);
      }
      else
      {
        s2 = numer / denom;
        clamp_sincos(s2, isValid);
        theta2 = incWY ? M_PI - asin(s2) : asin(s2);
      }

      testQ(WY) = theta2;

      double c2 = cos(theta2);

      double r =  L4*c2 - L4*c2*c3 - L3*s3*c2;
      double q = -L4*s3 + L3*c3 + L5;

      double det = -(q*q + r*r);

      if(std::abs(det) < zeroSize)
        isValid = false;

      double k = det < 0 ? -1 : 1;

      double ks1 = k*( q*x - r*z );
      double kc1 = k*(-r*x - q*z );

      double theta1 = atan2(ks1, kc1);
      testQ(WP) = theta1;

      Eigen::Quaterniond Rlower =
        Eigen::Quaterniond(Eigen::AngleAxisd(testQ(EP), Eigen::Vector3d::UnitY())) *
        Eigen::Quaterniond(Eigen::AngleAxisd(testQ(WY), Eigen::Vector3d::UnitZ())) *
        Eigen::Quaterniond(Eigen::AngleAxisd(testQ(WP), Eigen::Vector3d::UnitY()));

      Eigen::Matrix3d Rupper = B.rotation() * Rlower.inverse().matrix();

      Eigen::Vector3d euler = Rupper.eulerAngles(1, 0, 2);

      if(flipShoulder)
        euler = flipEuler3Axis(euler);

      testQ(SP) = euler[0];
      testQ(SR) = euler[1];
      testQ(SY) = euler[2];

      for(std::size_t j=0; j < 6; ++j)
      {
        testQ[j] = dart::math::wrapToPi(testQ[j]);
        if(std::abs(testQ[j]) < zeroSize)
          testQ[j] = 0.0;
      }

      int validity = isValid? VALID : OUT_OF_REACH;
      mSolutions.push_back(Solution(testQ, validity));
    }

    checkSolutionJointLimits();

    return mSolutions;
  }

  const std::vector<std::size_t>& getDofs() const override
  {
    if(!configured)
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
    if(nullptr == base)
    {
      dterr << "[HuboArmIK::configure] base link is a nullptr\n";
      assert(false);
      return;
    }

    const SkeletonPtr& skel = base->getSkeleton();
    const BodyNodePtr& pelvis = skel->getBodyNode("Body_TSY");
    if(nullptr == pelvis)
    {
      dterr << "[HuboArmIK::configure] Could not find Hubo's pelvis "
            << "(Body_TSY)\n";
      assert(false);
      return;
    }

    Eigen::Vector6d saved_q;

    DegreeOfFreedom* dofs[6];
    BodyNode* bn = base;
    for(std::size_t i=0; i < 6; ++i)
    {
      Joint* joint = bn->getParentJoint();
      if(joint->getNumDofs() != 1)
      {
        dterr << "[HuboArmIK::configure] Invalid number of DOFs ("
              << joint->getNumDofs() << ") in the Joint [" << joint->getName()
              << "]\n";
        assert(false);
        return;
      }

      dofs[i] = joint->getDof(0);
      saved_q[i] = dofs[i]->getPosition();
      dofs[i]->setPosition(0.0);
      bn = bn->getChildBodyNode(0);
    }

    BodyNode* elbow = dofs[3]->getChildBodyNode();
    L3 = std::abs(elbow->getTransform(dofs[2]->getParentBodyNode()).translation()[2]);
    L4 = std::abs(elbow->getTransform(dofs[3]->getParentBodyNode()).translation()[0]);

    BodyNode* wrist = dofs[5]->getChildBodyNode();
    Eigen::Isometry3d wrist_tf = wrist->getTransform(elbow);
    L5 = std::abs(wrist_tf.translation()[2]);

    shoulderTf = Eigen::Isometry3d::Identity();
    shoulderTf.translate(dofs[3]->getParentBodyNode()->getTransform(pelvis)
        .translation()[0] * Eigen::Vector3d::UnitX());
    shoulderTf.translate(dofs[2]->getParentBodyNode()->getTransform(pelvis)
        .translation()[1] * Eigen::Vector3d::UnitY());
    shoulderTf.translate(dofs[2]->getParentBodyNode()->getTransform(pelvis)
        .translation()[2] * Eigen::Vector3d::UnitZ());

    mWristEnd = dofs[5]->getChildBodyNode();

    alterantives <<
         1,  1,  1,
         1,  1,  0,
         1,  0,  1,
         1,  0,  0,
        -1,  1,  1,
        -1,  1,  0,
        -1,  0,  1,
        -1,  0,  0;

    for(std::size_t i=0; i < 6; ++i)
    {
      dofs[i]->setPosition(saved_q[i]);
      mDofs.push_back(dofs[i]->getIndexInSkeleton());
    }

    configured = true;
  }

  mutable bool configured;

  mutable Eigen::Isometry3d shoulderTf;
  mutable Eigen::Isometry3d wristTfInv;
  mutable Eigen::Isometry3d mNodeOffsetTfInv;
  mutable double L3, L4, L5;

  mutable Eigen::Matrix<int, 8, 3> alterantives;

  mutable std::vector<std::size_t> mDofs;

  std::string mBaseLinkName;
  mutable WeakBodyNodePtr mBaseLink;

  mutable JacobianNode* mWristEnd;
};

class HuboLegIK : public InverseKinematics::Analytical
{
public:

  /// baseLink should be Body_LHY or Body_RHY
  HuboLegIK(InverseKinematics* _ik, const std::string& baseLinkName,
            const Analytical::Properties& properties = Analytical::Properties())
    : Analytical(_ik, "HuboLegIK_"+baseLinkName, properties),
      configured(false),
      mBaseLinkName(baseLinkName)
  {
    // Do nothing
  }

  std::unique_ptr<GradientMethod> clone(InverseKinematics* _newIK) const override
  {
    return dart::common::make_unique<HuboLegIK>(
        _newIK, mBaseLinkName, getAnalyticalProperties());
  }

  const std::vector<Solution>& computeSolutions(
      const Eigen::Isometry3d& _desiredBodyTf) override
  {
    mSolutions.clear();
    mSolutions.reserve(8);

    if(!configured)
    {
      configure();

      if(!configured)
      {
        dtwarn << "[HuboLegIK::computeSolutions] This analytical IK was not able "
              << "to configure properly, so it will not be able to compute "
              << "solutions\n";
        return mSolutions;
      }
    }

    const BodyNodePtr& base = mBaseLink.lock();
    if(nullptr == base)
    {
      dterr << "[HuboLegIK::computeSolutions] Attempting to perform IK on a "
            << "limb that no longer exists!\n";
      assert(false);
      return mSolutions;
    }

    double nx, ny, sx, sy, ax, ay, az, px, py, pz;
    double q1, q2, q3, q4, q5, q6;
    double S2, S4, S6;
    double C2, C4, C5, C6;
    double C45, psi, q345;
    std::complex<double> radical;
    std::complex<double> sqrt_radical;
    Eigen::Isometry3d B, Binv;

    Eigen::Vector6d testQ;

    B = (base->getParentBodyNode()->getWorldTransform() * waist).inverse()
        * _desiredBodyTf * footTfInv;
    Binv = B.inverse();

    nx = Binv(0,0); sx = Binv(0,1); ax = Binv(0,2); px = Binv(0,3);
    ny = Binv(1,0); sy = Binv(1,1); ay = Binv(1,2); py = Binv(1,3);
                                    az = Binv(2,2); pz = Binv(2,3);

    for(std::size_t i=0; i < 8; ++i)
    {
      bool isValid = true;

      C4 = ((px+L6)*(px+L6) - L4*L4 - L5*L5 + py*py + pz*pz)/(2*L4*L5);
      radical = 1-C4*C4;
      sqrt_radical = std::sqrt(radical);
      if(sqrt_radical.imag() != 0)
          isValid = false;
      q4 = atan2(alternatives(i,0)*sqrt_radical.real(), C4);

      S4 = sin(q4);
      psi = atan2(S4*L4, C4*L4+L5);
      radical = (px+L6)*(px+L6) + py*py;
      sqrt_radical = std::sqrt(radical);
      if(sqrt_radical.imag() != 0)
          isValid = false;

      q5 = dart::math::wrapToPi(atan2(-pz, alternatives(i,1)*sqrt_radical.real())-psi);

      q6 = atan2(py, -(px+L6));
      C45 = cos(q4+q5);
      C5 = cos(q5);
      if( C45*L4 + C5*L5 < 0 )
          q6 = dart::math::wrapToPi(q6+M_PI);

      S6 = sin(q6);
      C6 = cos(q6);

      S2 = C6*ay + S6*ax;
      radical = 1-S2*S2;
      sqrt_radical = std::sqrt(radical);
      if(sqrt_radical.imag() != 0)
          isValid = false;
      q2 = atan2(S2, alternatives(i,2)*sqrt_radical.real());

      q1 = atan2(C6*sy + S6*sx, C6*ny + S6*nx);
      C2 = cos(q2);
      if( C2 < 0 )
          q1 = dart::math::wrapToPi(q1+M_PI);

      q345 = atan2(-az/C2, -(C6*ax - S6*ay)/C2);
      q3 = dart::math::wrapToPi(q345 - q4 - q5);

      testQ[0]=q1; testQ[1]=q2; testQ[2]=q3; testQ[3]=q4; testQ[4]=q5; testQ[5]=q6;

      for(int k=0; k<testQ.size(); ++k)
          if( fabs(testQ[k]) < zeroSize )
              testQ[k] = 0;

      int validity = isValid? VALID : OUT_OF_REACH;
      mSolutions.push_back(Solution(testQ, validity));
    }

    checkSolutionJointLimits();

    return mSolutions;
  }

  const std::vector<std::size_t>& getDofs() const override
  {
    if(!configured)
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
    if(nullptr == base)
    {
      dterr << "[HuboLegIK::configure] base link is a nullptr\n";
      assert(false);
      return;
    }

    const SkeletonPtr& skel = mIK->getNode()->getSkeleton();
    BodyNode* pelvis = skel->getBodyNode("Body_TSY");
    if(nullptr == pelvis)
    {
      dterr << "[HuboLegIK::configure] Could not find Hubo's pelvis "
            << "(Body_TSY)\n";
      assert(false);
      return;
    }

    Eigen::Vector6d saved_q;

    DegreeOfFreedom* dofs[6];
    BodyNode* bn = base;
    for(std::size_t i=0; i < 6; ++i)
    {
      Joint* joint = bn->getParentJoint();
      if(joint->getNumDofs() != 1)
      {
        dterr << "[HuboLegIK::configure] Invalid number of DOFs ("
              << joint->getNumDofs() << ") in the Joint [" << joint->getName()
              << "]\n";
        assert(false);
        return;
      }

      dofs[i] = joint->getDof(0);
      saved_q[i] = dofs[i]->getPosition();
      dofs[i]->setPosition(0.0);

      if(bn->getNumChildBodyNodes() > 0)
        bn = bn->getChildBodyNode(0);
    }

    L4 = std::abs(dofs[3]->getChildBodyNode()->
        getRelativeTransform().translation()[2]);

    L5 = std::abs(dofs[4]->getChildBodyNode()->
        getRelativeTransform().translation()[2]);

    // This offset will be taken care of with footTfInv
    L6 = 0.0;

    hipRotation = Eigen::Isometry3d::Identity();
    hipRotation.rotate(Eigen::AngleAxisd(90*M_PI/180.0,
                                         Eigen::Vector3d::UnitZ()));

    waist = dofs[2]->getChildBodyNode()->getTransform(
              dofs[0]->getParentBodyNode()) * hipRotation;

    footTfInv = Eigen::Isometry3d::Identity();
    footTfInv.rotate(Eigen::AngleAxisd(-90*M_PI/180.0,
                                       Eigen::Vector3d::UnitY()));
    footTfInv = footTfInv * mIK->getNode()->getTransform(dofs[5]->getChildBodyNode());
    footTfInv = footTfInv.inverse();

    alternatives <<
         1,  1,  1,
         1,  1, -1,
         1, -1,  1,
         1, -1, -1,
        -1,  1,  1,
        -1,  1, -1,
        -1, -1,  1,
        -1, -1, -1;

    for(std::size_t i=0; i < 6; ++i)
    {
      dofs[i]->setPosition(saved_q[i]);
      mDofs.push_back(dofs[i]->getIndexInSkeleton());
    }

    configured = true;
  }

  mutable double L4, L5, L6;
  mutable Eigen::Isometry3d waist;
  mutable Eigen::Isometry3d hipRotation;
  mutable Eigen::Isometry3d footTfInv;
  mutable Eigen::Matrix<int, 8, 3> alternatives;

  mutable std::vector<std::size_t> mDofs;

  mutable bool configured;

  std::string mBaseLinkName;

  mutable WeakBodyNodePtr mBaseLink;

};

class TeleoperationWorld : public dart::gui::osg::WorldNode
{
public:

  enum MoveEnum_t
  {
    MOVE_Q = 0,
    MOVE_W,
    MOVE_E,
    MOVE_A,
    MOVE_S,
    MOVE_D,
    MOVE_F,
    MOVE_Z,

    NUM_MOVE
  };

  TeleoperationWorld(WorldPtr _world, SkeletonPtr _robot)
    : dart::gui::osg::WorldNode(_world),
      mHubo(_robot),
      iter(0),
      l_foot(_robot->getEndEffector("l_foot")),
      r_foot(_robot->getEndEffector("r_foot")),
      l_hand(_robot->getEndEffector("l_hand")),
      r_hand(_robot->getEndEffector("r_hand"))
  {
    mMoveComponents.resize(NUM_MOVE, false);
    mAnyMovement = false;
    mAmplifyMovement = false;
  }

  void setMovement(const std::vector<bool>& moveComponents)
  {
    mMoveComponents = moveComponents;

    mAnyMovement = false;

    for(bool move : mMoveComponents)
    {
      if(move)
      {
        mAnyMovement = true;
        break;
      }
    }
  }

  void customPreRefresh() override
  {
    if(mAnyMovement)
    {
      Eigen::Isometry3d old_tf = mHubo->getBodyNode(0)->getWorldTransform();
      Eigen::Isometry3d new_tf = Eigen::Isometry3d::Identity();
      Eigen::Vector3d forward = old_tf.linear().col(0);
      forward[2] = 0.0;
      if(forward.norm() > 1e-10)
        forward.normalize();
      else
        forward.setZero();

      Eigen::Vector3d left = old_tf.linear().col(1);
      left[2] = 0.0;
      if(left.norm() > 1e-10)
        left.normalize();
      else
        left.setZero();

      const Eigen::Vector3d& up = Eigen::Vector3d::UnitZ();

      double linearStep = 0.01;
      double elevationStep = 0.2*linearStep;
      double rotationalStep = 2.0*M_PI/180.0;

      if(mAmplifyMovement)
      {
        linearStep *= 2.0;
        elevationStep *= 2.0;
        rotationalStep *= 2.0;
      }

      if(mMoveComponents[MOVE_W])
        new_tf.translate( linearStep*forward);

      if(mMoveComponents[MOVE_S])
        new_tf.translate(-linearStep*forward);

      if(mMoveComponents[MOVE_A])
        new_tf.translate( linearStep*left);

      if(mMoveComponents[MOVE_D])
        new_tf.translate(-linearStep*left);

      if(mMoveComponents[MOVE_F])
        new_tf.translate( elevationStep*up);

      if(mMoveComponents[MOVE_Z])
        new_tf.translate(-elevationStep*up);

      if(mMoveComponents[MOVE_Q])
        new_tf.rotate(Eigen::AngleAxisd( rotationalStep, up));

      if(mMoveComponents[MOVE_E])
        new_tf.rotate(Eigen::AngleAxisd(-rotationalStep, up));

      new_tf.pretranslate(old_tf.translation());
      new_tf.rotate(old_tf.rotation());

      mHubo->getJoint(0)->setPositions(FreeJoint::convertToPositions(new_tf));
    }

    mHubo->getIK(true)->solve();
  }

  bool mAmplifyMovement;

protected:

  SkeletonPtr mHubo;
  std::size_t iter;

  EndEffectorPtr l_foot;
  EndEffectorPtr r_foot;

  EndEffectorPtr l_hand;
  EndEffectorPtr r_hand;

  std::vector<IK::Analytical::Solution> mSolutions;

  Eigen::VectorXd grad;

  // Order: q, w, e, a, s, d
  std::vector<bool> mMoveComponents;

  bool mAnyMovement;
};

class InputHandler : public ::osgGA::GUIEventHandler
{
public:

  InputHandler(dart::gui::osg::Viewer* viewer, TeleoperationWorld* teleop,
               const SkeletonPtr& hubo, const WorldPtr& world)
    : mViewer(viewer),
      mTeleop(teleop),
      mHubo(hubo),
      mWorld(world)
  {
    initialize();
  }

  void initialize()
  {
    mRestConfig = mHubo->getPositions();

    for(std::size_t i=0; i < mHubo->getNumEndEffectors(); ++i)
    {
      const InverseKinematicsPtr ik = mHubo->getEndEffector(i)->getIK();
      if(ik)
      {
        mDefaultBounds.push_back(ik->getErrorMethod().getBounds());
        mDefaultTargetTf.push_back(ik->getTarget()->getRelativeTransform());
        mConstraintActive.push_back(false);
        mEndEffectorIndex.push_back(i);
      }
    }

    mPosture = std::dynamic_pointer_cast<RelaxedPosture>(
          mHubo->getIK(true)->getObjective());

    mBalance = std::dynamic_pointer_cast<dart::constraint::BalanceConstraint>(
          mHubo->getIK(true)->getProblem()->getEqConstraint(1));

    mOptimizationKey = 'r';

    mMoveComponents.resize(TeleoperationWorld::NUM_MOVE, false);
  }

  virtual bool handle(const ::osgGA::GUIEventAdapter& ea,
                      ::osgGA::GUIActionAdapter&) override
  {
    if(nullptr == mHubo)
    {
      return false;
    }

    if( ::osgGA::GUIEventAdapter::KEYDOWN == ea.getEventType() )
    {
      if( ea.getKey() == 'p' )
      {
        for(std::size_t i=0; i < mHubo->getNumDofs(); ++i)
          std::cout << mHubo->getDof(i)->getName() << ": "
                    << mHubo->getDof(i)->getPosition() << std::endl;
        return true;
      }

      if( ea.getKey() == 't' )
      {
        // Reset all the positions except for x, y, and yaw
        for(std::size_t i=0; i < mHubo->getNumDofs(); ++i)
        {
          if( i < 2 || 4 < i )
            mHubo->getDof(i)->setPosition(mRestConfig[i]);
        }
        return true;
      }

      if( '1' <= ea.getKey() && ea.getKey() <= '9' )
      {
        std::size_t index = ea.getKey() - '1';
        if(index < mConstraintActive.size())
        {
          EndEffector* ee = mHubo->getEndEffector(mEndEffectorIndex[index]);
          const InverseKinematicsPtr& ik = ee->getIK();
          if(ik && mConstraintActive[index])
          {
            mConstraintActive[index] = false;

            ik->getErrorMethod().setBounds(mDefaultBounds[index]);
            ik->getTarget()->setRelativeTransform(mDefaultTargetTf[index]);
            mWorld->removeSimpleFrame(ik->getTarget());
          }
          else if(ik)
          {
            mConstraintActive[index] = true;

            // Use the standard default bounds instead of our custom default
            // bounds
            ik->getErrorMethod().setBounds();
            ik->getTarget()->setTransform(ee->getTransform());
            mWorld->addSimpleFrame(ik->getTarget());
          }
        }
        return true;
      }

      if( 'x' == ea.getKey() )
      {
        EndEffector* ee = mHubo->getEndEffector("l_foot");
        ee->getSupport()->setActive(!ee->getSupport()->isActive());
        return true;
      }

      if( 'c' == ea.getKey() )
      {
        EndEffector* ee = mHubo->getEndEffector("r_foot");
        ee->getSupport()->setActive(!ee->getSupport()->isActive());
        return true;
      }

      if(ea.getKey() == ::osgGA::GUIEventAdapter::KEY_Shift_L)
        mTeleop->mAmplifyMovement = true;

      switch(ea.getKey())
      {
        case 'w': case 'W': mMoveComponents[TeleoperationWorld::MOVE_W] = true; break;
        case 'a': case 'A': mMoveComponents[TeleoperationWorld::MOVE_A] = true; break;
        case 's': case 'S': mMoveComponents[TeleoperationWorld::MOVE_S] = true; break;
        case 'd': case 'D': mMoveComponents[TeleoperationWorld::MOVE_D] = true; break;
        case 'q': case 'Q': mMoveComponents[TeleoperationWorld::MOVE_Q] = true; break;
        case 'e': case 'E': mMoveComponents[TeleoperationWorld::MOVE_E] = true; break;
        case 'f': case 'F': mMoveComponents[TeleoperationWorld::MOVE_F] = true; break;
        case 'z': case 'Z': mMoveComponents[TeleoperationWorld::MOVE_Z] = true; break;
      }

      switch(ea.getKey())
      {
        case 'w': case 'a': case 's': case 'd': case 'q': case 'e': case 'f': case 'z':
        case 'W': case 'A': case 'S': case 'D': case 'Q': case 'E': case 'F': case 'Z':
        {
          mTeleop->setMovement(mMoveComponents);
          return true;
        }
      }

      if(mOptimizationKey == ea.getKey())
      {
        if(mPosture)
          mPosture->enforceIdealPosture = true;

        if(mBalance)
          mBalance->setErrorMethod(dart::constraint::BalanceConstraint::OPTIMIZE_BALANCE);

        return true;
      }
    }

    if( ::osgGA::GUIEventAdapter::KEYUP == ea.getEventType() )
    {
      if(ea.getKey() == mOptimizationKey)
      {
        if(mPosture)
          mPosture->enforceIdealPosture = false;

        if(mBalance)
          mBalance->setErrorMethod(dart::constraint::BalanceConstraint::FROM_CENTROID);

        return true;
      }

      if(ea.getKey() == ::osgGA::GUIEventAdapter::KEY_Shift_L)
        mTeleop->mAmplifyMovement = false;

      switch(ea.getKey())
      {
        case 'w': case 'W': mMoveComponents[TeleoperationWorld::MOVE_W] = false; break;
        case 'a': case 'A': mMoveComponents[TeleoperationWorld::MOVE_A] = false; break;
        case 's': case 'S': mMoveComponents[TeleoperationWorld::MOVE_S] = false; break;
        case 'd': case 'D': mMoveComponents[TeleoperationWorld::MOVE_D] = false; break;
        case 'q': case 'Q': mMoveComponents[TeleoperationWorld::MOVE_Q] = false; break;
        case 'e': case 'E': mMoveComponents[TeleoperationWorld::MOVE_E] = false; break;
        case 'f': case 'F': mMoveComponents[TeleoperationWorld::MOVE_F] = false; break;
        case 'z': case 'Z': mMoveComponents[TeleoperationWorld::MOVE_Z] = false; break;
      }

      switch(ea.getKey())
      {
        case 'w': case 'a': case 's': case 'd': case 'q': case'e': case 'f': case 'z':
        case 'W': case 'A': case 'S': case 'D': case 'Q': case 'E': case 'F': case 'Z':
        {
          mTeleop->setMovement(mMoveComponents);
          return true;
        }
      }
    }

    return false;
  }

protected:

  dart::gui::osg::Viewer* mViewer;

  TeleoperationWorld* mTeleop;

  SkeletonPtr mHubo;

  WorldPtr mWorld;

  Eigen::VectorXd mRestConfig;

  std::vector<bool> mConstraintActive;

  std::vector<std::size_t> mEndEffectorIndex;

  std::vector< std::pair<Eigen::Vector6d, Eigen::Vector6d> > mDefaultBounds;

  dart::common::aligned_vector<Eigen::Isometry3d> mDefaultTargetTf;

  std::shared_ptr<RelaxedPosture> mPosture;

  std::shared_ptr<dart::constraint::BalanceConstraint> mBalance;

  char mOptimizationKey;

  std::vector<bool> mMoveComponents;
};


SkeletonPtr createGround()
{
  // Create a Skeleton to represent the ground
  SkeletonPtr ground = Skeleton::create("ground");
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  double thickness = 0.01;
  tf.translation() = Eigen::Vector3d(0,0,-thickness/2.0);
  WeldJoint::Properties joint;
  joint.mT_ParentBodyToJoint = tf;
  ground->createJointAndBodyNodePair<WeldJoint>(nullptr, joint);
  ShapePtr groundShape =
      std::make_shared<BoxShape>(Eigen::Vector3d(10,10,thickness));

  auto shapeNode = ground->getBodyNode(0)->createShapeNodeWith<
      VisualAspect, CollisionAspect, DynamicsAspect>(groundShape);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue(0.2));

  return ground;
}

SkeletonPtr createHubo()
{
  dart::io::DartLoader loader;
  loader.addPackageDirectory("drchubo", DART_DATA_PATH"/urdf/drchubo");
  SkeletonPtr hubo =
      loader.parseSkeleton(DART_DATA_PATH"/urdf/drchubo/drchubo.urdf");

  for(std::size_t i = 0; i < hubo->getNumBodyNodes(); ++i)
  {
    BodyNode* bn = hubo->getBodyNode(i);
    if(bn->getName().substr(0, 7) == "Body_LF"
       || bn->getName().substr(0, 7) == "Body_RF")
    {
      bn->remove();
      --i;
    }
  }

  return hubo;
}

void setStartupConfiguration(const SkeletonPtr& hubo)
{
  hubo->getDof("LHP")->setPosition(-45.0*M_PI/180.0);
  hubo->getDof("LKP")->setPosition( 90.0*M_PI/180.0);
  hubo->getDof("LAP")->setPosition(-45.0*M_PI/180.0);

  hubo->getDof("RHP")->setPosition(-45.0*M_PI/180.0);
  hubo->getDof("RKP")->setPosition( 90.0*M_PI/180.0);
  hubo->getDof("RAP")->setPosition(-45.0*M_PI/180.0);

  hubo->getDof("LSP")->setPosition(  30.0*M_PI/180.0);
  hubo->getDof("LEP")->setPosition(-120.0*M_PI/180.0);

  hubo->getDof("RSP")->setPosition(  30.0*M_PI/180.0);
  hubo->getDof("REP")->setPosition(-120.0*M_PI/180.0);


  hubo->getDof("LSY")->setPositionLowerLimit(-90.0*M_PI/180.0);
  hubo->getDof("LSY")->setPositionUpperLimit( 90.0*M_PI/180.0);
  hubo->getDof("LWY")->setPositionLowerLimit(-90.0*M_PI/180.0);
  hubo->getDof("LWY")->setPositionUpperLimit( 90.0*M_PI/180.0);

  hubo->getDof("RSY")->setPositionLowerLimit(-90.0*M_PI/180.0);
  hubo->getDof("RSY")->setPositionUpperLimit( 90.0*M_PI/180.0);
  hubo->getDof("RWY")->setPositionLowerLimit(-90.0*M_PI/180.0);
  hubo->getDof("RWY")->setPositionUpperLimit( 90.0*M_PI/180.0);
}

void setupEndEffectors(const SkeletonPtr& hubo)
{
  Eigen::VectorXd rootjoint_weights = Eigen::VectorXd::Ones(7);
  rootjoint_weights = 0.01*rootjoint_weights;

  double extra_error_clamp = 0.1;

  Eigen::Vector3d linearBounds =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());

  Eigen::Vector3d angularBounds =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());

  Eigen::Isometry3d tf_hand(Eigen::Isometry3d::Identity());
  tf_hand.translate(Eigen::Vector3d(0.0, 0.0, -0.09));

  EndEffector* l_hand = hubo->getBodyNode("Body_LWR")->
      createEndEffector("l_hand");
  l_hand->setDefaultRelativeTransform(tf_hand, true);

  dart::gui::osg::InteractiveFramePtr lh_target(new dart::gui::osg::InteractiveFrame(
                                           Frame::World(), "lh_target"));

  l_hand->getIK(true)->setTarget(lh_target);
  l_hand->getIK()->useWholeBody();

  l_hand->getIK()->setGradientMethod<HuboArmIK>("Body_LSP");

  l_hand->getIK()->getAnalytical()->setExtraDofUtilization(
        IK::Analytical::POST_ANALYTICAL);

  l_hand->getIK()->getAnalytical()->setExtraErrorLengthClamp(extra_error_clamp);

  l_hand->getIK()->getGradientMethod().setComponentWeights(rootjoint_weights);

  l_hand->getIK()->getErrorMethod().setLinearBounds(
        -linearBounds, linearBounds);

  l_hand->getIK()->getErrorMethod().setAngularBounds(
        -angularBounds, angularBounds);


  EndEffector* r_hand = hubo->getBodyNode("Body_RWR")->
      createEndEffector("r_hand");
  r_hand->setDefaultRelativeTransform(tf_hand, true);

  dart::gui::osg::InteractiveFramePtr rh_target(new dart::gui::osg::InteractiveFrame(
                                           Frame::World(), "rh_target"));

  r_hand->getIK(true)->setTarget(rh_target);
  r_hand->getIK()->useWholeBody();

  r_hand->getIK()->setGradientMethod<HuboArmIK>("Body_RSP");

  r_hand->getIK()->getAnalytical()->setExtraDofUtilization(
        IK::Analytical::POST_ANALYTICAL);

  r_hand->getIK()->getAnalytical()->setExtraErrorLengthClamp(extra_error_clamp);

  r_hand->getIK()->getGradientMethod().setComponentWeights(rootjoint_weights);

  r_hand->getIK()->getErrorMethod().setLinearBounds(
        -linearBounds, linearBounds);

  r_hand->getIK()->getErrorMethod().setAngularBounds(
        -angularBounds, angularBounds);


  dart::math::SupportGeometry foot_support;
  foot_support.push_back(Eigen::Vector3d(-0.08,  0.05, 0.0));
  foot_support.push_back(Eigen::Vector3d(-0.18,  0.05, 0.0));
  foot_support.push_back(Eigen::Vector3d(-0.18, -0.05, 0.0));
  foot_support.push_back(Eigen::Vector3d(-0.08, -0.05, 0.0));

  Eigen::Isometry3d tf_foot(Eigen::Isometry3d::Identity());
  double ground_dist = 0.01;
  tf_foot.translation() = Eigen::Vector3d(0.14, 0.0, -0.136+ground_dist);

  linearBounds[2] = 1e-8;
  Eigen::Vector3d ground_offset = ground_dist * Eigen::Vector3d::UnitZ();

  angularBounds[0] = 1e-8;
  angularBounds[1] = 1e-8;

  EndEffector* l_foot = hubo->getBodyNode("Body_LAR")->
      createEndEffector("l_foot");
  l_foot->setDefaultRelativeTransform(tf_foot, true);

  dart::gui::osg::InteractiveFramePtr lf_target(new dart::gui::osg::InteractiveFrame(
                                           Frame::World(), "lf_target"));

  l_foot->getIK(true)->setTarget(lf_target);

  l_foot->getIK()->setHierarchyLevel(1);

  l_foot->getIK()->getErrorMethod().setLinearBounds(
        -linearBounds + ground_offset, linearBounds + ground_offset);
  l_foot->getIK()->getErrorMethod().setAngularBounds(
        -angularBounds, angularBounds);

  l_foot->getIK()->setGradientMethod<HuboLegIK>("Body_LHY");

  l_foot->getSupport(true)->setGeometry(foot_support);
  l_foot->getSupport()->setActive();


  EndEffector* r_foot = hubo->getBodyNode("Body_RAR")->
      createEndEffector("r_foot");
  r_foot->setDefaultRelativeTransform(tf_foot, true);

  dart::gui::osg::InteractiveFramePtr rf_target(new dart::gui::osg::InteractiveFrame(
                                           Frame::World(), "rf_target"));

  r_foot->getIK(true)->setTarget(rf_target);

  r_foot->getIK()->setHierarchyLevel(1);

  r_foot->getIK()->getErrorMethod().setLinearBounds(
        -linearBounds + ground_offset, linearBounds + ground_offset);
  r_foot->getIK()->getErrorMethod().setAngularBounds(
        -angularBounds, angularBounds);

  r_foot->getIK()->setGradientMethod<HuboLegIK>("Body_RHY");

  r_foot->getSupport(true)->setGeometry(foot_support);
  r_foot->getSupport()->setActive();


  dart::math::SupportGeometry peg_support;
  peg_support.push_back(Eigen::Vector3d::Zero());

  linearBounds = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  angularBounds = linearBounds;

  Eigen::Isometry3d tf_peg(Eigen::Isometry3d::Identity());
  tf_peg.translation() = Eigen::Vector3d(0.0, 0.0, 0.09);

  EndEffector* l_peg = hubo->getBodyNode("Body_LWP")->createEndEffector("l_peg");
  l_peg->setDefaultRelativeTransform(tf_peg, true);

  dart::gui::osg::InteractiveFramePtr lp_target(new dart::gui::osg::InteractiveFrame(
                                           Frame::World(), "lp_target"));

  l_peg->getIK(true)->setTarget(lp_target);

  l_peg->getIK()->setGradientMethod<HuboArmIK>("Body_LSP");

  l_peg->getIK()->getErrorMethod().setLinearBounds(
        -linearBounds, linearBounds);
  l_peg->getIK()->getErrorMethod().setAngularBounds(
        -angularBounds, angularBounds);

  l_peg->getSupport(true)->setGeometry(peg_support);

  EndEffector* r_peg = hubo->getBodyNode("Body_RWP")->createEndEffector("r_peg");
  r_peg->setDefaultRelativeTransform(tf_peg, true);

  dart::gui::osg::InteractiveFramePtr rp_target(new dart::gui::osg::InteractiveFrame(
                                           Frame::World(), "rp_target"));

  r_peg->getIK(true)->setTarget(rp_target);

  r_peg->getIK()->setGradientMethod<HuboArmIK>("Body_RSP");

  r_peg->getIK()->getErrorMethod().setLinearBounds(
        -linearBounds, linearBounds);
  r_peg->getIK()->getErrorMethod().setAngularBounds(
        -angularBounds, angularBounds);

  r_peg->getSupport(true)->setGeometry(peg_support);

  double heightChange = -r_foot->getWorldTransform().translation()[2]+ground_dist;
  hubo->getDof("rootJoint_pos_z")->setPosition(heightChange);

  l_foot->getIK()->getTarget()->setTransform(l_foot->getTransform());
  r_foot->getIK()->getTarget()->setTransform(r_foot->getTransform());
}

void enableDragAndDrops(dart::gui::osg::Viewer& viewer, const SkeletonPtr& hubo)
{
  // Turn on drag-and-drop for the whole Skeleton
  for(std::size_t i=0; i < hubo->getNumBodyNodes(); ++i)
    viewer.enableDragAndDrop(hubo->getBodyNode(i), false, false);

  for(std::size_t i=0; i < hubo->getNumEndEffectors(); ++i)
  {
    EndEffector* ee = hubo->getEndEffector(i);
    if(!ee->getIK())
      continue;

    // Check whether the target is an interactive frame, and add it if it is
    if(const auto& frame = std::dynamic_pointer_cast<dart::gui::osg::InteractiveFrame>(
         ee->getIK()->getTarget()))
      viewer.enableDragAndDrop(frame.get());
  }
}

void setupWholeBodySolver(const SkeletonPtr& hubo)
{
  std::shared_ptr<dart::optimizer::GradientDescentSolver> solver =
      std::dynamic_pointer_cast<dart::optimizer::GradientDescentSolver>(
        hubo->getIK(true)->getSolver());

  std::size_t nDofs = hubo->getNumDofs();

  double default_weight = 0.01;
  Eigen::VectorXd weights = default_weight * Eigen::VectorXd::Ones(nDofs);
  weights[2] = 0.0;
  weights[3] = 0.0;
  weights[4] = 0.0;

  Eigen::VectorXd lower_posture = Eigen::VectorXd::Constant(nDofs,
        -std::numeric_limits<double>::infinity());
  lower_posture[0] = -0.35;
  lower_posture[1] = -0.35;
  lower_posture[5] =  0.55;

  Eigen::VectorXd upper_posture = Eigen::VectorXd::Constant(nDofs,
         std::numeric_limits<double>::infinity());
  upper_posture[0] =  0.35;
  upper_posture[1] =  0.50;
  upper_posture[5] =  0.95;

  std::shared_ptr<RelaxedPosture> objective = std::make_shared<RelaxedPosture>(
        hubo->getPositions(), lower_posture, upper_posture, weights);

  hubo->getIK()->setObjective(objective);

  std::shared_ptr<dart::constraint::BalanceConstraint> balance =
      std::make_shared<dart::constraint::BalanceConstraint>(hubo->getIK());
  hubo->getIK()->getProblem()->addEqConstraint(balance);

  balance->setErrorMethod(dart::constraint::BalanceConstraint::FROM_CENTROID);
  balance->setBalanceMethod(dart::constraint::BalanceConstraint::SHIFT_SUPPORT);

  solver->setNumMaxIterations(5);
}

int main()
{
  dart::simulation::WorldPtr world(new dart::simulation::World);

  SkeletonPtr hubo = createHubo();
  setStartupConfiguration(hubo);
  setupEndEffectors(hubo);

  Eigen::VectorXd positions = hubo->getPositions();
  // We make a clone to test whether the cloned version behaves the exact same
  // as the original version.
  hubo = hubo->cloneSkeleton("hubo_copy");
  hubo->setPositions(positions);

  world->addSkeleton(hubo);
  world->addSkeleton(createGround());

  setupWholeBodySolver(hubo);

  ::osg::ref_ptr<TeleoperationWorld> node = new TeleoperationWorld(world, hubo);

  dart::gui::osg::Viewer viewer;
  viewer.allowSimulation(false);
  viewer.addWorldNode(node);

  enableDragAndDrops(viewer, hubo);

  viewer.addEventHandler(new InputHandler(&viewer, node, hubo, world));

  double display_elevation = 0.05;
  viewer.addAttachment(new dart::gui::osg::SupportPolygonVisual(
                         hubo, display_elevation));

  std::cout << viewer.getInstructions() << std::endl;

  std::cout << "Alt + Click:   Try to translate a body without changing its orientation\n"
            << "Ctrl + Click:  Try to rotate a body without changing its translation\n"
            << "Shift + Click: Move a body using only its parent joint\n"
            << "1 -> 6:        Toggle the interactive target of an EndEffector\n"
            << "W A S D:       Move the robot around the scene\n"
            << "Q E:           Rotate the robot counter-clockwise and clockwise\n"
            << "F Z:           Shift the robot's elevation up and down\n"
            << "X C:           Toggle support on the left and right foot\n"
            << "R:             Optimize the robot's posture\n"
            << "T:             Reset the robot to its relaxed posture\n\n"
            << "  The green polygon is the support polygon of the robot, and the blue/red ball is\n"
            << "  the robot's center of mass. The green ball is the centroid of the polygon.\n\n"
            << "Note that this is purely kinematic. Physical simulation is not allowed in this app.\n"
            << std::endl;

  // Set up the window
  viewer.setUpViewInWindow(0, 0, 1280, 960);

  // Set up the default viewing position
  viewer.getCameraManipulator()->setHomePosition(::osg::Vec3( 5.34,  3.00, 1.91),
                                                 ::osg::Vec3( 0.00,  0.00, 0.50),
                                                 ::osg::Vec3(-0.20, -0.08, 0.98));

  // Reset the camera manipulator so that it starts in the new viewing position
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  viewer.run();
}
