/*
 * Copyright (c) 2011-2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>
 *            Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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

#ifndef DART_DYNAMICS_JOINT_H_
#define DART_DYNAMICS_JOINT_H_

#include <string>
#include <vector>

#include "dart/math/Geometry.h"
#include "dart/dynamics/GenCoordSystem.h"

namespace dart {
namespace renderer {
class RenderInterface;
}  // namespace renderer
}  // namespace dart

namespace dart {
namespace dynamics {

class BodyNode;
class Skeleton;

/// \brief Joint
class Joint : public GenCoordSystem
{
public:
  /// \brief Friend class declaration
  friend class BodyNode;

  /// \brief Friend class declaration
  friend class Skeleton;

public:
  /// \brief Constructor
  explicit Joint(const std::string& _name = "Noname Joint");

  /// \brief Destructor
  virtual ~Joint();

  //------------------------------ Properties ----------------------------------
  /// \brief Set joint name
  void setName(const std::string& _name);

  /// \brief Get joint name
  const std::string& getName() const;

  /// \brief Get skeleton that this joint belongs to. The skeleton set by
  /// init().
  Skeleton* getSkeleton() const;

  /// \brief Get index of this joint in the skeleton that this joint belongs to
  int getSkeletonIndex() const;

  /// \brief Set transformation from parent body node to this joint
  virtual void setTransformFromParentBodyNode(const Eigen::Isometry3d& _T);

  /// \brief Set transformation from child body node to this joint
  virtual void setTransformFromChildBodyNode(const Eigen::Isometry3d& _T);

  /// \brief Get transformation from parent body node to this joint
  const Eigen::Isometry3d& getTransformFromParentBodyNode() const;

  /// \brief Get transformation from child body node to this joint
  const Eigen::Isometry3d& getTransformFromChildBodyNode() const;

  /// \brief Set to enforce joint position limit
  void setPositionLimited(bool _isPositionLimited);

  /// \brief Get whether enforcing joint position limit
  bool isPositionLimited() const;

  /// \brief Set spring stiffness for spring force.
  /// \param[in] _idx Index of joint axis.
  /// \param[in] _k Spring stiffness.
  void setSpringStiffness(int _idx, double _k);

  /// \brief Get spring stiffnes for spring force.
  /// \param[in] _idx Index of joint axis.
  double getSpringStiffness(int _idx) const;

  /// \brief Set rest position for spring force.
  /// \param[in] _idx Index of joint axis.
  /// \param[in] _q0 Rest position.
  void setRestPosition(int _idx, double _q0);

  /// \brief Get rest position for spring force.
  /// \param[in] _idx Index of joint axis.
  /// \return Rest position.
  double getRestPosition(int _idx) const;

  /// \brief Set damping coefficient for viscous force.
  /// \param[in] _idx Index of joint axis.
  /// \param[in] _d Damping coefficient.
  void setDampingCoefficient(int _idx, double _d);

  /// \brief Get damping coefficient for viscous force.
  /// \param[in] _idx Index of joint axis.
  double getDampingCoefficient(int _idx) const;

  //----------------- Interface for generalized coordinates --------------------
  /// \brief Set single configuration in terms of generalized coordinates
  /// \param[in] _updateTransforms True to update transformations of body nodes
  /// \param[in] _updateVels True to update spacial velocities of body nodes
  /// \param[in] _updateAccs True to update spacial accelerations of body nodes
  virtual void setConfig(size_t _idx, double _config,
                         bool _updateTransforms = true,
                         bool _updateVels = false,
                         bool _updateAccs = false);

  /// \brief Set configurations in terms of generalized coordinates
  /// \param[in] _updateTransforms True to update transformations of body nodes
  /// \param[in] _updateVels True to update spacial velocities of body nodes
  /// \param[in] _updateAccs True to update spacial accelerations of body nodes
  virtual void setConfigs(const Eigen::VectorXd& _configs,
                          bool _updateTransforms = true,
                          bool _updateVels = false,
                          bool _updateAccs = false);

  /// \brief Set single generalized velocity
  /// \param[in] _updateVels True to update spacial velocities of body nodes
  /// \param[in] _updateAccs True to update spacial accelerations of body nodes
  virtual void setGenVel(size_t _idx, double _genVel,
                         bool _updateVels = true,
                         bool _updateAccs = false);

  /// \brief Set generalized velocities
  /// \param[in] _updateVels True to update spacial velocities of body nodes
  /// \param[in] _updateAccs True to update spacial accelerations of body nodes
  virtual void setGenVels(const Eigen::VectorXd& _genVels,
                          bool _updateVels = true,
                          bool _updateAccs = false);

  /// \brief Set single generalized acceleration
  /// \param[in] _updateAccs True to update spacial accelerations of body nodes
  virtual void setGenAcc(size_t _idx, double _genAcc,
                         bool _updateAccs );

  /// \brief Set generalized accelerations
  /// \param[in] _updateAccs True to update spacial accelerations of body nodes
  virtual void setGenAccs(const Eigen::VectorXd& _genAccs,
                          bool _updateAccs);

  //----------------------------------------------------------------------------
//  void updateVel

  //----------------------------------------------------------------------------
  /// \brief Get potential energy.
  double getPotentialEnergy() const;

  /// \brief Get transformation from parent body node to child body node
  const Eigen::Isometry3d& getLocalTransform() const;

  /// \brief Get generalized Jacobian from parent body node to child body node
  /// w.r.t. local generalized coordinate
  const math::Jacobian& getLocalJacobian() const;

  /// \brief Get time derivative of generalized Jacobian from parent body node
  /// to child body node w.r.t. local generalized coordinate
  const math::Jacobian& getLocalJacobianTimeDeriv() const;

  /// \brief Get whether this joint contains _genCoord.
  /// \param[in] Generalized coordinate to see.
  /// \return True if this joint contains _genCoord.
  bool contains(const GenCoord* _genCoord) const;

  /// \brief Get local index of the dof at this joint; if the dof is not
  /// presented at this joint, return -1.
  int getGenCoordLocalIndex(int _dofSkelIndex) const;

  /// \brief Get constraint wrench expressed in body node frame
  Eigen::Vector6d getBodyConstraintWrench() const;

  /// \brief Get spring force.
  ///
  /// We apply spring force in implicit manner. The spring force is
  /// F = -(springStiffness * q(k+1)), where q(k+1) is approximated as
  /// q(k) + h * dq(k) * h^2 * ddq(k). Since, in the recursive forward dynamics
  /// algorithm, ddq(k) is unknown variable that we want to obtain as the
  /// result, the spring force here is just
  /// F = -springStiffness * (q(k) + h * dq(k)) and
  /// -springStiffness * h^2 * ddq(k) term is rearranged at the recursive
  /// forward dynamics algorithm, and it affects on the articulated inertia.
  /// \sa BodyNode::updateArticulatedInertia(double).
  ///
  /// \param[in] _timeStep Time step used for approximating q(k+1).
  Eigen::VectorXd getSpringForces(double _timeStep) const;

  /// \brief Get damping force.
  ///
  /// We apply the damping force in implicit manner. The damping force is
  /// F = -(dampingCoefficient * dq(k+1)), where dq(k+1) is approximated as
  /// dq(k) + h * ddq(k). Since, in the recursive forward dynamics algorithm,
  /// ddq(k) is unknown variable that we want to obtain as the result, the
  /// damping force here is just F = -(dampingCoefficient * dq(k)) and
  /// -dampingCoefficient * h * ddq(k) term is rearranged at the recursive
  /// forward dynamics algorithm, and it affects on the articulated inertia.
  /// \sa BodyNode::updateArticulatedInertia(double).
  Eigen::VectorXd getDampingForces() const;

  //----------------------------- Rendering ------------------------------------
  /// \brief
  void applyGLTransform(renderer::RenderInterface* _ri);

protected:
  /// \brief Initialize this joint. This function is called by BodyNode::init()
  virtual void init(Skeleton* _skel, int _skelIdx);

  /// \brief Update transformation from parent body node to child body node
  virtual void updateTransform() = 0;

  /// \brief Update generalized Jacobian from parent body node to child body
  ///  node w.r.t. local generalized coordinate
  virtual void updateJacobian() = 0;

  /// \brief Update time derivative of generalized Jacobian from parent body
  /// node to child body node w.r.t. local generalized coordinate
  virtual void updateJacobianTimeDeriv() = 0;

protected:
  /// \brief Joint name
  std::string mName;

  /// \brief Skeleton pointer that this joint belongs to
  Skeleton* mSkeleton;

  /// \brief Unique dof id in skeleton
  int mSkelIndex;

  /// \brief Transformation from parent body node to this joint
  Eigen::Isometry3d mT_ParentBodyToJoint;

  /// \brief Transformation from child body node to this joint
  Eigen::Isometry3d mT_ChildBodyToJoint;

  /// \brief Local transformation.
  Eigen::Isometry3d mT;

  /// \brief Local Jacobian.
  math::Jacobian mS;

  /// \brief Time derivative of local Jacobian.
  math::Jacobian mdS;

  // TODO(JS): Temporary code
public:
  /// \brief Transmitting wrench from parent body to child body expressed in
  /// child body
  Eigen::Vector6d mWrench;

protected:
  /// \brief True if the joint limits are enforced in dynamic simulation.
  bool mIsPositionLimited;

  /// \brief Joint spring stiffness
  std::vector<double> mSpringStiffness;

  /// \brief Rest joint position for joint spring
  std::vector<double> mRestPosition;

  /// \brief Joint damping coefficient
  std::vector<double> mDampingCoefficient;
};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_JOINT_H_
