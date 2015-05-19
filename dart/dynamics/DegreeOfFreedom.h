/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <mxgrey@gatech.edu>
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

#ifndef DART_DYNAMICS_DEGREEOFFREEDOM_H_
#define DART_DYNAMICS_DEGREEOFFREEDOM_H_

#include <string>
#include <memory>
#include <Eigen/Core>

#include "dart/dynamics/BodyNode.h"

namespace dart {
namespace dynamics {

class Skeleton;
class Joint;
class BodyNode;
class SingleDofJoint;
template<size_t> class MultiDofJoint;

/// DegreeOfFreedom class is a proxy class for accessing single degrees of
/// freedom (aka generalized coordinates) of the Skeleton.
class DegreeOfFreedom : public virtual common::Subject
{
public:

  friend class Joint;
  friend class SingleDofJoint;
  template<size_t> friend class MultiDofJoint;
  friend class Skeleton;

  /// Change the name of this DegreeOfFreedom
  ///
  /// The _preserveName argument will be passed to the preserveName(bool)
  /// function. Set _preserveName to true when customizing the name of the
  /// DegreeOfFreedom; that way the name will not be overwritten if the Joint
  /// name changes.
  const std::string& setName(const std::string& _name, bool _preserveName=true);

  /// \brief Get the name of this DegreeOfFreedom
  ///
  /// DegreeOfFreedom's name will be automatically given by the joint it belongs
  /// to. Below is the naming policy:
  ///   - SingleDofJoint \n
  ///       Same name as the joint it belongs to.
  ///   - MultiDofJoint \n
  ///       "[Joint_name]+[affix]" is used. The affix is determined according
  ///       to the role they play in the joint. For example, suppose there's a
  ///       TranslationalJoint named "trans_joint". Then the each dof to be
  ///       named "trans_joint_x", "trans_joint_y", and "trans_joint_z".
  ///   - ZeroDofJoint \n
  ///       ZeroDofJoint doesn't have dof.
  ///
  /// The default name can be renamed by setName() as well.
  const std::string& getName() const;

  /// Prevent Joint::updateDegreeOfFreedomNames() from changing the name of this
  /// degree of freedom. This is useful if you (the user) have customized the
  /// name for this DegreeOfFreedom and want to prevent DART from automatically
  /// updating its name if its parent Joint properties ever change.
  void preserveName(bool _preserve);

  /// Check whether DegreeOfFreedom::lockName(bool) is activate
  bool isNamePreserved() const;

  /// Get this DegreeOfFreedom's index within its Skeleton
  size_t getIndexInSkeleton() const;

  /// Get this DegreeOfFreedom's index within its tree
  size_t getIndexInTree() const;

  /// Get this DegreeOfFreedom's index within its Joint
  size_t getIndexInJoint() const;

  /// Get the index of the tree that this DegreeOfFreedom belongs to
  size_t getTreeIndex() const;

  // -- Command functions ------------------------------------------------------

  /// Set the command of this DegreeOfFreedom
  void setCommand(double _command);

  /// Get the command of this DegreeOfFreedom
  double getCommand() const;

  /// Set the command of this DegreeOfFreedom to zero
  void resetCommand();

  // -- Position functions -----------------------------------------------------

  /// Set the position of this DegreeOfFreedom
  void setPosition(double _position);

  /// Get the position of this DegreeOfFreedom
  double getPosition() const;

  /// Set the position of this DegreeOfFreedom to zero
  void resetPosition();

  /// Set the position limits of this DegreeOfFreedom
  void setPositionLimits(double _lowerLimit, double _upperLimit);

  /// Set the position limits of this DegreeOfFreedom
  void setPositionLimits(const std::pair<double,double>& _limits);

  /// Get the position limits of this DegreeOfFreedom
  std::pair<double,double> getPositionLimits() const;

  /// Set the lower position limit of this DegreeOfFreedom
  void setPositionLowerLimit(double _limit);

  /// Get the lower position limit of this DegreeOfFreedom
  double getPositionLowerLimit() const;

  /// Set the upper position limit of this DegreeOfFreedom
  void setPositionUpperLimit(double _limit);

  /// Get the upper position limit of this DegreeOfFreedom
  double getPositionUpperLimit() const;

  // -- Velocity functions -----------------------------------------------------

  /// Set the velocity of this DegreeOfFreedom
  void setVelocity(double _velocity);

  /// Get the velocity of this DegreeOfFreedom
  double getVelocity() const;

  /// Set the velocity of this DegreeOfFreedom to zero
  void resetVelocity();

  /// Set the velocity limits of this DegreeOfFreedom
  void setVelocityLimits(double _lowerLimit, double _upperLimit);

  /// Set the velocity limtis of this DegreeOfFreedom
  void setVelocityLimits(const std::pair<double,double>& _limits);

  /// Get the velocity limits of this DegreeOfFreedom
  std::pair<double,double> getVelocityLimits() const;

  /// Set the lower velocity limit of this DegreeOfFreedom
  void setVelocityLowerLimit(double _limit);

  /// Get the lower velocity limit of this DegreeOfFreedom
  double getVelocityLowerLimit() const;

  /// Set the upper velocity limit of this DegreeOfFreedom
  void setVelocityUpperLimit(double _limit);

  /// Get the upper Velocity limit of this DegreeOfFreedom
  double getVelocityUpperLimit() const;

  // -- Acceleration functions -------------------------------------------------

  /// Set the acceleration of this DegreeOfFreedom
  void setAcceleration(double _acceleration);

  /// Get the acceleration of this DegreeOfFreedom
  double getAcceleration() const;

  /// Set the acceleration of this DegreeOfFreedom to zero
  void resetAcceleration();

  /// Set the acceleration limits of this DegreeOfFreedom
  void setAccelerationLimits(double _lowerLimit, double _upperLimit);

  /// Set the acceleartion limits of this DegreeOfFreedom
  void setAccelerationLimits(const std::pair<double,double>& _limits);

  /// Get the acceleration limits of this DegreeOfFreedom
  std::pair<double,double> getAccelerationLimits() const;

  /// Set the lower acceleration limit of this DegreeOfFreedom
  void setAccelerationLowerLimit(double _limit);

  /// Get the lower acceleration limit of this DegreeOfFreedom
  double getAccelerationLowerLimit() const;

  /// Set the upper acceleration limit of this DegreeOfFreedom
  void setAccelerationUpperLimit(double _limit);

  /// Get the upper acceleration limit of this DegreeOfFreedom
  double getAccelerationUpperLimit() const;

  // -- Force functions --------------------------------------------------------

  /// Set the generalized force of this DegreeOfFreedom
  void setForce(double _force);

  /// Get the generalized force of this DegreeOfFreedom
  double getForce() const;

  /// Set the generalized force of this DegreeOfFreedom to zero
  void resetForce();

  /// Set the generalized force limits of this DegreeOfFreedom
  void setForceLimits(double _lowerLimit, double _upperLimit);

  /// Set the generalized force limits of this DegreeOfFreedom
  void setForceLimits(const std::pair<double,double>& _limits);

  /// Get the generalized force limits of this DegreeOfFreedom
  std::pair<double,double> getForceLimits() const;

  /// Set the lower generalized force limit of this DegreeOfFreedom
  void setForceLowerLimit(double _limit);

  /// Get the lower generalized force limit of this DegreeOfFreedom
  double getForceLowerLimit() const;

  /// Set the upper generalized force limit of this DegreeOfFreedom
  void setForceUpperLimit(double _limit);

  /// Get the upper generalized force limit of this DegreeOfFreedom
  double getForceUpperLimit() const;

  // -- Velocity Change --------------------------------------------------------

  /// Set the velocity change of this DegreeOfFreedom
  void setVelocityChange(double _velocityChange);

  /// Get the velocity change of this DegreeOfFreedom
  double getVelocityChange() const;

  /// Set the velocity change of this DegreeOfFreedom to zero
  void resetVelocityChange();

  // -- Constraint Impulse -----------------------------------------------------

  /// Set the constraint impulse of this generalized coordinate
  void setConstraintImpulse(double _impulse);

  /// Get the constraint impulse of this generalized coordinate
  double getConstraintImpulse() const;

  /// Set the constraint impulse of this generalized coordinate to zero
  void resetConstraintImpulse();

  // -- Relationships ----------------------------------------------------------

  /// Get the Joint that this DegreeOfFreedom belongs to
  Joint* getJoint();

  /// Get the Joint that this DegreeOfFreedom belongs to
  const Joint* getJoint() const;

  /// Get the Skeleton that this DegreeOfFreedom is inside of
  std::shared_ptr<Skeleton> getSkeleton();

  /// Get the Skeleton that this DegreeOfFreedom is inside of
  std::shared_ptr<const Skeleton> getSkeleton() const;

  /// Get the BodyNode downstream of this DegreeOfFreedom
  BodyNode* getChildBodyNode();

  /// Get the BodyNode downstream of this DegreeOfFreedom
  const BodyNode* getChildBodyNode() const;

  /// Get the BodyNode upstream of this DegreeOfFreedom
  BodyNode* getParentBodyNode();

  /// Get the BodyNode upstream of this DegreeOfFreedom
  const BodyNode* getParentBodyNode() const;

protected:
  /// The constructor is protected so that only Joints can create
  /// DegreeOfFreedom classes
  DegreeOfFreedom(Joint* _joint, size_t _indexInJoint);

  /// \brief Index of this DegreeOfFreedom within its Joint
  ///
  /// The index is determined when this DegreeOfFreedom is created by the Joint
  /// it belongs to. Note that the index should be unique within the Joint.
  size_t mIndexInJoint;

  /// Index of this DegreeOfFreedom within its Skeleton
  size_t mIndexInSkeleton;

  /// Index of this DegreeOfFreedom within its tree
  size_t mIndexInTree;

  /// The joint that this DegreeOfFreedom belongs to
  Joint* mJoint;
  // Note that we do not need to store BodyNode or Skeleton, because we can
  // access them through this joint pointer. Moreover, we never need to check
  // whether mJoint is nullptr, because only Joints are allowed to create a
  // DegreeOfFreedom and every DegreeOfFreedom is deleted when its Joint is
  // destructed.

};

/// TemplateDegreeOfFreedomPtr is a templated class that enables users to create
/// a reference-counting DegreeOfFreedomPtr. Holding onto a DegreeOfFreedomPtr
/// will ensure that the BodyNode (and by extension, Skeleton) corresponding to
/// a DegreeOfFreedom does not get deleted. However, the DegreeOfFreedom itself
/// will be deleted if the parent Joint of the BodyNode is changed to a Joint
/// type that has a small number of DegreesOfFreedom than the local of the
/// DegreeOfFreedom that this DegreeOfFreedomPtr referred to. In such a case,
/// this will trigger and assertion in debug mode, or have a nullptr value if
/// not in debug mode.
template <class DegreeOfFreedomT, class BodyNodeT>
class TemplateDegreeOfFreedomPtr
{
public:

  template<class, class> friend class TemplateDegreeOfFreedomPtr;

  /// Default constructor
  TemplateDegreeOfFreedomPtr() = default;

  /// Typical constructor. _ptr must be a valid pointer (or a nullptr) when
  /// passed to this constructor
  TemplateDegreeOfFreedomPtr(DegreeOfFreedomT* _ptr) { set(_ptr); }

  /// Constructor that takes in a strong DegreeOfFreedomPtrs
  template <class OtherDegreeOfFreedomT, class OtherBodyNodeT>
  TemplateDegreeOfFreedomPtr(
      const TemplateDegreeOfFreedomPtr<OtherDegreeOfFreedomT,
      OtherBodyNodeT>& _dofp)
  {
    set(_dofp.get());
  }

  /// Assignment operator
  TemplateDegreeOfFreedomPtr& operator = (DegreeOfFreedomT* _ptr)
  {
    set(_ptr);
    return *this;
  }

  /// Assignment operator for DegreeOfFreedomPtrs
  template <class OtherDegreeOfFreedomT, class OtherBodyNodeT>
  TemplateDegreeOfFreedomPtr& operator = (
      const TemplateDegreeOfFreedomPtr<OtherDegreeOfFreedomT,
      OtherBodyNodeT>& _dofp)
  {
    set(_dofp.get());
    return *this;
  }

  /// Implicit conversion
  operator DegreeOfFreedomT*() const { return get(); }

  /// Dereferencing operator
  DegreeOfFreedomT& operator*() const { return *get(); }

  /// Dereferencing operation
  DegreeOfFreedomT* operator->() const { return get(); }

  /// Get the raw DegreeOfFreedom pointer
  DegreeOfFreedomT* get() const
  {
    if(nullptr == mBodyNodePtr)
      return nullptr;

    return mBodyNodePtr->getParentJoint()->getDof(mIndex);
  }

  /// Get the BodyNode that this DegreeOfFreedomPtr is tied to
  TemplateBodyNodePtr<BodyNodeT> getBodyNodePtr() const
  {
    return mBodyNodePtr;
  }

  /// Get the local generalized coordinate index that this DegreeOfFreedomPtr is
  /// tied to
  size_t getLocalIndex() const
  {
    if(nullptr == mBodyNodePtr)
      return (size_t)(-1);

    return mIndex;
  }

  /// Set the DegreeOfFreedom for this DegreeOfFreedomPtr
  void set(DegreeOfFreedomT* _ptr)
  {
    if(nullptr == _ptr)
    {
      mBodyNodePtr = nullptr;
      return;
    }

    mBodyNodePtr = _ptr->getChildBodyNode();
    mIndex = _ptr->getIndexInJoint();
  }

  //----------------------------------------------------------------------------
  /// \{ \name Comparison operators
  //----------------------------------------------------------------------------

  /// Equality
  template <class OtherDofT, class OtherBodyNodeT>
  bool operator == (const TemplateDegreeOfFreedomPtr<OtherDofT,
                    OtherBodyNodeT>& _rhs)
  {
    if(nullptr == mBodyNodePtr && nullptr == _rhs.mBodyNodePtr)
      return true;

    if( (mBodyNodePtr == _rhs.mBodyNodePtr) && (mIndex == _rhs.mIndex) )
      return true;

    return false;
  }

  /// Inequality
  template <class OtherDofT, class OtherBodyNodeT>
  bool operator != (const TemplateDegreeOfFreedomPtr<OtherDofT,
                    OtherBodyNodeT>& _rhs)
  {
    return !( *this == _rhs );
  }

  /// Less than
  template <class OtherDofT, class OtherBodyNodeT>
  bool operator < (const TemplateDegreeOfFreedomPtr<OtherDofT,
                   OtherBodyNodeT>& _rhs)
  {
    if( mBodyNodePtr == _rhs.mBodyNodePtr )
      return (mIndex < _rhs.mIndex);

    return (mBodyNodePtr < _rhs.mBodyNodePtr);
  }

  /// Greater than
  template <class OtherDofT, class OtherBodyNodeT>
  bool operator > (const TemplateDegreeOfFreedomPtr<OtherDofT,
                   OtherBodyNodeT>& _rhs)
  {
    if( mBodyNodePtr == _rhs.mBodyNodePtr )
      return (mIndex > _rhs.mIndex);

    return (mBodyNodePtr > _rhs.mBodyNodePtr);
  }

  /// Less than or equal to
  template <class OtherDofT, class OtherBodyNodeT>
  bool operator <= (const TemplateDegreeOfFreedomPtr<OtherDofT,
                    OtherBodyNodeT>& _rhs)
  {
    return (*this < _rhs) || (*this == _rhs);
  }

  /// Greater than or equal to
  template <class OtherDofT, class OtherBodyNodeT>
  bool operator >= (const TemplateDegreeOfFreedomPtr<OtherDofT,
                    OtherBodyNodeT>& _rhs)
  {
    return (*this > _rhs) || (*this == _rhs);
  }

  /// \}

private:
  /// Reference-holding pointer to the child BodyNode of this DegreeOfFreedom
  TemplateBodyNodePtr<BodyNodeT> mBodyNodePtr;

  /// Local index of this DegreeOfFreedom within its Joint
  size_t mIndex;
};

typedef TemplateDegreeOfFreedomPtr<DegreeOfFreedom, BodyNode> DegreeOfFreedomPtr;
typedef TemplateDegreeOfFreedomPtr<const DegreeOfFreedom, const BodyNode> ConstDegreeOfFreedomPtr;

/// TemplateWeakDegreeOfFreedomPtr is a templated class that enables users to
/// create a non-reference-holding WeakDegreeOfFreedomPtr. Holding onto a
/// WeakDegreeOfFreedomPtr will NOT prevent anything from getting deleted, but
/// you can use lock() to check whether the DegreeOfFreedom still exists. If it
/// does exist, it will return a valid strong DegreeOfFreedomPtr. Otherwise it
/// will return a nullptr DegreeOfFreedomPtr.
template <class DegreeOfFreedomT, class BodyNodeT>
class TemplateWeakDegreeOfFreedomPtr
{
public:

  template<class, class> friend class TemplateWeakDegreeOfFreedomPtr;

  /// Default constructor
  TemplateWeakDegreeOfFreedomPtr() { set(nullptr); }

  /// Typical constructor. _ptr must be a valid pointer (or a nullptr) when
  /// passed to this constructor
  TemplateWeakDegreeOfFreedomPtr(DegreeOfFreedomT* _ptr) { set(_ptr); }

  /// Constructor that takes in a WeakDegreeOfFreedomPtr
  template <class OtherDofT, class OtherBodyNodeT>
  TemplateWeakDegreeOfFreedomPtr(
      const TemplateWeakDegreeOfFreedomPtr<OtherDofT,
      OtherBodyNodeT>& _weakPtr)
  {
    set(_weakPtr);
  }

  /// Constructor that takes in a strong DegreeOfFreedomPtr
  template <class OtherDofT, class OtherBodyNodeT>
  TemplateWeakDegreeOfFreedomPtr(
      const TemplateDegreeOfFreedomPtr<OtherDofT,
      OtherBodyNodeT>& _strongPtr)
  {
    set(_strongPtr.get());
  }

  /// Assignment operator for raw DegreeOfFreedom pointers
  TemplateWeakDegreeOfFreedomPtr& operator = (DegreeOfFreedomT* _ptr)
  {
    set(_ptr);
    return *this;
  }

  /// Assignemnt operator for WeakDegreeOfFreedomPtrs
  template <class OtherDofT, class OtherBodyNodeT>
  TemplateWeakDegreeOfFreedomPtr& operator = (
      const TemplateWeakDegreeOfFreedomPtr<OtherDofT,
      OtherBodyNodeT>& _weakPtr)
  {
    set(_weakPtr);
    return *this;
  }

  /// Assignment operator for strong DegreeOfFreedomPtrs
  template <class OtherDofT, class OtherBodyNodeT>
  TemplateWeakDegreeOfFreedomPtr& operator = (
      const TemplateDegreeOfFreedomPtr<OtherDofT,
      OtherBodyNodeT>& _strongPtr)
  {
    set(_strongPtr.get());
    return *this;
  }

  /// Locks the DegreeOfFreedom reference to ensure that the referenced
  /// DegreeOfFreedom is currently still available. If the DegreeOfFreedom
  /// is not available any longer (i.e. has been deleted), then this will return
  /// a nullptr.
  TemplateDegreeOfFreedomPtr<DegreeOfFreedomT, BodyNodeT> lock() const
  {
    TemplateBodyNodePtr<BodyNodeT> bodyNode = mWeakBodyNode.lock();
    if(nullptr == bodyNode)
      return nullptr;

    return TemplateDegreeOfFreedomPtr<DegreeOfFreedomT, BodyNodeT>(
          bodyNode->getParentJoint()->getDof(mIndex));
  }

  /// Set the DegreeOfFreedom for this WeakDegreeOfFreedomPtr
  void set(DegreeOfFreedomT* _ptr)
  {
    if(nullptr == _ptr)
    {
      mWeakBodyNode = nullptr;
      mIndex = 0;
      return;
    }

    mWeakBodyNode = _ptr->getChildBodyNode();
    mIndex = _ptr->getIndexInJoint();
  }

  /// Attempt to set the DegreeOfFreedom for this WeakDegreeOfFreedomPtr based
  /// on another WeakDegreeOfFreedomPtr
  template <class OtherDofT, class OtherBodyNodeT>
  void set(const TemplateWeakDegreeOfFreedomPtr<OtherDofT,
           OtherBodyNodeT>& _weakPtr)
  {
    mWeakBodyNode = _weakPtr.mWeakBodyNode;
    mIndex = _weakPtr.mIndex;
  }

private:
  /// Weak pointer to the child BodyNode of this DegreeOfFreedom
  TemplateWeakBodyNodePtr<BodyNodeT> mWeakBodyNode;

  /// Local index of this DegreeOfFreedom within its Joint
  size_t mIndex;
};

typedef TemplateWeakDegreeOfFreedomPtr<DegreeOfFreedom, BodyNode> WeakDegreeOfFreedomPtr;
typedef TemplateWeakDegreeOfFreedomPtr<const DegreeOfFreedom, const BodyNode> WeakConstDegreeOfFreedomPtr;

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_DEGREEOFFREEDOM_H_
