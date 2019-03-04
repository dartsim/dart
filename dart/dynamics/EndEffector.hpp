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

#ifndef DART_DYNAMICS_ENDEFFECTOR_HPP_
#define DART_DYNAMICS_ENDEFFECTOR_HPP_

#include "dart/common/Aspect.hpp"
#include "dart/common/SpecializedForAspect.hpp"
#include "dart/common/AspectWithVersion.hpp"
#include "dart/dynamics/FixedJacobianNode.hpp"
#include "dart/dynamics/CompositeNode.hpp"
#include "dart/dynamics/detail/EndEffectorAspect.hpp"

namespace dart {
namespace dynamics {

class BodyNode;
class Skeleton;
class EndEffector;

//==============================================================================
class Support final :
    public common::AspectWithStateAndVersionedProperties<
        Support,
        detail::SupportStateData,
        detail::SupportPropertiesData,
        EndEffector,
        &detail::SupportUpdate>
{
public:

  DART_COMMON_ASPECT_STATE_PROPERTY_CONSTRUCTORS(Support)

  /// Set/Get the support geometry for this EndEffector. The SupportGeometry
  /// represents points in the EndEffector frame that can be used for contact
  /// when solving balancing or manipulation constraints.
  DART_COMMON_SET_GET_ASPECT_PROPERTY(math::SupportGeometry, Geometry)
  // void setGeometry(const math::SupportGeometry&);
  // const math::SupportGeometry& getGeometry() const;

  /// Pass in true if this EndEffector should be used to support the robot, like
  /// a foot
  void setActive(bool _supporting = true);

  /// Get whether this EndEffector is currently being used for support
  bool isActive() const;

};

//==============================================================================
class EndEffector final :
    public common::EmbedPropertiesOnTopOf<
        EndEffector, detail::EndEffectorProperties,
        detail::EndEffectorCompositeBase>
{
public:

  friend class Skeleton;
  friend class BodyNode;

  using UniqueProperties = detail::EndEffectorProperties;

  using BasicProperties = common::Composite::MakeProperties<
      NameAspect,
      FixedFrame,
      EndEffector>;

  using Properties = common::Composite::Properties;

  /// Destructor
  virtual ~EndEffector() = default;

  //----------------------------------------------------------------------------
  /// \{ \name Structural Properties
  //----------------------------------------------------------------------------

  /// Set the Properties of this EndEffector. If _useNow is true, the current
  /// Transform will be set to the new default transform.
  void setProperties(const BasicProperties& _properties);

  /// Set the Properties of this EndEffector. If _useNow is true, the current
  /// Transform will be set to the new default transform.
  void setProperties(const UniqueProperties& properties, bool useNow=false);

  /// Set the AspectProperties of this EndEffector
  void setAspectProperties(const AspectProperties& properties);

  /// Get the Properties of this EndEffector
  Properties getEndEffectorProperties() const;

  /// Copy the State and Properties of another EndEffector
  void copy(const EndEffector& _otherEndEffector);

  /// Copy the State and Properties of another EndEffector
  void copy(const EndEffector* _otherEndEffector);

  /// Copy the State and Properties of another EndEffector
  EndEffector& operator=(const EndEffector& _otherEndEffector);

  /// Set the default relative transform of this EndEffector. The relative
  /// transform of this EndEffector will be set to _newDefaultTf the next time
  /// resetRelativeTransform() is called. If _useNow is set to true, then
  /// resetRelativeTransform() will be called at the end of this function.
  void setDefaultRelativeTransform(const Eigen::Isometry3d& _newDefaultTf,
                                   bool _useNow = false);

  /// Set the current relative transform of this EndEffector to the default
  /// relative transform of this EndEffector. The default relative transform can
  /// be set with setDefaultRelativeTransform()
  void resetRelativeTransform();

  DART_BAKE_SPECIALIZED_ASPECT(Support)

  //----------------------------------------------------------------------------
  /// \{ \name Notifications
  //----------------------------------------------------------------------------

  // Documentation inherited
  void dirtyTransform() override;

  /// \}

protected:

  /// Constructor used by the Skeleton class
  explicit EndEffector(BodyNode* parent, const BasicProperties& properties);

  // Documentation inherited
  Node* cloneNode(BodyNode* _parent) const override;

};

} // namespace dynamics
} // namespace dart


#endif // DART_DYNAMICS_ENDEFFECTOR_HPP_
