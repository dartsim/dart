/*
 * Copyright (c) 2016, Georgia Tech Research Corporation
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

#ifndef DART_DYNAMICS_DETAIL_SOFTBODYNODEPROPERTIES_H_
#define DART_DYNAMICS_DETAIL_SOFTBODYNODEPROPERTIES_H_

#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/PointMass.h"
#include "dart/common/RequiresAspect.h"

namespace dart {
namespace dynamics {

const double DART_DEFAULT_VERTEX_STIFFNESS = 1.0;
const double DART_DEFAULT_EDGE_STIFNESS    = 1.0;
const double DART_DEFAULT_DAMPING_COEFF    = 0.01;

class SoftBodyNode;
class SoftMeshShape;

namespace detail {

//==============================================================================
class SoftBodyAspect;

//==============================================================================
struct SoftBodyNodeUniqueState
{
  /// Array of States for PointMasses
  std::vector<PointMass::State> mPointStates;
};

//==============================================================================
struct SoftBodyNodeUniqueProperties
{
  /// Spring stiffness for vertex deformation restoring spring force of the
  /// point masses
  double mKv;

  /// Spring stiffness for edge deformation restoring spring force of the
  /// point masses
  double mKe;

  /// Damping coefficient
  double mDampCoeff;

  /// Array of Properties for PointMasses
  std::vector<PointMass::Properties> mPointProps;

  // TODO(JS): Let's remove this because this is rendering part
  /// \brief Tri-mesh indexes for rendering.
  std::vector<Eigen::Vector3i> mFaces;

  SoftBodyNodeUniqueProperties(
      double _Kv = DART_DEFAULT_VERTEX_STIFFNESS,
      double _Ke = DART_DEFAULT_EDGE_STIFNESS,
      double _DampCoeff = DART_DEFAULT_DAMPING_COEFF,
      const std::vector<PointMass::Properties>& _points =
                                        std::vector<PointMass::Properties>(),
      const std::vector<Eigen::Vector3i>& _faces =
                                        std::vector<Eigen::Vector3i>());

  virtual ~SoftBodyNodeUniqueProperties() = default;

  /// Add a PointMass to this Properties struct
  void addPointMass(const PointMass::Properties& _properties);

  /// Connect two PointMasses together in this Properties struct
  bool connectPointMasses(size_t i1, size_t i2);

  /// Add a face to this Properties struct
  void addFace(const Eigen::Vector3i& _newFace);
};

//==============================================================================
struct SoftBodyNodeProperties
    : BodyNode::Properties, SoftBodyNodeUniqueProperties
{
  SoftBodyNodeProperties(
      const BodyNode::Properties& _bodyProperties = BodyNode::Properties(),
      const SoftBodyNodeUniqueProperties& _softProperties =
                                          SoftBodyNodeUniqueProperties());

  virtual ~SoftBodyNodeProperties() = default;
};

//==============================================================================
void SoftBodyNodeStateUpdate(SoftBodyAspect* aspect);

//==============================================================================
void SoftBodyNodePropertiesUpdate(SoftBodyAspect* aspect);

//==============================================================================
class SoftBodyAspect final :
    public common::AspectWithStateAndVersionedProperties<
        SoftBodyAspect, SoftBodyNodeUniqueState, SoftBodyNodeUniqueProperties,
        SoftBodyNode, &SoftBodyNodeStateUpdate, &SoftBodyNodePropertiesUpdate >
{
public:

  friend class dart::dynamics::SoftBodyNode;

  DART_COMMON_ASPECT_STATE_PROPERTY_CONSTRUCTORS(SoftBodyAspect)

  DART_COMMON_SET_GET_ASPECT_PROPERTY(double, Kv)
  DART_COMMON_SET_GET_ASPECT_PROPERTY(double, Ke)
  DART_COMMON_SET_GET_ASPECT_PROPERTY(double, DampCoeff)

protected:

  /// Get a direct reference to the State of this Aspect
  inline StateData& _getState() { return mState; }

  /// Get a direct reference to the State of this Aspect
  inline const StateData& _getState() const { return mState; }

  /// Get a direct reference to the Properties of this Aspect
  inline PropertiesData& _getProperties() { return mProperties; }

  /// Get a direct const reference to the Properties of this Aspect
  inline const PropertiesData& _getProperties() const { return mProperties; }

};

//==============================================================================
using SoftBodyNodeBase = common::CompositeJoiner<
    BodyNode, common::RequiresAspect<SoftBodyAspect> >;

} // namespace detail
} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_DETAIL_SOFTBODYNODEPROPERTIES_H_
