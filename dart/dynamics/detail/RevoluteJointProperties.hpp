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

#ifndef DART_DYNAMICS_DETAIL_REVOLUTEJOINTPROPERTIES_HPP_
#define DART_DYNAMICS_DETAIL_REVOLUTEJOINTPROPERTIES_HPP_

#include <string>

#include <Eigen/Dense>

#include "dart/dynamics/SingleDofJoint.hpp"

namespace dart {
namespace dynamics {

class RevoluteJoint;

namespace detail {

//==============================================================================
struct RevoluteJointUniqueProperties
{
  Eigen::Vector3d mAxis;

  RevoluteJointUniqueProperties(
      const Eigen::Vector3d& _axis = Eigen::Vector3d::UnitZ());

  virtual ~RevoluteJointUniqueProperties() = default;
};

//==============================================================================
struct RevoluteJointProperties :
    SingleDofJoint::Properties,
    RevoluteJointUniqueProperties
{
  RevoluteJointProperties(
      const SingleDofJoint::Properties& _singleDofJointProperties =
          SingleDofJoint::Properties(),
      const RevoluteJointUniqueProperties& _revoluteProperties =
          RevoluteJointUniqueProperties());

  virtual ~RevoluteJointProperties() = default;
};

//==============================================================================
class RevoluteJointAddon final :
    public common::AddonWithVersionedProperties<
        RevoluteJointAddon, RevoluteJointUniqueProperties, RevoluteJoint,
        detail::JointPropertyUpdate<RevoluteJointAddon> >
{
public:
  DART_COMMON_JOINT_ADDON_CONSTRUCTOR( RevoluteJointAddon )

  void setAxis(const Eigen::Vector3d& _axis);
  const Eigen::Vector3d& getAxis() const;
};

//==============================================================================
using RevoluteJointBase = common::AddonManagerJoiner<
    SingleDofJoint, common::RequiresAddon<RevoluteJointAddon> >;

} // namespace detail

} // namespace dynamics
} // namespace dart


#endif // DART_DYNAMICS_DETAIL_REVOLUTEJOINTPROPERTIES_HPP_
