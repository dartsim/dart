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

#ifndef DART_UTILS_MJCF_DETAIL_GEOMCOLLISIONFILTER_HPP_
#define DART_UTILS_MJCF_DETAIL_GEOMCOLLISIONFILTER_HPP_

#include <dart/collision/CollisionFilter.hpp>

#include <unordered_map>

namespace dart {

namespace dynamics {
class ShapeFrame;
} // namespace dynamics

namespace utils {
namespace MjcfParser {
namespace detail {

/// CollisionFilter that reproduces MuJoCo's contype/conaffinity pair rule
/// (http://mujoco.org/book/computation.html#coCollision) on top of DART's
/// default BodyNodeCollisionFilter behavior.
///
/// Two geoms are allowed to collide iff
///   (contype_A & conaffinity_B) != 0 || (contype_B & conaffinity_A) != 0
///
/// This is combined with (i.e. logically ANDed with) the inherited
/// BodyNodeCollisionFilter behavior (self-collision/adjacent-body handling,
/// resting-pair filtering, etc.), so installing this filter never re-enables
/// a pair that the base filter already excludes.
///
/// ShapeNodes that were not registered with setGeomBitmasks() (i.e. not
/// created from an MJCF <geom>) are unaffected by the bitmask rule: the pair
/// falls back to the inherited BodyNodeCollisionFilter behavior only.
class GeomCollisionFilter final : public collision::BodyNodeCollisionFilter
{
public:
  /// Registers the contype/conaffinity bitmasks of the MJCF <geom> that
  /// created \c shapeFrame. Overwrites any previously registered bitmasks for
  /// the same ShapeFrame.
  void setGeomBitmasks(
      const dynamics::ShapeFrame* shapeFrame, int contype, int conaffinity);

  // Documentation inherited
  bool ignoresCollision(
      const collision::CollisionObject* object1,
      const collision::CollisionObject* object2) const override;

private:
  struct Bitmasks
  {
    int mContype;
    int mConaffinity;
  };

  /// contype/conaffinity bitmasks keyed by the ShapeFrame of the ShapeNode
  /// created for the owning MJCF <geom>.
  std::unordered_map<const dynamics::ShapeFrame*, Bitmasks> mBitmasks;
};

} // namespace detail
} // namespace MjcfParser
} // namespace utils
} // namespace dart

#endif // #ifndef DART_UTILS_MJCF_DETAIL_GEOMCOLLISIONFILTER_HPP_
