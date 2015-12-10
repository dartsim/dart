/*
 * Copyright (c) 2011-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>
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

#ifndef DART_DYNAMICS_MARKER_H_
#define DART_DYNAMICS_MARKER_H_

#include <string>
#include <Eigen/Dense>
#include "dart/math/Helpers.h"

namespace dart {
namespace renderer {
class RenderInterface;
}  // namespace renderer
}  // namespace dart

namespace dart {
namespace dynamics {

class BodyNode;

class Marker
{
public:

  enum ConstraintType
  {
    NO,
    HARD,
    SOFT
  };

  struct Properties
  {
    std::string mName;
    Eigen::Vector3d mOffset;
    Eigen::Vector4d mColor;
    ConstraintType mType;

    Properties(const std::string& name = "",
               const Eigen::Vector3d& offset = Eigen::Vector3d::Zero(),
               const Eigen::Vector4d& color = Color::White(1.0),
               ConstraintType type = NO);

    // To get byte-aligned Eigen vectors
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  /// Constructor
  Marker(const std::string& name,
         const Eigen::Vector3d& offset,
         const Eigen::Vector4d& color,
         BodyNode* bodyNode,
         ConstraintType type = NO);

  /// Destructor
  virtual ~Marker();

  /// Render this marker
  void draw(renderer::RenderInterface* ri = nullptr,
            bool offset = true,
            const Eigen::Vector4d& color = Color::White(1.0),
            bool useDefaultColor = true) const;

  /// Get the BodyNode this Marker belongs to
  BodyNode* getBodyNode();

  /// Get the (const) BodyNode this Marker belongs to
  const BodyNode* getBodyNode() const;

  /// Get position of this marker in the parent body node coordinates
  const Eigen::Vector3d& getLocalPosition() const;

  /// Set position of this marker in the parent body node coordinates
  void setLocalPosition(const Eigen::Vector3d& offset);

  /// Get position in the world coordinates
  Eigen::Vector3d getWorldPosition() const;

  /// Deprecated; please use setIndexInSkeleton() instead
  DEPRECATED(6.0)
  void setSkeletonIndex(int index);

  /// Set index in skeleton this marker is belongs to
  void setIndexInSkeleton(int index);
  // TODO(JS): This function is not called by any. Remove?

  /// Get index in skeleton this marker is belongs to
  int getIndexInSkeleton() const;
  // TODO(JS): This function is not called by any. Remove?

  /// Get global unique ID
  int getID() const;

  /// Set name of this marker
  void setName(const std::string& name);

  /// Get name of this marker
  const std::string& getName() const;

  /// Set constraint type. which will be useful for inverse kinematics
  void setConstraintType(ConstraintType type);

  /// Get constraint type. which will be useful for inverse kinematics
  ConstraintType getConstraintType() const;

  friend class Skeleton;
  friend class BodyNode;

protected:

  /// Constructor used by BodyNode
  Marker(const Properties& properties, BodyNode* parent);

  /// \brief Properties of this Marker
  Properties mProperties;

  /// \brief BodyNode this marker belongs to
  BodyNode* mBodyNode;

  /// \brief position in the model class marker vector.
  int mSkelIndex;

private:
  /// Unique ID of this marker globally.
  int mID;

  /// Counts the number of markers globally.
  static int msMarkerCount;

public:
  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};
// TODO: Marker class should be refactored into a Node once pull request #531 is
// finished.

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_MARKER_H_

