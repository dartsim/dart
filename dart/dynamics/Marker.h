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

namespace dart {
namespace renderer {
class RenderInterface;
}  // namespace renderer
}  // namespace dart

namespace dart {
namespace dynamics {

class BodyNode;

class Marker {
public:
  enum ConstraintType {
    NO,
    HARD,
    SOFT
  };

  struct Properties
  {
    std::string mName;
    Eigen::Vector3d mOffset;
    ConstraintType mType;

    Properties(const std::string& _name = "",
               const Eigen::Vector3d& _offset = Eigen::Vector3d::Zero(),
               ConstraintType _type = NO);
  };

  /// \brief
  Marker(const std::string& _name, const Eigen::Vector3d& _offset,
         BodyNode* _bodyNode, ConstraintType _type = NO);

  /// \brief
  virtual ~Marker();

  /// \brief
  void draw(renderer::RenderInterface* _ri = nullptr, bool _offset = true,
            const Eigen::Vector4d& _color = Eigen::Vector4d::Identity(),
            bool _useDefaultColor = true) const;

  /// Get the BodyNode this Marker belongs to
  BodyNode* getBodyNode();

  /// Get the (const) BodyNode this Marker belongs to
  const BodyNode* getBodyNode() const;

  /// \brief
  const Eigen::Vector3d& getLocalPosition() const;

  /// \brief
  void setLocalPosition(const Eigen::Vector3d& _offset);

  /// \brief Get position w.r.t. world frame
  Eigen::Vector3d getWorldPosition() const;

  /// \brief
  int getIndexInSkeleton() const;

  /// \brief
  void setSkeletonIndex(int _idx);

  /// \brief
  int getID() const;

  /// \brief
  void setName(const std::string&);

  /// \brief
  const std::string& getName() const;

  // useful for IK
  /// \brief
  ConstraintType getConstraintType() const;

  /// \brief
  void setConstraintType(ConstraintType _type);

  friend class Skeleton;
  friend class BodyNode;

protected:

  /// Constructor used by BodyNode
  Marker(const Properties& _properties, BodyNode* _parent);

  /// \brief Properties of this Marker
  Properties mProperties;

  /// \brief BodyNode this marker belongs to
  BodyNode* mBodyNode;

  /// \brief position in the model class marker vector.
  int mSkelIndex;

private:
  /// \brief a unique ID of this marker globally.
  int mID;

  /// \brief counts the number of markers globally.
  static int msMarkerCount;

public:
  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_MARKER_H_

