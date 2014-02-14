/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
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

#ifndef SOFT_DYNAMICS_SOFTSKELETON_H_
#define SOFT_DYNAMICS_SOFTSKELETON_H_

#include <string>
#include <vector>

#include <dart/dynamics/Skeleton.h>

namespace dart {
namespace dynamics {

class SoftBodyNode;

/// \brief
class SoftSkeleton : public Skeleton
{
public:
  //--------------------------------------------------------------------------
  // Constructor and Desctructor
  //--------------------------------------------------------------------------
  /// \brief Constructor.
  explicit SoftSkeleton(const std::string& _name = "Unnamed SoftSkeleton");

  /// \brief Destructor.
  virtual ~SoftSkeleton();

  /// \brief Add a soft body node.
  void addSoftBodyNode(SoftBodyNode *_body);

  /// \brief Get soft body node.
  SoftBodyNode* getSoftBodyNode(int _idx) const;

  /// \brief Get soft body node.
  SoftBodyNode* getSoftBodyNode(const std::string& _name) const;

  /// \brief Get number of soft body nodes.
  int getNumSoftBodyNodes() const;

  /// \brief Get number of rigid body nodes.
  int getNumRigidBodyNodes() const;

  /// \brief
  void init(double _timeStep = 0.001, const Eigen::Vector3d& _gravity =
      Eigen::Vector3d(0.0, 0.0, -9.81));

protected:
  // Documentation inherited.
  virtual void updateExternalForceVector();

  // Documentation inherited.
  virtual void updateDampingForceVector();

  /// \brief Soft body node list.
  std::vector<SoftBodyNode*> mSoftBodyNodes;

private:
  /// \brief
  std::vector<GenCoord*> mPointMassGenCoords;
};

}  // namespace dynamics
}  // namespace dart

#endif  // SOFT_DYNAMICS_SOFTSKELETON_H_
