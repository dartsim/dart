/*
 * Copyright (c) 2013-2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
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

#ifndef SOFT_DYNAMICS_SOFTMESHSHAPE_H_
#define SOFT_DYNAMICS_SOFTMESHSHAPE_H_

#include <vector>

#include <assimp/scene.h>
#include <dart/dynamics/Shape.h>
#include <Eigen/Dense>

namespace dart {
namespace dynamics {

class SoftBodyNode;

// TODO(JS): Implement
class SoftMeshShape : public Shape
{
public:
  /// \brief Constructor.
  explicit SoftMeshShape(SoftBodyNode* _softBodyNode);

  /// \brief Destructor.
  virtual ~SoftMeshShape();

  /// \brief
  const aiMesh* getAssimpMesh() const;

  /// \brief Update positions of the vertices using the parent soft body node.
  void update();

  // Documentation inherited.
  virtual Eigen::Matrix3d computeInertia(double _mass) const;

  // Documentation inherited.
  virtual void draw(
      renderer::RenderInterface* _ri      = NULL,
      const Eigen::Vector4d&     _col     = Eigen::Vector4d::Ones(),
      bool                       _default = true) const;

protected:
  // Documentation inherited.
  virtual void computeVolume();

private:
  /// \brief Build mesh using SoftBodyNode data
  void _buildMesh();

  /// \brief
  SoftBodyNode* mSoftBodyNode;

  /// \brief
  aiMesh* mAssimpMesh;
};

}  // namespace dynamics
}  // namespace dart

#endif  // SOFT_DYNAMICS_SOFTMESHSHAPE_H_
