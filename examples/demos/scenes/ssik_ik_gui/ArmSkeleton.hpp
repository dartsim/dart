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

#ifndef DART_EXAMPLES_DEMOS_SCENES_SSIK_IK_GUI_ARMSKELETON_HPP_
#define DART_EXAMPLES_DEMOS_SCENES_SSIK_IK_GUI_ARMSKELETON_HPP_

#include "SsikBridge.hpp"

#include <dart/dart.hpp>

#include <vector>

namespace dart_demos {
namespace ssik_ik_gui {

//==============================================================================
/// Renders IK solutions as real DART skeletons built from the arm's
/// product-of-exponentials chain and posed with setPositions(q). Every
/// solution gets its own skeleton instance so they can be shown
/// simultaneously: the selected one is opaque and the rest are translucent
/// "ghosts". Ported verbatim from examples/ssik_ik_gui/main.cpp.
class ArmSkeleton
{
public:
  void setWorld(const dart::simulation::WorldPtr& world);

  /// Selects the chain for subsequent solutions. An empty chain clears the
  /// arm.
  void build(const std::vector<JointSpec>& chain);

  /// Poses one skeleton per solution, highlighting the selected one and
  /// drawing the others as translucent ghosts. With no solution, shows a
  /// single skeleton at its zero configuration.
  void showSolutions(const std::vector<SsikSolution>& solutions, int selected);

private:
  static constexpr double kLinkRadius = 0.028;
  static constexpr double kJointRadius = 0.045;
  static constexpr std::size_t kMaxInstances = 16;

  std::size_t numDofs() const;
  void clearInstances();
  void setConfig(
      const dart::dynamics::SkeletonPtr& skeleton, const Eigen::VectorXd& q);
  void setColor(
      const dart::dynamics::SkeletonPtr& skeleton,
      const Eigen::Vector4d& color);
  dart::dynamics::SkeletonPtr buildInstance();
  void addCylinder(
      dart::dynamics::BodyNode* body,
      const Eigen::Vector3d& a,
      const Eigen::Vector3d& b);
  void addSphere(
      dart::dynamics::BodyNode* body, const Eigen::Vector3d& position);

  dart::simulation::WorldPtr mWorld;
  std::vector<JointSpec> mChain;
  std::vector<dart::dynamics::SkeletonPtr> mInstances;
  std::size_t mInstanceCounter = 0;
};

} // namespace ssik_ik_gui
} // namespace dart_demos

#endif // DART_EXAMPLES_DEMOS_SCENES_SSIK_IK_GUI_ARMSKELETON_HPP_
