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

#ifndef DART_EXAMPLES_DEMOS_CONTACTVISUALIZER_HPP_
#define DART_EXAMPLES_DEMOS_CONTACTVISUALIZER_HPP_

#include <dart/gui/osg/osg.hpp>

#include <dart/dart.hpp>

#include <memory>
#include <vector>

#include <cstddef>

namespace dart_demos {

//==============================================================================
/// Host-level reusable contact-force visualizer, extracted from
/// RigidCubesScene's original bespoke version so every scene gets it for free.
/// Toggle lives in Diagnostics (host chrome, persists across scene switches);
/// the arrow pool itself is rebuilt per scene (the SimpleFrames it owns belong
/// to the active world).
class ContactVisualizer
{
public:
  /// Maximum number of contacts visualized in a single step, to bound the
  /// per-frame cost of a large contact manifold (documented in the toggle's
  /// UI label).
  static constexpr std::size_t kMaxArrows = 256;

  /// Binds to the newly-installed scene's world. Call from
  /// DemoHost::installScene after `world` is set as the current world.
  void onSceneInstalled(const dart::simulation::WorldPtr& world);

  /// Releases the arrow pool. Call from DemoHost::teardownCurrentScene
  /// before the world is destroyed.
  void reset();

  /// Reads world->getLastCollisionResult() and updates the arrow pool. Call
  /// from the host's composed postStep (once per simulation step, after the
  /// step that produced the current contacts).
  void applyPostStep();

  /// Renders the Diagnostics toggle checkbox.
  void renderToggle();

  /// Number of contacts visualized on the last applyPostStep() call (0 if
  /// disabled), for the Diagnostics stats line.
  std::size_t getVisualizedContactCount() const
  {
    return mLastVisualizedCount;
  }

  bool isEnabled() const
  {
    return mEnabled;
  }

private:
  void ensurePool(std::size_t count);
  void hideFrom(std::size_t start);

  dart::simulation::WorldPtr mWorld;
  bool mEnabled = true;

  std::vector<dart::dynamics::SimpleFramePtr> mFrames;
  std::vector<std::shared_ptr<dart::dynamics::ArrowShape>> mArrows;
  std::size_t mLastVisualizedCount = 0;
};

} // namespace dart_demos

#endif // DART_EXAMPLES_DEMOS_CONTACTVISUALIZER_HPP_
