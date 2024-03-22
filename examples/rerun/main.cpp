/*
 * Copyright (c) 2011-2024, The DART development contributors
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

#include <dart/gui/rerun/rerun.hpp>

#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>

#include <dart/simulation/World.hpp>

#include <dart/dart.hpp>

#include <rerun.hpp>
#include <rerun/demo_utils.hpp>

using namespace dart;
using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::utils;
using namespace dart::math;
using namespace rerun::demo;

int main()
{
  dart::utils::DartLoader urdfLoader;
  auto ground = urdfLoader.parseSkeleton("dart://sample/sdf/atlas/ground.urdf");
  auto atlas = dart::utils::SdfParser::readSkeleton(
      "dart://sample/sdf/atlas/atlas_v3_no_head.sdf");

  // Create a world and add the rigid body
  auto world = simulation::World::create();
  // world->addSkeleton(ground);
  world->addSkeleton(atlas);

  // Spawn Rerun viewer
  const auto rec = ::rerun::RecordingStream("rerun_example_cpp");
  rec.spawn().exit_on_failure();

  for (auto i = 0; i < 1; ++i) {
    world->step();
    gui::rerun::logWorld(rec, "world", *world);
  }
}
