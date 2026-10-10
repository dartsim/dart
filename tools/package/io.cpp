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

#include <dart/utils/urdf/DartLoader.hpp>
#include <dart/utils/utils.hpp>

#ifdef DART_PACKAGE_HAS_UTILS_ASSETS
  #include <dart/utils/assets/ModelResourceRetriever.hpp>
#endif

#include <array>
#include <string>

#include <cmath>

int main(int argc, char* argv[])
{
  if (argc != 2)
    return 1;
  const std::string data = argv[1];
#ifdef DART_PACKAGE_HAS_UTILS_ASSETS
  dart::utils::ModelResourceRetriever models(".", true);
  if (!models.addManifest(dart::common::Uri::createFromPath(
          data + "/robot_models/atlas-v5.xml"))
      || models.exists("model://package-smoke/v1/robot.urdf")
      || models.retrieve("model://package-smoke/v1/robot.urdf"))
    return 1;
#endif
  std::array<dart::simulation::WorldPtr, 3> worlds{
      dart::utils::SkelParser::readWorld(
          data + "/skel/test/single_pendulum.skel"),
      dart::utils::SdfParser::readWorld(
          data + "/sdf/test/single_bodynode_skeleton.world"),
      dart::utils::MjcfParser::readWorld(
          data + "/mjcf/openai/inverted_pendulum.xml")};
  if (!worlds[0] || worlds[0]->getNumSkeletons() != 1
      || worlds[0]->getSkeleton(0)->getNumBodyNodes() != 1 || !worlds[1]
      || worlds[1]->getNumSkeletons() != 1
      || worlds[1]->getSkeleton(0)->getNumBodyNodes() != 1 || !worlds[2]
      || worlds[2]->getNumSkeletons() != 2 || !worlds[2]->getSkeleton("cart")
      || worlds[2]->getSkeleton("cart")->getNumBodyNodes() != 2)
    return 1;

  for (const auto& world : worlds) {
    world->step();
    for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
      if (!std::isfinite(world->getSkeleton(i)->getPositions().norm()))
        return 1;
    }
  }

  dart::utils::DartLoader loader;
  auto skeleton
      = loader.parseSkeleton(data + "/urdf/test/primitive_geometry.urdf");
  return !skeleton || skeleton->getNumBodyNodes() != 1;
}
