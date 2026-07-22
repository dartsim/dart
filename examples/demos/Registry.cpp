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

#include "Registry.hpp"

#include "scenes/Scenes.hpp"

namespace dart_demos {

//==============================================================================
std::vector<DemoScene> makeDemoScenes()
{
  // Category order below is the navigator's first-appearance order. Keep this
  // in sync with examples/demos/README.md when adding or moving scenes.
  std::vector<DemoScene> scenes;
  scenes.push_back(makeEmptyScene());
  scenes.push_back(makeHelloWorldScene());
  scenes.push_back(makeSimpleFramesScene());
  scenes.push_back(makeBoxesScene());
  scenes.push_back(makeRigidCubesScene());
  scenes.push_back(makeRigidChainScene());
  scenes.push_back(makeAddDeleteSkelsScene());
  scenes.push_back(makeSimulationEventHandlerScene());
  scenes.push_back(makeRigidShapesScene());
  scenes.push_back(makeSleepingScene());

  // Research (new category; first appearance here).
  scenes.push_back(makeFbfPaperInclineScene());
  scenes.push_back(makeFbfAuthorInclineSweepCurrentSourceScene());
  scenes.push_back(makeFbfPaperBackspinScene());
  scenes.push_back(makeFbfAuthorBackspinCurrentSourceScene());
  scenes.push_back(makeFbfPaperTurntableScene());
  scenes.push_back(makeFbfPaperTurntableMu02Omega2Scene());
  scenes.push_back(makeFbfPaperTurntableMu02Omega5Scene());
  scenes.push_back(makeFbfPaperTurntableMu05Omega5Scene());
  scenes.push_back(makeFbfAuthorTurntableMu02Omega2Scene());
  scenes.push_back(makeFbfAuthorTurntableMu02Omega5Scene());
  scenes.push_back(makeFbfAuthorTurntableMu05Omega2Scene());
  scenes.push_back(makeFbfAuthorTurntableMu05Omega5Scene());
  scenes.push_back(makeFbfAuthorCardHouseScene());
  scenes.push_back(makeFbfAuthorCardHouse4ImpactCurrentSourceScene());
  scenes.push_back(makeFbfAuthorCardHouse5ImpactCurrentSourceScene());
  scenes.push_back(makeFbfAuthorCardHouse10ImpactCurrentSourceScene());
  scenes.push_back(
      makeFbfAuthorCardHouse4ImpactSourceContinuationCurrentSourceScene());
  scenes.push_back(
      makeFbfAuthorCardHouse5ImpactSourceContinuationCurrentSourceScene());
  scenes.push_back(
      makeFbfAuthorCardHouse10ImpactSourceContinuationCurrentSourceScene());
  scenes.push_back(makeFbfAuthorMasonryArch25CrownImpactCurrentSourceScene());
  scenes.push_back(
      makeFbfAuthorMasonryArch25CrownImpactSourceContinuationCurrentSourceScene());
  scenes.push_back(makeFbfAuthorMasonryArch101StandingCurrentSourceScene());
  scenes.push_back(makeFbfPaperPainleveScene());
  scenes.push_back(makeFbfPaperPainleveMu055Scene());
  scenes.push_back(makeFbfAuthorPainleveMu05Scene());
  scenes.push_back(makeFbfAuthorPainleveMu055Scene());
  scenes.push_back(makeFbfPaperCardAFrameScene());
  scenes.push_back(makeFbfPaperCardHouse26Scene());
  scenes.push_back(makeFbfPaperCardHouse10Scene());
  scenes.push_back(makeFbfPaperCardHouse10DynamicScene());
  scenes.push_back(makeFbfPaperMasonryArch25LiteralStandingScene());
  scenes.push_back(makeFbfPaperMasonryArch25Scene());
  scenes.push_back(makeFbfPaperMasonryArch101Scene());

  scenes.push_back(makeHardcodedDesignScene());
  scenes.push_back(makeRigidLoopScene());
  scenes.push_back(makeBoxStackingScene());
  scenes.push_back(makeDynamicJointConstraintsScene());
  scenes.push_back(makeTinkertoyScene());
#ifdef DART_DEMOS_HAVE_TINY_DNN
  scenes.push_back(makeHumanJointLimitsScene());
#endif

  // Control & IK (new category; first appearance here).
  scenes.push_back(makeJointConstraintsScene());
  scenes.push_back(makeHybridDynamicsScene());
  scenes.push_back(makeBipedStandScene());
  scenes.push_back(makeOperationalSpaceControlScene());
  scenes.push_back(makeContactInverseDynamicsScene());
#ifdef DART_DEMOS_HAVE_PYTHON
  scenes.push_back(makeSsikIkGuiScene());
#endif
  scenes.push_back(makeWamIkFastScene());
  scenes.push_back(makeAtlasPuppetScene());
  scenes.push_back(makeAtlasSimbiconScene());
  scenes.push_back(makeHuboPuppetScene());

  // Soft Bodies (new category; first appearance here).
  scenes.push_back(makeMixedChainScene());
  scenes.push_back(makeSoftBodiesScene());
  scenes.push_back(makeSoftCubesScene());
  scenes.push_back(makeSoftOpenChainScene());

  // Robots (new category; first appearance here).
  scenes.push_back(makeFetchScene());
  scenes.push_back(makeVehicleScene());

  // Visualization (existing category, first introduced by simple_frames).
  scenes.push_back(makeHeightmapScene());
#if HAVE_OCTOMAP
  scenes.push_back(makePointCloudScene());
#endif
  scenes.push_back(makeDragAndDropScene());
  return scenes;
}

} // namespace dart_demos
