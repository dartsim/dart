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

#ifndef DART_EXAMPLES_DEMOS_SCENES_FBFAUTHORMASONRYARCHSPEC_HPP_
#define DART_EXAMPLES_DEMOS_SCENES_FBFAUTHORMASONRYARCHSPEC_HPP_

#include <dart/math/detail/MasonryArchGeometry.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <cmath>
#include <cstddef>

#ifndef DART_FBF_AUTHOR_MASONRY_ARCH_SPEC_SHA256
  #error                                                                       \
      "FBF author-masonry-arch consumers must bind the shared spec source hash"
#endif

namespace fbf_author_masonry_arch {

inline constexpr const char* kContractSchema
    = "dart.fbf_author_masonry_arch_configuration_contract/v1";
inline constexpr const char* kContractKind = "configuration_only";
inline constexpr const char* kDemoSceneId
    = "fbf_author_masonry_arch_25_construction";
inline constexpr const char* kAuthorRepository
    = "https://github.com/matthcsong/fbf-sca-2026";
inline constexpr const char* kAuthorCommit
    = "b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0";
inline constexpr const char* kAuthorTree
    = "ffcdafb61adeda2239c8366d054b548b50d26685";
inline constexpr const char* kAuthorRunBlob
    = "35a052d7ef0975e7c828c9678d163054dfbb3ef2";
inline constexpr const char* kAuthorRunSha256
    = "7e9158240267bb0ec1d0316b1badd4f3c8e1cd10270322de2e205cfea96f6f73";
inline constexpr const char* kAuthorMeshTree
    = "2552017df4061c49f7df064d847a4268a6e02d1a";
// Path-sensitive two-level GNU sha256sum manifest hash over the 25 OBJ files,
// computed from the author repository root with:
//   find meshes/arch/num_stones=25 -maxdepth 1 -type f -name '*.obj' -print0
//     | LC_ALL=C sort -z | xargs -0 sha256sum | sha256sum
inline constexpr const char* kAuthorMeshTreeSha256
    = "a3f4e35073a2f4e74837fff277cd923f104b6af57f2cf995cf7524fe498e483d";
inline constexpr const char* kAuthorMeshDirectory = "meshes/arch/num_stones=25";
inline constexpr const char* kSpecSourceSha256
    = DART_FBF_AUTHOR_MASONRY_ARCH_SPEC_SHA256;

inline constexpr std::size_t kStoneCount = 25u;
inline constexpr std::size_t kFixedSpringerCount = 2u;
inline constexpr std::size_t kCubeCount = 3u;
inline constexpr const char* kGroundShape = "plane";
inline constexpr bool kGroundMobile = false;
inline constexpr bool kFixSpringers = true;
inline constexpr bool kCubesInitiallyKinematic = true;
inline constexpr double kFriction = 0.8;
inline constexpr double kStoneDensity = 2000.0;
inline constexpr double kCubeHalfSize = 1.5;
inline constexpr double kCubeEdgeLength = 2.0 * kCubeHalfSize;
inline constexpr double kCubeDensity = 2000.0;
inline constexpr double kCubeMass
    = kCubeDensity * kCubeEdgeLength * kCubeEdgeLength * kCubeEdgeLength;
inline constexpr double kCubeSpacing = 3.0 * kCubeHalfSize;
inline constexpr double kDropHeight = 10.0;
inline constexpr double kScale = 1.0;
inline constexpr double kContactGap = 0.005;
inline constexpr double kShapeStiffness = 1.0e4;
inline constexpr double kShapeDamping = 1.0e3;
inline constexpr double kDisplayTimeStep = 1.0 / 60.0;
inline constexpr std::size_t kSubstepsPerFrame = 4u;
inline constexpr double kRuntimeTimeStep
    = kDisplayTimeStep / static_cast<double>(kSubstepsPerFrame);
inline constexpr std::size_t kDefaultFrameCount = 400u;
inline constexpr std::size_t kDropFrame = 400u;
inline constexpr std::size_t kReleaseSubstep = kDropFrame * kSubstepsPerFrame;
inline constexpr std::size_t kDefaultTotalSubsteps
    = kDefaultFrameCount * kSubstepsPerFrame;
// The first value is the maximum six-decimal OBJ record. The author loads the
// records into a NumPy float32 array before taking the maximum; that runtime
// value is one float32 ULP higher and controls the actual cube pose.
inline constexpr double kSourceObjRecordArchTopZ = 65.273375;
inline constexpr double kSourceRuntimeArchTopZFloat32 = 65.27337646484375;
inline constexpr double kCubeInitialZ
    = kSourceRuntimeArchTopZFloat32 + kDropHeight;

inline constexpr std::size_t kSourceMaxContacts = 4096u;
inline constexpr int kSourceMaxOuterIterations = 200;
inline constexpr double kSourceOuterTolerance = 1e-6;
inline constexpr int kSourceResidualCheckInterval = 1;
inline constexpr const char* kSourceInnerSolver = "block_gs";
inline constexpr int kSourceInnerGaussSeidelSweeps = 30;
inline constexpr int kSourceInnerMaxIterations = 1000;
inline constexpr double kSourceInnerTolerance = 1e-6;
inline constexpr double kSourceGammaC = 15.5;
inline constexpr double kSourceGammaMax = 1e6;
inline constexpr bool kSourceAdaptiveGamma = true;
inline constexpr double kSourceArmijoRhoHigh = 0.9;
inline constexpr double kSourceArmijoShrink = 0.7;
inline constexpr double kSourceArmijoGrow = 1.0 / kSourceArmijoShrink;
inline constexpr double kSourceArmijoSkipThreshold = 1e-10;
inline constexpr int kSourceArmijoMaxBacktracks = 8;
inline constexpr int kSourcePlateauPatience = 0;
inline constexpr double kSourcePlateauRelativeTolerance = 0.01;
inline constexpr double kSourceWarmStartMatchRadius = 0.02;
inline constexpr double kSourceWarmStartNormalCosine = 0.9;
inline constexpr int kSourceWarmStartMaxAge = 3;
inline constexpr double kSourceWarmStartGammaThreshold = 1e-4;
inline constexpr double kSourceWarmStartGammaCap = 1e4;
inline constexpr double kSourceBaumgarteErp = 0.0;
inline constexpr bool kSourceProjectAfterCorrection = true;
inline constexpr const char* kSourceTerminationResidual = "coulomb_rel";
inline constexpr double kSourceTerminationTolerance = 1e-6;

// The pinned OBJ files print six decimal places. MasonryArchGeometry reverses
// the source generator's centimeter-to-meter conversion here; quantization
// then reproduces every numeric OBJ vertex record after the author's y/z axis
// swap. The returned coordinates retain the author's raw scale.
inline constexpr double kSourceCoordinateScale = 100.0;
inline constexpr double kSourceObjQuantum = 1.0e-6;
inline constexpr double kSourceObjDecimalScale = 1.0e6;

inline constexpr bool kConfigurationPortValid = true;
inline constexpr bool kSourceObjNumericRecordsEquivalent = true;
inline constexpr bool kSourceAxisSwapImplemented = true;
inline constexpr bool kSourceSolverConfigurationRecorded = true;
inline constexpr bool kSourceCollisionSemanticsImplemented = false;
inline constexpr bool kSourceContactGapSemanticsImplemented = false;
inline constexpr bool kSourceSolverBackendSemanticsImplemented = false;
inline constexpr bool kSourceFloat32SemanticsImplemented = false;
inline constexpr bool kSourceRuntimeFloat32InitialPosesEquivalent = false;
inline constexpr bool kDynamicsExecuted = false;
inline constexpr bool kTrajectoryEquivalence = false;
inline constexpr bool kDynamicsEquivalence = false;
inline constexpr bool kPhysicalOutcomeEquivalence = false;
inline constexpr bool kFig07Parity = false;
inline constexpr bool kVideo07Arch25Parity = false;
inline constexpr bool kTimingComparability = false;
inline constexpr bool kPaperParity = false;

using Triangle = std::array<std::size_t, 3>;
using Vertices = std::array<Eigen::Vector3d, 8>;

struct MassProperties
{
  Eigen::Vector3d localCenterOfMass = Eigen::Vector3d::Zero();
  double volume = 0.0;
  Eigen::Matrix3d momentPerUnitMass = Eigen::Matrix3d::Zero();
};

struct StoneSpec
{
  std::string name;
  std::size_t index = 0u;
  Vertices vertices{};
  Vertices localVertices{};
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  Eigen::Vector3d localCenterOfMass = Eigen::Vector3d::Zero();
  double volume = 0.0;
  double density = 0.0;
  double mass = 0.0;
  Eigen::Matrix3d moment = Eigen::Matrix3d::Zero();
  double friction = 0.0;
  bool mobile = true;
};

struct CubeSpec
{
  std::string name;
  std::size_t index = 0u;
  Eigen::Vector3d size = Eigen::Vector3d::Zero();
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  double density = 0.0;
  double mass = 0.0;
  Eigen::Matrix3d moment = Eigen::Matrix3d::Zero();
  double friction = 0.0;
  bool initiallyMobile = false;
};

//==============================================================================
inline const std::array<Triangle, 12>& triangles()
{
  return dart::math::detail::getMasonryArchStoneWedgeTriangles();
}

//==============================================================================
inline double quantizeSourceObjCoordinate(double value)
{
  return std::round(value * kSourceObjDecimalScale) / kSourceObjDecimalScale;
}

//==============================================================================
inline Vertices sourceRawVertices(
    const dart::math::detail::MasonryArchStoneWedgeGeometry& generated)
{
  Vertices vertices;
  for (std::size_t vertex = 0u; vertex < vertices.size(); ++vertex) {
    for (Eigen::Index axis = 0; axis < 3; ++axis) {
      vertices[vertex][axis] = quantizeSourceObjCoordinate(
          kSourceCoordinateScale * generated.vertices[vertex][axis]);
    }
  }
  return vertices;
}

//==============================================================================
inline Eigen::Vector3d vertexMean(const Vertices& vertices)
{
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const auto& vertex : vertices)
    mean += vertex;
  return mean / static_cast<double>(vertices.size());
}

//==============================================================================
inline MassProperties computeMassProperties(const Vertices& localVertices)
{
  MassProperties properties;
  Eigen::Vector3d firstMoment = Eigen::Vector3d::Zero();
  Eigen::Matrix3d secondMoment = Eigen::Matrix3d::Zero();

  // The body origin is the arithmetic vertex mean and therefore lies inside
  // each convex wedge. Decompose the hull into origin-face tetrahedra. Using
  // local coordinates avoids cancellation from the source's raw ~65-unit
  // crown height.
  for (const auto& triangle : triangles()) {
    const auto& a = localVertices[triangle[0]];
    const auto& b = localVertices[triangle[1]];
    const auto& c = localVertices[triangle[2]];
    const double tetrahedronVolume = std::abs(a.dot(b.cross(c))) / 6.0;
    const Eigen::Vector3d sum = a + b + c;
    properties.volume += tetrahedronVolume;
    firstMoment += 0.25 * tetrahedronVolume * sum;
    secondMoment += tetrahedronVolume
                    * (sum * sum.transpose() + a * a.transpose()
                       + b * b.transpose() + c * c.transpose())
                    / 20.0;
  }

  if (!(properties.volume > 0.0) || !std::isfinite(properties.volume))
    throw std::runtime_error("author masonry-arch stone has invalid volume");

  properties.localCenterOfMass = firstMoment / properties.volume;
  const Eigen::Matrix3d covariance
      = secondMoment / properties.volume
        - properties.localCenterOfMass
              * properties.localCenterOfMass.transpose();
  properties.momentPerUnitMass
      = covariance.trace() * Eigen::Matrix3d::Identity() - covariance;
  return properties;
}

//==============================================================================
inline std::vector<StoneSpec> makeStoneSpecs()
{
  const auto generated = dart::math::detail::generateMasonryArchStoneWedges(
      kStoneCount,
      {},
      dart::math::detail::MasonryArchBarrierGapPolicy::IncludeSourceOffsets);
  if (generated.size() != kStoneCount)
    throw std::runtime_error("author masonry-arch generator count mismatch");

  std::vector<StoneSpec> specs;
  specs.reserve(generated.size());
  for (std::size_t index = 0u; index < generated.size(); ++index) {
    StoneSpec spec;
    spec.name = "stone-" + (index < 9u ? std::string("0") : std::string())
                + std::to_string(index + 1u);
    spec.index = index;
    spec.vertices = sourceRawVertices(generated[index]);
    spec.transform.translation() = vertexMean(spec.vertices);
    for (std::size_t vertex = 0u; vertex < spec.vertices.size(); ++vertex) {
      spec.localVertices[vertex]
          = spec.vertices[vertex] - spec.transform.translation();
    }
    const auto properties = computeMassProperties(spec.localVertices);
    spec.localCenterOfMass = properties.localCenterOfMass;
    spec.volume = properties.volume;
    spec.density = kStoneDensity;
    spec.mass = spec.density * spec.volume;
    spec.moment = spec.mass * properties.momentPerUnitMass;
    spec.friction = kFriction;
    spec.mobile = index != 0u && index + 1u != kStoneCount;
    specs.push_back(spec);
  }
  return specs;
}

//==============================================================================
inline double archTopZ(const std::vector<StoneSpec>& stones)
{
  double top = -std::numeric_limits<double>::infinity();
  for (const auto& stone : stones) {
    for (const auto& vertex : stone.vertices)
      top = std::max(top, vertex.z());
  }
  if (!std::isfinite(top))
    throw std::runtime_error("author masonry-arch has no finite top vertex");
  return top;
}

//==============================================================================
inline std::vector<CubeSpec> makeCubeSpecs()
{
  std::vector<CubeSpec> specs;
  specs.reserve(kCubeCount);
  for (std::size_t index = 0u; index < kCubeCount; ++index) {
    CubeSpec spec;
    spec.name = "cube_" + std::to_string(index);
    spec.index = index;
    spec.size = Eigen::Vector3d::Constant(kCubeEdgeLength);
    spec.transform.translation() = Eigen::Vector3d(
        (static_cast<double>(index)
         - 0.5 * static_cast<double>(kCubeCount - 1u))
            * kCubeSpacing,
        0.0,
        kCubeInitialZ);
    spec.density = kCubeDensity;
    spec.mass = kCubeMass;
    spec.moment = Eigen::Matrix3d::Identity()
                  * (spec.mass * kCubeEdgeLength * kCubeEdgeLength / 6.0);
    spec.friction = kFriction;
    spec.initiallyMobile = false;
    specs.push_back(spec);
  }
  return specs;
}

//==============================================================================
constexpr bool releaseOccursWithinFrames(std::size_t frameCount)
{
  return kDropFrame < frameCount;
}

//==============================================================================
constexpr bool cubeIsMobileAtSubstep(
    std::size_t substep, std::size_t frameCount)
{
  return releaseOccursWithinFrames(frameCount) && substep >= kReleaseSubstep;
}

} // namespace fbf_author_masonry_arch

#endif // DART_EXAMPLES_DEMOS_SCENES_FBFAUTHORMASONRYARCHSPEC_HPP_
