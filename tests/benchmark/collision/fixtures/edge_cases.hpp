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

#pragma once

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cstddef>

namespace dart::benchmark::collision {

enum class EdgeCase
{
  kTouching,
  kDeepPenetration,
  kGrazing,
  kThinFeature
};

enum class PairKind
{
  kSphereSphere,
  kBoxBox,
  kCapsuleCapsule,
  kSphereBox,
  kCapsuleSphere,
  kCapsuleBox
};

struct PairTransforms
{
  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
};

struct SphereSpec
{
  double radius = 0.5;
};

struct CapsuleSpec
{
  double radius = 0.5;
  double height = 2.0;
};

struct BoxSpec
{
  Eigen::Vector3d halfExtents = Eigen::Vector3d(0.5, 0.5, 0.5);
  Eigen::Vector3d size = Eigen::Vector3d(1.0, 1.0, 1.0);
};

constexpr std::array<double, 3> kScaleSweep = {1e-3, 1.0, 1e3};

inline double ScaleFromIndex(int index)
{
  if (index < 0) {
    return kScaleSweep.front();
  }

  const auto idx = static_cast<std::size_t>(index);
  if (idx >= kScaleSweep.size()) {
    return kScaleSweep.back();
  }

  return kScaleSweep[idx];
}

inline const char* PairKindName(PairKind pair)
{
  switch (pair) {
    case PairKind::kSphereSphere:
      return "SphereSphere";
    case PairKind::kBoxBox:
      return "BoxBox";
    case PairKind::kCapsuleCapsule:
      return "CapsuleCapsule";
    case PairKind::kSphereBox:
      return "SphereBox";
    case PairKind::kCapsuleSphere:
      return "CapsuleSphere";
    case PairKind::kCapsuleBox:
      return "CapsuleBox";
  }
  return "UnknownPair";
}

inline const char* EdgeCaseName(EdgeCase edge)
{
  switch (edge) {
    case EdgeCase::kTouching:
      return "Touching";
    case EdgeCase::kDeepPenetration:
      return "DeepPenetration";
    case EdgeCase::kGrazing:
      return "Grazing";
    case EdgeCase::kThinFeature:
      return "ThinFeature";
  }
  return "UnknownCase";
}

inline Eigen::Isometry3d OffsetTransform(double x, double y, double z)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(x, y, z);
  return tf;
}

inline double GrazingEpsilon(double nominal)
{
  return std::max(1e-9, nominal * 1e-3);
}

inline SphereSpec MakeSphereSpec(double scale)
{
  return SphereSpec{0.5 * scale};
}

inline CapsuleSpec MakeCapsuleSpec(double scale)
{
  return CapsuleSpec{0.5 * scale, 2.0 * scale};
}

inline BoxSpec MakeBoxSpec(double scale, bool thinFeature)
{
  BoxSpec spec;
  spec.halfExtents = Eigen::Vector3d(0.5 * scale, 0.5 * scale, 0.5 * scale);
  if (thinFeature) {
    spec.halfExtents.x() = 0.05 * scale;
  }
  spec.size = 2.0 * spec.halfExtents;
  return spec;
}

inline double SeparationFromNominal(double nominal, EdgeCase edge)
{
  switch (edge) {
    case EdgeCase::kDeepPenetration:
      return nominal * 0.25;
    case EdgeCase::kGrazing:
      return nominal - GrazingEpsilon(nominal);
    case EdgeCase::kTouching:
    case EdgeCase::kThinFeature:
      return nominal;
  }
  return nominal;
}

inline PairTransforms MakeSphereSphereTransforms(
    double radius1, double radius2, EdgeCase edge)
{
  PairTransforms tfs;
  const double nominal = radius1 + radius2;
  const double sep = SeparationFromNominal(nominal, edge);
  tfs.tf2 = OffsetTransform(sep, 0.0, 0.0);
  return tfs;
}

inline PairTransforms MakeBoxBoxTransforms(
    const Eigen::Vector3d& half1, const Eigen::Vector3d& half2, EdgeCase edge)
{
  PairTransforms tfs;
  const double nominalX = half1.x() + half2.x();
  const double sepX = SeparationFromNominal(nominalX, edge);

  if (edge == EdgeCase::kGrazing) {
    const double nominalY = half1.y() + half2.y();
    const double sepY = nominalY - GrazingEpsilon(nominalY);
    tfs.tf2 = OffsetTransform(sepX, sepY, 0.0);
  } else {
    tfs.tf2 = OffsetTransform(sepX, 0.0, 0.0);
  }
  return tfs;
}

inline PairTransforms MakeCapsuleCapsuleTransforms(
    double radius1, double radius2, EdgeCase edge)
{
  PairTransforms tfs;
  const double nominal = radius1 + radius2;
  const double sep = SeparationFromNominal(nominal, edge);
  tfs.tf2 = OffsetTransform(sep, 0.0, 0.0);
  return tfs;
}

inline PairTransforms MakeSphereBoxTransforms(
    double sphereRadius, const Eigen::Vector3d& boxHalf, EdgeCase edge)
{
  PairTransforms tfs;
  const double nominalX = boxHalf.x() + sphereRadius;
  const double sepX = SeparationFromNominal(nominalX, edge);

  if (edge == EdgeCase::kDeepPenetration) {
    tfs.tf1 = OffsetTransform(0.0, 0.0, 0.0);
    return tfs;
  }

  if (edge == EdgeCase::kGrazing) {
    const double nominalY = boxHalf.y() + sphereRadius;
    const double sepY = nominalY - GrazingEpsilon(nominalY);
    tfs.tf1 = OffsetTransform(sepX, sepY, 0.0);
  } else {
    tfs.tf1 = OffsetTransform(sepX, 0.0, 0.0);
  }

  return tfs;
}

inline PairTransforms MakeCapsuleSphereTransforms(
    double capsuleRadius, double sphereRadius, EdgeCase edge)
{
  PairTransforms tfs;
  const double nominal = capsuleRadius + sphereRadius;
  const double sep = SeparationFromNominal(nominal, edge);
  tfs.tf2 = OffsetTransform(sep, 0.0, 0.0);
  return tfs;
}

inline PairTransforms MakeCapsuleBoxTransforms(
    double capsuleRadius, const Eigen::Vector3d& boxHalf, EdgeCase edge)
{
  PairTransforms tfs;
  const double nominalX = boxHalf.x() + capsuleRadius;
  const double sepX = SeparationFromNominal(nominalX, edge);

  if (edge == EdgeCase::kDeepPenetration) {
    tfs.tf1 = OffsetTransform(0.0, 0.0, 0.0);
    return tfs;
  }

  if (edge == EdgeCase::kGrazing) {
    const double nominalY = boxHalf.y() + capsuleRadius;
    const double sepY = nominalY - GrazingEpsilon(nominalY);
    tfs.tf1 = OffsetTransform(sepX, sepY, 0.0);
  } else {
    tfs.tf1 = OffsetTransform(sepX, 0.0, 0.0);
  }

  return tfs;
}

} // namespace dart::benchmark::collision
