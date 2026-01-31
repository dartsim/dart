// Copyright (c) 2011-2025, The DART development contributors
// All rights reserved.
//
// The list of contributors can be found at:
//   https://github.com/dartsim/dart/blob/main/LICENSE
//
// This file is provided under the following "BSD-style" License:
//   Redistribution and use in source and binary forms, with or
//   without modification, are permitted provided that the following
//   conditions are met:
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above
//     copyright notice, this list of conditions and the following
//     disclaimer in the documentation and/or other materials provided
//     with the distribution.
//   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
//   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
//   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
//   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
//   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
//   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//   POSSIBILITY OF SUCH DAMAGE.

#define DART_UNITTEST_SPECIALIZED_ASPECT_ACCESS

#include <dart/math/mesh.hpp>

#include <dart/common/aspect.hpp>
#include <dart/common/composite.hpp>
#include <dart/common/singleton.hpp>
#include <dart/common/specialized_for_aspect.hpp>

#include <gtest/gtest.h>

#include <memory>

namespace {

class DummyAspect final : public dart::common::Aspect
{
public:
  std::unique_ptr<dart::common::Aspect> cloneAspect() const override
  {
    return std::make_unique<DummyAspect>();
  }
};

class SpecialAspect final : public dart::common::Aspect
{
public:
  std::unique_ptr<dart::common::Aspect> cloneAspect() const override
  {
    return std::make_unique<SpecialAspect>();
  }
};

class TestComposite : public dart::common::Composite
{
};

class TestSpecializedComposite
  : public dart::common::SpecializedForAspect<SpecialAspect>
{
};

class TestSingleton final : public dart::common::Singleton<TestSingleton>
{
public:
  explicit TestSingleton(int value) : value(value) {}

  int value{0};
};

class MeshWrapper : public dart::math::Meshd
{
public:
  void normalize()
  {
    normalizeVertexNormals();
  }
};

} // namespace

//==============================================================================
TEST(TemplateCoverage, CompositeAndSpecialized)
{
  TestComposite composite;
  EXPECT_FALSE(composite.has<DummyAspect>());
  EXPECT_EQ(composite.get<DummyAspect>(), nullptr);
  EXPECT_FALSE(TestComposite::isSpecializedFor<DummyAspect>());

  auto* created = composite.createAspect<DummyAspect>();
  ASSERT_NE(created, nullptr);
  EXPECT_TRUE(composite.has<DummyAspect>());

  composite.removeAspect<DummyAspect>();
  EXPECT_FALSE(composite.has<DummyAspect>());

  auto owned = std::make_unique<DummyAspect>();
  composite.set<DummyAspect>(std::move(owned));
  EXPECT_TRUE(composite.has<DummyAspect>());

  auto released = composite.releaseAspect<DummyAspect>();
  EXPECT_NE(released, nullptr);
  EXPECT_FALSE(composite.has<DummyAspect>());

  TestSpecializedComposite specialized;
  EXPECT_FALSE(specialized.has<SpecialAspect>());
  EXPECT_FALSE(specialized.has<DummyAspect>());

  auto* specAspect = specialized.createAspect<SpecialAspect>();
  EXPECT_EQ(specAspect, specialized.get<SpecialAspect>());
  EXPECT_TRUE(specialized.has<SpecialAspect>());
  EXPECT_TRUE(TestSpecializedComposite::isSpecializedFor<SpecialAspect>());

  specialized.set<SpecialAspect>(specAspect);
  auto extracted = specialized.releaseAspect<SpecialAspect>();
  EXPECT_NE(extracted, nullptr);
  EXPECT_FALSE(specialized.has<SpecialAspect>());
}

//==============================================================================
TEST(TemplateCoverage, SingletonAndMesh)
{
  auto& singleton = TestSingleton::getSingleton(7);
  EXPECT_EQ(singleton.value, 7);
  auto* singletonPtr = TestSingleton::getSingletonPtr(9);
  EXPECT_EQ(singletonPtr, &singleton);

  MeshWrapper mesh;
  EXPECT_TRUE(mesh.isEmpty());

  mesh.getVertices().emplace_back(0.0, 0.0, 0.0);
  mesh.getVertexNormals().emplace_back(1.0, 0.0, 0.0);

  EXPECT_TRUE(mesh.hasVertices());
  EXPECT_TRUE(mesh.hasVertexNormals());

  mesh.translate(Eigen::Vector3d(1.0, 0.0, 0.0));
  mesh.normalize();

  MeshWrapper other;
  other.getVertices().emplace_back(0.5, 0.5, 0.5);
  other.getVertexNormals().emplace_back(0.0, 1.0, 0.0);

  auto combined = mesh + other;
  mesh += other;
  EXPECT_FALSE(combined.isEmpty());
}
