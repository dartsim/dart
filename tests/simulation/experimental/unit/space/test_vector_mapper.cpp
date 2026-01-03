/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include "dart/simulation/experimental/space/component_mapper.hpp"
#include "dart/simulation/experimental/space/state_space.hpp"
#include "dart/simulation/experimental/space/vector_mapper.hpp"

#include <entt/entt.hpp>
#include <gtest/gtest.h>

using namespace dart::simulation::experimental;

// Test component
struct Position
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

// Simple mapper for Position component
class PositionMapper : public ComponentMapper
{
public:
  size_t toVector(
      const entt::registry& registry,
      std::vector<double>& vec,
      size_t offset) const override
  {
    auto view = registry.view<Position>();
    size_t count = 0;

    for (auto entity : view) {
      const auto& pos = view.get<Position>(entity);
      vec[offset + count++] = pos.x;
      vec[offset + count++] = pos.y;
      vec[offset + count++] = pos.z;
    }

    return count;
  }

  size_t fromVector(
      entt::registry& registry,
      const std::vector<double>& vec,
      size_t offset) override
  {
    auto view = registry.view<Position>();
    size_t count = 0;

    for (auto entity : view) {
      auto& pos = view.get<Position>(entity);
      pos.x = vec[offset + count++];
      pos.y = vec[offset + count++];
      pos.z = vec[offset + count++];
    }

    return count;
  }

  size_t getDimension() const override
  {
    return 3;
  }
};

TEST(VectorMapper, BasicConstruction)
{
  StateSpace space;
  space.addVariable("pos", 3);
  space.finalize();

  VectorMapper mapper(space);
  EXPECT_EQ(mapper.getDimension(), 3);
}

TEST(VectorMapper, ToVectorWithScalarMapper)
{
  StateSpace space;
  space.addVariable("test", 1);
  space.finalize();

  VectorMapper mapper(space);

  // Add a scalar mapper
  mapper.addMapper(
      "test",
      std::make_unique<ScalarMapper>(
          [](const entt::registry&) { return 42.0; },
          [](entt::registry&, double) {}));

  entt::registry registry;
  auto vec = mapper.toVector(registry);

  ASSERT_EQ(vec.size(), 1);
  EXPECT_DOUBLE_EQ(vec[0], 42.0);
}

TEST(VectorMapper, RoundTripWithPositionMapper)
{
  StateSpace space;
  space.addVariable("positions", 6); // 2 entities × 3 DOF each
  space.finalize();

  VectorMapper mapper(space);
  mapper.addMapper("positions", std::make_unique<PositionMapper>());

  // Create entities with positions
  entt::registry registry;
  auto entity1 = registry.create();
  auto entity2 = registry.create();

  registry.emplace<Position>(entity1, Position{1.0, 2.0, 3.0});
  registry.emplace<Position>(entity2, Position{4.0, 5.0, 6.0});

  // Extract to vector
  auto vec1 = mapper.toVector(registry);

  ASSERT_EQ(vec1.size(), 6);
  // Note: EnTT doesn't guarantee entity iteration order,
  // so we just check that all values are present
  std::vector<double> expected = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
  for (double val : expected) {
    bool found = false;
    for (double v : vec1) {
      if (std::abs(v - val) < 1e-10) {
        found = true;
        break;
      }
    }
    EXPECT_TRUE(found) << "Value " << val << " not found in vector";
  }

  // Modify vector (multiply all by 10)
  std::vector<double> vec2 = vec1;
  for (auto& v : vec2) {
    v *= 10.0;
  }

  // Write back
  mapper.fromVector(registry, vec2);

  // Verify modifications - all values should be 10x original
  const auto& pos1 = registry.get<Position>(entity1);
  const auto& pos2 = registry.get<Position>(entity2);

  EXPECT_DOUBLE_EQ(pos1.x, 10.0);
  EXPECT_DOUBLE_EQ(pos1.y, 20.0);
  EXPECT_DOUBLE_EQ(pos1.z, 30.0);
  EXPECT_DOUBLE_EQ(pos2.x, 40.0);
  EXPECT_DOUBLE_EQ(pos2.y, 50.0);
  EXPECT_DOUBLE_EQ(pos2.z, 60.0);
}

TEST(VectorMapper, ToEigenConversion)
{
  StateSpace space;
  space.addVariable("test", 3);
  space.finalize();

  VectorMapper mapper(space);
  mapper.addMapper(
      "test",
      std::make_unique<ScalarMapper>(
          [](const entt::registry&) { return 1.5; },
          [](entt::registry&, double) {}));

  entt::registry registry;
  auto eigenVec = mapper.toEigen(registry);

  ASSERT_EQ(eigenVec.size(), 3);
  EXPECT_DOUBLE_EQ(eigenVec[0], 1.5);
  EXPECT_DOUBLE_EQ(eigenVec[1], 0.0);
  EXPECT_DOUBLE_EQ(eigenVec[2], 0.0);
}

TEST(VectorMapper, FromEigenConversion)
{
  StateSpace space;
  space.addVariable("positions", 3);
  space.finalize();

  VectorMapper mapper(space);

  entt::registry registry;
  auto entity = registry.create();
  registry.emplace<Position>(entity, Position{0.0, 0.0, 0.0});

  mapper.addMapper("positions", std::make_unique<PositionMapper>());

  // Create Eigen vector
  Eigen::VectorXd eigenVec(3);
  eigenVec << 7.0, 8.0, 9.0;

  // Write from Eigen
  mapper.fromEigen(registry, eigenVec);

  // Verify
  const auto& pos = registry.get<Position>(entity);
  EXPECT_DOUBLE_EQ(pos.x, 7.0);
  EXPECT_DOUBLE_EQ(pos.y, 8.0);
  EXPECT_DOUBLE_EQ(pos.z, 9.0);
}

TEST(VectorMapper, InPlaceToVector)
{
  StateSpace space;
  space.addVariable("test", 2);
  space.finalize();

  VectorMapper mapper(space);
  mapper.addMapper(
      "test",
      std::make_unique<ScalarMapper>(
          [](const entt::registry&) { return 3.14; },
          [](entt::registry&, double) {}));

  entt::registry registry;

  // Pre-allocate output
  std::vector<double> output(2);

  // In-place conversion
  mapper.toVector(registry, output);

  EXPECT_DOUBLE_EQ(output[0], 3.14);
  EXPECT_DOUBLE_EQ(output[1], 0.0);
}

TEST(VectorMapper, MultipleMappers)
{
  StateSpace space;
  space.addVariable("scalar1", 1);
  space.addVariable("scalar2", 1);
  space.addVariable("scalar3", 1);
  space.finalize();

  VectorMapper mapper(space);

  mapper.addMapper(
      "scalar1",
      std::make_unique<ScalarMapper>(
          [](const entt::registry&) { return 1.0; },
          [](entt::registry&, double) {}));

  mapper.addMapper(
      "scalar2",
      std::make_unique<ScalarMapper>(
          [](const entt::registry&) { return 2.0; },
          [](entt::registry&, double) {}));

  mapper.addMapper(
      "scalar3",
      std::make_unique<ScalarMapper>(
          [](const entt::registry&) { return 3.0; },
          [](entt::registry&, double) {}));

  entt::registry registry;
  auto vec = mapper.toVector(registry);

  ASSERT_EQ(vec.size(), 3);
  EXPECT_DOUBLE_EQ(vec[0], 1.0);
  EXPECT_DOUBLE_EQ(vec[1], 2.0);
  EXPECT_DOUBLE_EQ(vec[2], 3.0);
}

TEST(VectorMapper, InvalidVariableName)
{
  StateSpace space;
  space.addVariable("valid", 1);
  space.finalize();

  VectorMapper mapper(space);

  EXPECT_THROW(
      mapper.addMapper(
          "invalid",
          std::make_unique<ScalarMapper>(
              [](const entt::registry&) { return 0.0; },
              [](entt::registry&, double) {})),
      std::invalid_argument);
}

TEST(VectorMapper, OutputVectorTooSmall)
{
  StateSpace space;
  space.addVariable("test", 5);
  space.finalize();

  VectorMapper mapper(space);

  entt::registry registry;
  std::vector<double> tooSmall(3); // Should be at least 5

  EXPECT_THROW(mapper.toVector(registry, tooSmall), std::invalid_argument);
}

TEST(VectorMapper, InputVectorTooSmall)
{
  StateSpace space;
  space.addVariable("test", 5);
  space.finalize();

  VectorMapper mapper(space);

  entt::registry registry;
  std::vector<double> tooSmall(3); // Should be at least 5

  EXPECT_THROW(mapper.fromVector(registry, tooSmall), std::invalid_argument);
}

TEST(VectorMapper, NoMapperFillsZeros)
{
  StateSpace space;
  space.addVariable("unmapped", 3);
  space.finalize();

  VectorMapper mapper(space);
  // Don't add any mapper

  entt::registry registry;
  auto vec = mapper.toVector(registry);

  ASSERT_EQ(vec.size(), 3);
  EXPECT_DOUBLE_EQ(vec[0], 0.0);
  EXPECT_DOUBLE_EQ(vec[1], 0.0);
  EXPECT_DOUBLE_EQ(vec[2], 0.0);
}
