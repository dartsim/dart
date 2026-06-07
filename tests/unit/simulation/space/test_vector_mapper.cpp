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

#include "dart/simulation/detail/world_registry_access.hpp"
#include "dart/simulation/space/component_mapper.hpp"
#include "dart/simulation/space/state_space.hpp"
#include "dart/simulation/space/vector_mapper.hpp"
#include "dart/simulation/world.hpp"

#include <Eigen/Core>
#include <entt/entt.hpp>
#include <gtest/gtest.h>

#include <functional>
#include <span>
#include <stdexcept>
using namespace dart::simulation;

// Test component
struct Position
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct VectorField
{
  Eigen::Vector3d value{Eigen::Vector3d::Zero()};
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
      std::span<const double> vec,
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

TEST(VectorMapper, RoundTripWithWorldRegistryFieldMapper)
{
  StateSpace space;
  space.addVariable("position_x", 1);
  space.finalize();

  VectorMapper mapper(space);
  mapper.addMapper(
      "position_x",
      std::make_unique<FieldMapper<Position, double>>(&Position::x));

  World world;
  auto& registry = detail::registryOf(world);
  auto entity = registry.create();

  registry.emplace<Position>(entity, Position{1.0, 2.0, 3.0});

  auto vec = mapper.toVector(registry);

  ASSERT_EQ(vec.size(), 1);
  EXPECT_DOUBLE_EQ(vec[0], 1.0);

  EXPECT_EQ(mapper.getDimension(), 1);
  const auto eigenVec = mapper.toEigen(registry);
  ASSERT_EQ(eigenVec.size(), 1);
  EXPECT_DOUBLE_EQ(eigenVec[0], 1.0);

  const std::vector<double> updated{9.0};
  mapper.fromVector(registry, std::span<const double>(updated));

  const auto& pos = registry.get<Position>(entity);
  EXPECT_DOUBLE_EQ(pos.x, 9.0);
}

TEST(VectorMapper, FieldMapperHandlesEnttRegistryScalarAndEigenFields)
{
  StateSpace space;
  space.addVariable("position_x", 1);
  space.addVariable("field", 3);
  space.finalize();

  VectorMapper mapper(space);
  mapper.addMapper(
      "position_x",
      std::make_unique<FieldMapper<Position, double>>(&Position::x));
  mapper.addMapper(
      "field",
      std::make_unique<FieldMapper<VectorField, Eigen::Vector3d>>(
          &VectorField::value));

  entt::registry registry;
  auto entity = registry.create();
  registry.emplace<Position>(entity, Position{1.0, 2.0, 3.0});
  registry.emplace<VectorField>(
      entity, VectorField{Eigen::Vector3d(4.0, 5.0, 6.0)});

  auto vec = mapper.toVector(registry);

  ASSERT_EQ(vec.size(), 4);
  EXPECT_DOUBLE_EQ(vec[0], 1.0);
  EXPECT_DOUBLE_EQ(vec[1], 4.0);
  EXPECT_DOUBLE_EQ(vec[2], 5.0);
  EXPECT_DOUBLE_EQ(vec[3], 6.0);

  const std::vector<double> updated{7.0, 8.0, 9.0, 10.0};
  mapper.fromVector(registry, std::span<const double>(updated));

  const auto& pos = registry.get<Position>(entity);
  const auto& field = registry.get<VectorField>(entity);
  EXPECT_DOUBLE_EQ(pos.x, 7.0);
  EXPECT_DOUBLE_EQ(field.value.x(), 8.0);
  EXPECT_DOUBLE_EQ(field.value.y(), 9.0);
  EXPECT_DOUBLE_EQ(field.value.z(), 10.0);
}

TEST(VectorMapper, WorldRegistryFieldMapperCoversEigenAndSizeGuards)
{
  StateSpace space;
  space.addVariable("field", 3);
  space.finalize();

  VectorMapper mapper(space);
  mapper.addMapper(
      "field",
      std::make_unique<FieldMapper<VectorField, Eigen::Vector3d>>(
          &VectorField::value));

  World world;
  auto& registry = detail::registryOf(world);
  auto entity = registry.create();
  registry.emplace<VectorField>(
      entity, VectorField{Eigen::Vector3d(1.0, 2.0, 3.0)});

  std::vector<double> output(3);
  mapper.toVector(registry, output);
  EXPECT_DOUBLE_EQ(output[0], 1.0);
  EXPECT_DOUBLE_EQ(output[1], 2.0);
  EXPECT_DOUBLE_EQ(output[2], 3.0);

  Eigen::VectorXd eigenOutput(3);
  mapper.toEigen(registry, eigenOutput);
  EXPECT_DOUBLE_EQ(eigenOutput[0], 1.0);
  EXPECT_DOUBLE_EQ(eigenOutput[1], 2.0);
  EXPECT_DOUBLE_EQ(eigenOutput[2], 3.0);

  std::vector<double> tooSmallOutput(2);
  EXPECT_THROW(
      mapper.toVector(registry, tooSmallOutput), std::invalid_argument);

  Eigen::VectorXd tooSmallEigenOutput(2);
  EXPECT_THROW(
      mapper.toEigen(registry, tooSmallEigenOutput), std::invalid_argument);

  const std::vector<double> updated{4.0, 5.0, 6.0};
  mapper.fromVector(registry, std::span<const double>(updated));
  EXPECT_DOUBLE_EQ(registry.get<VectorField>(entity).value.x(), 4.0);
  EXPECT_DOUBLE_EQ(registry.get<VectorField>(entity).value.y(), 5.0);
  EXPECT_DOUBLE_EQ(registry.get<VectorField>(entity).value.z(), 6.0);

  const std::vector<double> tooSmallInput{7.0, 8.0};
  EXPECT_THROW(
      mapper.fromVector(registry, std::span<const double>(tooSmallInput)),
      std::invalid_argument);

  Eigen::VectorXd eigenUpdated(3);
  eigenUpdated << 9.0, 10.0, 11.0;
  mapper.fromEigen(registry, eigenUpdated);
  EXPECT_DOUBLE_EQ(registry.get<VectorField>(entity).value.x(), 9.0);
  EXPECT_DOUBLE_EQ(registry.get<VectorField>(entity).value.y(), 10.0);
  EXPECT_DOUBLE_EQ(registry.get<VectorField>(entity).value.z(), 11.0);

  Eigen::VectorXd tooSmallEigenInput(2);
  EXPECT_THROW(
      mapper.fromEigen(registry, tooSmallEigenInput), std::invalid_argument);
}

TEST(VectorMapper, WorldRegistryScalarMapperPreservesGenericCallbacks)
{
  StateSpace space;
  space.addVariable("position_x", 1);
  space.finalize();

  VectorMapper mapper(space);
  mapper.addMapper(
      "position_x",
      std::make_unique<ScalarMapper>(
          [](const auto& registry) {
            auto view = registry.template view<Position>();
            for (auto entity : view) {
              return registry.template get<Position>(entity).x;
            }
            return 0.0;
          },
          [](auto& registry, double value) {
            auto view = registry.template view<Position>();
            for (auto entity : view) {
              registry.template get<Position>(entity).x = value;
              return;
            }
          }));

  World world;
  auto& registry = detail::registryOf(world);
  auto entity = registry.create();
  registry.emplace<Position>(entity, Position{2.0, 3.0, 4.0});

  auto vec = mapper.toVector(registry);

  ASSERT_EQ(vec.size(), 1);
  EXPECT_DOUBLE_EQ(vec[0], 2.0);

  const std::vector<double> updated{8.0};
  mapper.fromVector(registry, std::span<const double>(updated));

  const auto& pos = registry.get<Position>(entity);
  EXPECT_DOUBLE_EQ(pos.x, 8.0);
}

TEST(VectorMapper, WorldRegistryScalarMapperSupportsWorldOnlyCallbacks)
{
  StateSpace space;
  space.addVariable("position_x", 1);
  space.finalize();

  VectorMapper mapper(space);
  mapper.addMapper(
      "position_x",
      std::make_unique<ScalarMapper>(
          std::function<double(const detail::WorldRegistry&)>(
              [](const detail::WorldRegistry& registry) {
                auto view = registry.view<Position>();
                for (auto entity : view) {
                  return registry.get<Position>(entity).x;
                }
                return 0.0;
              }),
          std::function<void(detail::WorldRegistry&, double)>(
              [](detail::WorldRegistry& registry, double value) {
                auto view = registry.view<Position>();
                for (auto entity : view) {
                  registry.get<Position>(entity).x = value;
                  return;
                }
              })));

  World world;
  auto& registry = detail::registryOf(world);
  auto entity = registry.create();
  registry.emplace<Position>(entity, Position{3.0, 4.0, 5.0});

  std::vector<double> output(1);
  mapper.toVector(registry, output);
  EXPECT_DOUBLE_EQ(output[0], 3.0);

  const auto eigenVec = mapper.toEigen(registry);
  ASSERT_EQ(eigenVec.size(), 1);
  EXPECT_DOUBLE_EQ(eigenVec[0], 3.0);

  Eigen::VectorXd updated(1);
  updated << 12.0;
  mapper.fromEigen(registry, updated);

  const auto& pos = registry.get<Position>(entity);
  EXPECT_DOUBLE_EQ(pos.x, 12.0);
}

TEST(VectorMapper, ScalarMapperRejectsUnsupportedRegistryDirectly)
{
  ScalarMapper enttOnlyMapper(
      [](const entt::registry&) { return 1.0; },
      [](entt::registry&, double) {});

  World world;
  auto& worldRegistry = detail::registryOf(world);
  std::vector<double> output(1);
  EXPECT_THROW(
      enttOnlyMapper.toVector(worldRegistry, output, 0), std::invalid_argument);

  const std::vector<double> input{2.0};
  EXPECT_THROW(
      enttOnlyMapper.fromVector(
          worldRegistry, std::span<const double>(input), 0),
      std::invalid_argument);

  ScalarMapper worldOnlyMapper(
      std::function<double(const detail::WorldRegistry&)>(
          [](const detail::WorldRegistry&) { return 3.0; }),
      std::function<void(detail::WorldRegistry&, double)>(
          [](detail::WorldRegistry&, double) {}));

  entt::registry enttRegistry;
  EXPECT_THROW(
      worldOnlyMapper.toVector(enttRegistry, output, 0), std::invalid_argument);
  EXPECT_THROW(
      worldOnlyMapper.fromVector(
          enttRegistry, std::span<const double>(input), 0),
      std::invalid_argument);
}

TEST(VectorMapper, WorldRegistryRejectsEnttOnlyScalarMapper)
{
  StateSpace space;
  space.addVariable("position_x", 1);
  space.finalize();

  VectorMapper mapper(space);
  mapper.addMapper(
      "position_x",
      std::make_unique<ScalarMapper>(
          [](const entt::registry&) { return 1.0; },
          [](entt::registry&, double) {}));

  World world;
  auto& registry = detail::registryOf(world);

  EXPECT_THROW((void)mapper.toVector(registry), std::invalid_argument);

  const std::vector<double> updated{2.0};
  EXPECT_THROW(
      mapper.fromVector(registry, std::span<const double>(updated)),
      std::invalid_argument);
}

TEST(VectorMapper, WorldRegistryRequiresWorldCapableMapper)
{
  StateSpace space;
  space.addVariable("position", 3);
  space.finalize();

  VectorMapper mapper(space);
  mapper.addMapper("position", std::make_unique<PositionMapper>());

  World world;
  auto& registry = detail::registryOf(world);
  auto entity = registry.create();
  registry.emplace<Position>(entity, Position{1.0, 2.0, 3.0});

  EXPECT_THROW((void)mapper.toVector(registry), std::invalid_argument);

  const std::vector<double> updated{1.0, 2.0, 3.0};
  EXPECT_THROW(
      mapper.fromVector(registry, std::span<const double>(updated)),
      std::invalid_argument);
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

  std::vector<double> output{1.0, 2.0, 3.0};
  mapper.toVector(registry, output);
  EXPECT_DOUBLE_EQ(output[0], 0.0);
  EXPECT_DOUBLE_EQ(output[1], 0.0);
  EXPECT_DOUBLE_EQ(output[2], 0.0);
}

TEST(VectorMapper, WorldRegistryNoMapperFillsZerosAndSkipsInput)
{
  StateSpace space;
  space.addVariable("unmapped_scalar", 1);
  space.addVariable("unmapped_vector", 2);
  space.finalize();

  VectorMapper mapper(space);
  World world;
  auto& registry = detail::registryOf(world);

  auto vec = mapper.toVector(registry);
  ASSERT_EQ(vec.size(), 3);
  EXPECT_DOUBLE_EQ(vec[0], 0.0);
  EXPECT_DOUBLE_EQ(vec[1], 0.0);
  EXPECT_DOUBLE_EQ(vec[2], 0.0);

  std::vector<double> output{1.0, 2.0, 3.0};
  mapper.toVector(registry, output);
  EXPECT_DOUBLE_EQ(output[0], 0.0);
  EXPECT_DOUBLE_EQ(output[1], 0.0);
  EXPECT_DOUBLE_EQ(output[2], 0.0);

  const auto eigenVec = mapper.toEigen(registry);
  ASSERT_EQ(eigenVec.size(), 3);
  EXPECT_DOUBLE_EQ(eigenVec[0], 0.0);
  EXPECT_DOUBLE_EQ(eigenVec[1], 0.0);
  EXPECT_DOUBLE_EQ(eigenVec[2], 0.0);

  const std::vector<double> updated{4.0, 5.0, 6.0};
  EXPECT_NO_THROW(
      mapper.fromVector(registry, std::span<const double>(updated)));

  Eigen::VectorXd eigenUpdated(3);
  eigenUpdated << 7.0, 8.0, 9.0;
  EXPECT_NO_THROW(mapper.fromEigen(registry, eigenUpdated));
}
