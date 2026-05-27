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
 *     copyright notice and this list of conditions in the documentation
 *     and/or other materials provided with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "deformable_body.hpp"

#include "dart/simulation/experimental/io/binary_io.hpp"
#include "dart/simulation/experimental/io/category_serializer.hpp"
#include "dart/simulation/experimental/io/serializer.hpp"

namespace dart::simulation::experimental::comps {
namespace {

//==============================================================================
template <typename T, typename Write>
void writeVector(
    std::ostream& output, const std::vector<T>& values, Write write)
{
  io::writePOD(output, values.size());
  for (const auto& value : values) {
    write(value);
  }
}

//==============================================================================
template <typename T, typename Read>
void readVector(std::istream& input, std::vector<T>& values, Read read)
{
  std::size_t size = 0;
  io::readPOD(input, size);
  values.resize(size);
  for (auto& value : values) {
    read(value);
  }
}

//==============================================================================
class DeformableNodeStateSerializer final
  : public io::TypedComponentSerializer<DeformableNodeState>
{
public:
  [[nodiscard]] std::string_view getTypeName() const override
  {
    return DeformableNodeState::getTypeName();
  }

private:
  void saveComponent(
      std::ostream& output,
      const DeformableNodeState& component,
      const io::EntityMap&) const override
  {
    const auto writeVector3 = [&](const Eigen::Vector3d& value) {
      io::writeVector3d(output, value);
    };
    const auto writeDouble = [&](double value) {
      io::writePOD(output, value);
    };
    const auto writeByte = [&](std::uint8_t value) {
      io::writePOD(output, value);
    };

    writeVector(output, component.positions, writeVector3);
    writeVector(output, component.previousPositions, writeVector3);
    writeVector(output, component.velocities, writeVector3);
    writeVector(output, component.masses, writeDouble);
    writeVector(output, component.fixed, writeByte);
  }

  void loadComponent(
      std::istream& input, DeformableNodeState& component) const override
  {
    const auto readVector3 = [&](Eigen::Vector3d& value) {
      io::readVector3d(input, value);
    };
    const auto readDouble = [&](double& value) {
      io::readPOD(input, value);
    };
    const auto readByte = [&](std::uint8_t& value) {
      io::readPOD(input, value);
    };

    readVector(input, component.positions, readVector3);
    readVector(input, component.previousPositions, readVector3);
    readVector(input, component.velocities, readVector3);
    readVector(input, component.masses, readDouble);
    readVector(input, component.fixed, readByte);
  }
};

//==============================================================================
class DeformableSpringModelSerializer final
  : public io::TypedComponentSerializer<DeformableSpringModel>
{
public:
  [[nodiscard]] std::string_view getTypeName() const override
  {
    return DeformableSpringModel::getTypeName();
  }

private:
  void saveComponent(
      std::ostream& output,
      const DeformableSpringModel& component,
      const io::EntityMap&) const override
  {
    writeVector(output, component.edges, [&](const DeformableSpringEdge& edge) {
      io::writePOD(output, edge.nodeA);
      io::writePOD(output, edge.nodeB);
      io::writePOD(output, edge.restLength);
    });
    io::writePOD(output, component.stiffness);
    io::writePOD(output, component.damping);
  }

  void loadComponent(
      std::istream& input, DeformableSpringModel& component) const override
  {
    readVector(input, component.edges, [&](DeformableSpringEdge& edge) {
      io::readPOD(input, edge.nodeA);
      io::readPOD(input, edge.nodeB);
      io::readPOD(input, edge.restLength);
    });
    io::readPOD(input, component.stiffness);
    io::readPOD(input, component.damping);
  }
};

DART_EXPERIMENTAL_REGISTER_COMPONENT(DeformableBodyTag)
[[maybe_unused]] io::SerializerRegistration<DeformableNodeStateSerializer>
    s_deformableNodeStateRegistration;
[[maybe_unused]] io::SerializerRegistration<DeformableSpringModelSerializer>
    s_deformableSpringModelRegistration;

} // namespace
} // namespace dart::simulation::experimental::comps
