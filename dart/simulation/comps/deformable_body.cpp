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

#include "dart/simulation/common/exceptions.hpp"
#include "dart/simulation/io/binary_io.hpp"
#include "dart/simulation/io/category_serializer.hpp"
#include "dart/simulation/io/serializer.hpp"

#include <cmath>

namespace dart::simulation::comps {

//==============================================================================
void validateDeformableVbdConfig(const DeformableVbdConfig& config)
{
  DART_SIMULATION_THROW_T_IF(
      config.iterations == 0u,
      InvalidArgumentException,
      "DeformableVbdConfig.iterations must be positive");
  DART_SIMULATION_THROW_T_IF(
      !std::isfinite(config.convergenceDisplacement)
          || config.convergenceDisplacement < 0.0,
      InvalidArgumentException,
      "DeformableVbdConfig.convergenceDisplacement must be finite and "
      "non-negative");
  DART_SIMULATION_THROW_T_IF(
      !std::isfinite(config.chebyshevRho) || config.chebyshevRho <= 0.0
          || config.chebyshevRho >= 1.0,
      InvalidArgumentException,
      "DeformableVbdConfig.chebyshevRho must be finite and strictly between "
      "0 and 1");
  DART_SIMULATION_THROW_T_IF(
      !std::isfinite(config.rayleighDamping) || config.rayleighDamping < 0.0,
      InvalidArgumentException,
      "DeformableVbdConfig.rayleighDamping must be finite and non-negative");
  DART_SIMULATION_THROW_T_IF(
      !std::isfinite(config.contactStiffness) || config.contactStiffness < 0.0,
      InvalidArgumentException,
      "DeformableVbdConfig.contactStiffness must be finite and non-negative");
  DART_SIMULATION_THROW_T_IF(
      !std::isfinite(config.avbdAttachmentStiffness)
          || config.avbdAttachmentStiffness < 0.0,
      InvalidArgumentException,
      "DeformableVbdConfig.avbdAttachmentStiffness must be finite and "
      "non-negative");
  DART_SIMULATION_THROW_T_IF(
      !std::isfinite(config.avbdFiniteStiffnessStart)
          || config.avbdFiniteStiffnessStart < 0.0,
      InvalidArgumentException,
      "DeformableVbdConfig.avbdFiniteStiffnessStart must be finite and "
      "non-negative");
  DART_SIMULATION_THROW_T_IF(
      !std::isfinite(config.avbdAlpha) || config.avbdAlpha < 0.0
          || config.avbdAlpha > 1.0,
      InvalidArgumentException,
      "DeformableVbdConfig.avbdAlpha must be finite and between 0 and 1");
  DART_SIMULATION_THROW_T_IF(
      !std::isfinite(config.avbdBeta) || config.avbdBeta < 0.0,
      InvalidArgumentException,
      "DeformableVbdConfig.avbdBeta must be finite and non-negative");
  DART_SIMULATION_THROW_T_IF(
      !std::isfinite(config.avbdGamma) || config.avbdGamma < 0.0
          || config.avbdGamma > 1.0,
      InvalidArgumentException,
      "DeformableVbdConfig.avbdGamma must be finite and between 0 and 1");
  DART_SIMULATION_THROW_T_IF(
      std::isnan(config.avbdMaxStiffness) || config.avbdMaxStiffness < 0.0,
      InvalidArgumentException,
      "DeformableVbdConfig.avbdMaxStiffness must be non-negative or positive "
      "infinity");
}

namespace {

//==============================================================================
template <typename Vector, typename Write>
void writeVector(std::ostream& output, const Vector& values, Write write)
{
  io::writePOD(output, values.size());
  for (const auto& value : values) {
    write(value);
  }
}

//==============================================================================
template <typename Vector, typename Read>
void readVector(std::istream& input, Vector& values, Read read)
{
  std::size_t size = 0;
  io::readPOD(input, size);
  values.resize(size);
  for (auto& value : values) {
    read(value);
  }
}

//==============================================================================
class DeformableNodeModelSerializer final
  : public io::TypedComponentSerializer<DeformableNodeModel>
{
public:
  [[nodiscard]] std::string_view getTypeName() const override
  {
    return DeformableNodeModel::getTypeName();
  }

private:
  void saveComponent(
      std::ostream& output,
      const DeformableNodeModel& component,
      const io::EntityMap&) const override
  {
    const auto writeDouble = [&](double value) {
      io::writePOD(output, value);
    };
    const auto writeByte = [&](std::uint8_t value) {
      io::writePOD(output, value);
    };

    writeVector(output, component.masses, writeDouble);
    writeVector(output, component.fixed, writeByte);
  }

  void loadComponent(
      std::istream& input, DeformableNodeModel& component) const override
  {
    const auto readDouble = [&](double& value) {
      io::readPOD(input, value);
    };
    const auto readByte = [&](std::uint8_t& value) {
      io::readPOD(input, value);
    };

    readVector(input, component.masses, readDouble);
    readVector(input, component.fixed, readByte);
  }
};

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

    writeVector(output, component.positions, writeVector3);
    writeVector(output, component.previousPositions, writeVector3);
    writeVector(output, component.velocities, writeVector3);
    writeVector(output, component.attachmentTargets, writeVector3);
  }

  void loadComponent(
      std::istream& input, DeformableNodeState& component) const override
  {
    loadComponent(input, component, io::kBinaryFormatVersion);
  }

  void loadComponent(
      std::istream& input,
      DeformableNodeState& component,
      std::uint32_t formatVersion) const override
  {
    const auto readVector3 = [&](Eigen::Vector3d& value) {
      io::readVector3d(input, value);
    };

    readVector(input, component.positions, readVector3);
    readVector(input, component.previousPositions, readVector3);
    readVector(input, component.velocities, readVector3);
    if (formatVersion >= 32u) {
      readVector(input, component.attachmentTargets, readVector3);
    } else {
      component.attachmentTargets = component.positions;
    }
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

//==============================================================================
class DeformableMeshTopologySerializer final
  : public io::TypedComponentSerializer<DeformableMeshTopology>
{
public:
  [[nodiscard]] std::string_view getTypeName() const override
  {
    return DeformableMeshTopology::getTypeName();
  }

private:
  void saveComponent(
      std::ostream& output,
      const DeformableMeshTopology& component,
      const io::EntityMap&) const override
  {
    const auto writeVector3 = [&](const Eigen::Vector3d& value) {
      io::writeVector3d(output, value);
    };
    writeVector(output, component.restPositions, writeVector3);
    writeVector(
        output,
        component.surfaceTriangles,
        [&](const DeformableSurfaceTriangle& triangle) {
          io::writePOD(output, triangle.nodeA);
          io::writePOD(output, triangle.nodeB);
          io::writePOD(output, triangle.nodeC);
        });
    writeVector(
        output,
        component.tetrahedra,
        [&](const DeformableTetrahedron& tetrahedron) {
          io::writePOD(output, tetrahedron.nodeA);
          io::writePOD(output, tetrahedron.nodeB);
          io::writePOD(output, tetrahedron.nodeC);
          io::writePOD(output, tetrahedron.nodeD);
        });
    writeVector(output, component.tetrahedronRestVolumes, [&](double volume) {
      io::writePOD(output, volume);
    });
  }

  void loadComponent(
      std::istream& input, DeformableMeshTopology& component) const override
  {
    const auto readVector3 = [&](Eigen::Vector3d& value) {
      io::readVector3d(input, value);
    };
    readVector(input, component.restPositions, readVector3);
    readVector(
        input,
        component.surfaceTriangles,
        [&](DeformableSurfaceTriangle& triangle) {
          io::readPOD(input, triangle.nodeA);
          io::readPOD(input, triangle.nodeB);
          io::readPOD(input, triangle.nodeC);
        });
    readVector(
        input, component.tetrahedra, [&](DeformableTetrahedron& tetrahedron) {
          io::readPOD(input, tetrahedron.nodeA);
          io::readPOD(input, tetrahedron.nodeB);
          io::readPOD(input, tetrahedron.nodeC);
          io::readPOD(input, tetrahedron.nodeD);
        });
    readVector(input, component.tetrahedronRestVolumes, [&](double& volume) {
      io::readPOD(input, volume);
    });
  }
};

//==============================================================================
class DeformableMaterialSerializer final
  : public io::TypedComponentSerializer<DeformableMaterial>
{
public:
  [[nodiscard]] std::string_view getTypeName() const override
  {
    return DeformableMaterial::getTypeName();
  }

private:
  void saveComponent(
      std::ostream& output,
      const DeformableMaterial& component,
      const io::EntityMap&) const override
  {
    io::writePOD(output, component.density);
    io::writePOD(output, component.youngsModulus);
    io::writePOD(output, component.poissonRatio);
    io::writePOD(output, component.frictionCoefficient);
    io::writePOD(output, component.useFiniteElementElasticity);
    io::writePOD(output, component.useFixedCorotationalElasticity);
    io::writePOD(output, component.useAdaptiveBarrierStiffness);
    io::writePOD(output, component.useIterativeLinearSolver);
    io::writePOD(output, component.useMatrixFreeLinearSolver);
  }

  void loadComponent(
      std::istream& input, DeformableMaterial& component) const override
  {
    loadComponent(input, component, io::kBinaryFormatVersion);
  }

  void loadComponent(
      std::istream& input,
      DeformableMaterial& component,
      std::uint32_t formatVersion) const override
  {
    io::readPOD(input, component.density);
    io::readPOD(input, component.youngsModulus);
    io::readPOD(input, component.poissonRatio);
    io::readPOD(input, component.frictionCoefficient);
    io::readPOD(input, component.useFiniteElementElasticity);
    io::readPOD(input, component.useFixedCorotationalElasticity);
    io::readPOD(input, component.useAdaptiveBarrierStiffness);
    io::readPOD(input, component.useIterativeLinearSolver);
    if (formatVersion >= 10u) {
      io::readPOD(input, component.useMatrixFreeLinearSolver);
    } else {
      component.useMatrixFreeLinearSolver = false;
    }
  }
};

//==============================================================================
class DeformableContactConfigSerializer final
  : public io::TypedComponentSerializer<DeformableContactConfig>
{
public:
  [[nodiscard]] std::string_view getTypeName() const override
  {
    return DeformableContactConfig::getTypeName();
  }

private:
  void saveComponent(
      std::ostream& output,
      const DeformableContactConfig& component,
      const io::EntityMap&) const override
  {
    io::writePOD(output, component.surfaceCandidateCapacity);
  }

  void loadComponent(
      std::istream& input, DeformableContactConfig& component) const override
  {
    io::readPOD(input, component.surfaceCandidateCapacity);
    DART_SIMULATION_THROW_T_IF(
        !input,
        InvalidArgumentException,
        "Truncated comps.DeformableContactConfig component payload");
  }
};

//==============================================================================
class DeformableVbdConfigSerializer final
  : public io::TypedComponentSerializer<DeformableVbdConfig>
{
public:
  [[nodiscard]] std::string_view getTypeName() const override
  {
    return "comps.DeformableVbdConfig";
  }

private:
  void saveComponent(
      std::ostream& output,
      const DeformableVbdConfig& component,
      const io::EntityMap&) const override
  {
    validateDeformableVbdConfig(component);

    io::writePOD(output, component.enabled);
    io::writePOD(output, component.iterations);
    io::writePOD(output, component.convergenceDisplacement);
    io::writePOD(output, component.useChebyshev);
    io::writePOD(output, component.chebyshevRho);
    io::writePOD(output, component.rayleighDamping);
    io::writePOD(output, component.contactStiffness);
    io::writePOD(output, component.requireVbdExecution);
    io::writePOD(output, component.useAvbdContactNormalRows);
    io::writePOD(output, component.useAvbdSelfContactNormalRows);
    io::writePOD(output, component.useAvbdAttachmentRows);
    io::writePOD(output, component.avbdAttachmentStiffness);
    io::writePOD(output, component.useAvbdFiniteStiffnessRows);
    io::writePOD(output, component.avbdFiniteStiffnessStart);
    io::writePOD(output, component.avbdAlpha);
    io::writePOD(output, component.avbdBeta);
    io::writePOD(output, component.avbdGamma);
    io::writePOD(output, component.avbdMaxStiffness);
  }

  void loadComponent(
      std::istream& input, DeformableVbdConfig& component) const override
  {
    io::readPOD(input, component.enabled);
    io::readPOD(input, component.iterations);
    io::readPOD(input, component.convergenceDisplacement);
    io::readPOD(input, component.useChebyshev);
    io::readPOD(input, component.chebyshevRho);
    io::readPOD(input, component.rayleighDamping);
    io::readPOD(input, component.contactStiffness);
    io::readPOD(input, component.requireVbdExecution);
    io::readPOD(input, component.useAvbdContactNormalRows);
    io::readPOD(input, component.useAvbdSelfContactNormalRows);
    io::readPOD(input, component.useAvbdAttachmentRows);
    io::readPOD(input, component.avbdAttachmentStiffness);
    io::readPOD(input, component.useAvbdFiniteStiffnessRows);
    io::readPOD(input, component.avbdFiniteStiffnessStart);
    io::readPOD(input, component.avbdAlpha);
    io::readPOD(input, component.avbdBeta);
    io::readPOD(input, component.avbdGamma);
    io::readPOD(input, component.avbdMaxStiffness);

    DART_SIMULATION_THROW_T_IF(
        !input,
        InvalidArgumentException,
        "Truncated comps.DeformableVbdConfig component payload");
    validateDeformableVbdConfig(component);
  }
};

//==============================================================================
class DeformableBoundaryConditionsSerializer final
  : public io::TypedComponentSerializer<DeformableBoundaryConditions>
{
public:
  [[nodiscard]] std::string_view getTypeName() const override
  {
    return DeformableBoundaryConditions::getTypeName();
  }

private:
  void saveComponent(
      std::ostream& output,
      const DeformableBoundaryConditions& component,
      const io::EntityMap&) const override
  {
    const auto writeIndex = [&](std::size_t value) {
      io::writePOD(output, value);
    };
    const auto writeVector3 = [&](const Eigen::Vector3d& value) {
      io::writeVector3d(output, value);
    };

    writeVector(
        output,
        component.dirichlet,
        [&](const DeformableDirichletBoundary& boundary) {
          writeVector(output, boundary.nodes, writeIndex);
          writeVector(output, boundary.referencePositions, writeVector3);
          io::writeVector3d(output, boundary.center);
          io::writeVector3d(output, boundary.linearVelocity);
          io::writeVector3d(output, boundary.angularVelocity);
          io::writePOD(output, boundary.startTime);
          io::writePOD(output, boundary.endTime);
        });
    writeVector(
        output,
        component.neumann,
        [&](const DeformableNeumannBoundary& boundary) {
          writeVector(output, boundary.nodes, writeIndex);
          io::writeVector3d(output, boundary.acceleration);
          io::writePOD(output, boundary.startTime);
          io::writePOD(output, boundary.endTime);
        });
  }

  void loadComponent(
      std::istream& input,
      DeformableBoundaryConditions& component) const override
  {
    const auto readIndex = [&](std::size_t& value) {
      io::readPOD(input, value);
    };
    const auto readVector3 = [&](Eigen::Vector3d& value) {
      io::readVector3d(input, value);
    };

    readVector(
        input, component.dirichlet, [&](DeformableDirichletBoundary& boundary) {
          readVector(input, boundary.nodes, readIndex);
          readVector(input, boundary.referencePositions, readVector3);
          io::readVector3d(input, boundary.center);
          io::readVector3d(input, boundary.linearVelocity);
          io::readVector3d(input, boundary.angularVelocity);
          io::readPOD(input, boundary.startTime);
          io::readPOD(input, boundary.endTime);
        });
    readVector(
        input, component.neumann, [&](DeformableNeumannBoundary& boundary) {
          readVector(input, boundary.nodes, readIndex);
          io::readVector3d(input, boundary.acceleration);
          io::readPOD(input, boundary.startTime);
          io::readPOD(input, boundary.endTime);
        });
  }
};

//==============================================================================
template <typename SerializerT>
void registerSerializerIfNeeded(io::SerializerRegistry& registry)
{
  SerializerT serializerProbe;
  if (registry.getSerializer(serializerProbe.getTypeName()) != nullptr) {
    return;
  }

  registry.registerSerializer(std::make_unique<SerializerT>());
}

} // namespace

//==============================================================================
void registerDeformableBodySerializers(io::SerializerRegistry& registry)
{
  registerSerializerIfNeeded<DeformableNodeModelSerializer>(registry);
  registerSerializerIfNeeded<DeformableNodeStateSerializer>(registry);
  registerSerializerIfNeeded<DeformableSpringModelSerializer>(registry);
  registerSerializerIfNeeded<DeformableMeshTopologySerializer>(registry);
  registerSerializerIfNeeded<DeformableMaterialSerializer>(registry);
  registerSerializerIfNeeded<DeformableContactConfigSerializer>(registry);
  registerSerializerIfNeeded<DeformableVbdConfigSerializer>(registry);
  registerSerializerIfNeeded<DeformableBoundaryConditionsSerializer>(registry);
}

namespace {

DART_SIMULATION_REGISTER_COMPONENT(DeformableBodyTag)

} // namespace
} // namespace dart::simulation::comps
