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

#include <dart/simulation/World.hpp>

#include <dart/constraint/ConstraintBase.hpp>
#include <dart/constraint/ConstraintSolver.hpp>

#include <dart/dynamics/fem/DeformableBody.hpp>

#include <dart/common/Memory.hpp>

#include <limits>
#include <stdexcept>
#include <string>

#include <cassert>

namespace dart {
namespace dynamics {
namespace fem {
namespace detail {

//==============================================================================
/// Owns the node state of one deformable body and advances it once per step.
class DeformableBodyImpl
{
public:
  DeformableBodyImpl(const TetMesh& mesh, const Material& material)
    : mMesh(mesh), mMaterial(material)
  {
    const std::size_t numNodes = mMesh.getNumNodes();

    mPositions.setZero(static_cast<int>(3 * numNodes));
    mVelocities.setZero(static_cast<int>(3 * numNodes));
    mMasses.setZero(static_cast<int>(numNodes));

    for (std::size_t i = 0; i < numNodes; ++i) {
      mPositions.segment<3>(static_cast<int>(3 * i)) = mMesh.getRestPosition(i);
    }

    // Lump each tetrahedron's mass evenly onto its four nodes, so the summed
    // node masses reproduce density times total rest volume exactly.
    for (std::size_t t = 0; t < mMesh.getNumTets(); ++t) {
      const double tetMass = mMaterial.mDensity * mMesh.getRestVolume(t);
      const Eigen::Vector4i tet = mMesh.getTet(t);
      for (int corner = 0; corner < 4; ++corner) {
        mMasses[tet[corner]] += 0.25 * tetMass;
      }
    }
  }

  /// Advances the node state by one step of the world's time step.
  ///
  /// Semi-implicit Euler, matching the order the world integrates rigid
  /// bodies: velocities first, then positions from the updated velocities.
  void integrate(double timeStep, const Eigen::Vector3d& gravity)
  {
    const double damping = mMaterial.mLinearDamping;
    const auto numNodes = static_cast<int>(mMasses.size());

    for (int i = 0; i < numNodes; ++i) {
      const double mass = mMasses[i];
      // A node that no tetrahedron references carries no mass and cannot be
      // accelerated; leave it where it is instead of dividing by zero.
      if (!(mass > 0.0)) {
        continue;
      }

      Eigen::Vector3d velocity = mVelocities.segment<3>(3 * i);
      const Eigen::Vector3d force = mass * gravity - damping * velocity;
      velocity += (force / mass) * timeStep;

      mVelocities.segment<3>(3 * i) = velocity;
      mPositions.segment<3>(3 * i) += velocity * timeStep;
    }
  }

  /// Advances one step using the attached world's gravity and time step.
  void integrateFromWorld()
  {
    const auto world = mWorld.lock();
    if (!world) {
      return;
    }

    integrate(world->getTimeStep(), world->getGravity());
  }

  void resetToRest()
  {
    for (std::size_t i = 0; i < mMesh.getNumNodes(); ++i) {
      mPositions.segment<3>(static_cast<int>(3 * i)) = mMesh.getRestPosition(i);
    }
    mVelocities.setZero();
  }

  /// Binds each surface vertex to the tetrahedron that contains it.
  void setEmbeddedSurface(const std::vector<Eigen::Vector3d>& restVertices)
  {
    mSurfaceTets.clear();
    mSurfaceTets.reserve(restVertices.size());
    mSurfaceWeights.clear();
    mSurfaceWeights.reserve(restVertices.size());

    for (const auto& vertex : restVertices) {
      std::size_t bestTet = 0;
      Eigen::Vector4d bestWeights = Eigen::Vector4d::Zero();
      double bestViolation = std::numeric_limits<double>::max();

      for (std::size_t t = 0; t < mMesh.getNumTets(); ++t) {
        const Eigen::Vector4i tet = mMesh.getTet(t);
        const Eigen::Vector3d offset = vertex - mMesh.getRestPosition(tet[0]);
        const Eigen::Vector3d solved = mMesh.getInverseRestShape(t) * offset;

        Eigen::Vector4d weights;
        weights[0] = 1.0 - solved[0] - solved[1] - solved[2];
        weights.tail<3>() = solved;

        // Zero violation means the vertex is inside this tetrahedron; a
        // positive value measures how far outside it falls.
        const double violation = -weights.minCoeff();
        if (violation < bestViolation) {
          bestViolation = violation;
          bestWeights = weights;
          bestTet = t;
        }

        if (violation <= 0.0) {
          break;
        }
      }

      mSurfaceTets.push_back(bestTet);
      mSurfaceWeights.push_back(bestWeights);
    }
  }

  Eigen::Vector3d getSurfaceVertexPosition(std::size_t vertexIndex) const
  {
    assert(vertexIndex < mSurfaceTets.size());

    const Eigen::Vector4i tet = mMesh.getTet(mSurfaceTets[vertexIndex]);
    const Eigen::Vector4d& weights = mSurfaceWeights[vertexIndex];

    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    for (int corner = 0; corner < 4; ++corner) {
      position += weights[corner] * mPositions.segment<3>(3 * tet[corner]);
    }

    return position;
  }

  TetMesh mMesh;

  Material mMaterial;

  /// Node positions, laid out as [x0, y0, z0, x1, ...].
  Eigen::VectorXd mPositions;

  /// Node velocities, laid out like mPositions.
  Eigen::VectorXd mVelocities;

  /// Lumped mass of each node.
  Eigen::VectorXd mMasses;

  /// Tetrahedron each embedded surface vertex is bound to.
  std::vector<std::size_t> mSurfaceTets;

  /// Barycentric weights of each embedded surface vertex.
  common::aligned_vector<Eigen::Vector4d> mSurfaceWeights;

  /// World the body is attached to, if any.
  std::weak_ptr<simulation::World> mWorld;

  /// Constraint the solver updates once per step.
  constraint::ConstraintBasePtr mConstraint;
};

namespace {

//==============================================================================
/// Carries a deformable body into the constraint solver's per-step update.
///
/// The solver calls update() on every manual constraint each time it solves,
/// before deciding which constraints join the LCP. Reporting the constraint as
/// inactive keeps the body out of the LCP entirely, so it advances every step
/// without changing any rigid-body result.
class DeformableBodyUpdateConstraint : public constraint::ConstraintBase
{
public:
  explicit DeformableBodyUpdateConstraint(DeformableBodyImpl* impl)
    : mImpl(impl)
  {
    mDim = 0;
  }

  static const std::string& getStaticType()
  {
    static const std::string type = "FemDeformableBodyUpdate";
    return type;
  }

  const std::string& getType() const override
  {
    return getStaticType();
  }

  void update() override
  {
    mImpl->integrateFromWorld();
  }

  void getInformation(constraint::ConstraintInfo* /*info*/) override
  {
    // Never reached: the constraint reports itself as inactive.
  }

  void applyUnitImpulse(std::size_t /*index*/) override {}

  void getVelocityChange(double* /*velocity*/, bool /*withCfm*/) override {}

  void excite() override {}

  void unexcite() override {}

  void applyImpulse(double* /*lambda*/) override {}

  bool isActive() const override
  {
    return false;
  }

  dynamics::SkeletonPtr getRootSkeleton() const override
  {
    return nullptr;
  }

private:
  DeformableBodyImpl* mImpl;
};

} // namespace

} // namespace detail

//==============================================================================
DeformableBody::DeformableBody(const TetMesh& mesh, const Material& material)
  : mImpl(std::make_unique<detail::DeformableBodyImpl>(mesh, material))
{
  mImpl->mConstraint
      = std::make_shared<detail::DeformableBodyUpdateConstraint>(mImpl.get());
}

//==============================================================================
DeformableBody::~DeformableBody()
{
  detach();
}

//==============================================================================
std::shared_ptr<DeformableBody> DeformableBody::create(
    const TetMesh& mesh, const Material& material)
{
  if (mesh.isEmpty()) {
    throw std::invalid_argument(
        "DeformableBody::create: mesh has no tetrahedra");
  }

  return std::shared_ptr<DeformableBody>(new DeformableBody(mesh, material));
}

//==============================================================================
const TetMesh& DeformableBody::getMesh() const
{
  return mImpl->mMesh;
}

//==============================================================================
const Material& DeformableBody::getMaterial() const
{
  return mImpl->mMaterial;
}

//==============================================================================
std::size_t DeformableBody::getNumNodes() const
{
  return mImpl->mMesh.getNumNodes();
}

//==============================================================================
Eigen::Vector3d DeformableBody::getNodePosition(std::size_t nodeIndex) const
{
  assert(nodeIndex < getNumNodes());
  return mImpl->mPositions.segment<3>(static_cast<int>(3 * nodeIndex));
}

//==============================================================================
void DeformableBody::setNodePosition(
    std::size_t nodeIndex, const Eigen::Vector3d& position)
{
  assert(nodeIndex < getNumNodes());
  mImpl->mPositions.segment<3>(static_cast<int>(3 * nodeIndex)) = position;
}

//==============================================================================
Eigen::Vector3d DeformableBody::getNodeVelocity(std::size_t nodeIndex) const
{
  assert(nodeIndex < getNumNodes());
  return mImpl->mVelocities.segment<3>(static_cast<int>(3 * nodeIndex));
}

//==============================================================================
void DeformableBody::setNodeVelocity(
    std::size_t nodeIndex, const Eigen::Vector3d& velocity)
{
  assert(nodeIndex < getNumNodes());
  mImpl->mVelocities.segment<3>(static_cast<int>(3 * nodeIndex)) = velocity;
}

//==============================================================================
double DeformableBody::getNodeMass(std::size_t nodeIndex) const
{
  assert(nodeIndex < getNumNodes());
  return mImpl->mMasses[static_cast<int>(nodeIndex)];
}

//==============================================================================
double DeformableBody::getTotalMass() const
{
  return mImpl->mMasses.sum();
}

//==============================================================================
void DeformableBody::resetToRest()
{
  mImpl->resetToRest();
}

//==============================================================================
void DeformableBody::setEmbeddedSurface(
    const std::vector<Eigen::Vector3d>& restVertices)
{
  mImpl->setEmbeddedSurface(restVertices);
}

//==============================================================================
std::size_t DeformableBody::getNumSurfaceVertices() const
{
  return mImpl->mSurfaceTets.size();
}

//==============================================================================
Eigen::Vector3d DeformableBody::getSurfaceVertexPosition(
    std::size_t vertexIndex) const
{
  return mImpl->getSurfaceVertexPosition(vertexIndex);
}

//==============================================================================
void DeformableBody::attachTo(const std::shared_ptr<simulation::World>& world)
{
  if (!world) {
    throw std::invalid_argument("DeformableBody::attachTo: world is null");
  }

  auto* solver = world->getConstraintSolver();
  if (!solver) {
    throw std::invalid_argument(
        "DeformableBody::attachTo: world has no constraint solver");
  }

  detach();

  solver->addConstraint(mImpl->mConstraint);
  mImpl->mWorld = world;
}

//==============================================================================
void DeformableBody::detach()
{
  if (const auto world = mImpl->mWorld.lock()) {
    if (auto* solver = world->getConstraintSolver()) {
      solver->removeConstraint(mImpl->mConstraint);
    }
  }

  mImpl->mWorld.reset();
}

//==============================================================================
bool DeformableBody::isAttached() const
{
  return !mImpl->mWorld.expired();
}

} // namespace fem
} // namespace dynamics
} // namespace dart
