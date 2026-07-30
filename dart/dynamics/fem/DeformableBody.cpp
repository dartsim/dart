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

#include <dart/common/Macros.hpp>
#include <dart/common/Memory.hpp>

#include <limits>
#include <stdexcept>
#include <string>

#include <cmath>

namespace dart {
namespace dynamics {
namespace fem {
namespace detail {

//==============================================================================
/// Returns the Lame parameters of an isotropic linear-elastic solid.
///
/// Shared by construction and by validation, so the values checked for
/// admissibility are exactly the ones the body goes on to use.
inline void computeLameParameters(
    const Material& material, double& shearModulus, double& lameLambda)
{
  const double youngsModulus = material.mYoungsModulus;
  const double poissonRatio = material.mPoissonRatio;
  shearModulus = youngsModulus / (2.0 * (1.0 + poissonRatio));
  lameLambda = youngsModulus * poissonRatio
               / ((1.0 + poissonRatio) * (1.0 - 2.0 * poissonRatio));
}

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
    mForces.setZero(static_cast<int>(3 * numNodes));
    mMasses.setZero(static_cast<int>(numNodes));
    mFixed.assign(numNodes, 0);

    computeLameParameters(mMaterial, mShearModulus, mLameLambda);

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
    computeElasticForces();

    // Damping is mass proportional and integrated implicitly, so the whole
    // update is
    //   v <- (v + g * dt) / (1 + c * dt).
    //
    // Two properties motivate that form. Dividing the damping force by the node
    // mass, rather than scaling it by the mass, would give every node its own
    // decay factor; lumped masses vary by more than ten times across an
    // ordinary box mesh, so a body translating uniformly would damp faster at
    // its corners than in its interior and tear itself apart even though no
    // elastic force exists yet. Scaling by mass makes the factor identical for
    // every node, which preserves rigid translation exactly. Applying it
    // implicitly then keeps the update stable for any positive damping and time
    // step, where an explicit factor of (1 - c * dt) would diverge past c * dt
    // of two. The undamped case stays bit-identical, and the damped case keeps
    // a closed form the tests check exactly.
    const double dampingDivisor = 1.0 + mMaterial.mLinearDamping * timeStep;
    const Eigen::Vector3d gravityStep = gravity * timeStep;
    const auto numNodes = static_cast<int>(mMasses.size());

    for (int i = 0; i < numNodes; ++i) {
      // A fixed node is anchored: it keeps zero velocity and ignores gravity,
      // elastic force, and damping alike.
      if (mFixed[static_cast<std::size_t>(i)] != 0) {
        mVelocities.segment<3>(3 * i).setZero();
        continue;
      }

      const double mass = mMasses[i];
      // A node that no tetrahedron references carries no mass and cannot be
      // accelerated; leave it where it is instead of dividing by zero.
      if (!(mass > 0.0)) {
        continue;
      }

      Eigen::Vector3d velocity = mVelocities.segment<3>(3 * i);
      velocity += mForces.segment<3>(3 * i) * (timeStep / mass);
      velocity = (velocity + gravityStep) / dampingDivisor;

      mVelocities.segment<3>(3 * i) = velocity;
      mPositions.segment<3>(3 * i) += velocity * timeStep;
    }
  }

  /// Returns the rotation part of a deformation gradient's polar decomposition.
  ///
  /// Taken from the singular value decomposition F = U S V^T as R = U V^T. When
  /// an element inverts, det(F) is negative and that product is a reflection
  /// rather than a rotation, so the smallest singular direction is flipped to
  /// recover the nearest true rotation.
  static Eigen::Matrix3d polarRotation(const Eigen::Matrix3d& F)
  {
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(
        F, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d u = svd.matrixU();
    const Eigen::Matrix3d& v = svd.matrixV();

    if ((u * v.transpose()).determinant() < 0.0) {
      u.col(2) = -u.col(2);
    }

    return u * v.transpose();
  }

  /// Returns the first Piola-Kirchhoff stress of one element.
  Eigen::Matrix3d computeStress(const Eigen::Matrix3d& F) const
  {
    if (mMaterial.mElasticModel == ElasticModel::Linear) {
      // Small-strain tensor. It cannot tell a rotation from a stretch, which is
      // exactly what the co-rotated model below fixes.
      const Eigen::Matrix3d strain
          = 0.5 * (F + F.transpose()) - Eigen::Matrix3d::Identity();
      return 2.0 * mShearModulus * strain
             + mLameLambda * strain.trace() * Eigen::Matrix3d::Identity();
    }

    // Co-rotated: measure strain in the element's own unrotated frame, so a
    // rigid rotation gives F == R and no stress at all.
    const Eigen::Matrix3d rotation = polarRotation(F);
    return 2.0 * mShearModulus * (F - rotation)
           + mLameLambda * ((rotation.transpose() * F).trace() - 3.0)
                 * rotation;
  }

  /// Returns the current deformation gradient of one element.
  Eigen::Matrix3d computeDeformationGradient(std::size_t tetIndex) const
  {
    const Eigen::Vector4i tet = mMesh.getTet(tetIndex);
    const auto x0 = mPositions.segment<3>(3 * tet[0]);

    Eigen::Matrix3d shape;
    shape.col(0) = mPositions.segment<3>(3 * tet[1]) - x0;
    shape.col(1) = mPositions.segment<3>(3 * tet[2]) - x0;
    shape.col(2) = mPositions.segment<3>(3 * tet[3]) - x0;

    return shape * mMesh.getInverseRestShape(tetIndex);
  }

  /// Recomputes every node's elastic force from the current positions.
  void computeElasticForces()
  {
    mForces.setZero();

    for (std::size_t t = 0; t < mMesh.getNumTets(); ++t) {
      const Eigen::Matrix3d stress
          = computeStress(computeDeformationGradient(t));

      // Nodal forces of one element: H = -volume * P * inverseRestShape^T gives
      // the forces on nodes one through three as its columns, and the force on
      // node zero follows from the element exerting no net force on itself.
      const Eigen::Matrix3d nodalForces
          = -mMesh.getRestVolume(t) * stress
            * mMesh.getInverseRestShape(t).transpose();

      const Eigen::Vector4i tet = mMesh.getTet(t);
      Eigen::Vector3d sum = Eigen::Vector3d::Zero();
      for (int corner = 0; corner < 3; ++corner) {
        const Eigen::Vector3d force = nodalForces.col(corner);
        mForces.segment<3>(3 * tet[corner + 1]) += force;
        sum += force;
      }
      mForces.segment<3>(3 * tet[0]) -= sum;
    }
  }

  /// Returns the elastic potential energy of the current configuration.
  double getElasticEnergy() const
  {
    double energy = 0.0;

    for (std::size_t t = 0; t < mMesh.getNumTets(); ++t) {
      const Eigen::Matrix3d F = computeDeformationGradient(t);
      double density = 0.0;

      if (mMaterial.mElasticModel == ElasticModel::Linear) {
        const Eigen::Matrix3d strain
            = 0.5 * (F + F.transpose()) - Eigen::Matrix3d::Identity();
        density = mShearModulus * strain.squaredNorm()
                  + 0.5 * mLameLambda * strain.trace() * strain.trace();
      } else {
        const Eigen::Matrix3d rotation = polarRotation(F);
        const double volumetric = (rotation.transpose() * F).trace() - 3.0;
        density = mShearModulus * (F - rotation).squaredNorm()
                  + 0.5 * mLameLambda * volumetric * volumetric;
      }

      energy += mMesh.getRestVolume(t) * density;
    }

    return energy;
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
    DART_ASSERT(vertexIndex < mSurfaceTets.size());

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

  /// Elastic force on each node, laid out like mPositions.
  Eigen::VectorXd mForces;

  /// Lumped mass of each node.
  Eigen::VectorXd mMasses;

  /// Whether each node is anchored in place.
  std::vector<char> mFixed;

  /// Lame parameters derived from the material's Young's modulus and
  /// Poisson's ratio.
  double mShearModulus = 0.0;
  double mLameLambda = 0.0;

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

  // TetMesh rejects non-positive element volumes so that lumped masses stay
  // positive; a non-positive density would defeat that from the other side,
  // leaving every node massless or negative and the body permanently inert
  // while still reporting itself attached and healthy. These comparisons are
  // written so that a NaN is rejected too.
  // Each comparison is written so that a NaN fails it, and each finite check
  // additionally rejects an infinity. An infinite modulus would otherwise pass
  // a bare positivity test, make both Lame parameters infinite, and then
  // multiply them by a zero strain at rest, so the very first step would fill
  // every node with NaN.
  if (!(material.mDensity > 0.0) || !std::isfinite(material.mDensity)) {
    throw std::invalid_argument(
        "DeformableBody::create: material density must be positive and finite");
  }

  if (!(material.mLinearDamping >= 0.0)
      || !std::isfinite(material.mLinearDamping)) {
    throw std::invalid_argument(
        "DeformableBody::create: material linear damping must be non-negative "
        "and finite");
  }

  if (!(material.mYoungsModulus > 0.0)
      || !std::isfinite(material.mYoungsModulus)) {
    throw std::invalid_argument(
        "DeformableBody::create: material Young's modulus must be positive and "
        "finite");
  }

  // Outside (-1, 0.5) the Lame parameter lambda is negative or divergent, so
  // the material is not physically admissible.
  if (!(material.mPoissonRatio > -1.0 && material.mPoissonRatio < 0.5)) {
    throw std::invalid_argument(
        "DeformableBody::create: material Poisson's ratio must lie strictly "
        "between -1 and 0.5");
  }

  // A ratio just short of 0.5, or a very large modulus, can still overflow the
  // derived parameters even though each input passed on its own.
  double shearModulus = 0.0;
  double lameLambda = 0.0;
  detail::computeLameParameters(material, shearModulus, lameLambda);
  if (!std::isfinite(shearModulus) || !std::isfinite(lameLambda)) {
    throw std::invalid_argument(
        "DeformableBody::create: material yields non-finite Lame parameters; "
        "reduce Young's modulus or move Poisson's ratio away from 0.5");
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
  DART_ASSERT(nodeIndex < getNumNodes());
  return mImpl->mPositions.segment<3>(static_cast<int>(3 * nodeIndex));
}

//==============================================================================
void DeformableBody::setNodePosition(
    std::size_t nodeIndex, const Eigen::Vector3d& position)
{
  DART_ASSERT(nodeIndex < getNumNodes());
  mImpl->mPositions.segment<3>(static_cast<int>(3 * nodeIndex)) = position;
}

//==============================================================================
Eigen::Vector3d DeformableBody::getNodeVelocity(std::size_t nodeIndex) const
{
  DART_ASSERT(nodeIndex < getNumNodes());
  return mImpl->mVelocities.segment<3>(static_cast<int>(3 * nodeIndex));
}

//==============================================================================
void DeformableBody::setNodeVelocity(
    std::size_t nodeIndex, const Eigen::Vector3d& velocity)
{
  DART_ASSERT(nodeIndex < getNumNodes());
  mImpl->mVelocities.segment<3>(static_cast<int>(3 * nodeIndex)) = velocity;
}

//==============================================================================
double DeformableBody::getNodeMass(std::size_t nodeIndex) const
{
  DART_ASSERT(nodeIndex < getNumNodes());
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
void DeformableBody::computeElasticForces()
{
  mImpl->computeElasticForces();
}

//==============================================================================
Eigen::Vector3d DeformableBody::getNodeForce(std::size_t nodeIndex) const
{
  DART_ASSERT(nodeIndex < getNumNodes());
  return mImpl->mForces.segment<3>(static_cast<int>(3 * nodeIndex));
}

//==============================================================================
double DeformableBody::getElasticEnergy() const
{
  return mImpl->getElasticEnergy();
}

//==============================================================================
void DeformableBody::setNodeFixed(std::size_t nodeIndex, bool fixed)
{
  DART_ASSERT(nodeIndex < getNumNodes());
  mImpl->mFixed[nodeIndex] = fixed ? 1 : 0;
  if (fixed) {
    mImpl->mVelocities.segment<3>(static_cast<int>(3 * nodeIndex)).setZero();
  }
}

//==============================================================================
bool DeformableBody::isNodeFixed(std::size_t nodeIndex) const
{
  DART_ASSERT(nodeIndex < getNumNodes());
  return mImpl->mFixed[nodeIndex] != 0;
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
std::size_t DeformableBody::getSurfaceVertexTet(std::size_t vertexIndex) const
{
  DART_ASSERT(vertexIndex < mImpl->mSurfaceTets.size());
  return mImpl->mSurfaceTets[vertexIndex];
}

//==============================================================================
Eigen::Vector4d DeformableBody::getSurfaceVertexWeights(
    std::size_t vertexIndex) const
{
  DART_ASSERT(vertexIndex < mImpl->mSurfaceWeights.size());
  return mImpl->mSurfaceWeights[vertexIndex];
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

  // The body advances from the constraint solver's per-step update, but
  // World::step() returns early without solving at all when automatic
  // deactivation decides the whole world is resting. A deformable body is not
  // part of that resting decision, so the world could sleep while the body
  // still had to move, and the body would silently freeze while simulated time
  // kept advancing. Turning the feature off keeps every step solving.
  //
  // This only affects worlds that opt into a deformable body; worlds without
  // one keep the resting fast path untouched. Detaching deliberately does not
  // restore the previous setting, because another deformable body may still be
  // attached and relying on it; re-enable it explicitly if every body is gone.
  auto deactivation = world->getDeactivationOptions();
  if (deactivation.mEnabled) {
    deactivation.mEnabled = false;
    world->setDeactivationOptions(deactivation);
  }

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
