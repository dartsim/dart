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
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER AND
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

#include <cuda_runtime.h>
#include <dart/simulation/experimental/compute/cuda/vbd_block_descent_cuda.cuh>

#include <stdexcept>
#include <string>

#include <cmath>

namespace dart::simulation::experimental::compute::cuda {
namespace {

constexpr double kMinSpringLength = 1e-12;

//==============================================================================
// Solve the symmetric positive-definite 3x3 system H d = f via the analytic
// cofactor inverse. Returns the zero step if H is not positive-definite.
__device__ inline void solveSym3(
    double h00,
    double h01,
    double h02,
    double h11,
    double h12,
    double h22,
    double fx,
    double fy,
    double fz,
    double& dx,
    double& dy,
    double& dz)
{
  const double c00 = h11 * h22 - h12 * h12;
  const double c01 = h02 * h12 - h01 * h22;
  const double c02 = h01 * h12 - h02 * h11;
  const double det = h00 * c00 + h01 * c01 + h02 * c02;
  if (!(det > 0.0)) {
    dx = 0.0;
    dy = 0.0;
    dz = 0.0;
    return;
  }
  const double c11 = h00 * h22 - h02 * h02;
  const double c12 = h02 * h01 - h00 * h12;
  const double c22 = h00 * h11 - h01 * h01;
  const double inv = 1.0 / det;
  dx = inv * (c00 * fx + c01 * fy + c02 * fz);
  dy = inv * (c01 * fx + c11 * fy + c12 * fz);
  dz = inv * (c02 * fx + c12 * fy + c22 * fz);
}

//==============================================================================
// One VBD block update for every vertex of a single color. Same-color vertices
// share no spring, so all updates here are independent (no data races); a
// vertex reads only its other-colored neighbors, which this kernel never
// writes.
__global__ void vbdColorSweepKernel(
    double* positions,
    const double* inertialTargets,
    const double* masses,
    const std::uint8_t* fixed,
    const std::uint32_t* springA,
    const std::uint32_t* springB,
    const double* springRest,
    const std::uint32_t* incidentOffsets,
    const std::uint32_t* incidentSprings,
    const std::uint32_t* colorVertices,
    std::uint32_t colorBegin,
    std::uint32_t colorEnd,
    double stiffness,
    double invDt2)
{
  const std::uint32_t slot = colorBegin + blockIdx.x * blockDim.x + threadIdx.x;
  if (slot >= colorEnd) {
    return;
  }
  const std::uint32_t v = colorVertices[slot];
  if (fixed[v] != 0u) {
    return;
  }

  const std::uint32_t b = 3u * v;
  const double px = positions[b];
  const double py = positions[b + 1];
  const double pz = positions[b + 2];

  // Inertia term: f = -(m/h^2)(x - y), H = (m/h^2) I.
  const double coeff = masses[v] * invDt2;
  double fx = -coeff * (px - inertialTargets[b]);
  double fy = -coeff * (py - inertialTargets[b + 1]);
  double fz = -coeff * (pz - inertialTargets[b + 2]);
  double h00 = coeff;
  double h11 = coeff;
  double h22 = coeff;
  double h01 = 0.0;
  double h02 = 0.0;
  double h12 = 0.0;

  const std::uint32_t begin = incidentOffsets[v];
  const std::uint32_t end = incidentOffsets[v + 1];
  for (std::uint32_t e = begin; e < end; ++e) {
    const std::uint32_t spring = incidentSprings[e];
    const std::uint32_t a = springA[spring];
    const std::uint32_t other = (a == v) ? springB[spring] : a;
    const std::uint32_t ob = 3u * other;
    const double dxp = positions[ob] - px;
    const double dyp = positions[ob + 1] - py;
    const double dzp = positions[ob + 2] - pz;
    const double length = sqrt(dxp * dxp + dyp * dyp + dzp * dzp);
    if (length <= kMinSpringLength) {
      continue;
    }
    const double invLen = 1.0 / length;
    const double nx = dxp * invLen;
    const double ny = dyp * invLen;
    const double nz = dzp * invLen;
    const double rest = springRest[spring];
    const double stretch = length - rest;

    // Force toward the neighbor when stretched.
    fx += stiffness * stretch * nx;
    fy += stiffness * stretch * ny;
    fz += stiffness * stretch * nz;

    // H += k[ transverse I + (1 - transverse) n n^T ], transverse clamped >= 0.
    double transverse = 1.0 - rest * invLen;
    if (transverse < 0.0) {
      transverse = 0.0;
    }
    const double longitudinal = stiffness * (1.0 - transverse);
    const double diag = stiffness * transverse;
    h00 += diag + longitudinal * nx * nx;
    h11 += diag + longitudinal * ny * ny;
    h22 += diag + longitudinal * nz * nz;
    h01 += longitudinal * nx * ny;
    h02 += longitudinal * nx * nz;
    h12 += longitudinal * ny * nz;
  }

  double dx;
  double dy;
  double dz;
  solveSym3(h00, h01, h02, h11, h12, h22, fx, fy, fz, dx, dy, dz);
  positions[b] = px + dx;
  positions[b + 1] = py + dy;
  positions[b + 2] = pz + dz;
}

//==============================================================================
// Predict the inertial target y = x + h v + h^2 g for every vertex, save the
// step-start position, and warm-start the optimization at y. Fixed vertices
// keep their position.
__global__ void vbdInertialTargetKernel(
    double* positions,
    const double* velocities,
    double* stepStartPositions,
    double* inertialTargets,
    const std::uint8_t* fixed,
    double gx,
    double gy,
    double gz,
    double timeStep,
    std::uint32_t vertexCount)
{
  const std::uint32_t v = blockIdx.x * blockDim.x + threadIdx.x;
  if (v >= vertexCount) {
    return;
  }
  const std::uint32_t b = 3u * v;
  const double px = positions[b];
  const double py = positions[b + 1];
  const double pz = positions[b + 2];
  stepStartPositions[b] = px;
  stepStartPositions[b + 1] = py;
  stepStartPositions[b + 2] = pz;
  if (fixed[v] != 0u) {
    inertialTargets[b] = px;
    inertialTargets[b + 1] = py;
    inertialTargets[b + 2] = pz;
    return;
  }
  const double h2 = timeStep * timeStep;
  const double yx = px + timeStep * velocities[b] + h2 * gx;
  const double yy = py + timeStep * velocities[b + 1] + h2 * gy;
  const double yz = pz + timeStep * velocities[b + 2] + h2 * gz;
  inertialTargets[b] = yx;
  inertialTargets[b + 1] = yy;
  inertialTargets[b + 2] = yz;
  positions[b] = yx;
  positions[b + 1] = yy;
  positions[b + 2] = yz;
}

//==============================================================================
// Backward-Euler velocity update v = (x - x^t) / h. Fixed vertices are held.
__global__ void vbdVelocityUpdateKernel(
    const double* positions,
    double* velocities,
    const double* stepStartPositions,
    const std::uint8_t* fixed,
    double timeStep,
    std::uint32_t vertexCount)
{
  const std::uint32_t v = blockIdx.x * blockDim.x + threadIdx.x;
  if (v >= vertexCount) {
    return;
  }
  const std::uint32_t b = 3u * v;
  if (fixed[v] != 0u) {
    velocities[b] = 0.0;
    velocities[b + 1] = 0.0;
    velocities[b + 2] = 0.0;
    return;
  }
  const double invDt = 1.0 / timeStep;
  velocities[b] = (positions[b] - stepStartPositions[b]) * invDt;
  velocities[b + 1] = (positions[b + 1] - stepStartPositions[b + 1]) * invDt;
  velocities[b + 2] = (positions[b + 2] - stepStartPositions[b + 2]) * invDt;
}

//==============================================================================
void throwIfCudaError(cudaError_t status, const char* operation)
{
  if (status != cudaSuccess) {
    throw std::runtime_error(
        std::string("CUDA VBD ") + operation
        + " failed: " + cudaGetErrorString(status));
  }
}

//==============================================================================
template <typename T>
class DeviceArray
{
public:
  explicit DeviceArray(const std::vector<T>& host) : m_count(host.size())
  {
    if (m_count == 0) {
      return;
    }
    throwIfCudaError(
        cudaMalloc(reinterpret_cast<void**>(&m_data), m_count * sizeof(T)),
        "cudaMalloc");
    throwIfCudaError(
        cudaMemcpy(
            m_data, host.data(), m_count * sizeof(T), cudaMemcpyHostToDevice),
        "cudaMemcpy H2D");
  }

  ~DeviceArray()
  {
    if (m_data != nullptr) {
      (void)cudaFree(m_data);
    }
  }

  DeviceArray(const DeviceArray&) = delete;
  DeviceArray& operator=(const DeviceArray&) = delete;

  [[nodiscard]] T* get() const noexcept
  {
    return m_data;
  }

  void download(std::vector<T>& host) const
  {
    if (m_count == 0) {
      return;
    }
    throwIfCudaError(
        cudaMemcpy(
            host.data(), m_data, m_count * sizeof(T), cudaMemcpyDeviceToHost),
        "cudaMemcpy D2H");
  }

private:
  std::size_t m_count = 0;
  T* m_data = nullptr;
};

} // namespace

//==============================================================================
void vbdStepMassSpringCuda(VbdCudaMassSpringProblem& problem)
{
  const std::size_t nodeCount = problem.nodeCount;
  if (nodeCount == 0 || problem.colorOffsets.size() < 2) {
    return;
  }

  DeviceArray<double> dPositions(problem.positions);
  DeviceArray<double> dInertial(problem.inertialTargets);
  DeviceArray<double> dMasses(problem.masses);
  DeviceArray<std::uint8_t> dFixed(problem.fixed);
  DeviceArray<std::uint32_t> dSpringA(problem.springA);
  DeviceArray<std::uint32_t> dSpringB(problem.springB);
  DeviceArray<double> dSpringRest(problem.springRest);
  DeviceArray<std::uint32_t> dIncidentOffsets(problem.incidentOffsets);
  DeviceArray<std::uint32_t> dIncidentSprings(problem.incidentSprings);
  DeviceArray<std::uint32_t> dColorVertices(problem.colorVertices);

  const double invDt2 = 1.0 / (problem.timeStep * problem.timeStep);
  const std::size_t colorCount = problem.colorOffsets.size() - 1;
  constexpr unsigned int kThreads = 128;

  for (std::size_t iteration = 0; iteration < problem.iterations; ++iteration) {
    for (std::size_t color = 0; color < colorCount; ++color) {
      const std::uint32_t begin = problem.colorOffsets[color];
      const std::uint32_t end = problem.colorOffsets[color + 1];
      if (end <= begin) {
        continue;
      }
      const unsigned int count = end - begin;
      const unsigned int blocks = (count + kThreads - 1) / kThreads;
      vbdColorSweepKernel<<<blocks, kThreads>>>(
          dPositions.get(),
          dInertial.get(),
          dMasses.get(),
          dFixed.get(),
          dSpringA.get(),
          dSpringB.get(),
          dSpringRest.get(),
          dIncidentOffsets.get(),
          dIncidentSprings.get(),
          dColorVertices.get(),
          begin,
          end,
          problem.stiffness,
          invDt2);
    }
  }

  throwIfCudaError(cudaGetLastError(), "kernel launch");
  throwIfCudaError(cudaDeviceSynchronize(), "synchronize");
  dPositions.download(problem.positions);
}

//==============================================================================
void vbdRolloutMassSpringCuda(VbdCudaRolloutProblem& problem)
{
  const std::size_t nodeCount = problem.nodeCount;
  if (nodeCount == 0 || problem.colorOffsets.size() < 2) {
    return;
  }

  DeviceArray<double> dPositions(problem.positions);
  DeviceArray<double> dVelocities(problem.velocities);
  DeviceArray<double> dMasses(problem.masses);
  DeviceArray<std::uint8_t> dFixed(problem.fixed);
  DeviceArray<std::uint32_t> dSpringA(problem.springA);
  DeviceArray<std::uint32_t> dSpringB(problem.springB);
  DeviceArray<double> dSpringRest(problem.springRest);
  DeviceArray<std::uint32_t> dIncidentOffsets(problem.incidentOffsets);
  DeviceArray<std::uint32_t> dIncidentSprings(problem.incidentSprings);
  DeviceArray<std::uint32_t> dColorVertices(problem.colorVertices);

  // Per-step scratch kept resident on the device for the whole rollout.
  const std::vector<double> zeros(3 * nodeCount, 0.0);
  DeviceArray<double> dStepStart(zeros);
  DeviceArray<double> dInertial(zeros);

  const double invDt2 = 1.0 / (problem.timeStep * problem.timeStep);
  const std::size_t colorCount = problem.colorOffsets.size() - 1;
  constexpr unsigned int kThreads = 128;
  const unsigned int vertexBlocks
      = (static_cast<unsigned int>(nodeCount) + kThreads - 1) / kThreads;

  for (std::size_t step = 0; step < problem.stepCount; ++step) {
    vbdInertialTargetKernel<<<vertexBlocks, kThreads>>>(
        dPositions.get(),
        dVelocities.get(),
        dStepStart.get(),
        dInertial.get(),
        dFixed.get(),
        problem.gravity[0],
        problem.gravity[1],
        problem.gravity[2],
        problem.timeStep,
        static_cast<std::uint32_t>(nodeCount));

    for (std::size_t iteration = 0; iteration < problem.iterations;
         ++iteration) {
      for (std::size_t color = 0; color < colorCount; ++color) {
        const std::uint32_t begin = problem.colorOffsets[color];
        const std::uint32_t end = problem.colorOffsets[color + 1];
        if (end <= begin) {
          continue;
        }
        const unsigned int count = end - begin;
        const unsigned int blocks = (count + kThreads - 1) / kThreads;
        vbdColorSweepKernel<<<blocks, kThreads>>>(
            dPositions.get(),
            dInertial.get(),
            dMasses.get(),
            dFixed.get(),
            dSpringA.get(),
            dSpringB.get(),
            dSpringRest.get(),
            dIncidentOffsets.get(),
            dIncidentSprings.get(),
            dColorVertices.get(),
            begin,
            end,
            problem.stiffness,
            invDt2);
      }
    }

    vbdVelocityUpdateKernel<<<vertexBlocks, kThreads>>>(
        dPositions.get(),
        dVelocities.get(),
        dStepStart.get(),
        dFixed.get(),
        problem.timeStep,
        static_cast<std::uint32_t>(nodeCount));
  }

  throwIfCudaError(cudaGetLastError(), "rollout kernel launch");
  throwIfCudaError(cudaDeviceSynchronize(), "rollout synchronize");
  dPositions.download(problem.positions);
  dVelocities.download(problem.velocities);
}

} // namespace dart::simulation::experimental::compute::cuda
