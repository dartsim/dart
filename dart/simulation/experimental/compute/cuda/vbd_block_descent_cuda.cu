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
// Templated on the scalar type so the same kernels serve double and the faster
// single-precision (mixed-precision) rollout.
template <typename T>
__device__ inline void solveSym3(
    T h00,
    T h01,
    T h02,
    T h11,
    T h12,
    T h22,
    T fx,
    T fy,
    T fz,
    T& dx,
    T& dy,
    T& dz)
{
  const T c00 = h11 * h22 - h12 * h12;
  const T c01 = h02 * h12 - h01 * h22;
  const T c02 = h01 * h12 - h02 * h11;
  const T det = h00 * c00 + h01 * c01 + h02 * c02;
  if (!(det > T(0))) {
    dx = T(0);
    dy = T(0);
    dz = T(0);
    return;
  }
  const T c11 = h00 * h22 - h02 * h02;
  const T c12 = h02 * h01 - h00 * h12;
  const T c22 = h00 * h11 - h01 * h01;
  const T inv = T(1) / det;
  dx = inv * (c00 * fx + c01 * fy + c02 * fz);
  dy = inv * (c01 * fx + c11 * fy + c12 * fz);
  dz = inv * (c02 * fx + c12 * fy + c22 * fz);
}

//==============================================================================
// One VBD block update for every vertex of a single color. Same-color vertices
// share no spring, so all updates here are independent (no data races); a
// vertex reads only its other-colored neighbors, which this kernel never
// writes.
template <typename T>
__global__ void vbdColorSweepKernel(
    T* positions,
    const T* inertialTargets,
    const T* masses,
    const std::uint8_t* fixed,
    const std::uint32_t* springA,
    const std::uint32_t* springB,
    const T* springRest,
    const std::uint32_t* incidentOffsets,
    const std::uint32_t* incidentSprings,
    const std::uint32_t* colorVertices,
    std::uint32_t colorBegin,
    std::uint32_t colorEnd,
    T stiffness,
    T invDt2)
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
  const T px = positions[b];
  const T py = positions[b + 1];
  const T pz = positions[b + 2];

  // Inertia term: f = -(m/h^2)(x - y), H = (m/h^2) I.
  const T coeff = masses[v] * invDt2;
  T fx = -coeff * (px - inertialTargets[b]);
  T fy = -coeff * (py - inertialTargets[b + 1]);
  T fz = -coeff * (pz - inertialTargets[b + 2]);
  T h00 = coeff;
  T h11 = coeff;
  T h22 = coeff;
  T h01 = T(0);
  T h02 = T(0);
  T h12 = T(0);

  const std::uint32_t begin = incidentOffsets[v];
  const std::uint32_t end = incidentOffsets[v + 1];
  for (std::uint32_t e = begin; e < end; ++e) {
    const std::uint32_t spring = incidentSprings[e];
    const std::uint32_t a = springA[spring];
    const std::uint32_t other = (a == v) ? springB[spring] : a;
    const std::uint32_t ob = 3u * other;
    const T dxp = positions[ob] - px;
    const T dyp = positions[ob + 1] - py;
    const T dzp = positions[ob + 2] - pz;
    const T length = sqrt(dxp * dxp + dyp * dyp + dzp * dzp);
    if (length <= T(kMinSpringLength)) {
      continue;
    }
    const T invLen = T(1) / length;
    const T nx = dxp * invLen;
    const T ny = dyp * invLen;
    const T nz = dzp * invLen;
    const T rest = springRest[spring];
    const T stretch = length - rest;

    // Force toward the neighbor when stretched.
    fx += stiffness * stretch * nx;
    fy += stiffness * stretch * ny;
    fz += stiffness * stretch * nz;

    // H += k[ transverse I + (1 - transverse) n n^T ], transverse clamped >= 0.
    T transverse = T(1) - rest * invLen;
    if (transverse < T(0)) {
      transverse = T(0);
    }
    const T longitudinal = stiffness * (T(1) - transverse);
    const T diag = stiffness * transverse;
    h00 += diag + longitudinal * nx * nx;
    h11 += diag + longitudinal * ny * ny;
    h22 += diag + longitudinal * nz * nz;
    h01 += longitudinal * nx * ny;
    h02 += longitudinal * nx * nz;
    h12 += longitudinal * ny * nz;
  }

  T dx;
  T dy;
  T dz;
  solveSym3<T>(h00, h01, h02, h11, h12, h22, fx, fy, fz, dx, dy, dz);
  positions[b] = px + dx;
  positions[b + 1] = py + dy;
  positions[b + 2] = pz + dz;
}

//==============================================================================
// Predict the inertial target y = x + h v + h^2 g for every vertex, save the
// step-start position, and warm-start the optimization at y. Fixed vertices
// keep their position.
template <typename T>
__global__ void vbdInertialTargetKernel(
    T* positions,
    const T* velocities,
    T* stepStartPositions,
    T* inertialTargets,
    const std::uint8_t* fixed,
    T gx,
    T gy,
    T gz,
    T timeStep,
    std::uint32_t vertexCount)
{
  const std::uint32_t v = blockIdx.x * blockDim.x + threadIdx.x;
  if (v >= vertexCount) {
    return;
  }
  const std::uint32_t b = 3u * v;
  const T px = positions[b];
  const T py = positions[b + 1];
  const T pz = positions[b + 2];
  stepStartPositions[b] = px;
  stepStartPositions[b + 1] = py;
  stepStartPositions[b + 2] = pz;
  if (fixed[v] != 0u) {
    inertialTargets[b] = px;
    inertialTargets[b + 1] = py;
    inertialTargets[b + 2] = pz;
    return;
  }
  const T h2 = timeStep * timeStep;
  const T yx = px + timeStep * velocities[b] + h2 * gx;
  const T yy = py + timeStep * velocities[b + 1] + h2 * gy;
  const T yz = pz + timeStep * velocities[b + 2] + h2 * gz;
  inertialTargets[b] = yx;
  inertialTargets[b + 1] = yy;
  inertialTargets[b + 2] = yz;
  positions[b] = yx;
  positions[b + 1] = yy;
  positions[b + 2] = yz;
}

//==============================================================================
// Backward-Euler velocity update v = (x - x^t) / h. Fixed vertices are held.
template <typename T>
__global__ void vbdVelocityUpdateKernel(
    const T* positions,
    T* velocities,
    const T* stepStartPositions,
    const std::uint8_t* fixed,
    T timeStep,
    std::uint32_t vertexCount)
{
  const std::uint32_t v = blockIdx.x * blockDim.x + threadIdx.x;
  if (v >= vertexCount) {
    return;
  }
  const std::uint32_t b = 3u * v;
  if (fixed[v] != 0u) {
    velocities[b] = T(0);
    velocities[b + 1] = T(0);
    velocities[b + 2] = T(0);
    return;
  }
  const T invDt = T(1) / timeStep;
  velocities[b] = (positions[b] - stepStartPositions[b]) * invDt;
  velocities[b + 1] = (positions[b + 1] - stepStartPositions[b + 1]) * invDt;
  velocities[b + 2] = (positions[b + 2] - stepStartPositions[b + 2]) * invDt;
}

//==============================================================================
// Minimal device 3-vector with the operations the Neo-Hookean tet term needs,
// templated on the scalar type so the tet kernels serve double and the faster
// single-precision (mixed-precision) rollout.
template <typename T>
struct DVec3
{
  T x, y, z;
};
template <typename T>
__device__ inline DVec3<T> operator+(DVec3<T> a, DVec3<T> b)
{
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}
template <typename T>
__device__ inline DVec3<T> operator*(T s, DVec3<T> a)
{
  return {s * a.x, s * a.y, s * a.z};
}
template <typename T>
__device__ inline T dot3(DVec3<T> a, DVec3<T> b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
template <typename T>
__device__ inline DVec3<T> cross3(DVec3<T> a, DVec3<T> b)
{
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

//==============================================================================
// Accumulate one tetrahedron's Stable Neo-Hookean force and (full 3x3) Hessian
// for local vertex `localVertex` into `f`/`H`. A device port of the analytic
// CPU kernel (addNeoHookeanTetTerm); `dmInv` is the tet's Dm^{-1} row-major.
template <typename T>
__device__ inline void accumulateNeoHookeanTet(
    int localVertex,
    const T* dmInv,
    T volume,
    DVec3<T> x0,
    DVec3<T> x1,
    DVec3<T> x2,
    DVec3<T> x3,
    T mu,
    T lambda,
    T* f,
    T* H)
{
  const DVec3<T> g1 = {dmInv[0], dmInv[1], dmInv[2]};
  const DVec3<T> g2 = {dmInv[3], dmInv[4], dmInv[5]};
  const DVec3<T> g3 = {dmInv[6], dmInv[7], dmInv[8]};
  const DVec3<T> g0
      = {-(g1.x + g2.x + g3.x), -(g1.y + g2.y + g3.y), -(g1.z + g2.z + g3.z)};
  const DVec3<T> g[4] = {g0, g1, g2, g3};
  const DVec3<T> X[4] = {x0, x1, x2, x3};

  // F columns f_b = sum_i g[i].component(b) * X[i].
  DVec3<T> c0 = {T(0), T(0), T(0)};
  DVec3<T> c1 = {T(0), T(0), T(0)};
  DVec3<T> c2 = {T(0), T(0), T(0)};
  for (int i = 0; i < 4; ++i) {
    c0 = c0 + g[i].x * X[i];
    c1 = c1 + g[i].y * X[i];
    c2 = c2 + g[i].z * X[i];
  }
  const DVec3<T> cof0 = cross3(c1, c2);
  const DVec3<T> cof1 = cross3(c2, c0);
  const DVec3<T> cof2 = cross3(c0, c1);
  const T j = dot3(c0, cof0);
  const T a = T(1) + mu / lambda;
  const T jlambda = lambda * (j - a);

  const DVec3<T> p0 = mu * c0 + jlambda * cof0;
  const DVec3<T> p1 = mu * c1 + jlambda * cof1;
  const DVec3<T> p2 = mu * c2 + jlambda * cof2;
  const DVec3<T> gi = g[localVertex];
  const DVec3<T> pgi = gi.x * p0 + gi.y * p1 + gi.z * p2;
  f[0] += -volume * pgi.x;
  f[1] += -volume * pgi.y;
  f[2] += -volume * pgi.z;

  for (int d = 0; d < 3; ++d) {
    const DVec3<T> ed
        = {d == 0 ? T(1) : T(0), d == 1 ? T(1) : T(0), d == 2 ? T(1) : T(0)};
    const DVec3<T> dc0 = gi.x * ed;
    const DVec3<T> dc1 = gi.y * ed;
    const DVec3<T> dc2 = gi.z * ed;
    const T dj = dot3(cof0, dc0) + dot3(cof1, dc1) + dot3(cof2, dc2);
    const DVec3<T> dcof0 = cross3(dc1, c2) + cross3(c1, dc2);
    const DVec3<T> dcof1 = cross3(dc2, c0) + cross3(c2, dc0);
    const DVec3<T> dcof2 = cross3(dc0, c1) + cross3(c0, dc1);
    const DVec3<T> dp0 = mu * dc0 + (lambda * dj) * cof0 + jlambda * dcof0;
    const DVec3<T> dp1 = mu * dc1 + (lambda * dj) * cof1 + jlambda * dcof1;
    const DVec3<T> dp2 = mu * dc2 + (lambda * dj) * cof2 + jlambda * dcof2;
    const DVec3<T> col = volume * (gi.x * dp0 + gi.y * dp1 + gi.z * dp2);
    H[0 * 3 + d] += col.x;
    H[1 * 3 + d] += col.y;
    H[2 * 3 + d] += col.z;
  }
}

//==============================================================================
// One Neo-Hookean tet VBD block update for every vertex of a single color.
template <typename T>
__global__ void vbdTetColorSweepKernel(
    T* positions,
    const T* inertialTargets,
    const T* masses,
    const std::uint8_t* fixed,
    const std::uint32_t* tetVertices,
    const T* tetRestShapeInverse,
    const T* tetRestVolume,
    const std::uint32_t* incidentTetOffsets,
    const std::uint32_t* incidentTetIndex,
    const std::uint8_t* incidentLocalVertex,
    const std::uint32_t* colorVertices,
    std::uint32_t colorBegin,
    std::uint32_t colorEnd,
    T mu,
    T lambda,
    T invDt2)
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
  const T px = positions[b];
  const T py = positions[b + 1];
  const T pz = positions[b + 2];

  const T coeff = masses[v] * invDt2;
  T f[3]
      = {-coeff * (px - inertialTargets[b]),
         -coeff * (py - inertialTargets[b + 1]),
         -coeff * (pz - inertialTargets[b + 2])};
  T H[9] = {coeff, T(0), T(0), T(0), coeff, T(0), T(0), T(0), coeff};

  const std::uint32_t begin = incidentTetOffsets[v];
  const std::uint32_t end = incidentTetOffsets[v + 1];
  for (std::uint32_t e = begin; e < end; ++e) {
    const std::uint32_t tet = incidentTetIndex[e];
    const int local = static_cast<int>(incidentLocalVertex[e]);
    const std::uint32_t* tv = tetVertices + 4u * tet;
    const auto loadVertex = [positions](std::uint32_t vertex) -> DVec3<T> {
      const std::uint32_t base = 3u * vertex;
      return {positions[base], positions[base + 1], positions[base + 2]};
    };
    accumulateNeoHookeanTet<T>(
        local,
        tetRestShapeInverse + 9u * tet,
        tetRestVolume[tet],
        loadVertex(tv[0]),
        loadVertex(tv[1]),
        loadVertex(tv[2]),
        loadVertex(tv[3]),
        mu,
        lambda,
        f,
        H);
  }

  // Symmetrize and solve.
  const T h01 = T(0.5) * (H[1] + H[3]);
  const T h02 = T(0.5) * (H[2] + H[6]);
  const T h12 = T(0.5) * (H[5] + H[7]);
  T dx;
  T dy;
  T dz;
  solveSym3<T>(H[0], h01, h02, H[4], h12, H[8], f[0], f[1], f[2], dx, dy, dz);
  positions[b] = px + dx;
  positions[b + 1] = py + dy;
  positions[b + 2] = pz + dz;
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

//==============================================================================
// Drive a device-resident rollout on `stream`. `recordStep` issues exactly one
// implicit-Euler step's kernel launches onto the stream. With `useCudaGraph`,
// one step is captured into a CUDA graph and replayed for every step (the
// per-step launch shape is identical, so a single graph replays exactly),
// amortizing per-launch overhead; otherwise the steps are launched directly.
template <typename RecordStep>
void runDeviceRollout(
    cudaStream_t stream,
    RecordStep&& recordStep,
    std::size_t stepCount,
    bool useCudaGraph)
{
  if (useCudaGraph) {
    cudaGraph_t graph = nullptr;
    cudaGraphExec_t graphExec = nullptr;
    throwIfCudaError(
        cudaStreamBeginCapture(stream, cudaStreamCaptureModeGlobal),
        "begin capture");
    recordStep();
    throwIfCudaError(cudaStreamEndCapture(stream, &graph), "end capture");
    throwIfCudaError(
        cudaGraphInstantiate(&graphExec, graph, 0), "graph instantiate");
    for (std::size_t step = 0; step < stepCount; ++step) {
      throwIfCudaError(cudaGraphLaunch(graphExec, stream), "graph launch");
    }
    throwIfCudaError(cudaStreamSynchronize(stream), "graph synchronize");
    (void)cudaGraphExecDestroy(graphExec);
    (void)cudaGraphDestroy(graph);
    return;
  }

  for (std::size_t step = 0; step < stepCount; ++step) {
    recordStep();
  }
  throwIfCudaError(cudaGetLastError(), "rollout kernel launch");
  throwIfCudaError(cudaStreamSynchronize(stream), "rollout synchronize");
}

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
      vbdColorSweepKernel<double><<<blocks, kThreads>>>(
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

namespace {

// Templated mass-spring rollout: device state and arithmetic use scalar type T.
// Host data (always double on the problem) is converted to T on upload and back
// to double on download, so T=float gives the mixed-precision rollout while
// T=double reproduces the full-precision path exactly.
template <typename T>
void rolloutMassSpringImpl(VbdCudaRolloutProblem& problem)
{
  const std::size_t nodeCount = problem.nodeCount;

  std::vector<T> hPositions(problem.positions.begin(), problem.positions.end());
  std::vector<T> hVelocities(
      problem.velocities.begin(), problem.velocities.end());
  const std::vector<T> hMasses(problem.masses.begin(), problem.masses.end());
  const std::vector<T> hSpringRest(
      problem.springRest.begin(), problem.springRest.end());

  DeviceArray<T> dPositions(hPositions);
  DeviceArray<T> dVelocities(hVelocities);
  DeviceArray<T> dMasses(hMasses);
  DeviceArray<std::uint8_t> dFixed(problem.fixed);
  DeviceArray<std::uint32_t> dSpringA(problem.springA);
  DeviceArray<std::uint32_t> dSpringB(problem.springB);
  DeviceArray<T> dSpringRest(hSpringRest);
  DeviceArray<std::uint32_t> dIncidentOffsets(problem.incidentOffsets);
  DeviceArray<std::uint32_t> dIncidentSprings(problem.incidentSprings);
  DeviceArray<std::uint32_t> dColorVertices(problem.colorVertices);

  // Per-step scratch kept resident on the device for the whole rollout.
  const std::vector<T> zeros(3 * nodeCount, T(0));
  DeviceArray<T> dStepStart(zeros);
  DeviceArray<T> dInertial(zeros);

  const T invDt2 = T(1) / (T(problem.timeStep) * T(problem.timeStep));
  const T timeStep = T(problem.timeStep);
  const T stiffness = T(problem.stiffness);
  const T gx = T(problem.gravity[0]);
  const T gy = T(problem.gravity[1]);
  const T gz = T(problem.gravity[2]);
  const std::size_t colorCount = problem.colorOffsets.size() - 1;
  constexpr unsigned int kThreads = 128;
  const unsigned int vertexBlocks
      = (static_cast<unsigned int>(nodeCount) + kThreads - 1) / kThreads;

  cudaStream_t stream = nullptr;
  throwIfCudaError(cudaStreamCreate(&stream), "stream create");

  const auto recordStep = [&]() {
    vbdInertialTargetKernel<T><<<vertexBlocks, kThreads, 0, stream>>>(
        dPositions.get(),
        dVelocities.get(),
        dStepStart.get(),
        dInertial.get(),
        dFixed.get(),
        gx,
        gy,
        gz,
        timeStep,
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
        vbdColorSweepKernel<T><<<blocks, kThreads, 0, stream>>>(
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
            stiffness,
            invDt2);
      }
    }

    vbdVelocityUpdateKernel<T><<<vertexBlocks, kThreads, 0, stream>>>(
        dPositions.get(),
        dVelocities.get(),
        dStepStart.get(),
        dFixed.get(),
        timeStep,
        static_cast<std::uint32_t>(nodeCount));
  };

  runDeviceRollout(stream, recordStep, problem.stepCount, problem.useCudaGraph);

  (void)cudaStreamDestroy(stream);
  dPositions.download(hPositions);
  dVelocities.download(hVelocities);
  problem.positions.assign(hPositions.begin(), hPositions.end());
  problem.velocities.assign(hVelocities.begin(), hVelocities.end());
}

} // namespace

//==============================================================================
void vbdRolloutMassSpringCuda(VbdCudaRolloutProblem& problem)
{
  if (problem.nodeCount == 0 || problem.colorOffsets.size() < 2) {
    return;
  }
  if (problem.useSinglePrecision) {
    rolloutMassSpringImpl<float>(problem);
  } else {
    rolloutMassSpringImpl<double>(problem);
  }
}

//==============================================================================
void vbdStepTetMeshCuda(VbdCudaTetProblem& problem)
{
  const std::size_t nodeCount = problem.nodeCount;
  if (nodeCount == 0 || problem.colorOffsets.size() < 2) {
    return;
  }

  DeviceArray<double> dPositions(problem.positions);
  DeviceArray<double> dInertial(problem.inertialTargets);
  DeviceArray<double> dMasses(problem.masses);
  DeviceArray<std::uint8_t> dFixed(problem.fixed);
  DeviceArray<std::uint32_t> dTetVertices(problem.tetVertices);
  DeviceArray<double> dTetRestShapeInverse(problem.tetRestShapeInverse);
  DeviceArray<double> dTetRestVolume(problem.tetRestVolume);
  DeviceArray<std::uint32_t> dIncidentTetOffsets(problem.incidentTetOffsets);
  DeviceArray<std::uint32_t> dIncidentTetIndex(problem.incidentTetIndex);
  DeviceArray<std::uint8_t> dIncidentLocalVertex(problem.incidentLocalVertex);
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
      vbdTetColorSweepKernel<double><<<blocks, kThreads>>>(
          dPositions.get(),
          dInertial.get(),
          dMasses.get(),
          dFixed.get(),
          dTetVertices.get(),
          dTetRestShapeInverse.get(),
          dTetRestVolume.get(),
          dIncidentTetOffsets.get(),
          dIncidentTetIndex.get(),
          dIncidentLocalVertex.get(),
          dColorVertices.get(),
          begin,
          end,
          problem.mu,
          problem.lambda,
          invDt2);
    }
  }

  throwIfCudaError(cudaGetLastError(), "tet kernel launch");
  throwIfCudaError(cudaDeviceSynchronize(), "tet synchronize");
  dPositions.download(problem.positions);
}

namespace {

// Templated tetrahedral rollout: device state and arithmetic use scalar type T.
// Host data (always double) is converted to T on upload and back on download,
// so T=float gives the mixed-precision tet rollout while T=double reproduces
// the full-precision path exactly.
template <typename T>
void rolloutTetMeshImpl(VbdCudaTetRolloutProblem& problem)
{
  const std::size_t nodeCount = problem.nodeCount;

  std::vector<T> hPositions(problem.positions.begin(), problem.positions.end());
  std::vector<T> hVelocities(
      problem.velocities.begin(), problem.velocities.end());
  const std::vector<T> hMasses(problem.masses.begin(), problem.masses.end());
  const std::vector<T> hRestShapeInverse(
      problem.tetRestShapeInverse.begin(), problem.tetRestShapeInverse.end());
  const std::vector<T> hRestVolume(
      problem.tetRestVolume.begin(), problem.tetRestVolume.end());

  DeviceArray<T> dPositions(hPositions);
  DeviceArray<T> dVelocities(hVelocities);
  DeviceArray<T> dMasses(hMasses);
  DeviceArray<std::uint8_t> dFixed(problem.fixed);
  DeviceArray<std::uint32_t> dTetVertices(problem.tetVertices);
  DeviceArray<T> dTetRestShapeInverse(hRestShapeInverse);
  DeviceArray<T> dTetRestVolume(hRestVolume);
  DeviceArray<std::uint32_t> dIncidentTetOffsets(problem.incidentTetOffsets);
  DeviceArray<std::uint32_t> dIncidentTetIndex(problem.incidentTetIndex);
  DeviceArray<std::uint8_t> dIncidentLocalVertex(problem.incidentLocalVertex);
  DeviceArray<std::uint32_t> dColorVertices(problem.colorVertices);

  // Per-step scratch kept resident on the device for the whole rollout.
  const std::vector<T> zeros(3 * nodeCount, T(0));
  DeviceArray<T> dStepStart(zeros);
  DeviceArray<T> dInertial(zeros);

  const T invDt2 = T(1) / (T(problem.timeStep) * T(problem.timeStep));
  const T timeStep = T(problem.timeStep);
  const T mu = T(problem.mu);
  const T lambda = T(problem.lambda);
  const T gx = T(problem.gravity[0]);
  const T gy = T(problem.gravity[1]);
  const T gz = T(problem.gravity[2]);
  const std::size_t colorCount = problem.colorOffsets.size() - 1;
  constexpr unsigned int kThreads = 128;
  const unsigned int vertexBlocks
      = (static_cast<unsigned int>(nodeCount) + kThreads - 1) / kThreads;

  cudaStream_t stream = nullptr;
  throwIfCudaError(cudaStreamCreate(&stream), "stream create");

  const auto recordStep = [&]() {
    vbdInertialTargetKernel<T><<<vertexBlocks, kThreads, 0, stream>>>(
        dPositions.get(),
        dVelocities.get(),
        dStepStart.get(),
        dInertial.get(),
        dFixed.get(),
        gx,
        gy,
        gz,
        timeStep,
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
        vbdTetColorSweepKernel<T><<<blocks, kThreads, 0, stream>>>(
            dPositions.get(),
            dInertial.get(),
            dMasses.get(),
            dFixed.get(),
            dTetVertices.get(),
            dTetRestShapeInverse.get(),
            dTetRestVolume.get(),
            dIncidentTetOffsets.get(),
            dIncidentTetIndex.get(),
            dIncidentLocalVertex.get(),
            dColorVertices.get(),
            begin,
            end,
            mu,
            lambda,
            invDt2);
      }
    }

    vbdVelocityUpdateKernel<T><<<vertexBlocks, kThreads, 0, stream>>>(
        dPositions.get(),
        dVelocities.get(),
        dStepStart.get(),
        dFixed.get(),
        timeStep,
        static_cast<std::uint32_t>(nodeCount));
  };

  runDeviceRollout(stream, recordStep, problem.stepCount, problem.useCudaGraph);

  (void)cudaStreamDestroy(stream);
  dPositions.download(hPositions);
  dVelocities.download(hVelocities);
  problem.positions.assign(hPositions.begin(), hPositions.end());
  problem.velocities.assign(hVelocities.begin(), hVelocities.end());
}

} // namespace

//==============================================================================
void vbdRolloutTetMeshCuda(VbdCudaTetRolloutProblem& problem)
{
  if (problem.nodeCount == 0 || problem.colorOffsets.size() < 2) {
    return;
  }
  if (problem.useSinglePrecision) {
    rolloutTetMeshImpl<float>(problem);
  } else {
    rolloutTetMeshImpl<double>(problem);
  }
}

} // namespace dart::simulation::experimental::compute::cuda
