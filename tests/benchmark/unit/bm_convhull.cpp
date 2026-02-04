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

#include <dart/math/detail/convhull.hpp>

#include <Eigen/Geometry>
#include <benchmark/benchmark.h>

#include <span>
#include <random>

using dart::math::detail::convexHull3dBuild;

//==============================================================================
// Helper to generate random points
//==============================================================================
template <typename S>
std::vector<Eigen::Matrix<S, 3, 1>> generateRandomPoints(
    int numPoints, int seed)
{
  std::mt19937 gen(seed);
  std::uniform_real_distribution<double> dist(-1.0, 1.0);

  std::vector<Eigen::Matrix<S, 3, 1>> vertices(numPoints);
  for (int i = 0; i < numPoints; ++i) {
    vertices[i] = Eigen::Matrix<S, 3, 1>(
        static_cast<S>(dist(gen)),
        static_cast<S>(dist(gen)),
        static_cast<S>(dist(gen)));
  }
  return vertices;
}

template <typename T>
std::span<const T> asSpan(const std::vector<T>& values)
{
  return std::span<const T>(values);
}

//==============================================================================
// Benchmark: Cube (8 points) - Float
//==============================================================================
static void BM_ConvexHull_Cube_Float(benchmark::State& state)
{
  std::vector<Eigen::Vector3f> vertices(8);
  vertices[0] = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  vertices[1] = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
  vertices[2] = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
  vertices[3] = Eigen::Vector3f(1.0f, 1.0f, 0.0f);
  vertices[4] = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
  vertices[5] = Eigen::Vector3f(1.0f, 0.0f, 1.0f);
  vertices[6] = Eigen::Vector3f(0.0f, 1.0f, 1.0f);
  vertices[7] = Eigen::Vector3f(1.0f, 1.0f, 1.0f);

  for (auto _ : state) {
    std::vector<int> faces;
    int numFaces = 0;
    convexHull3dBuild(asSpan(vertices), faces, numFaces);
    benchmark::DoNotOptimize(faces.data());
    benchmark::DoNotOptimize(numFaces);
  }
}
BENCHMARK(BM_ConvexHull_Cube_Float);

//==============================================================================
// Benchmark: Cube (8 points) - Double
//==============================================================================
static void BM_ConvexHull_Cube_Double(benchmark::State& state)
{
  std::vector<Eigen::Vector3d> vertices(8);
  vertices[0] = Eigen::Vector3d(0, 0, 0);
  vertices[1] = Eigen::Vector3d(1, 0, 0);
  vertices[2] = Eigen::Vector3d(0, 1, 0);
  vertices[3] = Eigen::Vector3d(1, 1, 0);
  vertices[4] = Eigen::Vector3d(0, 0, 1);
  vertices[5] = Eigen::Vector3d(1, 0, 1);
  vertices[6] = Eigen::Vector3d(0, 1, 1);
  vertices[7] = Eigen::Vector3d(1, 1, 1);

  for (auto _ : state) {
    std::vector<int> faces;
    int numFaces = 0;
    convexHull3dBuild(asSpan(vertices), faces, numFaces);
    benchmark::DoNotOptimize(faces.data());
    benchmark::DoNotOptimize(numFaces);
  }
}
BENCHMARK(BM_ConvexHull_Cube_Double);

//==============================================================================
// Benchmark: Random50 (50 random points) - Float
//==============================================================================
static void BM_ConvexHull_Random50_Float(benchmark::State& state)
{
  auto vertices = generateRandomPoints<float>(50, 12345);

  for (auto _ : state) {
    std::vector<int> faces;
    int numFaces = 0;
    convexHull3dBuild(asSpan(vertices), faces, numFaces);
    benchmark::DoNotOptimize(faces.data());
    benchmark::DoNotOptimize(numFaces);
  }
}
BENCHMARK(BM_ConvexHull_Random50_Float);

//==============================================================================
// Benchmark: Random50 (50 random points) - Double
//==============================================================================
static void BM_ConvexHull_Random50_Double(benchmark::State& state)
{
  auto vertices = generateRandomPoints<double>(50, 12345);

  for (auto _ : state) {
    std::vector<int> faces;
    int numFaces = 0;
    convexHull3dBuild(asSpan(vertices), faces, numFaces);
    benchmark::DoNotOptimize(faces.data());
    benchmark::DoNotOptimize(numFaces);
  }
}
BENCHMARK(BM_ConvexHull_Random50_Double);

//==============================================================================
// Benchmark: Random200 (200 random points) - Float
//==============================================================================
static void BM_ConvexHull_Random200_Float(benchmark::State& state)
{
  auto vertices = generateRandomPoints<float>(200, 54321);

  for (auto _ : state) {
    std::vector<int> faces;
    int numFaces = 0;
    convexHull3dBuild(asSpan(vertices), faces, numFaces);
    benchmark::DoNotOptimize(faces.data());
    benchmark::DoNotOptimize(numFaces);
  }
}
BENCHMARK(BM_ConvexHull_Random200_Float);

//==============================================================================
// Benchmark: Random200 (200 random points) - Double
//==============================================================================
static void BM_ConvexHull_Random200_Double(benchmark::State& state)
{
  auto vertices = generateRandomPoints<double>(200, 54321);

  for (auto _ : state) {
    std::vector<int> faces;
    int numFaces = 0;
    convexHull3dBuild(asSpan(vertices), faces, numFaces);
    benchmark::DoNotOptimize(faces.data());
    benchmark::DoNotOptimize(numFaces);
  }
}
BENCHMARK(BM_ConvexHull_Random200_Double);

//==============================================================================
// Benchmark: Random1000 (1000 random points) - Float
//==============================================================================
static void BM_ConvexHull_Random1000_Float(benchmark::State& state)
{
  auto vertices = generateRandomPoints<float>(1000, 99999);

  for (auto _ : state) {
    std::vector<int> faces;
    int numFaces = 0;
    convexHull3dBuild(asSpan(vertices), faces, numFaces);
    benchmark::DoNotOptimize(faces.data());
    benchmark::DoNotOptimize(numFaces);
  }
}
BENCHMARK(BM_ConvexHull_Random1000_Float);

//==============================================================================
// Benchmark: Random1000 (1000 random points) - Double
//==============================================================================
static void BM_ConvexHull_Random1000_Double(benchmark::State& state)
{
  auto vertices = generateRandomPoints<double>(1000, 99999);

  for (auto _ : state) {
    std::vector<int> faces;
    int numFaces = 0;
    convexHull3dBuild(asSpan(vertices), faces, numFaces);
    benchmark::DoNotOptimize(faces.data());
    benchmark::DoNotOptimize(numFaces);
  }
}
BENCHMARK(BM_ConvexHull_Random1000_Double);

BENCHMARK_MAIN();
