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

#ifndef DART_PYTHON_NATIVE_DEMO_MODELS_POINTCLOUDSENSOR_HPP_
#define DART_PYTHON_NATIVE_DEMO_MODELS_POINTCLOUDSENSOR_HPP_

#include <dart/config.hpp>

#if HAVE_OCTOMAP

  #include <dart/dynamics/PointCloudShape.hpp>
  #include <dart/dynamics/SimpleFrame.hpp>
  #include <dart/dynamics/Skeleton.hpp>
  #include <dart/dynamics/VoxelGridShape.hpp>

  #include <memory>

namespace dart_demos {

/// The native point-cloud scene's mesh sampling and occupancy update.
class PointCloudSensor
{
public:
  enum class SamplingMode
  {
    SampleOnRobot,
    SampleInBox
  };

  PointCloudSensor(
      dart::dynamics::SkeletonPtr robot,
      dart::dynamics::SimpleFramePtr sensorFrame,
      std::shared_ptr<dart::dynamics::PointCloudShape> pointCloud,
      std::shared_ptr<dart::dynamics::VoxelGridShape> voxelGrid);

  void update(double timeStep);

  const dart::dynamics::SkeletonPtr& getRobot() const;
  const dart::dynamics::SimpleFramePtr& getSensorFrame() const;
  const std::shared_ptr<dart::dynamics::PointCloudShape>& getPointCloud() const;
  const std::shared_ptr<dart::dynamics::VoxelGridShape>& getVoxelGrid() const;

  bool updating = true;
  SamplingMode samplingMode = SamplingMode::SampleOnRobot;

private:
  dart::dynamics::SkeletonPtr mRobot;
  dart::dynamics::SimpleFramePtr mSensorFrame;
  std::shared_ptr<dart::dynamics::PointCloudShape> mPointCloud;
  std::shared_ptr<dart::dynamics::VoxelGridShape> mVoxelGrid;
  double mOrbitTime = 0.0;
};

} // namespace dart_demos

#endif // HAVE_OCTOMAP

#endif // DART_PYTHON_NATIVE_DEMO_MODELS_POINTCLOUDSENSOR_HPP_
