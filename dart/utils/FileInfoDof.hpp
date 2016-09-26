/*
 * Copyright (c) 2011-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2011-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#ifndef DART_UTILS_FILEINFODOF_HPP_
#define DART_UTILS_FILEINFODOF_HPP_

#include <vector>

#include <Eigen/Dense>

namespace dart {

namespace dynamics {
class Skeleton;
}  // namespace dynamics

namespace utils {

/// \brief class FileInfoDof
class FileInfoDof
{
public:
  /// \brief Constructor
  FileInfoDof(dynamics::Skeleton* _skel, double _fps = 120.0);

  /// \brief Destructor
  virtual ~FileInfoDof();

  /// \brief Load file
  bool loadFile(const char* _fileName);

  /// \brief Save file
  /// \note Down sampling not implemented yet
  bool saveFile(const char* _fileName, std::size_t _start, std::size_t _end,
                double _sampleRate = 1.0);

  /// \brief Add Dof
  void addDof(const Eigen::VectorXd& _dofs);

  /// \brief Get Dof
  double getDofAt(std::size_t _frame, std::size_t _id) const;

  /// \brief Get pose at frame
  Eigen::VectorXd getPoseAtFrame(int _frame) const;

  /// \brief Set frames per second
  void setFPS(double _fps);

  /// \brief Get frames per second
  double getFPS() const;

  /// \brief Get number of frames
  int getNumFrames() const;

  /// \brief Get skeleton associated with
  dynamics::Skeleton* getSkel() const;

protected:
  /// \brief Model associated with
  dynamics::Skeleton* mSkel;

  /// \brief Frame rate
  double mFPS;

  /// \brief Number of frames
  std::size_t mNumFrames;

  /// \brief File name
  char mFileName[256];

  /// \brief Dof data [frame][dofIndex]
  std::vector<Eigen::VectorXd> mDofs;
};

}  // namespace utils
}  // namespace dart
#endif  // DART_UTILS_FILEINFODOF_HPP_
