/*
 * Copyright (c) 2011-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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

#ifndef KIDO_UTILS_FILEINFODOF_HPP_
#define KIDO_UTILS_FILEINFODOF_HPP_

#include <vector>

#include <Eigen/Dense>

namespace kido {

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
  bool saveFile(const char* _fileName, size_t _start, size_t _end,
                double _sampleRate = 1.0);

  /// \brief Add Dof
  void addDof(const Eigen::VectorXd& _dofs);

  /// \brief Get Dof
  double getDofAt(size_t _frame, size_t _id) const;

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
  size_t mNumFrames;

  /// \brief File name
  char mFileName[256];

  /// \brief Dof data [frame][dofIndex]
  std::vector<Eigen::VectorXd> mDofs;
};

}  // namespace utils
}  // namespace kido
#endif  // KIDO_UTILS_FILEINFODOF_HPP_
