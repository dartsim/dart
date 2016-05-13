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

#include "dart/utils/FileInfoDof.hpp"

#include <fstream>
#include <string>

#include "dart/dynamics/DegreeOfFreedom.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/Joint.hpp"
#include "dart/simulation/Recording.hpp"

namespace dart {
namespace utils {

//==============================================================================
FileInfoDof::FileInfoDof(dynamics::Skeleton* _skel, double _fps)
  : mSkel(_skel), mFPS(_fps), mNumFrames(0)
{
  std::strcpy(mFileName, "");
}

//==============================================================================
FileInfoDof::~FileInfoDof()
{
  mDofs.clear();
  mNumFrames = 0;
}

//==============================================================================
bool FileInfoDof::loadFile(const char* _fName)
{
  std::ifstream inFile(_fName);
  if (inFile.fail() == 1) return false;

  inFile.precision(20);
  char buffer[256];
  std::size_t nDof;

  // nFrames =
  inFile >> buffer;
  inFile >> buffer;
  inFile >> mNumFrames;

  // nDof =
  inFile >> buffer;
  inFile >> buffer;
  inFile >> nDof;

  if (mSkel == nullptr || mSkel->getNumDofs() != nDof)
    return false;

  mDofs.resize(mNumFrames);

  // dof names
  for (std::size_t i = 0; i < nDof; i++)
    inFile >> buffer;
  for (std::size_t j = 0; j < mNumFrames; j++)
  {
    mDofs[j].resize(nDof);
    for (std::size_t i = 0; i < nDof; i++)
    {
      double val;
      inFile >> val;
      mDofs[j][i] = val;
    }
  }

  // fps
  inFile >> buffer;
  if (!inFile.eof())
    inFile>>mFPS;

  inFile.close();

  std::string text = _fName;
  int lastSlash = text.find_last_of("/");
  text = text.substr(lastSlash+1);
  strcpy(mFileName, text.c_str());
  return true;
}

//==============================================================================
bool FileInfoDof::saveFile(const char* _fName, std::size_t _start, std::size_t _end,
                           double /*_sampleRate*/ )
{
  if (_end < _start) return false;

  std::ofstream outFile(_fName, std::ios::out);
  if (outFile.fail()) return false;

  std::size_t first = _start < mNumFrames ? _start : mNumFrames - 1;
  std::size_t last = _end < mNumFrames ? _end : mNumFrames - 1;

  outFile.precision(20);
  outFile << "frames = " << last-first+1 << " dofs = " << mSkel->getNumDofs() << std::endl;

  for (std::size_t i = 0; i < mSkel->getNumDofs(); i++)
  {
    const dynamics::DegreeOfFreedom* dof        = mSkel->getDof(i);
    const dynamics::Joint*           joint      = dof->getJoint();
    const std::size_t                     localIndex = dof->getIndexInJoint();

    outFile << joint->getName() << "." << localIndex << ' ';
  }

  outFile << std::endl;

  for (std::size_t i = first; i <= last; i++)
  {
    for (std::size_t j = 0; j < mSkel->getNumDofs(); j++)
      outFile << mDofs[i][j] << ' ';
    outFile << std::endl;
  }

  outFile << "FPS " << mFPS << std::endl;

  outFile.close();

  std::string text = _fName;
  std::size_t lastSlash = text.find_last_of("/");
  text = text.substr(lastSlash + 1);
  std::strcpy(mFileName, text.c_str());
  return true;
}

//==============================================================================
void FileInfoDof::addDof(const Eigen::VectorXd& _dofs)
{
  mDofs.push_back(_dofs); mNumFrames++;
}

//==============================================================================
double FileInfoDof::getDofAt(std::size_t _frame, std::size_t _id) const
{
  assert(_frame<mNumFrames); return mDofs.at(_frame)[_id];
}

//==============================================================================
Eigen::VectorXd FileInfoDof::getPoseAtFrame(int _frame) const
{
  return mDofs.at(_frame);
}

//==============================================================================
void FileInfoDof::setFPS(double _fps)
{
  mFPS = _fps;
}

//==============================================================================
double FileInfoDof::getFPS() const
{
  return mFPS;
}

//==============================================================================
int FileInfoDof::getNumFrames() const
{
  return mNumFrames;
}

//==============================================================================
dynamics::Skeleton*FileInfoDof::getSkel() const
{
  return mSkel;
}

}  // namespace utils
}  // namespace dart
