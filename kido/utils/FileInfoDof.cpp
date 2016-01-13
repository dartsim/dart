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

#include "kido/utils/FileInfoDof.hpp"

#include <fstream>
#include <string>

#include "kido/dynamics/DegreeOfFreedom.hpp"
#include "kido/dynamics/Skeleton.hpp"
#include "kido/dynamics/Joint.hpp"
#include "kido/simulation/Recording.hpp"

namespace kido {
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
  size_t nDof;

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
  for (size_t i = 0; i < nDof; i++)
    inFile >> buffer;
  for (size_t j = 0; j < mNumFrames; j++)
  {
    mDofs[j].resize(nDof);
    for (size_t i = 0; i < nDof; i++)
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
bool FileInfoDof::saveFile(const char* _fName, size_t _start, size_t _end,
                           double _sampleRate )
{
  if (_end < _start) return false;

  std::ofstream outFile(_fName, std::ios::out);
  if (outFile.fail()) return false;

  size_t first = _start < mNumFrames ? _start : mNumFrames - 1;
  size_t last = _end < mNumFrames ? _end : mNumFrames - 1;

  outFile.precision(20);
  outFile << "frames = " << last-first+1 << " dofs = " << mSkel->getNumDofs() << std::endl;

  for (size_t i = 0; i < mSkel->getNumDofs(); i++)
  {
    const dynamics::DegreeOfFreedom* dof        = mSkel->getDof(i);
    const dynamics::Joint*           joint      = dof->getJoint();
    const size_t                     localIndex = dof->getIndexInJoint();

    outFile << joint->getName() << "." << localIndex << ' ';
  }

  outFile << std::endl;

  for (size_t i = first; i <= last; i++)
  {
    for (size_t j = 0; j < mSkel->getNumDofs(); j++)
      outFile << mDofs[i][j] << ' ';
    outFile << std::endl;
  }

  outFile << "FPS " << mFPS << std::endl;

  outFile.close();

  std::string text = _fName;
  size_t lastSlash = text.find_last_of("/");
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
double FileInfoDof::getDofAt(size_t _frame, size_t _id) const
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
}  // namespace kido
