/*
 * Copyright (c) 2014-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2014-2016, Humanoid Lab, Georgia Tech Research Corporation
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

#include "dart/utils/FileInfoWorld.hpp"

#include <fstream>
#include <string>

#include "dart/simulation/Recording.hpp"

namespace dart {
namespace utils {

//==============================================================================
FileInfoWorld::FileInfoWorld()
  : mRecord(nullptr)
{
  std::strcpy(mFileName, "");
}

//==============================================================================
FileInfoWorld::~FileInfoWorld()
{
  delete mRecord;
}

//==============================================================================
bool FileInfoWorld::loadFile(const char* _fName)
{
  std::ifstream inFile(_fName);
  if (inFile.fail() == 1) return false;

  inFile.precision(8);
  char buffer[256];
  int numFrames;
  int numSkeletons;
  int intVal;
  double doubleVal;
  std::vector<int> numDofsForSkels;
  std::vector<double> tempState;
  Eigen::VectorXd state;

  inFile >> buffer;
  inFile >> numFrames;
  inFile >> buffer;
  inFile >> numSkeletons;

  for (int i = 0; i < numSkeletons; i++)
  {
    inFile >> buffer;
    inFile >> intVal;
    numDofsForSkels.push_back(intVal);
  }

  // Release the previous recording
  delete mRecord;

  mRecord = new simulation::Recording(numDofsForSkels);

  for (int i = 0; i < numFrames; i++)
  {
    for (int j = 0; j < numSkeletons; j++)
    {
      for (int k = 0; k < mRecord->getNumDofs(j); k++)
      {
        inFile >> doubleVal;
        tempState.push_back(doubleVal);
      }
    }

    inFile >> buffer;
    inFile >> intVal;
    for (int j = 0; j < intVal; j++)
      {
        for (int k = 0; k < 6; k++)
          {
            inFile >> doubleVal;
            tempState.push_back(doubleVal);
          }
      }

    state.resize(tempState.size());
    for (std::size_t j = 0; j < tempState.size(); j++)
      state[j] = tempState[j];
    mRecord->addState(state);
    tempState.clear();
  }
  inFile.close();

  std::string text = _fName;
  int lastSlash = text.find_last_of("/");
  text = text.substr(lastSlash+1);
  strcpy(mFileName, text.c_str());
  return true;
}

//==============================================================================
bool FileInfoWorld::saveFile(const char* _fName, simulation::Recording* _record)
{
  std::ofstream outFile(_fName, std::ios::out);
  if (outFile.fail()) return false;

  outFile.precision(8);

  outFile << "numFrames " << _record->getNumFrames() << std::endl;
  outFile << "numSkeletons " << _record->getNumSkeletons() << std::endl;
  for (int i = 0; i < _record->getNumSkeletons(); i++)
    outFile << "Skeleton" << i << " " << _record->getNumDofs(i) << " ";
  outFile << std::endl;
  for (int i = 0; i < _record->getNumFrames(); i++)
  {
    for (int j = 0; j < _record->getNumSkeletons(); j++)
    {
      for (int k = 0; k < _record->getNumDofs(j); k++)
        outFile << _record->getGenCoord(i, j, k) << " ";
      outFile << std::endl;
    }
    outFile << "Contacts " << _record->getNumContacts(i) << std::endl;

    for (int j = 0; j < _record->getNumContacts(i); j++)
    {
      outFile << _record->getContactPoint(i, j) << std::endl;
      outFile << _record->getContactForce(i, j) << std::endl;
    }
    outFile << std::endl;
  }
  
  outFile.close();

  std::string text = _fName;
  int lastSlash = text.find_last_of("/");
  text = text.substr(lastSlash+1);
  std::strcpy(mFileName, text.c_str());
  return true;
}

//==============================================================================
simulation::Recording* FileInfoWorld::getRecording() const
{
  return mRecord;
}

}  // namespace utils
}  // namespace dart
