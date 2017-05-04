/*
 * Copyright (c) 2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2017, Personal Robotics Lab, Carnegie Mellon University
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

#include "dart/dynamics/ImportedIkfastSolver.hpp"

#include <cassert>
#include <sstream>
#include "dart/common/SharedLibraryManager.hpp"

namespace dart {
namespace dynamics {

namespace {

//==============================================================================
template <typename FunctionType>
void loadFunction(
    const std::shared_ptr<common::SharedLibrary>& lib,
    const std::string& symbolName,
    const std::string& fileName,
    FunctionType& func)
{
  auto symbol = lib->getSymbol(symbolName);

  if (!symbol)
  {
    std::stringstream ss;
    ss << "Failed to load the symbol '<<" << symbolName << "' from the file '"
       << fileName << "'.";
    throw std::runtime_error(ss.str());
  }

  func = reinterpret_cast<FunctionType>(symbol);
  // TODO(JS): Is there a way to check the compatibility of the symbol?
}

} // namespace (anonymous)

//==============================================================================
ImportedIkfastSolver::ImportedIkfastSolver(const std::string& fileName)
{
  auto lib = common::SharedLibraryManager::getSingleton().load(fileName);

  if (!lib)
  {
    // TODO(JS): print warning
    return;
  }

  loadFunction<IkfastFuncGetInt>(
        lib, "GetNumFreeParameters", fileName, mGetNumFreeParameters);
  loadFunction<IkfastFuncGetIntPtr>(
        lib, "GetFreeParameters", fileName, mGetFreeParameters);
  loadFunction<IkfastFuncGetInt>(
        lib, "GetNumJoints", fileName, mGetNumJoints);
  loadFunction<IkfastFuncGetInt>(
        lib, "GetIkRealSize", fileName, mGetIkRealSize);
  loadFunction<IkfastFuncGetInt>(
        lib, "GetIkType", fileName, mGetIkType);
  loadFunction<IkfastFuncComputeIk>(
        lib, "ComputeIk", fileName, mComputeIk);
  loadFunction<IkfastFuncGetConstCharPtr>(
        lib, "GetKinematicsHash", fileName, mGetKinematicsHash);
  loadFunction<IkfastFuncGetConstCharPtr>(
        lib, "GetIkFastVersion", fileName, mGetIkFastVersion);

  mSharedLibrary = lib;
}

//==============================================================================
int ImportedIkfastSolver::getNumFreeParameters()
{
  assert(mGetNumFreeParameters);
  return mGetNumFreeParameters();
}

//==============================================================================
int* ImportedIkfastSolver::getFreeParameters()
{
  assert(mGetFreeParameters);
  return mGetFreeParameters();
}

//==============================================================================
int ImportedIkfastSolver::getNumJoints()
{
  assert(mGetNumJoints);
  return mGetNumJoints();
}

//==============================================================================
int ImportedIkfastSolver::getIkRealSize()
{
  assert(mGetIkRealSize);
  return mGetIkRealSize();
}

//==============================================================================
int ImportedIkfastSolver::getIkType()
{
  assert(mGetIkType);
  return mGetIkType();
}

//==============================================================================
bool ImportedIkfastSolver::computeIk(
    const IkReal* eetrans,
    const IkReal* eerot,
    const IkReal* pfree,
    ikfast::IkSolutionListBase<IkReal>& solutions)
{
  assert(mComputeIk);
  return mComputeIk(eetrans, eerot, pfree, solutions);
}

//==============================================================================
const char* ImportedIkfastSolver::getKinematicsHash()
{
  assert(mGetKinematicsHash);
  return mGetKinematicsHash();
}

//==============================================================================
const char* ImportedIkfastSolver::getIkFastVersion()
{
  assert(mGetIkFastVersion);
  return mGetIkFastVersion();
}

} // namespace dynamics
} // namespace dart
