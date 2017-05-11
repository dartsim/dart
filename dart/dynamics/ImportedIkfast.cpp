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

#include "dart/dynamics/ImportedIkfast.hpp"

#include <cassert>
#include <sstream>
#include "dart/common/SharedLibraryManager.hpp"
#include "dart/dynamics/Skeleton.hpp"

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
ImportedIkfast::ImportedIkfast(
    InverseKinematics* ik,
    const std::string& fileName,
    const std::string& methodName,
    const InverseKinematics::Analytical::Properties& properties)
  : Ikfast{ik, methodName, properties},
    mFileName{fileName},
    mSharedLibrary{nullptr},
    mGetNumFreeParameters{nullptr},
    mGetFreeParameters{nullptr},
    mGetNumJoints{nullptr},
    mGetIkRealSize{nullptr},
    mGetIkType{nullptr},
    mComputeIk{nullptr},
    mGetKinematicsHash{nullptr},
    mGetIkFastVersion{nullptr}
{
  configure();
}

//==============================================================================
auto ImportedIkfast::clone(InverseKinematics* newIK) const
    -> std::unique_ptr<GradientMethod>
{
  return dart::common::make_unique<ImportedIkfast>(
        newIK, mFileName, getMethodName(), getAnalyticalProperties());
}

//==============================================================================
int ImportedIkfast::getNumFreeParameters() const
{
  assert(mGetNumFreeParameters);
  // Shouldn't be called if this ImportedIkfast is not configured.

  return mGetNumFreeParameters();
}

//==============================================================================
int* ImportedIkfast::getFreeParameters() const
{
  assert(mGetFreeParameters);
  // Shouldn't be called if this ImportedIkfast is not configured.

  return mGetFreeParameters();
}

//==============================================================================
int ImportedIkfast::getNumJoints() const
{
  assert(mGetNumJoints);
  // Shouldn't be called if this ImportedIkfast is not configured.

  return mGetNumJoints();
}

//==============================================================================
int ImportedIkfast::getIkRealSize() const
{
  assert(mGetIkRealSize);
  // Shouldn't be called if this ImportedIkfast is not configured.

  return mGetIkRealSize();
}

//==============================================================================
int ImportedIkfast::getIkType() const
{
  assert(mGetIkType);
  // Shouldn't be called if this ImportedIkfast is not configured.

  return mGetIkType();
}

//==============================================================================
bool ImportedIkfast::computeIk(
    const IkReal* eetrans,
    const IkReal* eerot,
    const IkReal* pfree,
    ikfast::IkSolutionListBase<IkReal>& solutions)
{
  assert(mComputeIk);
  // Shouldn't be called if this ImportedIkfast is not configured.

  return mComputeIk(eetrans, eerot, pfree, solutions);
}

//==============================================================================
const char* ImportedIkfast::getKinematicsHash()
{
  assert(mGetKinematicsHash);
  // Shouldn't be called if this ImportedIkfast is not configured.

  return mGetKinematicsHash();
}

//==============================================================================
const char* ImportedIkfast::getIkFastVersion()
{
  assert(mGetIkFastVersion);
  // Shouldn't be called if this ImportedIkfast is not configured.

  return mGetIkFastVersion();
}

//==============================================================================
void ImportedIkfast::configure() const
{
  auto lib = common::SharedLibraryManager::getSingleton().load(mFileName);

  if (!lib)
  {
    dterr << "[ImportedIkfast] Could not load dynamic library '" << mFileName
          << "'.\n";
    return;
  }

  loadFunction<IkfastFuncGetInt>(
        lib, "GetNumFreeParameters", mFileName, mGetNumFreeParameters);
  loadFunction<IkfastFuncGetIntPtr>(
        lib, "GetFreeParameters", mFileName, mGetFreeParameters);
  loadFunction<IkfastFuncGetInt>(
        lib, "GetNumJoints", mFileName, mGetNumJoints);
  loadFunction<IkfastFuncGetInt>(
        lib, "GetIkRealSize", mFileName, mGetIkRealSize);
  loadFunction<IkfastFuncGetInt>(
        lib, "GetIkType", mFileName, mGetIkType);
  loadFunction<IkfastFuncComputeIk>(
        lib, "ComputeIk", mFileName, mComputeIk);
  loadFunction<IkfastFuncGetConstCharPtr>(
        lib, "GetKinematicsHash", mFileName, mGetKinematicsHash);
  loadFunction<IkfastFuncGetConstCharPtr>(
        lib, "GetIkFastVersion", mFileName, mGetIkFastVersion);

  mSharedLibrary = lib;

  Ikfast::configure();
}

} // namespace dynamics
} // namespace dart
