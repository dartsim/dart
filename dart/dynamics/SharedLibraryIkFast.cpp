/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#include "dart/dynamics/SharedLibraryIkFast.hpp"

#include <cassert>
#include <sstream>

#include "dart/common/Console.hpp"
#include "dart/common/SharedLibrary.hpp"
#include "dart/dynamics/Skeleton.hpp"

namespace dart {
namespace dynamics {

namespace {

//==============================================================================
template <typename FunctionType>
bool loadFunction(
    const std::shared_ptr<common::SharedLibrary>& lib,
    const std::string& symbolName,
    const std::string& fileName,
    FunctionType& func) {
  auto symbol = lib->getSymbol(symbolName);

  if (!symbol) {
    dterr << "Failed to load the symbol '" << symbolName << "' from the file '"
          << fileName << "'.\n";
    return false;
  }

  func = reinterpret_cast<FunctionType>(symbol);
  // TODO(JS): Is there a way to check the compatibility of the symbol?

  return true;
}

} // namespace

//==============================================================================
SharedLibraryIkFast::SharedLibraryIkFast(
    InverseKinematics* ik,
    const std::string& filePath,
    const std::vector<std::size_t>& dofMap,
    const std::vector<std::size_t>& freeDofMap,
    const std::string& methodName,
    const InverseKinematics::Analytical::Properties& properties)
  : IkFast{ik, dofMap, freeDofMap, methodName, properties},
    mFilePath{filePath},
    mSharedLibrary{nullptr},
    mGetNumFreeParameters{nullptr},
    mGetFreeParameters{nullptr},
    mGetNumJoints{nullptr},
    mGetIkRealSize{nullptr},
    mGetIkType{nullptr},
    mComputeIk{nullptr},
    mGetKinematicsHash{nullptr},
    mGetIkFastVersion{nullptr} {
  auto lib = common::SharedLibrary::create(mFilePath);

  if (!lib) {
    dterr << "[SharedLibraryIkFast] Could not load dynamic library '"
          << mFilePath << "'. This SharedLibraryIkFast is invalid.\n";
    return;
  }

  // It could fail to load the following symbols from the given shared library.
  // So we should check if the symbol is actually loaded before using it.

  loadFunction<IkFastFuncGetInt>(
      lib, "GetNumFreeParameters", mFilePath, mGetNumFreeParameters);

  loadFunction<IkFastFuncGetIntPtr>(
      lib, "GetFreeParameters", mFilePath, mGetFreeParameters);

  loadFunction<IkFastFuncGetInt>(lib, "GetNumJoints", mFilePath, mGetNumJoints);

  loadFunction<IkFastFuncGetInt>(
      lib, "GetIkRealSize", mFilePath, mGetIkRealSize);

  loadFunction<IkFastFuncGetInt>(lib, "GetIkType", mFilePath, mGetIkType);

  loadFunction<IkFastFuncComputeIk>(lib, "ComputeIk", mFilePath, mComputeIk);

  loadFunction<IkFastFuncGetConstCharPtr>(
      lib, "GetKinematicsHash", mFilePath, mGetKinematicsHash);

  loadFunction<IkFastFuncGetConstCharPtr>(
      lib, "GetIkFastVersion", mFilePath, mGetIkFastVersion);

  mSharedLibrary = lib;
}

//==============================================================================
auto SharedLibraryIkFast::clone(InverseKinematics* newIK) const
    -> std::unique_ptr<GradientMethod> {
  return std::make_unique<SharedLibraryIkFast>(
      newIK,
      mFilePath,
      mDofs,
      mFreeDofs,
      getMethodName(),
      getAnalyticalProperties());
}

//==============================================================================
int SharedLibraryIkFast::getNumFreeParameters() const {
  if (!mGetNumFreeParameters) {
    dterr << "[SharedLibraryIkFast::getNumFreeParameters] This SharedLibrary "
          << "is invalid. Returning 0.\n";
    return 0;
  }

  return mGetNumFreeParameters();
}

//==============================================================================
int* SharedLibraryIkFast::getFreeParameters() const {
  if (!mGetFreeParameters) {
    dterr << "[SharedLibraryIkFast::mGetFreeParameters] This SharedLibrary is "
          << "invalid. Returning nullptr.\n";
    return nullptr;
  }

  return mGetFreeParameters();
}

//==============================================================================
int SharedLibraryIkFast::getNumJoints() const {
  if (!mGetNumJoints) {
    dterr << "[SharedLibraryIkFast::mGetNumJoints] This SharedLibrary is "
          << "invalid. Returning 0.\n";
    return 0;
  }

  return mGetNumJoints();
}

//==============================================================================
int SharedLibraryIkFast::getIkRealSize() const {
  if (!mGetIkRealSize) {
    dterr << "[SharedLibraryIkFast::mGetIkRealSize] This SharedLibrary is "
          << "invalid. Returning 0.\n";
    return 0;
  }

  return mGetIkRealSize();
}

//==============================================================================
int SharedLibraryIkFast::getIkType() const {
  if (!mGetIkType) {
    dterr << "[SharedLibraryIkFast::mGetIkType] This SharedLibrary is "
          << "invalid. Returning 0.\n";
    return 0;
  }

  return mGetIkType();
}

//==============================================================================
bool SharedLibraryIkFast::computeIk(
    const IkReal* targetTranspose,
    const IkReal* targetRotation,
    const IkReal* freeParams,
    ikfast::IkSolutionListBase<IkReal>& solutions) {
  if (!mComputeIk) {
    dterr << "[SharedLibraryIkFast::mComputeIk] This SharedLibrary is "
          << "invalid. Returning false.\n";
    return false;
  }

  return mComputeIk(targetTranspose, targetRotation, freeParams, solutions);
}

//==============================================================================
void SharedLibraryIkFast::computeFk(
    const IkReal* parameters, IkReal* targetTranspose, IkReal* targetRotation) {
  if (!mComputeFk) {
    dterr << "[SharedLibraryIkFast::mComputeFk] This SharedLibrary is "
          << "invalid.\n";
    return;
  }

  mComputeFk(parameters, targetTranspose, targetRotation);
}

//==============================================================================
const char* SharedLibraryIkFast::getKinematicsHash() {
  if (!mGetKinematicsHash) {
    dterr << "[SharedLibraryIkFast::mGetKinematicsHash] This SharedLibrary is "
          << "invalid. Returning nullptr.\n";
    return nullptr;
  }

  return mGetKinematicsHash();
}

//==============================================================================
const char* SharedLibraryIkFast::getIkFastVersion() {
  if (!mGetIkFastVersion) {
    dterr << "[SharedLibraryIkFast::mGetIkFastVersion] This SharedLibrary is "
          << "invalid. Returning nullptr.\n";
    return nullptr;
  }

  return mGetIkFastVersion();
}

//==============================================================================
void SharedLibraryIkFast::configure() const {
  if (!mSharedLibrary)
    return;

  IkFast::configure();
}

} // namespace dynamics
} // namespace dart
