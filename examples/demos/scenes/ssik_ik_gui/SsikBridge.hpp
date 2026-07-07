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

// Python bridge for the ssik_ik_gui scene port (examples/ssik_ik_gui). Split
// into its own header per BRIEF-phase2's convention for multi-file scenes
// (examples/demos/scenes/<name>/, namespaced dart_demos::<name>).
//
// Python.h must be included before any standard or system headers because it
// configures feature-test macros (e.g. _POSIX_C_SOURCE) that those headers
// latch on first inclusion; every translation unit that includes this header
// must do so before anything else (see SsikIkGuiScene.cpp).

#ifndef DART_EXAMPLES_DEMOS_SCENES_SSIK_IK_GUI_SSIKBRIDGE_HPP_
#define DART_EXAMPLES_DEMOS_SCENES_SSIK_IK_GUI_SSIKBRIDGE_HPP_

// clang-format off
#include <Python.h>
// clang-format on

#include <dart/dart.hpp>

#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace dart_demos {
namespace ssik_ik_gui {

//==============================================================================
/// RAII helper that ensures the calling thread holds the Python GIL for the
/// duration of a scope. PyGILState_Ensure/Release is reentrant, so this is
/// safe even when the current thread already holds the GIL. This host is
/// single-threaded end to end (see ensurePythonInitialized()'s comment), so
/// there is never actually a separate draw thread contending for the GIL
/// here; the guard is kept anyway both for parity with the original and as
/// insurance against a future threading change.
class GilGuard
{
public:
  GilGuard() : mState(PyGILState_Ensure()) {}
  ~GilGuard()
  {
    PyGILState_Release(mState);
  }

  GilGuard(const GilGuard&) = delete;
  GilGuard& operator=(const GilGuard&) = delete;

private:
  PyGILState_STATE mState;
};

//==============================================================================
/// Ensures exactly one CPython interpreter is initialized for the lifetime of
/// the process, safe to call every time this scene is activated (including
/// repeated Rebuild/Reset and scene switches). Unlike the original example's
/// PythonSession RAII (which owns Py_Initialize/Py_Finalize for the lifetime
/// of a dedicated single-purpose process), a consolidated demos host may
/// activate this scene many times per run, and CPython does not reliably
/// support repeated Py_Finalize/Py_Initialize cycles with extension modules
/// loaded (numpy in particular); this deliberately never finalizes, leaking
/// the interpreter until process exit. Also forces the viewer to
/// SingleThreaded (already the host-wide default set in
/// DemoHost::ensureViewerConfigured(), but asserted here too as a hard
/// invariant this scene relies on): an OSG draw thread calling into Python
/// while the main thread holds the GIL would deadlock on the first solve.
inline void ensurePythonInitialized()
{
  static const bool initialized = [] {
    if (!Py_IsInitialized())
      Py_Initialize();
    return Py_IsInitialized();
  }();
  if (!initialized)
    throw std::runtime_error("failed to initialize Python");
}

//==============================================================================
class PyObjectPtr
{
public:
  explicit PyObjectPtr(PyObject* ptr = nullptr) : mPtr(ptr) {}
  ~PyObjectPtr()
  {
    Py_XDECREF(mPtr);
  }

  PyObjectPtr(const PyObjectPtr&) = delete;
  PyObjectPtr& operator=(const PyObjectPtr&) = delete;

  PyObjectPtr(PyObjectPtr&& other) noexcept : mPtr(other.release()) {}
  PyObjectPtr& operator=(PyObjectPtr&& other) noexcept
  {
    if (this != &other)
      reset(other.release());
    return *this;
  }

  PyObject* get() const
  {
    return mPtr;
  }
  PyObject* release()
  {
    PyObject* ptr = mPtr;
    mPtr = nullptr;
    return ptr;
  }
  void reset(PyObject* ptr = nullptr)
  {
    if (mPtr == ptr)
      return;
    Py_XDECREF(mPtr);
    mPtr = ptr;
  }

  explicit operator bool() const
  {
    return mPtr != nullptr;
  }

private:
  PyObject* mPtr;
};

//==============================================================================
std::string consumePythonError();

//==============================================================================
struct ArmSpec
{
  std::string label;
  std::string module;
};

//==============================================================================
/// The 19 arms bundled with the optional ssik package, in the original's
/// combo-box order.
std::vector<ArmSpec> getArmSpecs();

//==============================================================================
/// One joint of an ssik product-of-exponentials chain: the fixed transforms
/// before/after the joint and its rotation axis. Forward kinematics composes
/// tLeft * exp(axis * q) * tRight per joint.
struct JointSpec
{
  Eigen::Isometry3d tLeft = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tRight = Eigen::Isometry3d::Identity();
  Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
  bool prismatic = false;
};

//==============================================================================
struct ArmInfo
{
  std::string baseLink;
  std::string eeLink;
  int dof = 0;
  std::vector<JointSpec> chain;
  Eigen::Isometry3d home = [] {
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.translation() = Eigen::Vector3d(0.45, 0.0, 0.35);
    return tf;
  }();
};

//==============================================================================
struct SsikSolution
{
  Eigen::VectorXd q;
  double fkResidual = std::numeric_limits<double>::quiet_NaN();
  std::string refinementUsed;
  bool hasFkPose = false;
  Eigen::Isometry3d fkPose = Eigen::Isometry3d::Identity();
};

//==============================================================================
/// Python bridge to the optional ssik package: loads a prebuilt analytical IK
/// module and solves for a target pose, also reporting the module's
/// product-of-exponentials chain so the scene can render an articulated arm.
/// One instance is created per scene activation (see SsikIkGuiScene.cpp);
/// construction only requires numpy (bundled in the pixi environment), not
/// ssik itself -- ssik is imported lazily per-arm in loadArm(), with load
/// failures reported through `error` rather than thrown, so a missing ssik
/// install degrades to a status message instead of disabling the scene.
class SsikPythonBridge
{
public:
  SsikPythonBridge();

  bool loadArm(
      const std::string& moduleName, ArmInfo& info, std::string& error);

  bool solve(
      const Eigen::Isometry3d& target,
      int maxSolutions,
      bool useSeed,
      const Eigen::VectorXd& seed,
      bool respectLimits,
      bool allowRefinement,
      const std::string& seedMetric,
      std::vector<SsikSolution>& solutions,
      std::string& error);

private:
  PyObjectPtr mGlobals;
  PyObjectPtr mBridge;
};

} // namespace ssik_ik_gui
} // namespace dart_demos

#endif // DART_EXAMPLES_DEMOS_SCENES_SSIK_IK_GUI_SSIKBRIDGE_HPP_
