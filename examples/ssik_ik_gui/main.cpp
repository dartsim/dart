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
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

// Python.h must be included before any standard or system headers because it
// configures feature-test macros (e.g. _POSIX_C_SOURCE) that those headers
// latch on first inclusion. Keep it first; clang-format would otherwise sort it
// in with the other angle-bracket includes.
// clang-format off
#include <Python.h>
// clang-format on

#include <dart/gui/osg/osg.hpp>

#include <dart/dart.hpp>

#include <osg/Camera>
#include <osg/GraphicsContext>
#include <osg/Viewport>

#include <algorithm>
#include <array>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <cmath>
#include <cstdlib>
#include <cstring>

namespace {

constexpr int kViewerWidth = 1180;
constexpr int kViewerHeight = 860;

constexpr double pi()
{
  return 3.14159265358979323846;
}

//==============================================================================
// RAII helper that ensures the calling thread holds the Python GIL for the
// duration of a scope. The OSG viewer may invoke ImGui widget rendering (and
// therefore the embedded-Python solver calls) from a draw thread that is not
// the thread that initialized the interpreter, so every block that touches the
// CPython C API must hold the GIL. PyGILState_Ensure/Release is reentrant, so
// this is safe even when the current thread already holds the GIL.
class GilGuard
{
public:
  GilGuard() : mState(PyGILState_Ensure())
  {
    // Do nothing
  }

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
class PyObjectPtr
{
public:
  explicit PyObjectPtr(PyObject* ptr = nullptr) : mPtr(ptr)
  {
    // Do nothing
  }

  ~PyObjectPtr()
  {
    Py_XDECREF(mPtr);
  }

  PyObjectPtr(const PyObjectPtr&) = delete;
  PyObjectPtr& operator=(const PyObjectPtr&) = delete;

  PyObjectPtr(PyObjectPtr&& other) noexcept : mPtr(other.release())
  {
    // Do nothing
  }

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
std::string consumePythonError()
{
  if (!PyErr_Occurred())
    return "unknown Python error";

  PyObject* type = nullptr;
  PyObject* value = nullptr;
  PyObject* traceback = nullptr;
  PyErr_Fetch(&type, &value, &traceback);
  PyErr_NormalizeException(&type, &value, &traceback);

  PyObjectPtr typePtr(type);
  PyObjectPtr valuePtr(value);
  PyObjectPtr tracebackPtr(traceback);

  PyObjectPtr text(value ? PyObject_Str(value) : nullptr);
  if (!text)
    return "Python exception";

  const char* utf8 = PyUnicode_AsUTF8(text.get());
  if (!utf8) {
    PyErr_Clear();
    return "Python exception";
  }

  return utf8;
}

//==============================================================================
class PythonSession
{
public:
  PythonSession()
  {
    mOwnsInterpreter = !Py_IsInitialized();
    if (mOwnsInterpreter)
      Py_Initialize();

    if (!Py_IsInitialized())
      throw std::runtime_error("failed to initialize Python");
  }

  ~PythonSession()
  {
    if (mOwnsInterpreter && Py_IsInitialized())
      Py_Finalize();
  }

private:
  bool mOwnsInterpreter;
};

//==============================================================================
struct ArmSpec
{
  std::string label;
  std::string module;
};

//==============================================================================
std::vector<ArmSpec> getArmSpecs()
{
  return {
      {"Universal Robots UR5", "ur5_ik"},
      {"KUKA Puma 560", "puma560_ik"},
      {"Kinova JACO 2", "jaco2_ik"},
      {"KUKA iiwa14", "iiwa14_ik"},
      {"Kinova Gen3", "gen3_ik"},
      {"Franka Panda", "franka_panda_ik"},
      {"UFactory xArm7", "xarm7_ik"},
      {"UFactory xArm6", "xarm6_ik"},
      {"Unitree Z1", "z1_ik"},
      {"AgileX PiPER", "piper_ik"},
      {"Flexiv Rizon 4", "rizon4_ik"},
      {"Kassow KR810", "kassow_kr810_ik"},
      {"Flexiv Rizon 10", "rizon10_ik"},
      {"FANUC CRX-10iA/L", "fanuc_crx10ial_ik"},
      {"I2RT YAM", "yam_ik"},
      {"I2RT big_yam", "big_yam_ik"},
      {"Franka Research 3", "fr3_ik"},
      {"OpenArm left", "openarm_left_ik"},
      {"OpenArm right", "openarm_right_ik"},
  };
}

//==============================================================================
// One joint of an ssik product-of-exponentials chain: the fixed transforms
// before/after the joint and its rotation axis. Forward kinematics composes
// tLeft * exp(axis * q) * tRight per joint.
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
// Python bridge to the optional ssik package. It loads a prebuilt analytical IK
// module, solves for a target pose, and also reports the per-joint world frames
// of each solution (computed from the module's product-of-exponentials chain)
// so the example can render the articulated arm rather than just a marker.
const char* kBridgeScript = R"PY(
import importlib

import numpy as np


class DARTSsikBridge:
    def __init__(self):
        self.module = None

    def _flat(self, matrix):
        m = np.asarray(matrix, dtype=np.float64).reshape((4, 4))
        return [float(v) for v in m.reshape(-1)]

    def _has_chain(self):
        attrs = ("_JOINT_T_LEFTS", "_JOINT_AXES", "_JOINT_TYPES", "_JOINT_T_RIGHTS")
        return self.module is not None and all(
            hasattr(self.module, a) for a in attrs
        )

    def chain(self):
        # Per-joint product-of-exponentials structure used to build a DART
        # skeleton: each joint contributes T_left * exp(axis * q) * T_right.
        if not self._has_chain():
            return None
        t_lefts = self.module._JOINT_T_LEFTS
        t_rights = self.module._JOINT_T_RIGHTS
        axes = self.module._JOINT_AXES
        types = self.module._JOINT_TYPES
        out = []
        for i in range(len(t_lefts)):
            out.append(
                {
                    "t_left": self._flat(t_lefts[i]),
                    "t_right": self._flat(t_rights[i]),
                    "axis": [float(x) for x in np.asarray(axes[i], dtype=np.float64)],
                    "type": str(types[i]),
                }
            )
        return out

    def load(self, module_name):
        self.module = importlib.import_module("ssik.prebuilt." + module_name)
        return {
            "BASE_LINK": str(getattr(self.module, "BASE_LINK", "")),
            "EE_LINK": str(getattr(self.module, "EE_LINK", "")),
            "DOF": int(getattr(self.module, "DOF", 0)),
            "T_HOME": self._flat(getattr(self.module, "T_HOME")),
            "CHAIN": self.chain(),
        }

    def solve(
        self,
        flat_target,
        max_solutions,
        use_seed,
        seed,
        respect_limits,
        allow_refinement,
        seed_metric,
    ):
        if self.module is None:
            raise RuntimeError("no ssik prebuilt module is loaded")
        target = np.asarray(flat_target, dtype=np.float64).reshape((4, 4))
        kwargs = {
            "max_solutions": None if int(max_solutions) <= 0 else int(max_solutions),
            "respect_limits": bool(respect_limits),
            "allow_refinement": bool(allow_refinement),
            "seed_metric": str(seed_metric),
        }
        if use_seed:
            kwargs["q_seed"] = np.asarray(seed, dtype=np.float64)
        solutions = self.module.solve(target, **kwargs)
        out = []
        has_fk = hasattr(self.module, "fk")
        for solution in solutions:
            q = np.asarray(solution.q, dtype=np.float64)
            fk_pose = self._flat(self.module.fk(q)) if has_fk else None
            out.append(
                {
                    "q": q.tolist(),
                    "fk_residual": float(
                        getattr(solution, "fk_residual", float("nan"))
                    ),
                    "refinement_used": str(
                        getattr(solution, "refinement_used", "")
                    ),
                    "fk_pose": fk_pose,
                }
            )
        return out


bridge = DARTSsikBridge()
)PY";

//==============================================================================
PyObjectPtr makePythonList(const Eigen::Matrix4d& matrix)
{
  PyObjectPtr list(PyList_New(16));
  if (!list)
    return PyObjectPtr();

  int index = 0;
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      PyObject* value = PyFloat_FromDouble(matrix(r, c));
      if (!value)
        return PyObjectPtr();
      PyList_SET_ITEM(list.get(), index++, value);
    }
  }

  return list;
}

//==============================================================================
PyObjectPtr makePythonList(const Eigen::VectorXd& vector)
{
  PyObjectPtr list(PyList_New(vector.size()));
  if (!list)
    return PyObjectPtr();

  for (int i = 0; i < vector.size(); ++i) {
    PyObject* value = PyFloat_FromDouble(vector[i]);
    if (!value)
      return PyObjectPtr();
    PyList_SET_ITEM(list.get(), i, value);
  }

  return list;
}

//==============================================================================
bool parseMatrix(PyObject* object, Eigen::Isometry3d& tf, std::string& error)
{
  PyObjectPtr sequence(PySequence_Fast(object, "expected a flat matrix"));
  if (!sequence) {
    error = consumePythonError();
    return false;
  }

  if (PySequence_Fast_GET_SIZE(sequence.get()) != 16) {
    error = "expected a flat 4x4 matrix from ssik";
    return false;
  }

  Eigen::Matrix4d matrix;
  PyObject** items = PySequence_Fast_ITEMS(sequence.get());
  for (int i = 0; i < 16; ++i) {
    const double value = PyFloat_AsDouble(items[i]);
    if (PyErr_Occurred()) {
      error = consumePythonError();
      return false;
    }
    matrix(i / 4, i % 4) = value;
  }

  tf.matrix() = matrix;
  return true;
}

//==============================================================================
bool parseVector(PyObject* object, Eigen::VectorXd& vector, std::string& error)
{
  PyObjectPtr sequence(PySequence_Fast(object, "expected a vector"));
  if (!sequence) {
    error = consumePythonError();
    return false;
  }

  const Py_ssize_t size = PySequence_Fast_GET_SIZE(sequence.get());
  vector.resize(size);

  PyObject** items = PySequence_Fast_ITEMS(sequence.get());
  for (Py_ssize_t i = 0; i < size; ++i) {
    const double value = PyFloat_AsDouble(items[i]);
    if (PyErr_Occurred()) {
      error = consumePythonError();
      return false;
    }
    vector[static_cast<int>(i)] = value;
  }

  return true;
}

//==============================================================================
bool parseVec3(PyObject* object, Eigen::Vector3d& vec, std::string& error)
{
  PyObjectPtr sequence(PySequence_Fast(object, "expected a 3-vector"));
  if (!sequence) {
    error = consumePythonError();
    return false;
  }
  if (PySequence_Fast_GET_SIZE(sequence.get()) != 3) {
    error = "expected a 3-vector";
    return false;
  }
  PyObject** items = PySequence_Fast_ITEMS(sequence.get());
  for (int i = 0; i < 3; ++i) {
    vec[i] = PyFloat_AsDouble(items[i]);
    if (PyErr_Occurred()) {
      error = consumePythonError();
      return false;
    }
  }
  return true;
}

//==============================================================================
std::string getStringItem(PyObject* dict, const char* key)
{
  PyObject* item = PyDict_GetItemString(dict, key);
  if (!item)
    return "";

  const char* utf8 = PyUnicode_AsUTF8(item);
  if (!utf8) {
    PyErr_Clear();
    return "";
  }

  return utf8;
}

//==============================================================================
bool parseChain(
    PyObject* object, std::vector<JointSpec>& chain, std::string& error)
{
  chain.clear();
  if (!object || object == Py_None)
    return true; // module without product-of-exponentials metadata

  PyObjectPtr sequence(PySequence_Fast(object, "expected a joint chain"));
  if (!sequence) {
    error = consumePythonError();
    return false;
  }

  const Py_ssize_t size = PySequence_Fast_GET_SIZE(sequence.get());
  PyObject** items = PySequence_Fast_ITEMS(sequence.get());
  chain.reserve(static_cast<std::size_t>(size));
  for (Py_ssize_t i = 0; i < size; ++i) {
    PyObject* entry = items[i];
    if (!PyDict_Check(entry)) {
      error = "invalid joint chain entry";
      return false;
    }
    JointSpec spec;
    PyObject* tLeft = PyDict_GetItemString(entry, "t_left");
    PyObject* tRight = PyDict_GetItemString(entry, "t_right");
    PyObject* axis = PyDict_GetItemString(entry, "axis");
    if (!tLeft || !tRight || !axis) {
      error = "incomplete joint chain entry";
      return false;
    }
    if (!parseMatrix(tLeft, spec.tLeft, error)
        || !parseMatrix(tRight, spec.tRight, error)
        || !parseVec3(axis, spec.axis, error))
      return false;
    const std::string type = getStringItem(entry, "type");
    spec.prismatic = type.find("pris") != std::string::npos
                     || type.find("Pris") != std::string::npos;
    chain.push_back(spec);
  }

  return true;
}

//==============================================================================
class SsikPythonBridge
{
public:
  SsikPythonBridge()
  {
    GilGuard gil;

    mGlobals.reset(PyDict_New());
    if (!mGlobals)
      throw std::runtime_error("failed to allocate Python globals");

    PyDict_SetItemString(mGlobals.get(), "__builtins__", PyEval_GetBuiltins());

    PyObjectPtr result(PyRun_String(
        kBridgeScript, Py_file_input, mGlobals.get(), mGlobals.get()));
    if (!result)
      throw std::runtime_error(consumePythonError());

    PyObject* bridge = PyDict_GetItemString(mGlobals.get(), "bridge");
    if (!bridge)
      throw std::runtime_error("failed to create ssik Python bridge");

    Py_INCREF(bridge);
    mBridge.reset(bridge);
  }

  bool loadArm(const std::string& moduleName, ArmInfo& info, std::string& error)
  {
    GilGuard gil;

    PyObjectPtr result(
        PyObject_CallMethod(mBridge.get(), "load", "s", moduleName.c_str()));
    if (!result) {
      error = consumePythonError();
      return false;
    }

    if (!PyDict_Check(result.get())) {
      error = "ssik bridge returned an invalid arm description";
      return false;
    }

    info.baseLink = getStringItem(result.get(), "BASE_LINK");
    info.eeLink = getStringItem(result.get(), "EE_LINK");

    PyObject* dof = PyDict_GetItemString(result.get(), "DOF");
    info.dof = dof ? static_cast<int>(PyLong_AsLong(dof)) : 0;
    if (PyErr_Occurred()) {
      error = consumePythonError();
      return false;
    }
    if (info.dof <= 0) {
      error = "ssik prebuilt module reported a non-positive DOF";
      return false;
    }

    PyObject* home = PyDict_GetItemString(result.get(), "T_HOME");
    if (!home || !parseMatrix(home, info.home, error))
      return false;

    PyObject* chain = PyDict_GetItemString(result.get(), "CHAIN");
    if (!parseChain(chain, info.chain, error))
      return false;

    return true;
  }

  bool solve(
      const Eigen::Isometry3d& target,
      int maxSolutions,
      bool useSeed,
      const Eigen::VectorXd& seed,
      bool respectLimits,
      bool allowRefinement,
      const std::string& seedMetric,
      std::vector<SsikSolution>& solutions,
      std::string& error)
  {
    GilGuard gil;

    PyObjectPtr method(PyObject_GetAttrString(mBridge.get(), "solve"));
    if (!method) {
      error = consumePythonError();
      return false;
    }

    PyObjectPtr flatTarget(makePythonList(target.matrix()));
    PyObjectPtr seedList(makePythonList(seed));
    PyObjectPtr maxObject(PyLong_FromLong(maxSolutions));
    PyObjectPtr useSeedObject(PyBool_FromLong(useSeed));
    PyObjectPtr respectLimitsObject(PyBool_FromLong(respectLimits));
    PyObjectPtr allowRefinementObject(PyBool_FromLong(allowRefinement));
    PyObjectPtr seedMetricObject(PyUnicode_FromString(seedMetric.c_str()));
    if (!flatTarget || !seedList || !maxObject || !useSeedObject
        || !respectLimitsObject || !allowRefinementObject
        || !seedMetricObject) {
      error = consumePythonError();
      return false;
    }

    PyObjectPtr args(PyTuple_New(7));
    if (!args) {
      error = consumePythonError();
      return false;
    }

    PyTuple_SET_ITEM(args.get(), 0, flatTarget.release());
    PyTuple_SET_ITEM(args.get(), 1, maxObject.release());
    PyTuple_SET_ITEM(args.get(), 2, useSeedObject.release());
    PyTuple_SET_ITEM(args.get(), 3, seedList.release());
    PyTuple_SET_ITEM(args.get(), 4, respectLimitsObject.release());
    PyTuple_SET_ITEM(args.get(), 5, allowRefinementObject.release());
    PyTuple_SET_ITEM(args.get(), 6, seedMetricObject.release());

    PyObjectPtr result(PyObject_CallObject(method.get(), args.get()));
    if (!result) {
      error = consumePythonError();
      return false;
    }

    PyObjectPtr sequence(PySequence_Fast(result.get(), "expected solutions"));
    if (!sequence) {
      error = consumePythonError();
      return false;
    }

    solutions.clear();
    const Py_ssize_t size = PySequence_Fast_GET_SIZE(sequence.get());
    PyObject** items = PySequence_Fast_ITEMS(sequence.get());
    solutions.reserve(static_cast<std::size_t>(size));

    for (Py_ssize_t i = 0; i < size; ++i) {
      PyObject* item = items[i];
      if (!PyDict_Check(item)) {
        error = "ssik bridge returned an invalid solution";
        return false;
      }

      SsikSolution solution;
      PyObject* q = PyDict_GetItemString(item, "q");
      if (!q || !parseVector(q, solution.q, error))
        return false;

      PyObject* residual = PyDict_GetItemString(item, "fk_residual");
      if (residual) {
        solution.fkResidual = PyFloat_AsDouble(residual);
        if (PyErr_Occurred()) {
          PyErr_Clear();
          solution.fkResidual = std::numeric_limits<double>::quiet_NaN();
        }
      }

      solution.refinementUsed = getStringItem(item, "refinement_used");

      PyObject* fkPose = PyDict_GetItemString(item, "fk_pose");
      if (fkPose && fkPose != Py_None) {
        if (!parseMatrix(fkPose, solution.fkPose, error))
          return false;
        solution.hasFkPose = true;
      }

      solutions.push_back(solution);
    }

    return true;
  }

private:
  PyObjectPtr mGlobals;
  PyObjectPtr mBridge;
};

//==============================================================================
// Renders the IK solutions as real DART skeletons built from the arm's
// product-of-exponentials chain and posed with setPositions(q). Every solution
// gets its own skeleton instance so they can be shown simultaneously: the
// selected one is opaque and the rest are translucent "ghosts". Using
// articulated skeletons keeps each arm connected by construction; switching
// arms rebuilds the pool, so no per-link visual state leaks across arms.
class ArmSkeleton
{
public:
  void setWorld(const dart::simulation::WorldPtr& world)
  {
    mWorld = world;
  }

  // Selects the chain for subsequent solutions. An empty chain clears the arm.
  void build(const std::vector<JointSpec>& chain)
  {
    clearInstances();
    mChain = chain;
  }

  // Poses one skeleton per solution, highlighting the selected one and drawing
  // the others as translucent ghosts. With no solution, shows a single skeleton
  // at its zero configuration.
  void showSolutions(const std::vector<SsikSolution>& solutions, int selected)
  {
    if (mChain.empty()) {
      clearInstances();
      return;
    }

    const std::size_t wanted
        = solutions.empty()
              ? 1
              : std::min<std::size_t>(solutions.size(), kMaxInstances);
    while (mInstances.size() < wanted)
      mInstances.push_back(buildInstance());
    while (mInstances.size() > wanted) {
      mWorld->removeSkeleton(mInstances.back());
      mInstances.pop_back();
    }

    const Eigen::Vector4d selectedColor(0.90, 0.20, 0.18, 1.0);
    const Eigen::Vector4d ghostColor(0.85, 0.32, 0.30, 0.22);
    const Eigen::Vector4d restColor(0.55, 0.58, 0.62, 0.5);

    if (solutions.empty()) {
      setConfig(mInstances[0], Eigen::VectorXd::Zero(numDofs()));
      setColor(mInstances[0], restColor);
      return;
    }

    for (std::size_t i = 0; i < wanted; ++i) {
      setConfig(mInstances[i], solutions[i].q);
      setColor(
          mInstances[i],
          static_cast<int>(i) == selected ? selectedColor : ghostColor);
    }
  }

private:
  static constexpr double kLinkRadius = 0.028;
  static constexpr double kJointRadius = 0.045;
  static constexpr std::size_t kMaxInstances = 16;

  std::size_t numDofs() const
  {
    return mInstances.empty() ? mChain.size()
                              : mInstances.front()->getNumDofs();
  }

  void clearInstances()
  {
    for (const auto& skeleton : mInstances)
      mWorld->removeSkeleton(skeleton);
    mInstances.clear();
  }

  void setConfig(
      const dart::dynamics::SkeletonPtr& skeleton, const Eigen::VectorXd& q)
  {
    const std::size_t dofs = skeleton->getNumDofs();
    Eigen::VectorXd positions = Eigen::VectorXd::Zero(dofs);
    const Eigen::Index n
        = std::min<Eigen::Index>(static_cast<Eigen::Index>(dofs), q.size());
    positions.head(n) = q.head(n);
    skeleton->setPositions(positions);
  }

  void setColor(
      const dart::dynamics::SkeletonPtr& skeleton, const Eigen::Vector4d& color)
  {
    for (std::size_t b = 0; b < skeleton->getNumBodyNodes(); ++b) {
      auto* body = skeleton->getBodyNode(b);
      for (std::size_t s = 0; s < body->getNumShapeNodes(); ++s) {
        auto* node = body->getShapeNode(s);
        if (auto* visual = node->getVisualAspect())
          visual->setRGBA(color);
      }
    }
  }

  dart::dynamics::SkeletonPtr buildInstance()
  {
    auto skeleton = dart::dynamics::Skeleton::create(
        "ssik_arm_" + std::to_string(mInstanceCounter++));

    dart::dynamics::WeldJoint::Properties baseProps;
    baseProps.mName = "base_joint";
    dart::dynamics::BodyNode* parent
        = skeleton
              ->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(
                  nullptr, baseProps)
              .second;
    parent->setName("base");
    addSphere(parent, Eigen::Vector3d::Zero());
    addCylinder(
        parent, Eigen::Vector3d::Zero(), mChain.front().tLeft.translation());

    for (std::size_t k = 0; k < mChain.size(); ++k) {
      const JointSpec& spec = mChain[k];
      Eigen::Vector3d axis = spec.axis;
      if (axis.norm() > 1e-9)
        axis.normalize();

      dart::dynamics::BodyNode* body = nullptr;
      if (spec.prismatic) {
        dart::dynamics::PrismaticJoint::Properties props;
        props.mName = "j" + std::to_string(k);
        props.mAxis = axis;
        props.mT_ParentBodyToJoint = spec.tLeft;
        props.mT_ChildBodyToJoint = spec.tRight.inverse();
        body = skeleton
                   ->createJointAndBodyNodePair<dart::dynamics::PrismaticJoint>(
                       parent, props)
                   .second;
      } else {
        dart::dynamics::RevoluteJoint::Properties props;
        props.mName = "j" + std::to_string(k);
        props.mAxis = axis;
        props.mT_ParentBodyToJoint = spec.tLeft;
        props.mT_ChildBodyToJoint = spec.tRight.inverse();
        body = skeleton
                   ->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>(
                       parent, props)
                   .second;
      }
      body->setName("b" + std::to_string(k));

      addSphere(body, Eigen::Vector3d::Zero());
      // The body's parent and child joints are fixed in its frame, so the link
      // cylinders connecting them are static geometry that moves with the body.
      addCylinder(
          body, spec.tRight.inverse().translation(), Eigen::Vector3d::Zero());
      if (k + 1 < mChain.size())
        addCylinder(
            body, Eigen::Vector3d::Zero(), mChain[k + 1].tLeft.translation());

      parent = body;
    }

    mWorld->addSkeleton(skeleton);
    return skeleton;
  }

  void addCylinder(
      dart::dynamics::BodyNode* body,
      const Eigen::Vector3d& a,
      const Eigen::Vector3d& b)
  {
    const Eigen::Vector3d delta = b - a;
    const double length = delta.norm();
    if (length < 1e-5)
      return;
    auto shape
        = std::make_shared<dart::dynamics::CylinderShape>(kLinkRadius, length);
    auto* node = body->createShapeNodeWith<dart::dynamics::VisualAspect>(shape);
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.translation() = 0.5 * (a + b);
    tf.linear() = Eigen::Quaterniond::FromTwoVectors(
                      Eigen::Vector3d::UnitZ(), delta / length)
                      .toRotationMatrix();
    node->setRelativeTransform(tf);
  }

  void addSphere(
      dart::dynamics::BodyNode* body, const Eigen::Vector3d& position)
  {
    auto shape = std::make_shared<dart::dynamics::SphereShape>(kJointRadius);
    auto* node = body->createShapeNodeWith<dart::dynamics::VisualAspect>(shape);
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.translation() = position;
    node->setRelativeTransform(tf);
  }

  dart::simulation::WorldPtr mWorld;
  std::vector<JointSpec> mChain;
  std::vector<dart::dynamics::SkeletonPtr> mInstances;
  std::size_t mInstanceCounter = 0;
};

//==============================================================================
Eigen::Isometry3d makeTransform(
    const std::array<float, 3>& xyz, const std::array<float, 3>& rpyDegrees)
{
  const double roll = rpyDegrees[0] * pi() / 180.0;
  const double pitch = rpyDegrees[1] * pi() / 180.0;
  const double yaw = rpyDegrees[2] * pi() / 180.0;

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(xyz[0], xyz[1], xyz[2]);
  tf.linear() = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                 * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                 * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
                    .toRotationMatrix();
  return tf;
}

//==============================================================================
void setTargetControlsFromTransform(
    const Eigen::Isometry3d& tf,
    std::array<float, 3>& xyz,
    std::array<float, 3>& rpyDegrees)
{
  xyz[0] = static_cast<float>(tf.translation().x());
  xyz[1] = static_cast<float>(tf.translation().y());
  xyz[2] = static_cast<float>(tf.translation().z());

  const Eigen::Vector3d zyx = tf.linear().eulerAngles(2, 1, 0);
  rpyDegrees[0] = static_cast<float>(zyx[2] * 180.0 / pi());
  rpyDegrees[1] = static_cast<float>(zyx[1] * 180.0 / pi());
  rpyDegrees[2] = static_cast<float>(zyx[0] * 180.0 / pi());
}

//==============================================================================
std::string formatVector(const Eigen::VectorXd& vector)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(4);
  for (int i = 0; i < vector.size(); ++i) {
    if (i > 0)
      stream << ", ";
    stream << vector[i];
  }
  return stream.str();
}

//==============================================================================
Eigen::Isometry3d offsetPose(
    const Eigen::Isometry3d& pose, const Eigen::Vector3d& offset)
{
  Eigen::Isometry3d out = pose;
  out.translation() += offset;
  return out;
}

//==============================================================================
class SsikIkWidget : public dart::gui::osg::ImGuiWidget
{
public:
  SsikIkWidget(
      dart::gui::osg::ImGuiViewer* viewer,
      dart::gui::osg::InteractiveFramePtr target,
      dart::dynamics::SimpleFramePtr solvedFrame,
      ArmSkeleton* arm,
      int initialArmIndex)
    : mViewer(viewer),
      mTarget(std::move(target)),
      mSolvedFrame(std::move(solvedFrame)),
      mArm(arm),
      mArms(getArmSpecs())
  {
    if (initialArmIndex >= 0
        && initialArmIndex < static_cast<int>(mArms.size()))
      mArmIndex = initialArmIndex;
    loadSelectedArm();
  }

  void render() override
  {
    // If the target gizmo was dragged in the 3D view, its frame transform no
    // longer matches the panel controls. Pick that up, reflect it in the
    // controls, and re-solve so the arm follows the dragged target.
    const Eigen::Isometry3d targetTf = mTarget->getTransform();
    if (!targetTf.isApprox(mAppliedTargetTf, 1e-9)) {
      setTargetControlsFromTransform(targetTf, mTargetXyz, mTargetRpyDegrees);
      mAppliedTargetTf = targetTf;
      if (mAutoSolve)
        solveCurrent();
    }

    const auto guiScale
        = static_cast<float>(mViewer->getImGuiHandler()->getGuiScale());
    ImGui::SetNextWindowPos(
        ImVec2(10 * guiScale, 20 * guiScale), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowBgAlpha(0.85f);

    // Auto-resize so the panel always grows to fit its content (the amount of
    // text varies with the loaded arm and number of solutions); the user never
    // has to resize it manually to reveal clipped controls.
    if (!ImGui::Begin(
            "ssik Analytical IK",
            nullptr,
            ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_AlwaysAutoResize)) {
      ImGui::End();
      return;
    }

    // Constrain field and wrapped-text widths so the auto-resized window has a
    // stable, readable width instead of stretching to the longest message.
    const float fieldWidth = 240.0f * guiScale;
    ImGui::PushItemWidth(fieldWidth);
    ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + 360.0f * guiScale);

    if (ImGui::BeginMenuBar()) {
      if (ImGui::BeginMenu("Menu")) {
        if (ImGui::MenuItem("Exit"))
          mViewer->setDone(true);
        ImGui::EndMenu();
      }
      if (ImGui::BeginMenu("Help")) {
        if (ImGui::MenuItem("About DART"))
          mViewer->showAbout();
        ImGui::EndMenu();
      }
      ImGui::EndMenuBar();
    }

    renderArmSelector();
    ImGui::Separator();
    renderTargetControls();
    ImGui::Separator();
    renderSolverControls();
    ImGui::Separator();
    renderSolutions();

    ImGui::PopTextWrapPos();
    ImGui::PopItemWidth();
    ImGui::End();
  }

  int armCount() const
  {
    return static_cast<int>(mArms.size());
  }

  // Test hook: switches to the given arm (the same path as choosing it in the
  // combo) and returns a one-line summary. Used by --self-test to exercise
  // online arm switching across every prebuilt module without opening a window.
  std::string selectArmForTest(int index, bool& loaded, int& solutionCount)
  {
    loaded = false;
    solutionCount = 0;
    if (index < 0 || index >= static_cast<int>(mArms.size()))
      return "invalid arm index";

    mArmIndex = index;
    loadSelectedArm();

    // "loaded" means ssik imported and the kinematic chain was read, i.e. the
    // optional IK integration is healthy. Zero solutions for a particular
    // target is not a per-arm failure: several arms' home pose sits on a
    // joint-limit boundary, so ssik legitimately returns no limit-valid
    // solution there. The caller fails the self-test when any arm fails to
    // load, or when no arm produces any solution at all (which would indicate a
    // broken solver).
    loaded = mLoaded;
    solutionCount = static_cast<int>(mSolutions.size());

    std::ostringstream stream;
    stream << mArms[index].label << " (" << mArms[index].module << "): ";
    if (mLoaded)
      stream << "loaded DOF=" << mInfo.dof << ", " << mSolutions.size()
             << " solution(s)";
    else
      stream << "not loaded (" << mStatus << ")";
    return stream.str();
  }

private:
  // Attaches an explanatory tooltip to the most recently submitted widget,
  // shown after a short hover. Keeps the panel usable by newcomers who may not
  // know the IK terminology without cluttering the controls themselves.
  static void itemTooltip(const char* text)
  {
    ImGui::SetItemTooltip("%s", text);
  }

  void renderArmSelector()
  {
    if (ImGui::BeginCombo("Arm", mArms[mArmIndex].label.c_str())) {
      for (std::size_t i = 0; i < mArms.size(); ++i) {
        const bool selected = static_cast<int>(i) == mArmIndex;
        if (ImGui::Selectable(mArms[i].label.c_str(), selected)) {
          mArmIndex = static_cast<int>(i);
          loadSelectedArm();
        }
        if (selected)
          ImGui::SetItemDefaultFocus();
      }
      ImGui::EndCombo();
    }
    itemTooltip(
        "Robot arm to control. Each entry is an ssik prebuilt analytical IK "
        "module; switching rebuilds the arm and re-solves immediately.");

    if (mLoaded) {
      ImGui::Text("Module: ssik.prebuilt.%s", mArms[mArmIndex].module.c_str());
      itemTooltip(
          "The Python module providing this arm's closed-form IK and "
          "kinematics.");
      ImGui::Text(
          "Chain: %s -> %s", mInfo.baseLink.c_str(), mInfo.eeLink.c_str());
      itemTooltip(
          "Kinematic chain solved: from the base link to the end-effector "
          "(tip) link.");
      ImGui::Text("DOF: %d", mInfo.dof);
      itemTooltip(
          "Degrees of freedom: the number of independently actuated joints in "
          "the arm.");
    } else {
      ImGui::TextColored(
          ImVec4(1.0f, 0.78f, 0.20f, 1.0f), "ssik is not available");
      ImGui::TextWrapped("%s", mStatus.c_str());
      ImGui::TextWrapped(
          "Install into this Pixi environment with: "
          "pixi run python -m pip install ssik");
    }
  }

  void renderTargetControls()
  {
    bool changed = false;
    changed |= ImGui::DragFloat3("Position", mTargetXyz.data(), 0.005f);
    itemTooltip(
        "Desired end-effector position (x, y, z) in meters, in the arm's base "
        "frame. Drag a value to edit, or drag the gizmo in the 3D view.");
    changed |= ImGui::DragFloat3("RPY deg", mTargetRpyDegrees.data(), 0.5f);
    itemTooltip(
        "Desired end-effector orientation as roll, pitch, yaw (degrees) about "
        "the X, Y, Z axes.");

    if (changed) {
      syncTargetFrame();
      if (mAutoSolve)
        solveCurrent();
    }

    if (ImGui::Button("Home target")) {
      setTargetControlsFromTransform(mInfo.home, mTargetXyz, mTargetRpyDegrees);
      syncTargetFrame();
      if (mAutoSolve)
        solveCurrent();
    }
    itemTooltip("Reset the target to the arm's home (rest) pose.");
    ImGui::SameLine();
    if (ImGui::Button("Read dragged")) {
      setTargetControlsFromTransform(
          mTarget->getTransform(), mTargetXyz, mTargetRpyDegrees);
      if (mAutoSolve)
        solveCurrent();
    }
    itemTooltip(
        "Copy the pose of the draggable gizmo in the 3D view back into these "
        "fields.");
  }

  void renderSolverControls()
  {
    bool changed = false;
    changed |= ImGui::Checkbox("Auto solve", &mAutoSolve);
    itemTooltip(
        "Re-run the IK automatically whenever the target or an option changes "
        "(including while dragging the gizmo).");
    changed |= ImGui::Checkbox("Respect limits", &mRespectLimits);
    itemTooltip(
        "Discard solutions that violate the arm's joint limits. Turn off to "
        "also see out-of-limit branches.");
    changed |= ImGui::Checkbox("Allow refinement", &mAllowRefinement);
    itemTooltip(
        "Let ssik numerically refine near-singular solutions for extra "
        "accuracy, at some extra cost.");
    changed |= ImGui::Checkbox("Use q seed", &mUseSeed);
    itemTooltip(
        "Provide a seed (reference) joint configuration so solutions are "
        "ranked by closeness to it, keeping motion continuous.");
    changed |= ImGui::SliderInt("Max solutions", &mMaxSolutions, 0, 256);
    itemTooltip(
        "Maximum number of IK branches to return (0 = all). Analytical IK "
        "yields several distinct elbow/wrist configurations for one target.");

    const char* metrics[] = {"wrap_linf", "wrap_l2"};
    changed |= ImGui::Combo("Seed metric", &mSeedMetricIndex, metrics, 2);
    itemTooltip(
        "How distance to the seed is measured when ranking solutions: "
        "wrap_linf = largest single-joint angular distance; "
        "wrap_l2 = Euclidean distance across joints.");

    if (ImGui::Button("Solve"))
      solveCurrent();
    itemTooltip("Run the analytical IK once for the current target.");

    ImGui::SameLine();
    if (ImGui::Button("Seed from selection")) {
      if (mSelectedSolution >= 0
          && mSelectedSolution < static_cast<int>(mSolutions.size())) {
        mSeed = mSolutions[mSelectedSolution].q;
        mUseSeed = true;
        if (mAutoSolve)
          solveCurrent();
      }
    }
    itemTooltip(
        "Use the currently selected solution below as the seed configuration "
        "and enable 'Use q seed'.");

    if (changed && mAutoSolve)
      solveCurrent();
  }

  void renderSolutions()
  {
    ImGui::TextWrapped("%s", mStatus.c_str());

    if (mSolutions.empty())
      return;

    const int maxIndex = static_cast<int>(mSolutions.size()) - 1;
    if (mSelectedSolution > maxIndex)
      mSelectedSolution = maxIndex;

    if (mSolutions.size() > 1) {
      if (ImGui::SliderInt("Solution", &mSelectedSolution, 0, maxIndex))
        updateSolvedFrame();
      itemTooltip(
          "Browse the IK solutions: distinct joint configurations that all "
          "reach the same target. The selected one is opaque; the others are "
          "shown translucent.");
    }

    const SsikSolution& solution = mSolutions[mSelectedSolution];
    ImGui::Text("fk residual: %.3e", solution.fkResidual);
    itemTooltip(
        "Forward-kinematics residual: how far this solution's actual "
        "end-effector pose is from the target (smaller is better).");
    ImGui::Text("refinement: %s", solution.refinementUsed.c_str());
    itemTooltip("Whether ssik numerically refined this solution.");
    ImGui::TextWrapped("q: %s", formatVector(solution.q).c_str());
    itemTooltip("Joint angles of this solution in radians, one per DOF.");

    if (solution.hasFkPose) {
      const Eigen::Vector3d& p = solution.fkPose.translation();
      ImGui::Text("FK xyz: %.3f, %.3f, %.3f", p.x(), p.y(), p.z());
      itemTooltip(
          "End-effector position computed by forward kinematics from q; should "
          "match the target position.");
    }
  }

  void loadSelectedArm()
  {
    mLoaded = false;
    mSolutions.clear();
    mSelectedSolution = 0;
    mStatus = "Loading ssik.prebuilt." + mArms[mArmIndex].module;

    std::string error;
    if (!mBridge.loadArm(mArms[mArmIndex].module, mInfo, error)) {
      mStatus = "Install ssik in this Python environment to use this example: "
                + error;
      mInfo = ArmInfo();
      if (mArm)
        mArm->build({});
      setTargetControlsFromTransform(mInfo.home, mTargetXyz, mTargetRpyDegrees);
      syncTargetFrame();
      mSolvedFrame->setTransform(
          offsetPose(mInfo.home, Eigen::Vector3d(0.0, 0.25, 0.0)));
      return;
    }

    mLoaded = true;
    mSeed = Eigen::VectorXd::Zero(mInfo.dof);
    // Rebuild the articulated skeleton for the newly selected arm.
    if (mArm)
      mArm->build(mInfo.chain);
    setTargetControlsFromTransform(mInfo.home, mTargetXyz, mTargetRpyDegrees);
    syncTargetFrame();
    mSolvedFrame->setTransform(mInfo.home);
    solveCurrent();
  }

  void syncTargetFrame()
  {
    mTarget->setTransform(makeTransform(mTargetXyz, mTargetRpyDegrees));
    // Record the panel-driven pose so render()'s drag detection only reacts to
    // direct gizmo dragging, not to our own control edits.
    mAppliedTargetTf = mTarget->getTransform();
  }

  void solveCurrent()
  {
    if (!mLoaded)
      return;

    if (mSeed.size() != mInfo.dof)
      mSeed = Eigen::VectorXd::Zero(mInfo.dof);

    const char* metrics[] = {"wrap_linf", "wrap_l2"};
    std::string error;
    if (!mBridge.solve(
            mTarget->getTransform(),
            mMaxSolutions,
            mUseSeed,
            mSeed,
            mRespectLimits,
            mAllowRefinement,
            metrics[mSeedMetricIndex],
            mSolutions,
            error)) {
      mSolutions.clear();
      mStatus = "Solve failed: " + error;
      if (mArm)
        mArm->showSolutions(mSolutions, -1);
      return;
    }

    std::ostringstream stream;
    stream << "Solutions: " << mSolutions.size();
    if (mMaxSolutions == 0)
      stream << " (all branches)";
    mStatus = stream.str();

    if (!mSolutions.empty())
      mSelectedSolution = std::min<int>(
          mSelectedSolution, static_cast<int>(mSolutions.size()) - 1);
    updateSolvedFrame();
  }

  void updateSolvedFrame()
  {
    if (mArm)
      mArm->showSolutions(mSolutions, mSelectedSolution);

    if (mSelectedSolution < 0
        || mSelectedSolution >= static_cast<int>(mSolutions.size()))
      return;

    const SsikSolution& solution = mSolutions[mSelectedSolution];
    if (solution.hasFkPose)
      mSolvedFrame->setTransform(solution.fkPose);
  }

  dart::gui::osg::ImGuiViewer* mViewer;
  dart::gui::osg::InteractiveFramePtr mTarget;
  dart::dynamics::SimpleFramePtr mSolvedFrame;
  ArmSkeleton* mArm;
  Eigen::Isometry3d mAppliedTargetTf = Eigen::Isometry3d::Identity();
  SsikPythonBridge mBridge;
  std::vector<ArmSpec> mArms;
  int mArmIndex = 0;
  ArmInfo mInfo;
  bool mLoaded = false;
  bool mAutoSolve = true;
  bool mRespectLimits = true;
  bool mAllowRefinement = false;
  bool mUseSeed = false;
  int mMaxSolutions = 0;
  int mSeedMetricIndex = 0;
  int mSelectedSolution = 0;
  std::array<float, 3> mTargetXyz{{0.0f, 0.0f, 0.0f}};
  std::array<float, 3> mTargetRpyDegrees{{0.0f, 0.0f, 0.0f}};
  Eigen::VectorXd mSeed;
  std::vector<SsikSolution> mSolutions;
  std::string mStatus;
};

//==============================================================================
class SsikWorldNode : public dart::gui::osg::WorldNode
{
public:
  explicit SsikWorldNode(const dart::simulation::WorldPtr& world)
    : dart::gui::osg::WorldNode(world)
  {
    // Do nothing
  }
};

//==============================================================================
struct Options
{
  double scale = 1.0;
  bool showHelp = false;
  bool headless = false;
  bool selfTest = false;
  std::string shotPath = "ssik_ik_gui.png";
  int armIndex = 0;
};

//==============================================================================
void printUsage(const char* executable)
{
  dart::gui::osg::printGuiScaleUsage(std::cout, executable);
  std::cout << "\nAdditional options:\n"
            << "  --arm INDEX      Initial arm index to load (default 0).\n"
            << "  --headless       Render one off-screen frame to a PNG and "
               "exit.\n"
            << "  --shot PATH      Output PNG path for --headless (default "
               "ssik_ik_gui.png).\n"
            << "  --self-test      Load and solve every prebuilt arm in turn "
               "(exercises online switching) and exit.\n";
}

//==============================================================================
// Parses example-specific options and delegates the shared GUI scale flags
// (--gui-scale, -h/--help) to the common dart-gui-osg helper.
Options parseOptions(int argc, char* argv[])
{
  Options options;

  std::vector<char*> forwarded;
  forwarded.push_back(argv[0]);
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--headless") == 0) {
      options.headless = true;
    } else if (std::strcmp(argv[i], "--self-test") == 0) {
      options.selfTest = true;
    } else if (std::strcmp(argv[i], "--shot") == 0 && i + 1 < argc) {
      options.shotPath = argv[++i];
    } else if (std::strcmp(argv[i], "--arm") == 0 && i + 1 < argc) {
      options.armIndex = std::atoi(argv[++i]);
    } else {
      forwarded.push_back(argv[i]);
    }
  }

  const dart::gui::osg::GuiScaleOptions gui
      = dart::gui::osg::parseGuiScaleOptions(
          static_cast<int>(forwarded.size()), forwarded.data(), &std::cerr);
  options.scale = gui.scale;
  options.showHelp = gui.showHelp;
  return options;
}

//==============================================================================
// Render one frame off-screen into a pbuffer and write it to a PNG without
// opening a window. Useful for smoke tests / CI and for verifying the arm
// rendering. Returns a process exit code.
int runHeadless(
    dart::gui::osg::ImGuiViewer* viewer,
    const Options& options,
    const ::osg::Vec3& eye,
    const ::osg::Vec3& center,
    const ::osg::Vec3& up)
{
  const int width
      = dart::gui::osg::scaleWindowExtent(kViewerWidth, options.scale);
  const int height
      = dart::gui::osg::scaleWindowExtent(kViewerHeight, options.scale);

  ::osg::ref_ptr<::osg::GraphicsContext::Traits> traits
      = new ::osg::GraphicsContext::Traits;
  traits->readDISPLAY();
  traits->setUndefinedScreenDetailsToDefaultScreen();
  traits->x = 0;
  traits->y = 0;
  traits->width = width;
  traits->height = height;
  traits->red = traits->green = traits->blue = 8;
  traits->alpha = 8;
  traits->depth = 24;
  traits->windowDecoration = false;
  traits->pbuffer = true;
  traits->doubleBuffer = true;

  ::osg::ref_ptr<::osg::GraphicsContext> gc
      = ::osg::GraphicsContext::createGraphicsContext(traits.get());
  if (!gc) {
    std::cerr << "[headless] Failed to create an off-screen GL context "
                 "(no usable DISPLAY?).\n";
    return 1;
  }

  auto* camera = viewer->getCamera();
  camera->setGraphicsContext(gc.get());
  camera->setViewport(new ::osg::Viewport(0, 0, width, height));
  camera->setProjectionMatrixAsPerspective(
      30.0, static_cast<double>(width) / height, 0.1, 1000.0);
  const GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
  camera->setDrawBuffer(buffer);
  camera->setReadBuffer(buffer);

  viewer->setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
  viewer->setCameraManipulator(nullptr);
  viewer->simulate(false);
  camera->setViewMatrixAsLookAt(eye, center, up);

  viewer->realize();
  if (!viewer->isRealized()) {
    std::cerr << "[headless] Viewer failed to realize off-screen.\n";
    return 1;
  }
  if (auto* queue = viewer->getEventQueue()) {
    queue->windowResize(0, 0, width, height);
    queue->setMouseInputRange(0.0f, 0.0f, width, height);
  }

  // A few frames let ImGui build its first frame and the widget solve and pose
  // the arm before the screen capture.
  for (int i = 0; i < 10; ++i) {
    camera->setViewMatrixAsLookAt(eye, center, up);
    viewer->frame();
  }

  viewer->captureScreen(options.shotPath);
  viewer->frame(); // SaveScreen writes the PNG during this frame.

  std::cout << "[headless] wrote " << options.shotPath << " (" << width << "x"
            << height << ")\n";
  return 0;
}

} // namespace

//==============================================================================
int main(int argc, char* argv[])
{
  const Options options = parseOptions(argc, argv);
  if (options.showHelp) {
    printUsage(argv[0]);
    return 0;
  }

  PythonSession python;

  dart::simulation::WorldPtr world(new dart::simulation::World);

  Eigen::Isometry3d defaultTarget = Eigen::Isometry3d::Identity();
  defaultTarget.translation() = Eigen::Vector3d(0.45, 0.0, 0.35);

  // A draggable 6-DOF InteractiveFrame gizmo (translation arrows + rotation
  // rings + planar handles) serves as the IK target, like the atlas_puppet
  // example. Sized so it stays visible where the end effector reaches it.
  dart::gui::osg::InteractiveFramePtr target(
      new dart::gui::osg::InteractiveFrame(
          dart::dynamics::Frame::World(), "ssik_target", defaultTarget, 0.25));
  world->addSimpleFrame(target);

  // The solved end-effector pose is an output indicator (not interactive), so a
  // plain cyan sphere marker rather than a second draggable gizmo.
  dart::dynamics::SimpleFramePtr solvedFrame(new dart::dynamics::SimpleFrame(
      dart::dynamics::Frame::World(),
      "ssik_fk_solution",
      offsetPose(defaultTarget, Eigen::Vector3d(0.0, 0.25, 0.0))));
  solvedFrame->setShape(std::make_shared<dart::dynamics::SphereShape>(0.05));
  solvedFrame->createVisualAspect()->setColor(
      Eigen::Vector4d(0.15, 0.85, 1.0, 0.85));
  world->addSimpleFrame(solvedFrame);

  ArmSkeleton arm;
  arm.setWorld(world);

  osg::ref_ptr<SsikWorldNode> node = new SsikWorldNode(world);

  osg::ref_ptr<dart::gui::osg::ImGuiViewer> viewer
      = new dart::gui::osg::ImGuiViewer();
  // Render on the calling (main) thread. The embedded Python interpreter is
  // initialized on this thread and keeps the GIL, so widget rendering -- which
  // calls back into Python to solve and to switch arms -- must also run here.
  // With OSG's default multi-threaded model the draw thread would block forever
  // trying to acquire the GIL, freezing the UI on the first solve/arm switch.
  viewer->setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
  viewer->addWorldNode(node);
  viewer->getImGuiHandler()->setGuiScale(options.scale);

  auto widget = std::make_shared<SsikIkWidget>(
      viewer.get(), target, solvedFrame, &arm, options.armIndex);
  viewer->getImGuiHandler()->addWidget(widget);

  if (options.selfTest) {
    int notLoaded = 0;
    long long totalSolutions = 0;
    for (int i = 0; i < widget->armCount(); ++i) {
      bool loaded = false;
      int solutionCount = 0;
      std::cout << "[self-test] "
                << widget->selectArmForTest(i, loaded, solutionCount)
                << (loaded ? "" : "  [FAIL]") << "\n";
      if (!loaded)
        ++notLoaded;
      totalSolutions += solutionCount;
    }
    if (notLoaded > 0) {
      std::cout << "[self-test] " << notLoaded << " of " << widget->armCount()
                << " arms failed to load (is ssik installed?)\n";
      return 1;
    }
    if (totalSolutions == 0) {
      std::cout << "[self-test] no arm produced any IK solution; the solver "
                   "integration looks broken\n";
      return 1;
    }
    std::cout << "[self-test] all " << widget->armCount() << " arms loaded ("
              << totalSolutions << " total solutions)\n";
    return 0;
  }

  viewer->enableDragAndDrop(target.get());
  viewer->addInstructionText(
      "\nRed arm: the selected IK solution. Yellow marker: target transform. "
      "Cyan marker: solved end-effector.\n");
  viewer->addInstructionText(
      "Install optional ssik with: pixi run python -m pip install ssik\n");
  viewer->addInstructionText(
      "Use --gui-scale <value> to scale the ImGui panel and viewer window.\n");
  std::cout << viewer->getInstructions() << std::endl;

  osg::ref_ptr<dart::gui::osg::GridVisual> grid
      = new dart::gui::osg::GridVisual();
  grid->setPlaneType(dart::gui::osg::GridVisual::PlaneType::XY);
  viewer->addAttachment(grid);

  // Framed to fit the prebuilt arms, whose reach at home ranges from ~0.4 m
  // (compact 6R) up to ~1.4 m (tall 7R such as iiwa14/Rizon10).
  const ::osg::Vec3 eye(2.6f, -3.0f, 1.6f);
  const ::osg::Vec3 center(0.1f, 0.0f, 0.55f);
  const ::osg::Vec3 up(0.0f, 0.0f, 1.0f);

  if (options.headless)
    return runHeadless(viewer.get(), options, eye, center, up);

  viewer->setUpViewInWindow(
      0,
      0,
      dart::gui::osg::scaleWindowExtent(kViewerWidth, options.scale),
      dart::gui::osg::scaleWindowExtent(kViewerHeight, options.scale));
  viewer->getCameraManipulator()->setHomePosition(eye, center, up);
  viewer->setCameraManipulator(viewer->getCameraManipulator());

  viewer->run();
}
