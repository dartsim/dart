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

#include "SsikBridge.hpp"

namespace dart_demos {
namespace ssik_ik_gui {

namespace {

//==============================================================================
// Python bridge to the optional ssik package. It loads a prebuilt analytical
// IK module, solves for a target pose, and also reports the per-joint world
// frames of each solution (computed from the module's product-of-exponentials
// chain) so the scene can render the articulated arm rather than just a
// marker. Copied verbatim from examples/ssik_ik_gui/main.cpp.
const char* const kBridgeScript = R"PY(
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

} // namespace

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
SsikPythonBridge::SsikPythonBridge()
{
  ensurePythonInitialized();
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

//==============================================================================
bool SsikPythonBridge::loadArm(
    const std::string& moduleName, ArmInfo& info, std::string& error)
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

//==============================================================================
bool SsikPythonBridge::solve(
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
      || !respectLimitsObject || !allowRefinementObject || !seedMetricObject) {
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

} // namespace ssik_ik_gui
} // namespace dart_demos
