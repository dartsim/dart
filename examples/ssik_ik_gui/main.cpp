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

#include <dart/gui/osg/osg.hpp>

#include <dart/dart.hpp>

#include <Python.h>

#include <algorithm>
#include <array>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <cmath>

namespace {

constexpr double pi()
{
  return 3.14159265358979323846;
}

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
struct ArmInfo
{
  std::string baseLink;
  std::string eeLink;
  int dof = 0;
  Eigen::Isometry3d home = Eigen::Isometry3d::Identity();
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
const char* kBridgeScript = R"PY(
import importlib

class DARTSsikBridge:
    def __init__(self):
        self.module = None

    def _flat(self, matrix):
        return [float(matrix[r][c]) for r in range(4) for c in range(4)]

    def load(self, module_name):
        self.module = importlib.import_module("ssik.prebuilt." + module_name)
        return {
            "BASE_LINK": str(getattr(self.module, "BASE_LINK", "")),
            "EE_LINK": str(getattr(self.module, "EE_LINK", "")),
            "DOF": int(getattr(self.module, "DOF", 0)),
            "T_HOME": self._flat(getattr(self.module, "T_HOME")),
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
        import numpy as np
        T = np.asarray(flat_target, dtype=np.float64).reshape((4, 4))
        kwargs = {
            "max_solutions": None if int(max_solutions) <= 0 else int(max_solutions),
            "respect_limits": bool(respect_limits),
            "allow_refinement": bool(allow_refinement),
            "seed_metric": str(seed_metric),
        }
        if use_seed:
            kwargs["q_seed"] = np.asarray(seed, dtype=np.float64)
        solutions = self.module.solve(T, **kwargs)
        out = []
        has_fk = hasattr(self.module, "fk")
        for solution in solutions:
            q = np.asarray(solution.q, dtype=np.float64)
            fk_pose = None
            if has_fk:
                fk_pose = self._flat(self.module.fk(q))
            out.append({
                "q": q.tolist(),
                "fk_residual": float(getattr(solution, "fk_residual", float("nan"))),
                "refinement_used": str(getattr(solution, "refinement_used", "")),
                "fk_pose": fk_pose,
            })
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
class SsikPythonBridge
{
public:
  SsikPythonBridge()
  {
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

    PyObject* home = PyDict_GetItemString(result.get(), "T_HOME");
    if (!home || !parseMatrix(home, info.home, error))
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
class SsikIkWidget : public dart::gui::osg::ImGuiWidget
{
public:
  SsikIkWidget(
      dart::gui::osg::ImGuiViewer* viewer,
      dart::gui::osg::InteractiveFramePtr target,
      dart::gui::osg::InteractiveFramePtr solvedFrame)
    : mViewer(viewer),
      mTarget(std::move(target)),
      mSolvedFrame(std::move(solvedFrame)),
      mArms(getArmSpecs())
  {
    loadSelectedArm();
  }

  void render() override
  {
    ImGui::SetNextWindowPos(ImVec2(10, 20), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(430, 640), ImGuiCond_FirstUseEver);

    if (!ImGui::Begin(
            "ssik Analytical IK", nullptr, ImGuiWindowFlags_MenuBar)) {
      ImGui::End();
      return;
    }

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

    ImGui::End();
  }

private:
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

    if (mLoaded) {
      ImGui::Text("Module: ssik.prebuilt.%s", mArms[mArmIndex].module.c_str());
      ImGui::Text(
          "Chain: %s -> %s", mInfo.baseLink.c_str(), mInfo.eeLink.c_str());
      ImGui::Text("DOF: %d", mInfo.dof);
    } else {
      ImGui::TextWrapped("%s", mStatus.c_str());
    }
  }

  void renderTargetControls()
  {
    bool changed = false;
    changed |= ImGui::DragFloat3("Position", mTargetXyz.data(), 0.005f);
    changed |= ImGui::DragFloat3("RPY deg", mTargetRpyDegrees.data(), 0.5f);

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
    ImGui::SameLine();
    if (ImGui::Button("Read dragged")) {
      setTargetControlsFromTransform(
          mTarget->getTransform(), mTargetXyz, mTargetRpyDegrees);
      if (mAutoSolve)
        solveCurrent();
    }
  }

  void renderSolverControls()
  {
    bool changed = false;
    changed |= ImGui::Checkbox("Auto solve", &mAutoSolve);
    changed |= ImGui::Checkbox("Respect limits", &mRespectLimits);
    changed |= ImGui::Checkbox("Allow refinement", &mAllowRefinement);
    changed |= ImGui::Checkbox("Use q seed", &mUseSeed);
    changed |= ImGui::SliderInt("Max solutions", &mMaxSolutions, 0, 256);

    const char* metrics[] = {"wrap_linf", "wrap_l2"};
    changed |= ImGui::Combo("Seed metric", &mSeedMetricIndex, metrics, 2);

    if (ImGui::Button("Solve"))
      solveCurrent();

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
    }

    const SsikSolution& solution = mSolutions[mSelectedSolution];
    ImGui::Text("fk residual: %.3e", solution.fkResidual);
    ImGui::Text("refinement: %s", solution.refinementUsed.c_str());
    ImGui::TextWrapped("q: %s", formatVector(solution.q).c_str());

    if (solution.hasFkPose) {
      const Eigen::Vector3d& p = solution.fkPose.translation();
      ImGui::Text("FK xyz: %.3f, %.3f, %.3f", p.x(), p.y(), p.z());
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
      return;
    }

    mLoaded = true;
    mSeed = Eigen::VectorXd::Zero(mInfo.dof);
    setTargetControlsFromTransform(mInfo.home, mTargetXyz, mTargetRpyDegrees);
    syncTargetFrame();
    solveCurrent();
  }

  void syncTargetFrame()
  {
    mTarget->setTransform(makeTransform(mTargetXyz, mTargetRpyDegrees));
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
      return;
    }

    std::ostringstream stream;
    stream << "Solutions: " << mSolutions.size();
    if (mMaxSolutions == 0)
      stream << " (all branches)";
    mStatus = stream.str();

    if (!mSolutions.empty()) {
      mSelectedSolution = std::min<int>(
          mSelectedSolution, static_cast<int>(mSolutions.size()) - 1);
      updateSolvedFrame();
    }
  }

  void updateSolvedFrame()
  {
    if (mSelectedSolution < 0
        || mSelectedSolution >= static_cast<int>(mSolutions.size()))
      return;

    const SsikSolution& solution = mSolutions[mSelectedSolution];
    if (solution.hasFkPose)
      mSolvedFrame->setTransform(solution.fkPose);
  }

  osg::ref_ptr<dart::gui::osg::ImGuiViewer> mViewer;
  dart::gui::osg::InteractiveFramePtr mTarget;
  dart::gui::osg::InteractiveFramePtr mSolvedFrame;
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

} // namespace

//==============================================================================
int main()
{
  PythonSession python;

  dart::simulation::WorldPtr world(new dart::simulation::World);

  dart::gui::osg::InteractiveFramePtr target(
      new dart::gui::osg::InteractiveFrame(dart::dynamics::Frame::World()));
  target->setName("ssik_target");
  world->addSimpleFrame(target);

  dart::gui::osg::InteractiveFramePtr solvedFrame(
      new dart::gui::osg::InteractiveFrame(dart::dynamics::Frame::World()));
  solvedFrame->setName("ssik_fk_solution");
  world->addSimpleFrame(solvedFrame);

  osg::ref_ptr<SsikWorldNode> node = new SsikWorldNode(world);

  osg::ref_ptr<dart::gui::osg::ImGuiViewer> viewer
      = new dart::gui::osg::ImGuiViewer();
  viewer->addWorldNode(node);

  viewer->getImGuiHandler()->addWidget(
      std::make_shared<SsikIkWidget>(viewer, target, solvedFrame));

  viewer->enableDragAndDrop(target.get());

  osg::ref_ptr<dart::gui::osg::GridVisual> grid
      = new dart::gui::osg::GridVisual();
  grid->setPlaneType(dart::gui::osg::GridVisual::PlaneType::XY);
  viewer->addAttachment(grid);

  viewer->setUpViewInWindow(0, 0, 980, 720);
  viewer->getCameraManipulator()->setHomePosition(
      ::osg::Vec3(2.5f, -3.0f, 1.8f),
      ::osg::Vec3(0.0f, 0.0f, 0.5f),
      ::osg::Vec3(0.0f, 0.0f, 1.0f));
  viewer->setCameraManipulator(viewer->getCameraManipulator());

  viewer->run();
}
