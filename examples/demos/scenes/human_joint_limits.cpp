/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License.
 */

#include "scenes.hpp"

#include <dart/config.hpp>

#include <dart/gui/panel.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/simulation/world.hpp>

#include <dart/constraint/constraint_base.hpp>
#include <dart/constraint/constraint_solver.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/joint.hpp>
#include <dart/dynamics/shape_frame.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <dart/math/constants.hpp>

#include <dart/common/uri.hpp>

#include <dart/io/read.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <fstream>
#include <iterator>
#include <limits>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace dart::examples::demos {

namespace {

constexpr const char* kGroundSkeletonName = "human_joint_limits_ground";
constexpr const char* kHumanSkeletonName = "human_joint_limits_human";

constexpr double kErrorAllowance = 0.0;
constexpr double kErrorReductionParameter = 0.01;
constexpr double kMaxErrorReductionVelocity = 1e+1;
constexpr double kConstraintForceMixing = 1e-9;

class BinaryReader
{
public:
  explicit BinaryReader(const std::string& path)
  {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
      throw std::runtime_error("Failed to open neural network file: " + path);
    }

    mBytes.assign(
        std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
  }

  std::uint64_t readUint64()
  {
    return readPod<std::uint64_t>();
  }

  bool readBool()
  {
    if (mOffset >= mBytes.size()) {
      throw std::runtime_error("Unexpected end of neural network file");
    }
    return mBytes[mOffset++] != 0;
  }

  std::string readString()
  {
    const std::uint64_t size = readUint64();
    if (size > remaining()) {
      throw std::runtime_error("Invalid string length in neural network file");
    }

    std::string value(
        reinterpret_cast<const char*>(mBytes.data() + mOffset),
        static_cast<std::size_t>(size));
    mOffset += static_cast<std::size_t>(size);
    return value;
  }

  std::vector<double> readFloatVector()
  {
    const std::uint64_t size = readUint64();
    if (size > remaining() / sizeof(float)) {
      throw std::runtime_error("Invalid vector length in neural network file");
    }

    std::vector<double> values(static_cast<std::size_t>(size));
    for (double& value : values) {
      value = static_cast<double>(readPod<float>());
    }
    return values;
  }

  void expectEnd() const
  {
    if (mOffset != mBytes.size()) {
      throw std::runtime_error("Trailing bytes in neural network file");
    }
  }

private:
  std::size_t remaining() const
  {
    return mBytes.size() - mOffset;
  }

  template <typename T>
  T readPod()
  {
    if (sizeof(T) > remaining()) {
      throw std::runtime_error("Unexpected end of neural network file");
    }

    T value;
    std::memcpy(&value, mBytes.data() + mOffset, sizeof(T));
    mOffset += sizeof(T);
    return value;
  }

  std::vector<unsigned char> mBytes;
  std::size_t mOffset = 0;
};

struct NetworkLayer
{
  std::string type;
  std::size_t inputSize = 0;
  std::size_t outputSize = 0;
  bool hasBias = false;
  std::vector<double> weights;
  std::vector<double> biases;
};

class TinyDnnSequentialNetwork
{
public:
  explicit TinyDnnSequentialNetwork(const std::string& path)
  {
    BinaryReader reader(path);
    const std::uint64_t layerCount = reader.readUint64();
    mLayers.reserve(static_cast<std::size_t>(layerCount));

    for (std::uint64_t i = 0; i < layerCount; ++i) {
      NetworkLayer layer;
      layer.type = reader.readString();
      if (layer.type == "fully_connected") {
        layer.inputSize = static_cast<std::size_t>(reader.readUint64());
        layer.outputSize = static_cast<std::size_t>(reader.readUint64());
        layer.hasBias = reader.readBool();
      } else if (layer.type == "tanh" || layer.type == "sigmoid") {
        layer.inputSize = static_cast<std::size_t>(reader.readUint64());
        layer.outputSize = layer.inputSize;
        const auto channelCount = reader.readUint64();
        const auto phase = reader.readUint64();
        if (channelCount != 1 || phase != 1) {
          throw std::runtime_error(
              "Unsupported activation metadata in neural network file");
        }
      } else {
        throw std::runtime_error(
            "Unsupported neural network layer type: " + layer.type);
      }
      mLayers.push_back(std::move(layer));
    }

    for (NetworkLayer& layer : mLayers) {
      const std::string payloadType = reader.readString();
      if (payloadType != layer.type) {
        throw std::runtime_error("Neural network payload order mismatch");
      }

      if (layer.type != "fully_connected") {
        continue;
      }

      layer.weights = reader.readFloatVector();
      if (layer.weights.size() != layer.inputSize * layer.outputSize) {
        throw std::runtime_error(
            "Unexpected fully-connected weight count in neural network file");
      }

      if (layer.hasBias) {
        layer.biases = reader.readFloatVector();
        if (layer.biases.size() != layer.outputSize) {
          throw std::runtime_error(
              "Unexpected fully-connected bias count in neural network file");
        }
      } else {
        layer.biases.assign(layer.outputSize, 0.0);
      }
    }

    reader.expectEnd();
  }

  std::pair<double, std::vector<double>> predictWithInputGradient(
      std::span<const double> input) const
  {
    std::vector<std::vector<double>> activations;
    activations.reserve(mLayers.size() + 1);
    activations.emplace_back(input.begin(), input.end());

    for (const NetworkLayer& layer : mLayers) {
      const std::vector<double>& previous = activations.back();
      if (previous.size() != layer.inputSize) {
        throw std::runtime_error(
            "Neural network layer input size mismatch during prediction");
      }

      std::vector<double> current(layer.outputSize, 0.0);
      if (layer.type == "fully_connected") {
        current = layer.biases;
        for (std::size_t i = 0; i < layer.inputSize; ++i) {
          for (std::size_t j = 0; j < layer.outputSize; ++j) {
            current[j] += previous[i] * layer.weights[i * layer.outputSize + j];
          }
        }
      } else if (layer.type == "tanh") {
        for (std::size_t i = 0; i < layer.outputSize; ++i) {
          current[i] = std::tanh(previous[i]);
        }
      } else if (layer.type == "sigmoid") {
        for (std::size_t i = 0; i < layer.outputSize; ++i) {
          current[i] = 1.0 / (1.0 + std::exp(-previous[i]));
        }
      }

      activations.push_back(std::move(current));
    }

    if (activations.back().size() != 1) {
      throw std::runtime_error("Expected scalar neural network output");
    }

    std::vector<double> gradient(1, 1.0);
    for (std::size_t layerIndex = mLayers.size(); layerIndex > 0;
         --layerIndex) {
      const NetworkLayer& layer = mLayers[layerIndex - 1];
      const std::vector<double>& output = activations[layerIndex];

      if (layer.type == "tanh") {
        for (std::size_t i = 0; i < gradient.size(); ++i) {
          gradient[i] *= 1.0 - output[i] * output[i];
        }
      } else if (layer.type == "sigmoid") {
        for (std::size_t i = 0; i < gradient.size(); ++i) {
          gradient[i] *= output[i] * (1.0 - output[i]);
        }
      } else if (layer.type == "fully_connected") {
        std::vector<double> inputGradient(layer.inputSize, 0.0);
        for (std::size_t i = 0; i < layer.inputSize; ++i) {
          for (std::size_t j = 0; j < layer.outputSize; ++j) {
            inputGradient[i]
                += layer.weights[i * layer.outputSize + j] * gradient[j];
          }
        }
        gradient = std::move(inputGradient);
      }
    }

    return {activations.back()[0], std::move(gradient)};
  }

private:
  std::vector<NetworkLayer> mLayers;
};

using NetworkPtr = std::shared_ptr<const TinyDnnSequentialNetwork>;

void validateJoint(
    const dart::dynamics::Joint* joint,
    std::string_view name,
    std::size_t expectedDofs)
{
  if (joint == nullptr) {
    throw std::runtime_error(
        "human_joint_limits world is missing joint: " + std::string(name));
  }
  if (joint->getNumDofs() != expectedDofs) {
    throw std::runtime_error(
        "human_joint_limits joint " + std::string(name)
        + " has unexpected DOF count");
  }
}

void writeJointImpulses(
    const std::vector<std::pair<dart::dynamics::Joint*, std::size_t>>& dofs,
    const std::vector<double>& values)
{
  for (std::size_t i = 0; i < dofs.size(); ++i) {
    dofs[i].first->setConstraintImpulse(dofs[i].second, values[i]);
  }
}

void clearJointImpulses(
    const std::vector<std::pair<dart::dynamics::Joint*, std::size_t>>& dofs)
{
  for (const auto& dof : dofs) {
    dof.first->setConstraintImpulse(dof.second, 0.0);
  }
}

void addJointImpulses(
    const std::vector<std::pair<dart::dynamics::Joint*, std::size_t>>& dofs,
    const std::vector<double>& values,
    double scale)
{
  for (std::size_t i = 0; i < dofs.size(); ++i) {
    auto* joint = dofs[i].first;
    const std::size_t index = dofs[i].second;
    joint->setConstraintImpulse(
        index, joint->getConstraintImpulse(index) + values[i] * scale);
  }
}

double dotVelocity(
    const std::vector<std::pair<dart::dynamics::Joint*, std::size_t>>& dofs,
    const std::vector<double>& jacobian)
{
  double value = 0.0;
  for (std::size_t i = 0; i < dofs.size(); ++i) {
    value += jacobian[i] * dofs[i].first->getVelocity(dofs[i].second);
  }
  return value;
}

double dotVelocityChange(
    const std::vector<std::pair<dart::dynamics::Joint*, std::size_t>>& dofs,
    const std::vector<double>& jacobian)
{
  double value = 0.0;
  for (std::size_t i = 0; i < dofs.size(); ++i) {
    value += jacobian[i] * dofs[i].first->getVelocityChange(dofs[i].second);
  }
  return value;
}

class NeuralJointLimitConstraint : public dart::constraint::ConstraintBase
{
public:
  NeuralJointLimitConstraint(
      std::vector<std::pair<dart::dynamics::Joint*, std::size_t>> dofs,
      dart::dynamics::BodyNode* terminalBody,
      NetworkPtr network,
      bool mirror)
    : mDofs(std::move(dofs)),
      mTerminalBody(terminalBody),
      mNetwork(std::move(network)),
      mMirror(mirror),
      mJacobian(mDofs.size(), 0.0)
  {
    if (mDofs.empty() || mTerminalBody == nullptr || mNetwork == nullptr) {
      throw std::runtime_error(
          "Invalid human joint limit constraint configuration");
    }
  }

  void getInformation(dart::constraint::ConstraintInfo* lcp) override
  {
    const double bouncingVelocity = std::clamp(
        (-mViolation - kErrorAllowance) * lcp->invTimeStep
            * kErrorReductionParameter,
        0.0,
        kMaxErrorReductionVelocity);

    lcp->b[0] = mNegativeVelocity + bouncingVelocity;
    lcp->lo[0] = 0.0;
    lcp->hi[0] = std::numeric_limits<double>::infinity();
    lcp->x[0] = mLifeTime == 0 ? 0.0 : mOldImpulse;
  }

  void applyUnitImpulse(std::size_t index) override
  {
    if (index >= mDim) {
      throw std::runtime_error("Invalid human joint limit impulse index");
    }

    const auto skeleton = mDofs.front().first->getSkeleton();
    skeleton->clearConstraintImpulses();

    writeJointImpulses(mDofs, mJacobian);
    skeleton->updateBiasImpulse(mTerminalBody);
    skeleton->updateVelocityChange();
    clearJointImpulses(mDofs);

    mAppliedImpulseIndex = index;
  }

  void getVelocityChange(double* delVel, bool withCfm) override
  {
    if (delVel == nullptr) {
      throw std::runtime_error("Null velocity-change buffer");
    }

    const auto skeleton = mDofs.front().first->getSkeleton();
    delVel[0] = skeleton->isImpulseApplied()
                    ? dotVelocityChange(mDofs, mJacobian)
                    : 0.0;
    if (withCfm) {
      delVel[mAppliedImpulseIndex]
          += delVel[mAppliedImpulseIndex] * kConstraintForceMixing;
    }
  }

  void excite() override
  {
    mDofs.front().first->getSkeleton()->setImpulseApplied(true);
  }

  void unexcite() override
  {
    mDofs.front().first->getSkeleton()->setImpulseApplied(false);
  }

  void applyImpulse(double* lambda) override
  {
    mOldImpulse = lambda[0];
    addJointImpulses(mDofs, mJacobian, lambda[0]);
  }

  dart::dynamics::SkeletonPtr getRootSkeleton() const override
  {
    return dart::constraint::ConstraintBase::getRootSkeleton(
        mDofs.front().first->getSkeleton()->getSkeleton());
  }

  bool isActive() const override
  {
    return mActive;
  }

protected:
  void activateIfViolated(
      double networkOutput, std::vector<double> jacobian, double velocity)
  {
    const bool wasActive = mActive;
    mViolation = networkOutput - 0.5;
    mActive = false;
    mDim = 0;

    if (mViolation > 0.0) {
      return;
    }

    mActive = true;
    mLifeTime = wasActive ? mLifeTime + 1 : 0;
    mJacobian = std::move(jacobian);
    mNegativeVelocity = -velocity;
    mDim = 1;
  }

  const std::vector<std::pair<dart::dynamics::Joint*, std::size_t>>& getDofs()
      const
  {
    return mDofs;
  }

  const NetworkPtr& getNetwork() const
  {
    return mNetwork;
  }

  bool isMirror() const
  {
    return mMirror;
  }

private:
  std::vector<std::pair<dart::dynamics::Joint*, std::size_t>> mDofs;
  dart::dynamics::BodyNode* mTerminalBody = nullptr;
  NetworkPtr mNetwork;
  bool mMirror = false;
  std::vector<double> mJacobian;
  std::size_t mAppliedImpulseIndex = 0;
  std::size_t mLifeTime = 0;
  double mViolation = 0.0;
  double mNegativeVelocity = 0.0;
  double mOldImpulse = 0.0;
  bool mActive = false;
};

class HumanArmJointLimitConstraint final : public NeuralJointLimitConstraint
{
public:
  HumanArmJointLimitConstraint(
      dart::dynamics::Joint* shoulder,
      dart::dynamics::Joint* elbow,
      bool mirror,
      NetworkPtr network)
    : NeuralJointLimitConstraint(
          {{shoulder, 0}, {shoulder, 1}, {shoulder, 2}, {elbow, 0}},
          elbow->getChildBodyNode(),
          std::move(network),
          mirror),
      mShoulder(shoulder),
      mElbow(elbow)
  {
  }

  std::string_view getType() const override
  {
    return "HumanArmJointLimitConstraint";
  }

  void update() override
  {
    double qz = mShoulder->getPosition(0);
    const double qx = mShoulder->getPosition(1);
    double qy = mShoulder->getPosition(2);
    const double qe = mElbow->getPosition(0);

    if (isMirror()) {
      qz = -qz;
      qy = -qy;
    }

    const std::vector<double> input
        = {std::cos(qz),
           std::sin(qz),
           std::cos(qx),
           std::sin(qx),
           std::cos(qy + 2.0 * dart::math::pi / 3.0),
           std::cos(qe)};
    auto [networkOutput, inputGradient]
        = getNetwork()->predictWithInputGradient(input);

    std::vector<double> jacobian(4, 0.0);
    jacobian[0]
        = inputGradient[0] * (-std::sin(qz)) + inputGradient[1] * std::cos(qz);
    jacobian[1]
        = inputGradient[2] * (-std::sin(qx)) + inputGradient[3] * std::cos(qx);
    jacobian[2]
        = inputGradient[4] * (-std::sin(qy + 2.0 * dart::math::pi / 3.0));
    jacobian[3] = inputGradient[5] * (-std::sin(qe));

    if (isMirror()) {
      jacobian[0] = -jacobian[0];
      jacobian[2] = -jacobian[2];
    }

    const double velocity = dotVelocity(getDofs(), jacobian);
    activateIfViolated(networkOutput, std::move(jacobian), velocity);
  }

private:
  dart::dynamics::Joint* mShoulder = nullptr;
  dart::dynamics::Joint* mElbow = nullptr;
};

class HumanLegJointLimitConstraint final : public NeuralJointLimitConstraint
{
public:
  HumanLegJointLimitConstraint(
      dart::dynamics::Joint* hip,
      dart::dynamics::Joint* knee,
      dart::dynamics::Joint* ankle,
      bool mirror,
      NetworkPtr network)
    : NeuralJointLimitConstraint(
          {{hip, 0}, {hip, 1}, {hip, 2}, {knee, 0}, {ankle, 0}, {ankle, 1}},
          ankle->getChildBodyNode(),
          std::move(network),
          mirror),
      mHip(hip),
      mKnee(knee),
      mAnkle(ankle)
  {
  }

  std::string_view getType() const override
  {
    return "HumanLegJointLimitConstraint";
  }

  void update() override
  {
    double qz = mHip->getPosition(0);
    const double qx = mHip->getPosition(1);
    double qy = mHip->getPosition(2);
    const double qe = mKnee->getPosition(0);
    const double hx = mAnkle->getPosition(0);
    const double hy = mAnkle->getPosition(1);

    if (isMirror()) {
      qz = -qz;
      qy = -qy;
    }

    const std::vector<double> input
        = {std::cos(qz),
           std::sin(qz),
           std::cos(qx),
           std::sin(qx),
           std::cos(qy + dart::math::half_pi),
           std::cos(qe),
           std::cos(hx + dart::math::half_pi),
           std::cos(hy + dart::math::half_pi)};
    auto [networkOutput, inputGradient]
        = getNetwork()->predictWithInputGradient(input);

    std::vector<double> jacobian(6, 0.0);
    jacobian[0]
        = inputGradient[0] * (-std::sin(qz)) + inputGradient[1] * std::cos(qz);
    jacobian[1]
        = inputGradient[2] * (-std::sin(qx)) + inputGradient[3] * std::cos(qx);
    jacobian[2] = inputGradient[4] * (-std::sin(qy + dart::math::half_pi));
    jacobian[3] = inputGradient[5] * (-std::sin(qe));
    jacobian[4] = inputGradient[6] * (-std::sin(hx + dart::math::half_pi));
    jacobian[5] = inputGradient[7] * (-std::sin(hy + dart::math::half_pi));

    if (isMirror()) {
      jacobian[0] = -jacobian[0];
      jacobian[2] = -jacobian[2];
    }

    const double velocity = dotVelocity(getDofs(), jacobian);
    activateIfViolated(networkOutput, std::move(jacobian), velocity);
  }

private:
  dart::dynamics::Joint* mHip = nullptr;
  dart::dynamics::Joint* mKnee = nullptr;
  dart::dynamics::Joint* mAnkle = nullptr;
};

dart::dynamics::Joint* getRequiredJoint(
    const dart::dynamics::SkeletonPtr& skeleton,
    std::string_view name,
    std::size_t expectedDofs)
{
  auto* joint = skeleton->getJoint(std::string(name));
  validateJoint(joint, name, expectedDofs);
  return joint;
}

std::size_t installHumanJointLimitConstraints(
    dart::simulation::World& world, const dart::dynamics::SkeletonPtr& human)
{
  if (human == nullptr) {
    throw std::runtime_error("human_joint_limits world is missing human");
  }
  auto* solver = world.getConstraintSolver();
  if (solver == nullptr) {
    throw std::runtime_error("human_joint_limits world is missing solver");
  }

  const NetworkPtr armNetwork = std::make_shared<TinyDnnSequentialNetwork>(
      dart::config::dataPath("humanJointLimits/neuralnets/net-larm"));
  const NetworkPtr legNetwork = std::make_shared<TinyDnnSequentialNetwork>(
      dart::config::dataPath("humanJointLimits/neuralnets/net-lleg"));

  auto* leftShoulder = getRequiredJoint(human, "j_bicep_left", 3);
  auto* leftElbow = getRequiredJoint(human, "j_forearm_left", 1);
  solver->addConstraint(
      std::make_shared<HumanArmJointLimitConstraint>(
          leftShoulder, leftElbow, false, armNetwork));

  auto* rightShoulder = getRequiredJoint(human, "j_bicep_right", 3);
  auto* rightElbow = getRequiredJoint(human, "j_forearm_right", 1);
  solver->addConstraint(
      std::make_shared<HumanArmJointLimitConstraint>(
          rightShoulder, rightElbow, true, armNetwork));

  auto* leftHip = getRequiredJoint(human, "j_thigh_left", 3);
  auto* leftKnee = getRequiredJoint(human, "j_shin_left", 1);
  auto* leftAnkle = getRequiredJoint(human, "j_heel_left", 2);
  solver->addConstraint(
      std::make_shared<HumanLegJointLimitConstraint>(
          leftHip, leftKnee, leftAnkle, false, legNetwork));

  auto* rightHip = getRequiredJoint(human, "j_thigh_right", 3);
  auto* rightKnee = getRequiredJoint(human, "j_shin_right", 1);
  auto* rightAnkle = getRequiredJoint(human, "j_heel_right", 2);
  solver->addConstraint(
      std::make_shared<HumanLegJointLimitConstraint>(
          rightHip, rightKnee, rightAnkle, true, legNetwork));

  return 4;
}

struct HumanJointLimitsScene
{
  dart::simulation::WorldPtr world;
  std::size_t customConstraintCount = 0;
};

void makeStaticGround(const dart::dynamics::SkeletonPtr& skeleton)
{
  if (skeleton == nullptr) {
    return;
  }

  skeleton->setMobile(false);
  for (std::size_t i = 0; i < skeleton->getNumBodyNodes(); ++i) {
    auto* body = skeleton->getBodyNode(i);
    if (body == nullptr) {
      continue;
    }
    body->setGravityMode(false);
  }
}

void colorHuman(const dart::dynamics::SkeletonPtr& human)
{
  const std::size_t numBodies = human->getNumBodyNodes();
  for (std::size_t i = 0; i < numBodies; ++i) {
    auto* body = human->getBodyNode(i);
    if (body == nullptr) {
      continue;
    }
    const double t = numBodies <= 1 ? 0.0
                                    : static_cast<double>(i)
                                          / static_cast<double>(numBodies - 1);
    body->setColor(
        Eigen::Vector3d(0.68 + 0.10 * t, 0.52 + 0.10 * t, 0.38 + 0.08 * t));
  }

  if (auto* pelvis = human->getBodyNode("pelvis")) {
    pelvis->setColor(Eigen::Vector3d(0.42, 0.50, 0.66));
  }
  if (auto* thorax = human->getBodyNode("thorax")) {
    thorax->setColor(Eigen::Vector3d(0.36, 0.54, 0.70));
  }
  if (auto* head = human->getBodyNode("head")) {
    head->setColor(Eigen::Vector3d(0.84, 0.68, 0.52));
  }
}

HumanJointLimitsScene createHumanJointLimitsScene()
{
  const dart::common::Uri worldUri = dart::common::Uri::createFromPath(
      dart::config::dataPath("skel/kima/kima_human_edited.skel"));
  auto world = dart::io::readWorld(worldUri);
  if (world == nullptr) {
    throw std::runtime_error(
        "Failed to load human_joint_limits world from " + worldUri.toString());
  }

  auto ground = world->getSkeleton("ground skeleton");
  if (ground == nullptr) {
    throw std::runtime_error("human_joint_limits world is missing ground");
  }
  ground->setName(kGroundSkeletonName);
  makeStaticGround(ground);
  if (auto* body = ground->getBodyNode("ground")) {
    body->setColor(Eigen::Vector3d(0.58, 0.62, 0.60));
  }

  auto human = world->getSkeleton("human");
  if (human == nullptr) {
    throw std::runtime_error("human_joint_limits world is missing human");
  }
  human->setName(kHumanSkeletonName);
  human->setMobile(true);
  human->eachJoint(
      [](dart::dynamics::Joint* joint) { joint->setLimitEnforcement(true); });
  const std::size_t customConstraintCount
      = installHumanJointLimitConstraints(*world, human);

  colorHuman(human);
  return {world, customConstraintCount};
}

dart::gui::Panel createHumanJointLimitsPanel(std::size_t customConstraintCount)
{
  dart::gui::Panel panel;
  panel.title = "Human Joint Limits";
  panel.buildWithContext = [customConstraintCount](
                               dart::gui::PanelBuilder& builder,
                               dart::gui::PanelContext& context) {
    builder.text("Kima human skeleton with DART joint-limit enforcement");
    builder.text(
        "Neural arm/leg custom constraints: "
        + std::to_string(customConstraintCount));
    builder.text("space bar: simulation on/off");
    builder.separator();
    if (context.lifecycle != nullptr) {
      if (builder.button(context.lifecycle->paused ? "Resume" : "Pause")) {
        dart::gui::togglePaused(*context.lifecycle);
      }
      builder.sameLine();
      if (builder.button("Step")) {
        dart::gui::requestSingleStep(*context.lifecycle);
      }
    }
    builder.text("time: " + std::to_string(context.simulationTime));
    builder.text("contacts: " + std::to_string(context.contactCount));
    builder.text("selected: " + context.selectedLabel);
  };
  return panel;
}

} // namespace

dart::gui::ApplicationOptions makeHumanJointLimitsScene()
{
  auto scene = createHumanJointLimitsScene();

  dart::gui::ApplicationOptions options;
  options.world = scene.world;
  options.panels.push_back(
      createHumanJointLimitsPanel(scene.customConstraintCount));
  return options;
}

} // namespace dart::examples::demos
