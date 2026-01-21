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

#include "simulation_event_handler.hpp"

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/Material>
#include <osg/Shape>
#include <osg/ShapeDrawable>

#include <iomanip>
#include <iostream>

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::gui;
using namespace dart::math;

SimulationEventHandler::SimulationEventHandler(
    WorldPtr world, dart::gui::Viewer* viewer)
  : mWorld(world),
    mViewer(viewer),
    mSelectedBody(nullptr),
    mSelectedBodyIndex(0),
    mTimeStep(DEFAULT_TIME_STEP),
    mSimulationRunning(false),
    mForceMagnitude(DEFAULT_FORCE_MAGNITUDE),
    mTorqueMagnitude(DEFAULT_TORQUE_MAGNITUDE),
    mShowForceArrows(true),
    mFrameCounter(0)
{
  if (mWorld) {
    mWorld->setTimeStep(mTimeStep);
    mRigidBodies = getRigidBodies();

    if (!mRigidBodies.empty()) {
      mSelectedBody = mRigidBodies[0];
      std::cout << "Selected body: " << mSelectedBody->getName() << std::endl;
    }

    printInstructions();
  }
}

bool SimulationEventHandler::handle(
    const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& /*aa*/)
{
  if (ea.getEventType() != osgGA::GUIEventAdapter::KEYDOWN) {
    return false;
  }

  switch (ea.getKey()) {
    // Simulation control
    case ' ': // Space bar - toggle simulation
      toggleSimulation();
      return true;

    case 's': // Step simulation
    case 'S':
      stepSimulation();
      return true;

    case 'r': // Reset simulation
    case 'R':
      resetSimulation();
      return true;

    // Body selection
    case osgGA::GUIEventAdapter::KEY_Tab: // Next body
      selectNextBody();
      return true;

    case osgGA::GUIEventAdapter::KEY_BackSpace: // Previous body
      selectPreviousBody();
      return true;

    // Force application (world coordinates)
    case osgGA::GUIEventAdapter::KEY_Up: // +Y force
      applyForceToSelectedBody(Eigen::Vector3d(0, mForceMagnitude, 0));
      return true;

    case osgGA::GUIEventAdapter::KEY_Down: // -Y force
      applyForceToSelectedBody(Eigen::Vector3d(0, -mForceMagnitude, 0));
      return true;

    case osgGA::GUIEventAdapter::KEY_Left: // -X force
      applyForceToSelectedBody(Eigen::Vector3d(-mForceMagnitude, 0, 0));
      return true;

    case osgGA::GUIEventAdapter::KEY_Right: // +X force
      applyForceToSelectedBody(Eigen::Vector3d(mForceMagnitude, 0, 0));
      return true;

    case 'u': // +Z force (up)
    case 'U':
      applyForceToSelectedBody(Eigen::Vector3d(0, 0, mForceMagnitude));
      return true;

    case 'd': // -Z force (down)
    case 'D':
      applyForceToSelectedBody(Eigen::Vector3d(0, 0, -mForceMagnitude));
      return true;

    // Torque application (world coordinates)
    case 'q': // +X torque
    case 'Q':
      applyTorqueToSelectedBody(Eigen::Vector3d(mTorqueMagnitude, 0, 0));
      return true;

    case 'w': // +Y torque
    case 'W':
      applyTorqueToSelectedBody(Eigen::Vector3d(0, mTorqueMagnitude, 0));
      return true;

    case 'e': // +Z torque
    case 'E':
      applyTorqueToSelectedBody(Eigen::Vector3d(0, 0, mTorqueMagnitude));
      return true;

    case 'a': // -X torque
    case 'A':
      applyTorqueToSelectedBody(Eigen::Vector3d(-mTorqueMagnitude, 0, 0));
      return true;

    case 'z': // -Y torque
    case 'Z':
      applyTorqueToSelectedBody(Eigen::Vector3d(0, -mTorqueMagnitude, 0));
      return true;

    case 'c': // -Z torque
    case 'C':
      applyTorqueToSelectedBody(Eigen::Vector3d(0, 0, -mTorqueMagnitude));
      return true;

    // Force/torque magnitude adjustment
    case '+': // Increase magnitude
    case '=':
      mForceMagnitude *= 1.5;
      mTorqueMagnitude *= 1.5;
      std::cout << "Force magnitude: " << mForceMagnitude
                << ", Torque magnitude: " << mTorqueMagnitude << std::endl;
      return true;

    case '-': // Decrease magnitude
    case '_':
      mForceMagnitude *= 0.67;
      mTorqueMagnitude *= 0.67;
      std::cout << "Force magnitude: " << mForceMagnitude
                << ", Torque magnitude: " << mTorqueMagnitude << std::endl;
      return true;

    // Time step adjustment
    case '>': // Increase time step
    case '.':
      mTimeStep *= 1.5;
      mWorld->setTimeStep(mTimeStep);
      std::cout << "Time step: " << mTimeStep << " seconds" << std::endl;
      return true;

    case '<': // Decrease time step
    case ',':
      mTimeStep *= 0.67;
      mWorld->setTimeStep(mTimeStep);
      std::cout << "Time step: " << mTimeStep << " seconds" << std::endl;
      return true;

    // Visualization
    case 'v': // Toggle force arrows
    case 'V':
      mShowForceArrows = !mShowForceArrows;
      if (!mShowForceArrows) {
        clearForceArrows();
      }
      std::cout << "Force arrows: " << (mShowForceArrows ? "ON" : "OFF")
                << std::endl;
      return true;

    // Information
    case 'i': // Print simulation state
    case 'I':
      printSimulationState();
      return true;

    case 'h': // Print help
    case 'H':
    case '?':
      printInstructions();
      return true;

    default:
      return false;
  }
}

void SimulationEventHandler::update()
{
  ++mFrameCounter;

  if (mShowForceArrows && (mFrameCounter % ARROW_UPDATE_FREQUENCY == 0)) {
    updateForceArrows();
  }
}

void SimulationEventHandler::applyForceToSelectedBody(
    const Eigen::Vector3d& force)
{
  if (!mSelectedBody) {
    std::cout << "No body selected!" << std::endl;
    return;
  }

  // Apply force at center of mass
  mSelectedBody->addExtForce(force);

  // Store for visualization
  mAppliedForces.push_back(std::make_pair(mSelectedBody, force));

  std::cout << "Applied force [" << force.transpose()
            << "] to body: " << mSelectedBody->getName() << std::endl;
}

void SimulationEventHandler::applyTorqueToSelectedBody(
    const Eigen::Vector3d& torque)
{
  if (!mSelectedBody) {
    std::cout << "No body selected!" << std::endl;
    return;
  }

  mSelectedBody->addExtTorque(torque);

  // Store for visualization
  mAppliedTorques.push_back(std::make_pair(mSelectedBody, torque));

  std::cout << "Applied torque [" << torque.transpose()
            << "] to body: " << mSelectedBody->getName() << std::endl;
}

void SimulationEventHandler::updateForceArrows()
{
  if (!mShowForceArrows || !mViewer) {
    return;
  }

  clearForceArrows();

  // Create arrows for applied forces
  for (const auto& force : mAppliedForces) {
    addForceArrow(force.first, force.second);
  }

  // Create arrows for applied torques (as curved arrows would be complex,
  // we'll show them as straight arrows from COM)
  for (const auto& torque : mAppliedTorques) {
    addForceArrow(torque.first, torque.second);
  }

  // Clear applied forces after visualization update
  mAppliedForces.clear();
  mAppliedTorques.clear();
}

void SimulationEventHandler::addForceArrow(
    BodyNodePtr bodyNode, const Eigen::Vector3d& force)
{
  if (!mViewer || force.norm() < 1e-6) {
    return;
  }

  // Get body position
  Eigen::Vector3d bodyPos = bodyNode->getCOM();

  // Normalize force for visualization (scale to reasonable size)
  Eigen::Vector3d normalizedForce = force.normalized() * 0.5; // 0.5 meter arrow
  Eigen::Vector3d arrowEnd = bodyPos + normalizedForce;

  // Create arrow geometry
  ::osg::ref_ptr<::osg::Geode> geode = new ::osg::Geode;
  ::osg::ref_ptr<::osg::Geometry> geometry = new ::osg::Geometry;

  // Arrow vertices (line + arrowhead)
  ::osg::ref_ptr<::osg::Vec3Array> vertices = new ::osg::Vec3Array;
  vertices->push_back(::osg::Vec3(bodyPos.x(), bodyPos.y(), bodyPos.z()));
  vertices->push_back(::osg::Vec3(arrowEnd.x(), arrowEnd.y(), arrowEnd.z()));

  // Create simple arrowhead
  Eigen::Vector3d perpendicular1, perpendicular2;
  if (std::abs(normalizedForce.z()) < 0.9) {
    perpendicular1
        = normalizedForce.cross(Eigen::Vector3d::UnitZ()).normalized() * 0.1;
  } else {
    perpendicular1
        = normalizedForce.cross(Eigen::Vector3d::UnitX()).normalized() * 0.1;
  }
  perpendicular2 = normalizedForce.cross(perpendicular1).normalized() * 0.1;

  Eigen::Vector3d head1 = arrowEnd - 0.2 * normalizedForce + perpendicular1;
  Eigen::Vector3d head2 = arrowEnd - 0.2 * normalizedForce - perpendicular1;
  Eigen::Vector3d head3 = arrowEnd - 0.2 * normalizedForce + perpendicular2;
  Eigen::Vector3d head4 = arrowEnd - 0.2 * normalizedForce - perpendicular2;

  vertices->push_back(::osg::Vec3(head1.x(), head1.y(), head1.z()));
  vertices->push_back(::osg::Vec3(arrowEnd.x(), arrowEnd.y(), arrowEnd.z()));
  vertices->push_back(::osg::Vec3(head2.x(), head2.y(), head2.z()));
  vertices->push_back(::osg::Vec3(arrowEnd.x(), arrowEnd.y(), arrowEnd.z()));
  vertices->push_back(::osg::Vec3(head3.x(), head3.y(), head3.z()));
  vertices->push_back(::osg::Vec3(arrowEnd.x(), arrowEnd.y(), arrowEnd.z()));
  vertices->push_back(::osg::Vec3(head4.x(), head4.y(), head4.z()));
  vertices->push_back(::osg::Vec3(arrowEnd.x(), arrowEnd.y(), arrowEnd.z()));

  geometry->setVertexArray(vertices);

  // Create colors (red for forces)
  ::osg::ref_ptr<::osg::Vec4Array> colors = new ::osg::Vec4Array;
  for (size_t i = 0; i < vertices->size(); ++i) {
    colors->push_back(::osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f)); // Red
  }
  geometry->setColorArray(colors, ::osg::Array::BIND_PER_VERTEX);

  // Create line primitives
  geometry->addPrimitiveSet(
      new ::osg::DrawArrays(::osg::PrimitiveSet::LINES, 0, vertices->size()));

  // Set line width
  ::osg::ref_ptr<::osg::LineWidth> lineWidth = new ::osg::LineWidth(3.0f);
  geometry->getOrCreateStateSet()->setAttributeAndModes(
      lineWidth, ::osg::StateAttribute::ON);

  geode->addDrawable(geometry);

  // Create group to hold the arrow
  ::osg::ref_ptr<::osg::Group> arrowGroup = new ::osg::Group;
  arrowGroup->addChild(geode);

  mForceArrowNodes.push_back(arrowGroup);

  // Add to scene (this is a simplified approach; in practice you'd want to add
  // to appropriate scene nodes)
  if (mViewer->getSceneData()) {
    ::osg::Group* root = dynamic_cast<::osg::Group*>(mViewer->getSceneData());
    if (root) {
      root->addChild(arrowGroup);
    }
  }
}

void SimulationEventHandler::clearForceArrows()
{
  if (!mViewer || !mViewer->getSceneData()) {
    return;
  }

  ::osg::Group* root = dynamic_cast<::osg::Group*>(mViewer->getSceneData());
  if (!root) {
    return;
  }

  // Remove all arrow nodes from scene
  for (auto& arrowNode : mForceArrowNodes) {
    root->removeChild(arrowNode);
  }

  for (auto& arrowNode : mTorqueArrowNodes) {
    root->removeChild(arrowNode);
  }

  mForceArrowNodes.clear();
  mTorqueArrowNodes.clear();
}

void SimulationEventHandler::selectNextBody()
{
  if (mRigidBodies.empty()) {
    return;
  }

  mSelectedBodyIndex = (mSelectedBodyIndex + 1) % mRigidBodies.size();
  mSelectedBody = mRigidBodies[mSelectedBodyIndex];

  std::cout << "Selected body: " << mSelectedBody->getName() << " ("
            << (mSelectedBodyIndex + 1) << "/" << mRigidBodies.size() << ")"
            << std::endl;
}

void SimulationEventHandler::selectPreviousBody()
{
  if (mRigidBodies.empty()) {
    return;
  }

  if (mSelectedBodyIndex == 0) {
    mSelectedBodyIndex = mRigidBodies.size() - 1;
  } else {
    --mSelectedBodyIndex;
  }

  mSelectedBody = mRigidBodies[mSelectedBodyIndex];

  std::cout << "Selected body: " << mSelectedBody->getName() << " ("
            << (mSelectedBodyIndex + 1) << "/" << mRigidBodies.size() << ")"
            << std::endl;
}

void SimulationEventHandler::resetSimulation()
{
  if (!mWorld) {
    return;
  }

  // Reset all skeletons to initial state
  for (std::size_t i = 0; i < mWorld->getNumSkeletons(); ++i) {
    auto skel = mWorld->getSkeleton(i);
    skel->resetPositions();
    skel->resetVelocities();
    skel->clearExternalForces();
  }

  // Reset simulation time
  mWorld->reset();

  // Clear visualization
  clearForceArrows();
  mAppliedForces.clear();
  mAppliedTorques.clear();

  std::cout << "Simulation reset" << std::endl;
}

void SimulationEventHandler::stepSimulation()
{
  if (!mWorld) {
    return;
  }

  mWorld->step();
  std::cout << "Simulation time: " << std::fixed << std::setprecision(4)
            << mWorld->getTime() << " seconds" << std::endl;
}

void SimulationEventHandler::toggleSimulation()
{
  mSimulationRunning = !mSimulationRunning;

  if (mViewer) {
    mViewer->allowSimulation(mSimulationRunning);
  }

  std::cout << "Simulation " << (mSimulationRunning ? "STARTED" : "PAUSED")
            << std::endl;
}

void SimulationEventHandler::printSimulationState()
{
  if (!mWorld) {
    return;
  }

  std::cout << "\n=== Simulation State ===" << std::endl;
  std::cout << "Time: " << std::fixed << std::setprecision(4)
            << mWorld->getTime() << " seconds" << std::endl;
  std::cout << "Time step: " << mTimeStep << " seconds" << std::endl;
  std::cout << "Running: " << (mSimulationRunning ? "YES" : "NO") << std::endl;
  std::cout << "Force magnitude: " << mForceMagnitude << " N" << std::endl;
  std::cout << "Torque magnitude: " << mTorqueMagnitude << " Nm" << std::endl;
  std::cout << "Show arrows: " << (mShowForceArrows ? "YES" : "NO")
            << std::endl;

  if (mSelectedBody) {
    std::cout << "Selected body: " << mSelectedBody->getName() << std::endl;
    Eigen::Vector3d pos = mSelectedBody->getCOM();
    Eigen::Vector3d vel = mSelectedBody->getCOMLinearVelocity();
    std::cout << "  Position: [" << pos.transpose() << "]" << std::endl;
    std::cout << "  Velocity: [" << vel.transpose() << "]" << std::endl;
  }

  std::cout << "Number of rigid bodies: " << mRigidBodies.size() << std::endl;
  std::cout << "========================\n" << std::endl;
}

void SimulationEventHandler::printInstructions()
{
  std::cout << "\n=== Simulation Event Handler Controls ===" << std::endl;
  std::cout << "Simulation Control:" << std::endl;
  std::cout << "  Space      - Toggle simulation play/pause" << std::endl;
  std::cout << "  S          - Step simulation one frame" << std::endl;
  std::cout << "  R          - Reset simulation" << std::endl;
  std::cout << "\nBody Selection:" << std::endl;
  std::cout << "  Tab        - Select next rigid body" << std::endl;
  std::cout << "  Backspace  - Select previous rigid body" << std::endl;
  std::cout << "\nForce Application (on selected body):" << std::endl;
  std::cout << "  Arrow Keys - Apply force in X/Y directions" << std::endl;
  std::cout << "  U          - Apply upward force (+Z)" << std::endl;
  std::cout << "  D          - Apply downward force (-Z)" << std::endl;
  std::cout << "\nTorque Application (on selected body):" << std::endl;
  std::cout << "  Q/A        - Apply torque around X axis (+/-)" << std::endl;
  std::cout << "  W/Z        - Apply torque around Y axis (+/-)" << std::endl;
  std::cout << "  E/C        - Apply torque around Z axis (+/-)" << std::endl;
  std::cout << "\nMagnitude Adjustment:" << std::endl;
  std::cout << "  +/=        - Increase force/torque magnitude" << std::endl;
  std::cout << "  -/_        - Decrease force/torque magnitude" << std::endl;
  std::cout << "\nTime Step Adjustment:" << std::endl;
  std::cout << "  >/.        - Increase simulation time step" << std::endl;
  std::cout << "  </, 	   - Decrease simulation time step" << std::endl;
  std::cout << "\nVisualization:" << std::endl;
  std::cout << "  V          - Toggle force arrow visualization" << std::endl;
  std::cout << "\nInformation:" << std::endl;
  std::cout << "  I          - Print current simulation state" << std::endl;
  std::cout << "  H/?        - Show this help" << std::endl;
  std::cout << "==========================================\n" << std::endl;
}

std::vector<BodyNodePtr> SimulationEventHandler::getRigidBodies()
{
  std::vector<BodyNodePtr> rigidBodies;

  if (!mWorld) {
    return rigidBodies;
  }

  for (std::size_t i = 0; i < mWorld->getNumSkeletons(); ++i) {
    auto skel = mWorld->getSkeleton(i);

    for (std::size_t j = 0; j < skel->getNumBodyNodes(); ++j) {
      auto body = skel->getBodyNode(j);
      auto joint = body->getParentJoint();

      // Consider bodies with FreeJoint or similar as rigid bodies
      // Also include bodies that are not fixed to the world
      if (joint
          && (joint->getType() == "FreeJoint" || joint->getType() == "BallJoint"
              || joint->getType() == "EulerJoint"
              || joint->getType() == "TranslationalJoint"
              || joint->getNumDofs() > 0)) {
        rigidBodies.push_back(body);
      }
    }
  }

  return rigidBodies;
}
