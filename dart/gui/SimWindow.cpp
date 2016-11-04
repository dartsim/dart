/*
 * Copyright (c) 2013-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2013-2016, Humanoid Lab, Georgia Tech Research Corporation
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
 *   * This code incorporates portions of Open Dynamics Engine
 *     (Copyright (c) 2001-2004, Russell L. Smith. All rights
 *     reserved.) and portions of FCL (Copyright (c) 2011, Willow
 *     Garage, Inc. All rights reserved.), which were released under
 *     the same BSD license as below
 *
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

#include "dart/gui/SimWindow.hpp"

#include <cstdio>
#include <iostream>
#include <string>

#include "dart/simulation/World.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/SoftBodyNode.hpp"
#include "dart/dynamics/SphereShape.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/EllipsoidShape.hpp"
#include "dart/dynamics/CylinderShape.hpp"
#include "dart/dynamics/CapsuleShape.hpp"
#include "dart/dynamics/ConeShape.hpp"
#include "dart/dynamics/PlaneShape.hpp"
#include "dart/dynamics/MultiSphereShape.hpp"
#include "dart/dynamics/MeshShape.hpp"
#include "dart/dynamics/SoftMeshShape.hpp"
#include "dart/dynamics/LineSegmentShape.hpp"
#include "dart/dynamics/Marker.hpp"
#include "dart/constraint/ConstraintSolver.hpp"
#include "dart/collision/CollisionDetector.hpp"
#include "dart/gui/LoadGlut.hpp"
#include "dart/gui/GLFuncs.hpp"
#include "dart/gui/GraphWindow.hpp"
#include "dart/utils/FileInfoWorld.hpp"

namespace dart {
namespace gui {

SimWindow::SimWindow()
  : Win3D() {
  mBackground[0] = 1.0;
  mBackground[1] = 1.0;
  mBackground[2] = 1.0;
  mBackground[3] = 1.0;

  mPlay = false;
  mSimulating = false;
  mPlayFrame = 0;
  mShowPointMasses = false;
  mShowMarkers = true;
  mPersp = 45.f;
  mTrans[1] = 300.f;
}

SimWindow::~SimWindow() {
  for (const auto& graphWindow : mGraphWindows)
    delete graphWindow;
}

void SimWindow::timeStepping() {
  mWorld->step();
}

//==============================================================================
void SimWindow::drawWorld() const
{
  drawSkeletons();

  for (auto i = 0u; i < mWorld->getNumSimpleFrames(); ++i)
    drawShapeFrame(mWorld->getSimpleFrame(i).get());
}

//==============================================================================
void SimWindow::drawSkeletons() const
{
  for (auto i = 0u; i < mWorld->getNumSkeletons(); ++i)
    drawSkeleton(mWorld->getSkeleton(i).get());
}

//==============================================================================
void SimWindow::drawSkels()
{
  drawSkeletons();
}

//==============================================================================
void SimWindow::drawEntities()
{
  for (std::size_t i = 0; i < mWorld->getNumSimpleFrames(); ++i)
    drawShapeFrame(mWorld->getSimpleFrame(i).get());
}

void SimWindow::displayTimer(int _val) {
  int numIter = mDisplayTimeout / (mWorld->getTimeStep() * 1000);
  if (mPlay) {
    mPlayFrame += 16;
    if (mPlayFrame >= mWorld->getRecording()->getNumFrames())
      mPlayFrame = 0;
  } else if (mSimulating) {
    for (int i = 0; i < numIter; i++) {
      timeStepping();
      mWorld->bake();
    }
  }
  glutPostRedisplay();
  glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
}

void SimWindow::draw() {
  glDisable(GL_LIGHTING);
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  if (!mSimulating) {
      if (mPlayFrame < mWorld->getRecording()->getNumFrames()) {
      std::size_t nSkels = mWorld->getNumSkeletons();
      for (std::size_t i = 0; i < nSkels; i++) {
        // std::size_t start = mWorld->getIndex(i);
        // std::size_t size = mWorld->getSkeleton(i)->getNumDofs();
        mWorld->getSkeleton(i)->setPositions(mWorld->getRecording()->getConfig(mPlayFrame, i));
      }
      if (mShowMarkers) {
        // std::size_t sumDofs = mWorld->getIndex(nSkels);
        int nContact = mWorld->getRecording()->getNumContacts(mPlayFrame);
        for (int i = 0; i < nContact; i++) {
            Eigen::Vector3d v = mWorld->getRecording()->getContactPoint(mPlayFrame, i);
            Eigen::Vector3d f = mWorld->getRecording()->getContactForce(mPlayFrame, i);

          glBegin(GL_LINES);
          glVertex3f(v[0], v[1], v[2]);
          glVertex3f(v[0] + f[0], v[1] + f[1], v[2] + f[2]);
          glEnd();
          mRI->setPenColor(Eigen::Vector3d(0.2, 0.2, 0.8));
          mRI->pushMatrix();
          glTranslated(v[0], v[1], v[2]);
          mRI->drawSphere(0.01);
          mRI->popMatrix();
        }
      }
    }
  } else {
    if (mShowMarkers) {
      const auto result =
          mWorld->getConstraintSolver()->getLastCollisionResult();
      for (const auto& contact : result.getContacts()) {
        Eigen::Vector3d v = contact.point;
        Eigen::Vector3d f = contact.force / 10.0;
        glBegin(GL_LINES);
        glVertex3f(v[0], v[1], v[2]);
        glVertex3f(v[0] + f[0], v[1] + f[1], v[2] + f[2]);
        glEnd();
        mRI->setPenColor(Eigen::Vector3d(0.2, 0.2, 0.8));
        mRI->pushMatrix();
        glTranslated(v[0], v[1], v[2]);
        mRI->drawSphere(0.01);
        mRI->popMatrix();
      }
    }
  }

  drawWorld();

  // display the frame count in 2D text
  char buff[64];
  if (!mSimulating)
#ifdef _WIN32
    _snprintf(buff, sizeof(buff), "%d", mPlayFrame);
#else
    std::snprintf(buff, sizeof(buff), "%d", mPlayFrame);
#endif
  else
#ifdef _WIN32
    _snprintf(buff, sizeof(buff), "%d", mWorld->getSimFrames());
#else
    std::snprintf(buff, sizeof(buff), "%d", mWorld->getSimFrames());
#endif
  std::string frame(buff);
  glColor3f(0.0, 0.0, 0.0);
  gui::drawStringOnScreen(0.02f, 0.02f, frame);
  glEnable(GL_LIGHTING);
}

void SimWindow::keyboard(unsigned char _key, int _x, int _y) {
  switch (_key) {
    case ' ':  // use space key to play or stop the motion
      mSimulating = !mSimulating;
      if (mSimulating)
        mPlay = false;
      break;
    case 'p':  // playBack
      mPlay = !mPlay;
      if (mPlay)
        mSimulating = false;
      break;
    case '[':  // step backward
      if (!mSimulating) {
        mPlayFrame--;
        if (mPlayFrame < 0)
          mPlayFrame = 0;
        glutPostRedisplay();
      }
      break;
    case ']':  // step forwardward
      if (!mSimulating) {
        mPlayFrame++;
        if (mPlayFrame >= mWorld->getRecording()->getNumFrames())
          mPlayFrame = 0;
        glutPostRedisplay();
      }
      break;
    case 'v':  // show or hide markers
      mShowMarkers = !mShowMarkers;
      break;
  case 's':
      saveWorld();
      std::cout << "World Saved in 'tempWorld.txt'" << std::endl;
      break;
    default:
      Win3D::keyboard(_key, _x, _y);
  }
  glutPostRedisplay();
}

void SimWindow::setWorld(simulation::WorldPtr _world) {
  mWorld = _world;
}

void SimWindow::saveWorld() {
  if (!mWorld)
    return;
  dart::utils::FileInfoWorld worldFile;
  worldFile.saveFile("tempWorld.txt", mWorld->getRecording());
}

void SimWindow::plot(Eigen::VectorXd& _data) {
  GraphWindow* figure = new GraphWindow();
  figure->setData(_data);
  figure->initWindow(480, 240, "figure");
  mGraphWindows.push_back(figure);
}

//==============================================================================
void SimWindow::drawSkeleton(const dynamics::Skeleton* skeleton,
                             const Eigen::Vector4d& color,
                             bool useDefaultColor) const
{
  if (!skeleton)
    return;

  for (auto i = 0u; i < skeleton->getNumTrees(); ++i)
    drawBodyNode(skeleton->getRootBodyNode(i), color, useDefaultColor, true);
}

//==============================================================================
void SimWindow::drawEntity(const dynamics::Entity* entity,
                           const Eigen::Vector4d& color,
                           bool useDefaultColor) const
{
  if (!entity)
    return;

  const auto& bodyNode = dynamic_cast<const dynamics::BodyNode*>(entity);
  if (bodyNode)
  {
    drawBodyNode(bodyNode, color, useDefaultColor, true);
    return;
  }

  const auto& shapeFrame = dynamic_cast<const dynamics::ShapeFrame*>(entity);
  if (shapeFrame)
  {
    drawShapeFrame(shapeFrame, color, useDefaultColor);
    return;
  }
}

//==============================================================================
void SimWindow::drawBodyNode(const dynamics::BodyNode* bodyNode,
                             const Eigen::Vector4d& color,
                             bool useDefaultColor,
                             bool recursive) const
{
  if (!bodyNode)
    return;

  if (!mRI)
    return;

  mRI->pushMatrix();

  // Use the relative transform of this Frame. We assume that we are being
  // called from the parent Frame's renderer.
  // TODO(MXG): This can cause trouble if the draw function is originally called
  // on an Entity or Frame which is not a child of the World Frame
  mRI->transform(bodyNode->getRelativeTransform());

  // _ri->pushName(???); TODO(MXG): What should we do about this for Frames?
  auto shapeNodes = bodyNode->getShapeNodesWith<dynamics::VisualAspect>();
  for (const auto& shapeNode : shapeNodes)
    drawShapeFrame(shapeNode, color, useDefaultColor);
  // _ri.popName();

  if (mShowPointMasses)
  {
    const auto& softBodyNode
        = dynamic_cast<const dynamics::SoftBodyNode*>(bodyNode);
    if (softBodyNode)
      drawPointMasses(softBodyNode->getPointMasses(), color);
  }

  if (mShowMarkers)
  {
    for (auto i = 0u; i < bodyNode->getNumMarkers(); ++i)
      drawMarker(bodyNode->getMarker(i));
  }

  // render the subtree
  if (recursive)
  {
    for (const auto& entity : bodyNode->getChildEntities())
      drawEntity(entity, color, useDefaultColor);
  }

  mRI->popMatrix();
}

//==============================================================================
void SimWindow::drawShapeFrame(const dynamics::ShapeFrame* shapeFrame,
                               const Eigen::Vector4d& color,
                               bool useDefaultColor) const
{
  if (!shapeFrame)
    return;

  if (!mRI)
    return;

  const auto& visualAspect = shapeFrame->getVisualAspect();

  if (!visualAspect || visualAspect->isHidden())
    return;

  mRI->pushMatrix();
  mRI->transform(shapeFrame->getRelativeTransform());

  if (useDefaultColor)
    drawShape(shapeFrame->getShape().get(), visualAspect->getRGBA());
  else
    drawShape(shapeFrame->getShape().get(), color);

  mRI->popMatrix();
}

//==============================================================================
void SimWindow::drawShape(const dynamics::Shape* shape,
                          const Eigen::Vector4d& color) const
{
  if (!shape)
    return;

  if (!mRI)
    return;

  glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
  glEnable(GL_COLOR_MATERIAL);

  mRI->setPenColor(color);

  using dynamics::Shape;
  using dynamics::SphereShape;
  using dynamics::BoxShape;
  using dynamics::EllipsoidShape;
  using dynamics::CylinderShape;
  using dynamics::CapsuleShape;
  using dynamics::ConeShape;
  using dynamics::PlaneShape;
  using dynamics::MultiSphereShape;
  using dynamics::MeshShape;
  using dynamics::SoftMeshShape;
  using dynamics::LineSegmentShape;

  if (shape->is<SphereShape>())
  {
    const auto* sphere = static_cast<const SphereShape*>(shape);
    mRI->drawSphere(sphere->getRadius());
  }
  else if (shape->is<BoxShape>())
  {
    const auto* box = static_cast<const BoxShape*>(shape);
    mRI->drawCube(box->getSize());
  }
  else if (shape->is<EllipsoidShape>())
  {
    const auto* ellipsoid = static_cast<const EllipsoidShape*>(shape);
    mRI->drawEllipsoid(ellipsoid->getSize());
  }
  else if (shape->is<CylinderShape>())
  {
    const auto* cylinder = static_cast<const CylinderShape*>(shape);
    mRI->drawCylinder(cylinder->getRadius(), cylinder->getHeight());
  }
  else if (shape->is<CapsuleShape>())
  {
    const auto* capsule = static_cast<const CapsuleShape*>(shape);
    mRI->drawCapsule(capsule->getRadius(), capsule->getHeight());
  }
  else if (shape->is<ConeShape>())
  {
    const auto* cone = static_cast<const ConeShape*>(shape);
    mRI->drawCone(cone->getRadius(), cone->getHeight());
  }
  else if (shape->is<MultiSphereShape>())
  {
    const auto* multiSphere = static_cast<const MultiSphereShape*>(shape);
    const auto& spheres = multiSphere->getSpheres();
    for (const auto& sphere : spheres)
    {
      glTranslated(sphere.second.x(), sphere.second.y(), sphere.second.z());
      mRI->drawSphere(sphere.first);
      glTranslated(-sphere.second.x(), -sphere.second.y(), -sphere.second.z());
    }
    // TODO(JS): This is an workaround that draws only spheres rather than the
    // actual convex hull.
  }
  else if (shape->is<MeshShape>())
  {
    const auto& mesh = static_cast<const MeshShape*>(shape);

    glDisable(GL_COLOR_MATERIAL); // Use mesh colors to draw

    if (mesh->getDisplayList())
      mRI->drawList(mesh->getDisplayList());
    else
      mRI->drawMesh(mesh->getScale(), mesh->getMesh());
  }
  else if (shape->is<SoftMeshShape>())
  {
    const auto& softMesh = static_cast<const SoftMeshShape*>(shape);
    mRI->drawSoftMesh(softMesh->getAssimpMesh());
  }
  else if (shape->is<LineSegmentShape>())
  {
    const auto& lineSegmentShape
        = static_cast<const LineSegmentShape*>(shape);
    mRI->drawLineSegments(lineSegmentShape->getVertices(),
                          lineSegmentShape->getConnections());
  }
  else
  {
    dterr << "[SimWindow::drawShape] Attempting to draw an unsupported shape "
          << "type [" << shape->getType() << "].\n";
  }

  glDisable(GL_COLOR_MATERIAL);
}

//==============================================================================
void SimWindow::drawPointMasses(
    const std::vector<dynamics::PointMass*> pointMasses,
    const Eigen::Vector4d& color,
    bool useDefaultColor) const
{
  if (!mRI)
    return;

  for (const auto& pointMass : pointMasses)
  {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    // render point at the current position
    mRI->pushMatrix();
    T.translation() = pointMass->getLocalPosition();
    mRI->transform(T);
    if (useDefaultColor)
      mRI->setPenColor(Eigen::Vector4d(0.8, 0.3, 0.3, 1.0));
    else
      mRI->setPenColor(color);
    mRI->drawSphere(0.005);
    mRI->popMatrix();

    // render point at the resting position
    mRI->pushMatrix();
    T.translation() = pointMass->getRestingPosition();
    mRI->transform(T);
    if (useDefaultColor)
      mRI->setPenColor(Eigen::Vector4d(0.8, 0.3, 0.3, 1.0));
    else
      mRI->setPenColor(color);
    mRI->drawSphere(0.005);
    mRI->popMatrix();
  }
}

//==============================================================================
void SimWindow::drawMarker(const dynamics::Marker* marker,
                           const Eigen::Vector4d& color,
                           bool useDefaultColor) const
{
  if (!marker)
    return;

  if (!mRI)
    return;

  mRI->pushName(marker->getID());

  if (marker->getConstraintType() == dynamics::Marker::HARD)
  {
    mRI->setPenColor(Color::Red(1.0));
  }
  else if (marker->getConstraintType() == dynamics::Marker::SOFT)
  {
    mRI->setPenColor(Color::Green(1.0));
  }
  else
  {
    if (useDefaultColor)
      mRI->setPenColor(marker->getColor());
    else
      mRI->setPenColor(color);
  }

  mRI->pushMatrix();
  mRI->translate(marker->getLocalPosition());
  mRI->drawSphere(0.005);
  mRI->popMatrix();

  mRI->popName();
}

}  // namespace gui
}  // namespace dart
