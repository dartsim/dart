/*
 * Copyright (c) 2011-2018, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include "dart/gui/glfw/WorldWindow.hpp"

#include "dart/collision/CollisionDetector.hpp"
#include "dart/constraint/ConstraintSolver.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/CapsuleShape.hpp"
#include "dart/dynamics/ConeShape.hpp"
#include "dart/dynamics/CylinderShape.hpp"
#include "dart/dynamics/EllipsoidShape.hpp"
#include "dart/dynamics/LineSegmentShape.hpp"
#include "dart/dynamics/Marker.hpp"
#include "dart/dynamics/MeshShape.hpp"
#include "dart/dynamics/MultiSphereConvexHullShape.hpp"
#include "dart/dynamics/PlaneShape.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/SoftBodyNode.hpp"
#include "dart/dynamics/SoftBodyNode.hpp"
#include "dart/dynamics/SoftMeshShape.hpp"
#include "dart/dynamics/SphereShape.hpp"
#include "dart/gui/OpenGLRenderInterface.hpp"
#include "dart/gui/glut/GLUTFuncs.hpp"
#include "dart/gui/glut/GraphWindow.hpp"
#include "dart/gui/glut/LoadGlut.hpp"
#include "dart/simulation/World.hpp"
#include "dart/utils/FileInfoWorld.hpp"
#include "dart/gui/ShaderCode.hpp"

namespace dart {
namespace gui {
namespace glfw {

//==============================================================================
namespace {

const char* DEFAULT_VERTEX_SHADER_CODE
    = R"(
      #version 330
      layout(location = 0) in vec3 Position;
      void main()
      {
        gl_Position = vec4(Position, 1.0);
      }
      )";

const char* DEFAULT_FRAGMENT_SHADER_CODE
    = R"(
      #version 330
      layout(location = 0) out vec4 color;
      void main()
      {
        color = vec4(1.0f, 0.5f, 0.2f, 1.0f);
      }
      )";

} // (anonymous) namespace

//==============================================================================
WorldWindow::WorldWindow(
    simulation::WorldPtr world,
    const std::string& title,
    int width,
    int height,
    bool show)
  : Window(title, width, height, show), mWorld(std::move(world))
{
  mPlay = false;
  mSimulating = false;
  mPlayFrame = 0;
  mShowPointMasses = false;
  mShowMarkers = true;
  mPersp = 45.f;
  mTrans[1] = 300.0;

  mRI = std::make_shared<OpenGLRenderInterface>();
  mRI->initialize();

  // TODO: Just for test
  mBox.initialize();

  VertexShaderCode vertexShader(DEFAULT_VERTEX_SHADER_CODE);
  FragmentShaderCode fragmentShader(DEFAULT_FRAGMENT_SHADER_CODE);
  mProgram.reset(new Program(vertexShader, fragmentShader));
  mProgram->generateUniformLists();
}

//==============================================================================
WorldWindow::~WorldWindow()
{
  // Do nothing

  mBox.release();
}

//==============================================================================
void WorldWindow::renderScene()
{
//  glMatrixMode(GL_PROJECTION);
//  glLoadIdentity();
//  gluPerspective(
//      static_cast<GLdouble>(mPersp),
//      static_cast<GLdouble>(mWindowWidth)
//          / static_cast<GLdouble>(mWindowHeight),
//      0.1,
//      10.0);
//  gluLookAt(mEye[0], mEye[1], mEye[2], 0.0, 0.0, -1.0, mUp[0], mUp[1], mUp[2]);

//  glMatrixMode(GL_MODELVIEW);
//  glLoadIdentity();

//  glEnable(GL_BLEND);
//  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//  glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
//  glShadeModel(GL_SMOOTH);
//  glPolygonMode(GL_FRONT, GL_FILL);

//  mTrackBall.applyGLRotation();

//  // Draw world origin indicator
//  if (!mCapture)
//  {
//    glEnable(GL_DEPTH_TEST);
//    glDisable(GL_TEXTURE_2D);
//    glDisable(GL_LIGHTING);
//    glLineWidth(2.0);
//    if (mRotate || mTranslate || mZooming)
//    {
//      glColor3f(1.0f, 0.0f, 0.0f);
//      glBegin(GL_LINES);
//      glVertex3f(-0.1f, 0.0f, -0.0f);
//      glVertex3f(0.15f, 0.0f, -0.0f);
//      glEnd();

//      glColor3f(0.0f, 1.0f, 0.0f);
//      glBegin(GL_LINES);
//      glVertex3f(0.0f, -0.1f, 0.0f);
//      glVertex3f(0.0f, 0.15f, 0.0f);
//      glEnd();

//      glColor3f(0.0f, 0.0f, 1.0f);
//      glBegin(GL_LINES);
//      glVertex3f(0.0f, 0.0f, -0.1f);
//      glVertex3f(0.0f, 0.0f, 0.15f);
//      glEnd();
//    }
//  }

//  glScalef(mZoom, mZoom, mZoom);
//  glTranslated(mTrans[0] * 0.001, mTrans[1] * 0.001, mTrans[2] * 0.001);

  // TODO: Set lights

  mProgram->bind();



  renderWorld();

  // Draw trackball indicator
  //  if (mRotate && !mCapture)
  //    mTrackBall.draw(mWinWidth, mWinHeight);
}

//==============================================================================
void WorldWindow::renderWorld()
{
  // For test
  mBox.render();

  for (auto i = 0u; i < mWorld->getNumSkeletons(); ++i)
    drawSkeleton(mWorld->getSkeleton(i).get());
}

//==============================================================================
void WorldWindow::drawSkeleton(
    const dynamics::Skeleton* skeleton,
    const Eigen::Vector4d& color,
    bool useDefaultColor) const
{
  if (!skeleton)
    return;

  for (auto i = 0u; i < skeleton->getNumTrees(); ++i)
    drawBodyNode(skeleton->getRootBodyNode(i), color, useDefaultColor, true);
}

//==============================================================================
void WorldWindow::drawEntity(
    const dynamics::Entity* entity,
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
void WorldWindow::drawBodyNode(
    const dynamics::BodyNode* bodyNode,
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
void WorldWindow::drawShapeFrame(
    const dynamics::ShapeFrame* shapeFrame,
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
void WorldWindow::drawShape(
    const dynamics::Shape* shape, const Eigen::Vector4d& color) const
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
  using dynamics::MultiSphereConvexHullShape;
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
    mRI->drawEllipsoid(ellipsoid->getDiameters());
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
  else if (shape->is<MultiSphereConvexHullShape>())
  {
    const auto* multiSphere
        = static_cast<const MultiSphereConvexHullShape*>(shape);
    mRI->drawMultiSphere(multiSphere->getSpheres());
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
    const auto& lineSegmentShape = static_cast<const LineSegmentShape*>(shape);
    mRI->drawLineSegments(
        lineSegmentShape->getVertices(), lineSegmentShape->getConnections());
  }
  else
  {
    dterr << "[WorldWindow::drawShape] Attempting to draw an unsupported shape "
          << "type [" << shape->getType() << "].\n";
  }

  glDisable(GL_COLOR_MATERIAL);
}

//==============================================================================
void WorldWindow::drawPointMasses(
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
void WorldWindow::drawMarker(
    const dynamics::Marker* marker,
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

} // namespace glfw
} // namespace gui
} // namespace dart
