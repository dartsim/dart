/*
 * Copyright (c) 2011-2019, The DART development contributors
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

#include <cmath>

#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>

using namespace dart;
using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::math;

const std::string& robotName = "KR5";

class PointCloudWorld : public gui::osg::WorldNode
{
public:
  explicit PointCloudWorld(
      simulation::WorldPtr world, dynamics::SkeletonPtr robot)
    : gui::osg::WorldNode(std::move(world)), mRobot(std::move(robot))
  {
    auto voxelFrame = mWorld->getSimpleFrame("voxel");

    mVoxelShape = std::dynamic_pointer_cast<dynamics::VoxelGridShape>(
        voxelFrame->getShape());

    assert(mVoxelShape);
  }

  // Triggered at the beginning of each simulation step
  void customPreStep() override
  {
    if (!mRobot)
      return;

    // Set robot pose
    Eigen::VectorXd pos = mRobot->getPositions();
    pos += 0.01 * Eigen::VectorXd::Random(pos.size());
    mRobot->setPositions(pos);

    // Generate point cloud from robot meshes
    auto pointCloud = generatePointCloud(100);

    // Update sensor position
    static double time = 0.0;
    const double dt = 0.001;
    const double radius = 1.0;
    Eigen::Vector3d center = Eigen::Vector3d(0.0, 0.1, 0.0);
    Eigen::Vector3d sensorPos = center;
    sensorPos[0] = radius * std::sin(time);
    sensorPos[1] = radius * std::cos(time);
    sensorPos[2] = 0.5 + 0.25 * std::sin(time * 2.0);
    time += dt;
    auto sensorFrame = mWorld->getSimpleFrame("sensor");
    assert(sensorFrame);
    sensorFrame->setTranslation(sensorPos);

    // Update voxel
    mVoxelShape->updateOccupancy(pointCloud, sensorPos);
  }

protected:
  octomap::Pointcloud generatePointCloud(std::size_t numPoints)
  {
    octomap::Pointcloud pointCloud;

    const auto numBodies = mRobot->getNumBodyNodes();
    assert(numBodies > 0);
    while (true)
    {
      const auto bodyIndex
          = math::Random::uniform<std::size_t>(0, numBodies - 1);
      auto body = mRobot->getBodyNode(bodyIndex);
      auto shapeNodes = body->getShapeNodesWith<dynamics::VisualAspect>();
      if (shapeNodes.empty())
        continue;

      const auto shapeIndex
          = math::Random::uniform<std::size_t>(0, shapeNodes.size() - 1);
      auto shapeNode = shapeNodes[shapeIndex];
      auto shape = shapeNode->getShape();
      assert(shape);

      if (!shape->is<dynamics::MeshShape>())
        continue;
      auto mesh = std::static_pointer_cast<dynamics::MeshShape>(shape);

      auto assimpScene = mesh->getMesh();
      assert(assimpScene);

      if (assimpScene->mNumMeshes < 1)
        continue;
      const auto meshIndex
          = math::Random::uniform<std::size_t>(0, assimpScene->mNumMeshes - 1);

      auto assimpMesh = assimpScene->mMeshes[meshIndex];
      auto numVertices = assimpMesh->mNumVertices;

      auto vertexIndex
          = math::Random::uniform<unsigned int>(0, numVertices - 1);
      auto vertex = assimpMesh->mVertices[vertexIndex];

      Eigen::Isometry3d tf = shapeNode->getWorldTransform();
      Eigen::Vector3d eigenVertex
          = Eigen::Vector3d(vertex.x, vertex.y, vertex.z);
      eigenVertex = tf * eigenVertex;

      pointCloud.push_back(eigenVertex.x(), eigenVertex.y(), eigenVertex.z());

      if (pointCloud.size() == numPoints)
        return pointCloud;
    }
  }

  SkeletonPtr mRobot;

  std::shared_ptr<dynamics::VoxelGridShape> mVoxelShape;
};

dynamics::SkeletonPtr createRobot(const std::string& name)
{
  auto urdfParser = dart::utils::DartLoader();

  // Load the robot
  auto robot
      = urdfParser.parseSkeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf");

  // Set the colors of the models to obey the shape's color specification
  for (std::size_t i = 0; i < robot->getNumBodyNodes(); ++i)
  {
    BodyNode* bn = robot->getBodyNode(i);
    auto shapeNodes = bn->getShapeNodesWith<dart::dynamics::VisualAspect>();
    for (auto shapeNode : shapeNodes)
    {
      std::shared_ptr<MeshShape> mesh
          = std::dynamic_pointer_cast<MeshShape>(shapeNode->getShape());
      if (mesh)
        mesh->setColorMode(MeshShape::SHAPE_COLOR);
    }
  }

  // Rotate the robot so that z is upwards (default transform is not Identity)
  robot->getJoint(0)->setTransformFromParentBodyNode(
      Eigen::Isometry3d::Identity());

  robot->setName(name);

  return robot;
}

dynamics::SkeletonPtr createGround()
{
  auto urdfParser = dart::utils::DartLoader();

  auto ground = urdfParser.parseSkeleton("dart://sample/urdf/KR5/ground.urdf");

  // Rotate and move the ground so that z is upwards
  Eigen::Isometry3d ground_tf
      = ground->getJoint(0)->getTransformFromParentBodyNode();
  ground_tf.pretranslate(Eigen::Vector3d(0, 0, 0.5));
  ground_tf.rotate(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(1, 0, 0)));
  ground->getJoint(0)->setTransformFromParentBodyNode(ground_tf);

  return ground;
}

dynamics::SimpleFramePtr createVoxelFrame(double resolution = 0.01)
{
  auto voxelShape
      = ::std::make_shared<dart::dynamics::VoxelGridShape>(resolution);
  auto voxelFrame = ::dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World());
  voxelFrame->setName("voxel");
  voxelFrame->setShape(voxelShape);
  auto visualAspect = voxelFrame->createVisualAspect();
  visualAspect->setRGBA(Color::Orange(0.5));

  return voxelFrame;
}

dynamics::SimpleFramePtr createSensorFrame()
{
  auto sphereShape = ::std::make_shared<dart::dynamics::SphereShape>(0.05);
  auto sensorFrame = ::dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World());
  sensorFrame->setName("sensor");
  sensorFrame->setShape(sphereShape);
  auto visualAspect = sensorFrame->createVisualAspect();
  visualAspect->setRGB(Color::Red());

  return sensorFrame;
}

int main()
{
  auto world = dart::simulation::World::create();
  world->setGravity(Eigen::Vector3d::Zero());

  auto robot = createRobot(robotName);
  world->addSkeleton(robot);

  auto ground = createGround();
  world->addSkeleton(ground);

  auto voxel = createVoxelFrame(0.05);
  world->addSimpleFrame(voxel);

  auto sensor = createSensorFrame();
  world->addSimpleFrame(sensor);

  // Create an instance of our customized WorldNode
  ::osg::ref_ptr<PointCloudWorld> node = new PointCloudWorld(world, robot);
  node->setNumStepsPerCycle(16);

  // Create the Viewer instance
  dart::gui::osg::Viewer viewer;
  viewer.addWorldNode(node);
  viewer.simulate(true);

  // Print out instructions
  std::cout << viewer.getInstructions() << std::endl;

  // Set up the window to be 640x480 pixels
  viewer.setUpViewInWindow(0, 0, 640, 480);

  viewer.getCameraManipulator()->setHomePosition(
      ::osg::Vec3(2.57, 3.14, 1.64),
      ::osg::Vec3(0.00, 0.00, 0.00),
      ::osg::Vec3(-0.24, -0.25, 0.94));
  // We need to re-dirty the CameraManipulator by passing it into the viewer
  // again, so that the viewer knows to update its HomePosition setting
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  // Begin the application loop
  viewer.run();
}
