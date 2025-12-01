/*
 * Minimal VulkanSceneGraph viewer example for DART.
 */

#include <dart/gui/vsg/Viewer.hpp>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/ShapeNode.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/WeldJoint.hpp>
#include <dart/simulation/World.hpp>

#include <Eigen/Geometry>

using namespace dart;

namespace {

dynamics::SkeletonPtr createFloor()
{
  auto floor = dynamics::Skeleton::create("floor");
  auto [joint, body]
      = floor->createJointAndBodyNodePair<dynamics::WeldJoint, dynamics::BodyNode>();
  auto shape = std::make_shared<dynamics::BoxShape>(
      Eigen::Vector3d(5.0, 5.0, 0.2));
  auto* shapeNode = body->createShapeNodeWith<
      dynamics::VisualAspect,
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);
  shapeNode->getVisualAspect()->setColor(Eigen::Vector4d(0.8, 0.8, 0.8, 1.0));
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0.0, 0.0, -0.1);
  joint->setTransformFromParentBodyNode(tf);
  return floor;
}

dynamics::SkeletonPtr createBox()
{
  auto box = dynamics::Skeleton::create("box");
  auto [joint, body]
      = box->createJointAndBodyNodePair<dynamics::FreeJoint, dynamics::BodyNode>();
  auto shape
      = std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Constant(0.2));
  auto* shapeNode = body->createShapeNodeWith<
      dynamics::VisualAspect,
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);
  shapeNode->getVisualAspect()->setColor(Eigen::Vector4d(0.2, 0.4, 0.8, 1.0));
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0.0, 0.0, 1.0);
  joint->setTransformFromParentBodyNode(tf);
  return box;
}

} // namespace

int main()
{
  auto world = std::make_shared<simulation::World>();
  world->setName("vsg-example");
  world->addSkeleton(createFloor());
  world->addSkeleton(createBox());

  gui::vsg::Viewer viewer(world);
  viewer.simulate(true);
  viewer.setNumStepsPerCycle(4);
  viewer.run();
}
