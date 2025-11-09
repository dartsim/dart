#include <dart/gui/osg/all.hpp>

#include <dart/all.hpp>

#include <algorithm>
#include <array>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

using namespace dart;

namespace {

constexpr int kSegmentCount = 6;
constexpr double kSegmentLength = 0.45;
constexpr double kSegmentRadius = 0.05;
constexpr double kJointSphereRadius = 0.06;
constexpr double kChainElevation = 0.5;

constexpr double kLetterHeight = 0.7;
constexpr double kLetterThickness = 0.08;
constexpr double kLetterDepth = 0.08;
constexpr double kLetterSpacing = 0.55;
constexpr double kLetterElevation = 0.1;

constexpr double kRampDuration = 5.0;

const std::array<Eigen::Vector3d, kSegmentCount> kRainbowColors = {
    Eigen::Vector3d(0.88, 0.18, 0.23), // red
    Eigen::Vector3d(0.98, 0.50, 0.15), // orange
    Eigen::Vector3d(0.99, 0.80, 0.12), // yellow
    Eigen::Vector3d(0.37, 0.69, 0.32), // green
    Eigen::Vector3d(0.0, 0.67, 0.56),  // teal
    Eigen::Vector3d(0.57, 0.33, 0.66)  // purple
};

void setStaticInertia(dynamics::BodyNode* body)
{
  dynamics::Inertia inertia;
  inertia.setMass(0.1);
  inertia.setMoment(Eigen::Matrix3d::Identity() * 1e-4);
  body->setInertia(inertia);
}

void addBoxShape(
    dynamics::BodyNode* body,
    const Eigen::Vector3d& dims,
    const Eigen::Vector3d& offset,
    const Eigen::Vector3d& color)
{
  auto shape = std::make_shared<dynamics::BoxShape>(dims);
  auto node = body->createShapeNodeWith<dynamics::VisualAspect>(shape);
  node->getVisualAspect()->setColor(color);
  node->setRelativeTranslation(offset);
}

void addCapsuleShape(
    dynamics::BodyNode* body,
    const Eigen::Vector3d& dims,
    const Eigen::Vector3d& offset,
    const Eigen::Vector3d& axis,
    const Eigen::Vector3d& color)
{
  auto shape
      = std::make_shared<dynamics::CapsuleShape>(dims[0], dims[1]); // radius, height
  auto node = body->createShapeNodeWith<dynamics::VisualAspect>(shape);
  node->getVisualAspect()->setColor(color);

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  if (!axis.isApprox(Eigen::Vector3d::UnitZ())) {
    tf.linear() = Eigen::Quaterniond::FromTwoVectors(
                      Eigen::Vector3d::UnitZ(), axis.normalized())
                      .toRotationMatrix();
  }
  tf.translation() = offset;
  node->setRelativeTransform(tf);
}

void addSphereShape(
    dynamics::BodyNode* body,
    double radius,
    const Eigen::Vector3d& offset,
    const Eigen::Vector3d& color)
{
  auto shape = std::make_shared<dynamics::SphereShape>(radius);
  auto node = body->createShapeNodeWith<dynamics::VisualAspect>(shape);
  node->getVisualAspect()->setColor(color);
  node->setRelativeTranslation(offset);
}

dynamics::SkeletonPtr createGround()
{
  auto ground = dynamics::Skeleton::create("ground");
  auto pair
      = ground->createJointAndBodyNodePair<dynamics::WeldJoint>(nullptr);
  auto body = pair.second;

  auto shape = std::make_shared<dynamics::BoxShape>(
      Eigen::Vector3d(12.0, 12.0, 0.05));
  auto node = body->createShapeNodeWith<dynamics::VisualAspect>(shape);
  node->getVisualAspect()->setColor(dart::Color::LightGray());
  node->setRelativeTranslation(Eigen::Vector3d(0.0, 0.0, -0.025));

  setStaticInertia(body);

  return ground;
}

dynamics::SkeletonPtr createLetterD(const Eigen::Vector3d& position)
{
  auto skeleton = dynamics::Skeleton::create("letter_D");
  auto pair
      = skeleton->createJointAndBodyNodePair<dynamics::WeldJoint>(nullptr);
  auto joint = pair.first;
  auto body = pair.second;
  joint->setTransformFromParentBodyNode(Eigen::Translation3d(position));

  const Eigen::Vector3d color = Eigen::Vector3d::Constant(0.05);
  const double width = 0.52;

  addBoxShape(
      body,
      Eigen::Vector3d(kLetterThickness, kLetterHeight, kLetterDepth),
      Eigen::Vector3d(-width / 2.0 + kLetterThickness / 2.0, 0, 0),
      color);
  addBoxShape(
      body,
      Eigen::Vector3d(width - kLetterThickness / 2.0, kLetterThickness, kLetterDepth),
      Eigen::Vector3d(kLetterThickness / 2.0, kLetterHeight / 2.0 - kLetterThickness / 2.0, 0),
      color);
  addBoxShape(
      body,
      Eigen::Vector3d(width - kLetterThickness / 2.0, kLetterThickness, kLetterDepth),
      Eigen::Vector3d(kLetterThickness / 2.0, -kLetterHeight / 2.0 + kLetterThickness / 2.0, 0),
      color);

  addCapsuleShape(
      body,
      Eigen::Vector3d((width - kLetterThickness) / 2.0, kLetterHeight - 2.0 * kLetterThickness, 0),
      Eigen::Vector3d(width / 2.0 - kLetterThickness / 2.0, 0, 0),
      Eigen::Vector3d::UnitY(),
      color);

  setStaticInertia(body);

  return skeleton;
}

dynamics::SkeletonPtr createLetterA(const Eigen::Vector3d& position)
{
  auto skeleton = dynamics::Skeleton::create("letter_A");
  auto pair
      = skeleton->createJointAndBodyNodePair<dynamics::WeldJoint>(nullptr);
  auto joint = pair.first;
  auto body = pair.second;
  joint->setTransformFromParentBodyNode(Eigen::Translation3d(position));

  const Eigen::Vector3d color = Eigen::Vector3d::Constant(0.05);
  const double width = 0.55;

  addCapsuleShape(
      body,
      Eigen::Vector3d(kLetterThickness / 2.0, kLetterHeight, 0),
      Eigen::Vector3d(-width / 2.0, 0, 0),
      Eigen::Vector3d(0.3, 0, 1).normalized(),
      color);
  addCapsuleShape(
      body,
      Eigen::Vector3d(kLetterThickness / 2.0, kLetterHeight, 0),
      Eigen::Vector3d(width / 2.0, 0, 0),
      Eigen::Vector3d(-0.3, 0, 1).normalized(),
      color);
  addBoxShape(
      body,
      Eigen::Vector3d(width - kLetterThickness, kLetterThickness, kLetterDepth),
      Eigen::Vector3d(0, -0.05, 0),
      color);

  setStaticInertia(body);

  return skeleton;
}

dynamics::SkeletonPtr createLetterR(const Eigen::Vector3d& position)
{
  auto skeleton = dynamics::Skeleton::create("letter_R");
  auto pair
      = skeleton->createJointAndBodyNodePair<dynamics::WeldJoint>(nullptr);
  auto joint = pair.first;
  auto body = pair.second;
  joint->setTransformFromParentBodyNode(Eigen::Translation3d(position));

  const Eigen::Vector3d color = Eigen::Vector3d::Constant(0.05);
  const double width = 0.52;

  addBoxShape(
      body,
      Eigen::Vector3d(kLetterThickness, kLetterHeight, kLetterDepth),
      Eigen::Vector3d(-width / 2.0 + kLetterThickness / 2.0, 0, 0),
      color);
  addBoxShape(
      body,
      Eigen::Vector3d(width - kLetterThickness / 2.0, kLetterThickness, kLetterDepth),
      Eigen::Vector3d(kLetterThickness / 2.0, kLetterHeight / 2.0 - kLetterThickness / 2.0, 0),
      color);
  addBoxShape(
      body,
      Eigen::Vector3d(width - kLetterThickness / 2.0, kLetterThickness, kLetterDepth),
      Eigen::Vector3d(kLetterThickness / 2.0, 0.05, 0),
      color);
  addCapsuleShape(
      body,
      Eigen::Vector3d((width - kLetterThickness) / 2.0, (kLetterHeight / 2.0) - kLetterThickness, 0),
      Eigen::Vector3d(width / 2.0 - kLetterThickness / 2.0, 0.2, 0),
      Eigen::Vector3d::UnitY(),
      color);
  addCapsuleShape(
      body,
      Eigen::Vector3d(kLetterThickness / 2.0, kLetterHeight / 2.0, 0),
      Eigen::Vector3d(width / 2.0 - 0.1, -kLetterHeight / 4.0, 0),
      (Eigen::Vector3d(width / 2.0 - 0.1, -kLetterHeight / 4.0, 0)).normalized(),
      color);

  setStaticInertia(body);

  return skeleton;
}

dynamics::SkeletonPtr createLetterT(const Eigen::Vector3d& position)
{
  auto skeleton = dynamics::Skeleton::create("letter_T");
  auto pair
      = skeleton->createJointAndBodyNodePair<dynamics::WeldJoint>(nullptr);
  auto joint = pair.first;
  auto body = pair.second;
  joint->setTransformFromParentBodyNode(Eigen::Translation3d(position));

  const Eigen::Vector3d color = Eigen::Vector3d::Constant(0.05);
  const double width = 0.55;

  addBoxShape(
      body,
      Eigen::Vector3d(width, kLetterThickness, kLetterDepth),
      Eigen::Vector3d(0, kLetterHeight / 2.0 - kLetterThickness / 2.0, 0),
      color);
  addBoxShape(
      body,
      Eigen::Vector3d(kLetterThickness, kLetterHeight, kLetterDepth),
      Eigen::Vector3d(0, 0, 0),
      color);

  setStaticInertia(body);

  return skeleton;
}

dynamics::SkeletonPtr createArrow(const Eigen::Vector3d& position)
{
  auto skeleton = dynamics::Skeleton::create("logo_arrow");
  auto pair
      = skeleton->createJointAndBodyNodePair<dynamics::WeldJoint>(nullptr);
  auto joint = pair.first;
  auto body = pair.second;
  joint->setTransformFromParentBodyNode(Eigen::Translation3d(position));

  const Eigen::Vector3d color = Eigen::Vector3d::Constant(0.05);

  addBoxShape(
      body,
      Eigen::Vector3d(0.55, kLetterThickness * 0.8, kLetterDepth),
      Eigen::Vector3d(-0.25, 0, 0),
      color);

  auto head = std::make_shared<dynamics::ConeShape>(0.18, 0.35);
  auto node = body->createShapeNodeWith<dynamics::VisualAspect>(head);
  node->getVisualAspect()->setColor(color);
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0.15, 0, 0);
  tf.linear()
      = Eigen::AngleAxisd(math::constantsd::pi() / 2.0, Eigen::Vector3d::UnitY())
            .toRotationMatrix();
  node->setRelativeTransform(tf);

  setStaticInertia(body);

  return skeleton;
}

dynamics::SkeletonPtr createRainbowChain(Eigen::VectorXd& targetAngles)
{
  auto skeleton = dynamics::Skeleton::create("rainbow_chain");

  auto basePair
      = skeleton->createJointAndBodyNodePair<dynamics::WeldJoint>(nullptr);
  auto baseJoint = basePair.first;
  auto baseBody = basePair.second;
  baseJoint->setTransformFromParentBodyNode(
      Eigen::Translation3d(Eigen::Vector3d(0.0, 0.0, kChainElevation)));
  setStaticInertia(baseBody);

  dynamics::BodyNode* parentBody = baseBody;

  targetAngles = Eigen::VectorXd::Zero(kSegmentCount);
  for (int i = 0; i < kSegmentCount; ++i) {
    auto pair = skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
        parentBody);
    auto joint = pair.first;
    auto body = pair.second;

    joint->setAxis(Eigen::Vector3d::UnitZ());
    joint->setTransformFromParentBodyNode(
        (i == 0) ? Eigen::Isometry3d::Identity()
                 : Eigen::Translation3d(kSegmentLength, 0, 0));
    joint->setTransformFromChildBodyNode(Eigen::Isometry3d::Identity());
    joint->setPositionLimits(
        Eigen::Vector2d(-math::constantsd::pi(), math::constantsd::pi()));
    joint->setDampingCoefficient(0, 0.5);

    auto shape = std::make_shared<dynamics::BoxShape>(
        Eigen::Vector3d(kSegmentLength, kSegmentRadius * 0.9, kSegmentRadius * 0.9));
    auto shapeNode = body->createShapeNodeWith<
        dynamics::VisualAspect,
        dynamics::CollisionAspect,
        dynamics::DynamicsAspect>(shape);
    shapeNode->getVisualAspect()->setColor(kRainbowColors[i]);
    shapeNode->setRelativeTranslation(
        Eigen::Vector3d(kSegmentLength / 2.0, 0.0, 0.0));

    body->setLocalCOM(Eigen::Vector3d(kSegmentLength / 2.0, 0.0, 0.0));
    dynamics::Inertia inertia;
    inertia.setMass(0.35);
    inertia.setMoment(shape->computeInertia(inertia.getMass()));
    inertia.setLocalCOM(body->getLocalCOM());
    body->setInertia(inertia);

    addSphereShape(
        parentBody,
        kJointSphereRadius,
        (i == 0) ? Eigen::Vector3d::Zero()
                 : Eigen::Vector3d(kSegmentLength, 0.0, 0.0),
        kRainbowColors[i]);

    parentBody = body;
    targetAngles[i] = (i == 0) ? 0.0 : math::constantsd::pi() / 3.0;
  }

  return skeleton;
}

class LogoChainWorldNode final : public gui::osg::RealTimeWorldNode
{
public:
  LogoChainWorldNode(
      const simulation::WorldPtr& world,
      dynamics::SkeletonPtr chain,
      Eigen::VectorXd target)
    : gui::osg::RealTimeWorldNode(world),
      mChain(std::move(chain)),
      mTarget(std::move(target)),
      mKp(Eigen::VectorXd::Constant(mTarget.size(), 120.0)),
      mKd(Eigen::VectorXd::Constant(mTarget.size(), 12.0))
  {
  }

  void customPreStep() override
  {
    if (mTarget.size() == 0)
      return;

    const double t = getWorld()->getTime();
    const double alpha = std::clamp(t / kRampDuration, 0.0, 1.0);
    const Eigen::VectorXd desired = alpha * mTarget;

    const Eigen::VectorXd q = mChain->getPositions();
    const Eigen::VectorXd dq = mChain->getVelocities();

    const Eigen::VectorXd error = desired - q;
    const Eigen::VectorXd dError = -dq;

    const Eigen::VectorXd tau = mKp.cwiseProduct(error) + mKd.cwiseProduct(dError);
    mChain->setForces(tau);
  }

private:
  dynamics::SkeletonPtr mChain;
  Eigen::VectorXd mTarget;
  Eigen::VectorXd mKp;
  Eigen::VectorXd mKd;
};

void addLettering(simulation::WorldPtr world)
{
  const Eigen::Vector3d baseline(0.8, -0.85, kLetterElevation);
  std::vector<dynamics::SkeletonPtr> letters;
  letters.emplace_back(createLetterD(baseline + Eigen::Vector3d(0.0, 0.0, 0.0)));
  letters.emplace_back(createLetterA(
      baseline + Eigen::Vector3d(kLetterSpacing, 0.0, 0.0)));
  letters.emplace_back(createLetterR(
      baseline + Eigen::Vector3d(2.0 * kLetterSpacing, 0.0, 0.0)));
  letters.emplace_back(createLetterT(
      baseline + Eigen::Vector3d(3.0 * kLetterSpacing, 0.0, 0.0)));

  auto arrow = createArrow(baseline + Eigen::Vector3d(-0.7, 0.1, 0.0));

  for (auto& skel : letters) {
    world->addSkeleton(skel);
  }
  world->addSkeleton(arrow);
}

} // namespace

int main()
{
  auto world = simulation::World::create();
  world->setGravity(Eigen::Vector3d::Zero());
  world->setTimeStep(1.0 / 1000.0);

  Eigen::VectorXd targetAngles;
  auto chain = createRainbowChain(targetAngles);

  chain->setPositions(Eigen::VectorXd::Zero(chain->getNumDofs()));
  chain->setVelocities(Eigen::VectorXd::Zero(chain->getNumDofs()));

  world->addSkeleton(createGround());
  world->addSkeleton(chain);

  addLettering(world);

  auto node = ::osg::ref_ptr<LogoChainWorldNode>(
      new LogoChainWorldNode(world, chain, targetAngles));

  gui::osg::Viewer viewer;
  viewer.addWorldNode(node);

  auto shadow
      = gui::osg::WorldNode::createDefaultShadowTechnique(&viewer);
  node->setShadowTechnique(shadow);

  viewer.setUpViewInWindow(0, 0, 960, 720);
  viewer.getCameraManipulator()->setHomePosition(
      ::osg::Vec3(2.5f, 3.5f, 1.6f),
      ::osg::Vec3(0.5f, 0.0f, kChainElevation),
      ::osg::Vec3(-0.2f, -0.2f, 0.95f));
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  viewer.addInstructionText(
      "Press space to let the rainbow chain settle into the DART logo pose.");
  std::cout << viewer.getInstructions() << std::endl;

  viewer.run();

  return 0;
}
