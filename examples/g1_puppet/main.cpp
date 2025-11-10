#include <dart/gui/osg/all.hpp>

#include <dart/utils/CompositeResourceRetriever.hpp>
#include <dart/utils/DartResourceRetriever.hpp>
#include <dart/utils/HttpResourceRetriever.hpp>
#include <dart/utils/PackageResourceRetriever.hpp>
#include <dart/utils/urdf/DartLoader.hpp>

#include <dart/simulation/World.hpp>

#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/ShapeNode.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/WeldJoint.hpp>

#include <dart/common/LocalResourceRetriever.hpp>

#include <dart/all.hpp>

#include <iostream>
#include <memory>
#include <string>

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::utils;

namespace {

struct Options
{
  std::string packageName = "g1_description";
  std::string packageUri
      = "https://raw.githubusercontent.com/unitreerobotics/unitree_ros/"
        "master/robots/g1_description";
  std::string robotUri = "package://g1_description/g1_29dof.urdf";
};

void printUsage(const char* executable)
{
  std::cout
      << "Usage: " << executable << " [--package-uri URI] [--robot-uri URI]"
      << "\n\n"
      << "This example downloads the Unitree G1 humanoid URDF directly "
         "from GitHub.\n"
      << "Provide --package-uri to point at a different ROS package root, "
         "and\n"
      << "--robot-uri for an alternative URDF/SDF.\n";
}

Options parseCommandLine(int argc, char* argv[])
{
  Options options;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if ((arg == "--package-uri" || arg == "-p") && i + 1 < argc) {
      options.packageUri = argv[++i];
      continue;
    }

    if ((arg == "--robot-uri" || arg == "-r") && i + 1 < argc) {
      options.robotUri = argv[++i];
      continue;
    }

    if (arg == "--help" || arg == "-h") {
      printUsage(argv[0]);
      std::exit(0);
    }

    std::cerr << "Unknown or incomplete argument '" << arg << "'.\n";
    printUsage(argv[0]);
    std::exit(1);
  }

  return options;
}

ResourceRetrieverPtr createResourceRetriever(const Options& options)
{
  auto local = std::make_shared<LocalResourceRetriever>();
  auto dartRetriever = std::make_shared<DartResourceRetriever>();
  auto http = std::make_shared<HttpResourceRetriever>();

  auto passthrough = std::make_shared<CompositeResourceRetriever>();
  passthrough->addSchemaRetriever("file", local);
  passthrough->addSchemaRetriever("dart", dartRetriever);
  passthrough->addSchemaRetriever("http", http);
  passthrough->addSchemaRetriever("https", http);
  passthrough->addDefaultRetriever(local);

  auto packageRetriever
      = std::make_shared<PackageResourceRetriever>(passthrough);
  packageRetriever->addPackageDirectory(
      options.packageName, options.packageUri);

  auto resolver = std::make_shared<CompositeResourceRetriever>();
  resolver->addSchemaRetriever("package", packageRetriever);
  resolver->addSchemaRetriever("file", local);
  resolver->addSchemaRetriever("dart", dartRetriever);
  resolver->addSchemaRetriever("http", http);
  resolver->addSchemaRetriever("https", http);
  resolver->addDefaultRetriever(local);

  return resolver;
}

SkeletonPtr createGround()
{
  SkeletonPtr ground = Skeleton::create("ground");

  WeldJoint::Properties joint;
  joint.mName = "ground_joint";

  BodyNode::Properties body;
  body.mName = "ground_body";

  auto [weld, bodyNode]
      = ground->createJointAndBodyNodePair<WeldJoint>(nullptr, joint, body);
  DART_UNUSED(weld);

  const auto thickness = 0.1;
  auto shape = std::make_shared<BoxShape>(Eigen::Vector3d(8, 8, thickness));
  auto shapeNode = bodyNode->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(shape);
  shapeNode->getVisualAspect()->setColor(dart::Color::Gray(0.4));
  shapeNode->setRelativeTranslation(Eigen::Vector3d(0, 0, -thickness / 2.0));

  return ground;
}

SkeletonPtr loadG1(
    const Options& options, const ResourceRetrieverPtr& retriever)
{
  DartLoader::Options loaderOptions;
  loaderOptions.mResourceRetriever = retriever;
  DartLoader loader(loaderOptions);

  SkeletonPtr robot = loader.parseSkeleton(options.robotUri);
  if (!robot) {
    std::cerr << "Failed to load robot from '" << options.robotUri << "'.\n";
    return nullptr;
  }

  if (auto* freeJoint = dynamic_cast<FreeJoint*>(robot->getJoint(0))) {
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.translation().z() = 1.0;
    FreeJoint::setTransformOf(freeJoint, tf);
  }

  return robot;
}

void enableDragAndDrop(dart::gui::osg::Viewer& viewer, const SkeletonPtr& robot)
{
  for (std::size_t i = 0; i < robot->getNumBodyNodes(); ++i)
    viewer.enableDragAndDrop(robot->getBodyNode(i), false, false);
}

} // namespace

int main(int argc, char* argv[])
{
  const Options options = parseCommandLine(argc, argv);
  auto retriever = createResourceRetriever(options);

  WorldPtr world = World::create();
  world->setGravity(Eigen::Vector3d(0, 0, -9.81));

  SkeletonPtr ground = createGround();
  world->addSkeleton(ground);

  SkeletonPtr g1 = loadG1(options, retriever);
  if (!g1)
    return 1;

  g1->setName("G1");
  world->addSkeleton(g1);

  std::cout << "Loaded G1 robot from '" << options.robotUri << "'.\n"
            << "Package root for '" << options.packageName << "' set to '"
            << options.packageUri << "'.\n";

  auto worldNode = new dart::gui::osg::WorldNode(world);
  dart::gui::osg::Viewer viewer;
  viewer.addWorldNode(worldNode);
  viewer.allowSimulation(false);

  enableDragAndDrop(viewer, g1);

  viewer.setUpViewInWindow(0, 0, 1280, 960);
  viewer.getCameraManipulator()->setHomePosition(
      ::osg::Vec3(3.5, 2.0, 1.8),
      ::osg::Vec3(0.0, 0.0, 1.2),
      ::osg::Vec3(0.0, 0.0, 1.0));

  std::cout << "Use the mouse to drag body nodes. Close the viewer window to "
               "exit."
            << std::endl;

  return viewer.run();
}
