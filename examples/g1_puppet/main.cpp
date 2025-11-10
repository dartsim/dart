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

#include <dart/gui/osg/SupportPolygonVisual.hpp>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::utils;

namespace {

const double kSupportVisualElevation = 0.02;

struct IkHandle
{
  dart::dynamics::EndEffector* effector = nullptr;
  dart::dynamics::InverseKinematicsPtr ik;
  dart::gui::osg::InteractiveFramePtr frame;
  dart::simulation::WorldPtr world;
  int hotkey = 0;
  bool active = false;
  bool attached = false;
};

class G1WorldNode : public dart::gui::osg::WorldNode
{
public:
  using dart::gui::osg::WorldNode::WorldNode;

  void addIkHandle(const std::shared_ptr<IkHandle>& handle)
  {
    mHandles.push_back(handle);
  }

  void customPreRefresh() override
  {
    for (const auto& handle : mHandles) {
      if (!handle->active)
        continue;

      handle->ik->getTarget()->setTransform(handle->frame->getTransform());
      handle->ik->solveAndApply(true);
    }
  }

private:
  std::vector<std::shared_ptr<IkHandle>> mHandles;
};

class IkInputHandler : public ::osgGA::GUIEventHandler
{
public:
  explicit IkInputHandler(std::vector<std::shared_ptr<IkHandle>> handles)
    : mHandles(std::move(handles))
  {
  }

  bool handle(
      const ::osgGA::GUIEventAdapter& ea, ::osgGA::GUIActionAdapter&) override
  {
    if (ea.getEventType() != ::osgGA::GUIEventAdapter::KEYDOWN)
      return false;

    const int key = ea.getKey();
    for (const auto& handle : mHandles) {
      if (key != handle->hotkey)
        continue;

      handle->active = !handle->active;
      if (handle->active) {
        if (!handle->attached) {
          handle->world->addSimpleFrame(handle->frame);
          handle->attached = true;
        }
        handle->frame->setTransform(handle->effector->getTransform());
        std::cout << "Activated IK target '" << handle->effector->getName()
                  << "'.\n";
      } else {
        if (handle->attached) {
          handle->world->removeSimpleFrame(handle->frame);
          handle->attached = false;
        }
        std::cout << "Deactivated IK target '" << handle->effector->getName()
                  << "'.\n";
      }
      return true;
    }

    return false;
  }

private:
  std::vector<std::shared_ptr<IkHandle>> mHandles;
};

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

std::vector<std::shared_ptr<IkHandle>> setupIkHandles(
    const SkeletonPtr& robot,
    const WorldPtr& world,
    dart::gui::osg::Viewer& viewer,
    const ::osg::ref_ptr<G1WorldNode>& worldNode)
{
  struct Config
  {
    std::string bodyNode;
    Eigen::Vector3d offset;
    int key;
  };

  const std::vector<Config> configs = {
      {"left_rubber_hand", Eigen::Vector3d(0.0, 0.0, 0.0), '1'},
      {"right_rubber_hand", Eigen::Vector3d(0.0, 0.0, 0.0), '2'},
      {"left_ankle_roll_link", Eigen::Vector3d(0.0, 0.0, 0.0), '3'},
      {"right_ankle_roll_link", Eigen::Vector3d(0.0, 0.0, 0.0), '4'},
      {"left_knee_link", Eigen::Vector3d(0.0, 0.0, 0.2), '5'},
      {"right_knee_link", Eigen::Vector3d(0.0, 0.0, 0.2), '6'}};

  std::vector<std::shared_ptr<IkHandle>> handles;
  handles.reserve(configs.size());

  for (const auto& config : configs) {
    dart::dynamics::BodyNode* bn = robot->getBodyNode(config.bodyNode);
    if (!bn) {
      DART_WARN(
          "Unable to find body node '{}' for IK target.", config.bodyNode);
      continue;
    }

    auto* ee = bn->createEndEffector(config.bodyNode + "_target");
    auto ik = ee->getIK(true);

    const Eigen::Isometry3d tf
        = Eigen::Translation3d(config.offset) * ee->getTransform();
    auto frame = dart::gui::osg::InteractiveFrame::createShared(
        dart::dynamics::Frame::World(), ee->getName() + "_frame", tf, 0.15);

    viewer.enableDragAndDrop(frame.get());

    auto simpleTarget
        = std::static_pointer_cast<dart::dynamics::SimpleFrame>(frame);
    ik->setTarget(simpleTarget);

    auto handle = std::make_shared<IkHandle>();
    handle->effector = ee;
    handle->ik = ik;
    handle->frame = frame;
    handle->world = world;
    handle->hotkey = config.key;

    handles.push_back(handle);
    worldNode->addIkHandle(handle);
  }

  return handles;
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
    tf.translation().z() = 0.75;
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

  ::osg::ref_ptr<G1WorldNode> worldNode = new G1WorldNode(world);
  dart::gui::osg::Viewer viewer;
  viewer.addWorldNode(worldNode);
  viewer.allowSimulation(false);

  auto grid = ::osg::ref_ptr<dart::gui::osg::GridVisual>(
      new dart::gui::osg::GridVisual());
  grid->setPlaneType(dart::gui::osg::GridVisual::PlaneType::XY);
  grid->setNumCells(40);
  grid->setMinorLineStepSize(0.1);
  grid->setOffset(Eigen::Vector3d::Zero());
  viewer.addAttachment(grid);
  viewer.addAttachment(new dart::gui::osg::SupportPolygonVisual(
      g1, kSupportVisualElevation));

  enableDragAndDrop(viewer, g1);

  auto ikHandles = setupIkHandles(g1, world, viewer, worldNode);
  viewer.addEventHandler(new IkInputHandler(ikHandles));

  viewer.setUpViewInWindow(0, 0, 1280, 960);
  viewer.getCameraManipulator()->setHomePosition(
      ::osg::Vec3(3.0, 1.6, 1.4),
      ::osg::Vec3(0.0, 0.0, 1.0),
      ::osg::Vec3(0.0, 0.0, 1.0));
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  std::cout << "Use the mouse to drag body nodes.\n"
            << "Press keys 1-6 to toggle IK targets (hands, feet, elbows). "
               "Close the viewer window to exit."
            << std::endl;

  return viewer.run();
}
