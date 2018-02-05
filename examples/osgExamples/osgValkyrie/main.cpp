#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/utils.hpp>
#include <dart/utils/urdf/urdf.hpp>

#include "worldnode.hpp"
#include "Configuration.h"

void setMeshColor(dart::dynamics::SkeletonPtr robot) {
    for(std::size_t i=0; i<robot->getNumBodyNodes(); ++i) {
        dart::dynamics::BodyNode* bn = robot->getBodyNode(i);
        auto shapeNodes = bn->getShapeNodesWith<dart::dynamics::VisualAspect>();
        for(auto shapeNode : shapeNodes) {
            std::shared_ptr<dart::dynamics::MeshShape> mesh =
                std::dynamic_pointer_cast<dart::dynamics::MeshShape>(
                        shapeNode->getShape());
            if(mesh)
                mesh->setColorMode(dart::dynamics::MeshShape::MATERIAL_COLOR);
        }
    }
}

int main() {
    //// Generate world and add skeletons
    dart::simulation::WorldPtr world(new dart::simulation::World);
    dart::utils::DartLoader urdfLoader;
    dart::dynamics::SkeletonPtr ground = urdfLoader.parseSkeleton(
            "dart://sample/urdf/valkyrie/ground.urdf");
    dart::dynamics::SkeletonPtr robot = urdfLoader.parseSkeleton(
            "dart://sample/urdf/valkyrie/valkyrie.urdf");
    world->addSkeleton(ground);
    world->addSkeleton(robot);
    Eigen::Vector3d gravity(0.0, 0.0, -9.81);
    world->setGravity(gravity);
    world->setTimeStep(1.0/1000);

    // Initial configuration
    Eigen::VectorXd q = robot->getPositions();
    q[5] = 1.2;
    robot->setPositions(q);


    // Set mesh color
//    setMeshColor(robot);

    //// Wrap a worldnode
    osg::ref_ptr<WorldNode> node
        = new WorldNode(world);
    node->setNumStepsPerCycle(10);

    //// Create viewer
    dart::gui::osg::Viewer viewer;
    viewer.addWorldNode(node);
//    viewer.simulate(true);
    std::cout << viewer.getInstructions() << std::endl;
    viewer.setUpViewInWindow(0, 0, 1280, 960);
    viewer.getCameraManipulator()->setHomePosition(
            ::osg::Vec3( 2.57,  3.14, 2.)*2.0,
            ::osg::Vec3( 0.00,  0.00, 1.00),
            ::osg::Vec3(-0.24, -0.25, 0.94));    viewer.setCameraManipulator(viewer.getCameraManipulator());
    viewer.run();
}
