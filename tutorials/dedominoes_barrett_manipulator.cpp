#include "dart/dart.h"

using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::math;

class Controller
{
public:
	Controller(const SkeletonPtr& manipulator)
		: mManipulator(manipulator)
	{
	}
protected:
	SkeletonPtr mManipulator;
	
	//simpleframe inherit from frame
	SimpleFramePtr mTarget;

	BodyNodePtr mEndEffector;

	//desired joint positions when not applying the operational space controller
	Eigen::VectorXd mQDesired;

	double mKpPD;
	double mKdPD;
	double mKpOS;
	double mKdOS;

	//joint forces for the manipulator (output of the Controller)
	Eigen::VectorXd mForces;
};

class MyWindow : public dart::gui::SimWindow
{
public:
	MyWindow(const WorldPtr& world)
		: mHasEverRun(false)
	{
		setWorld(world);
		mController = std::unique_ptr<Controller> (
			new Controller(world->getSkeleton("manipulator")));
	}
	
	void keyboard(unsigned char key, int x, int y) override
	{
		if (!mHasEverRun)
		{
			switch (key)
			{
				case ' ':
					mHasEverRun = !mHasEverRun;
				break;
			}
		}
		SimWindow::keyboard(key, x, y);
	}

	
	void timeStepping() override
	{
		SimWindow::timeStepping();
	}
protected:
	SkeletonPtr mDomino;
	SkeletonPtr mFloor;
	bool mHasEverRun;
	int mPushCountCount;
	std::unique_ptr<Controller> mController;
};

SkeletonPtr createFloor()
{
	SkeletonPtr floor = Skeleton::create("floor");
	
	// Give the floor a body
	BodyNodePtr body = floor->createJointAndBodyNodePair<WeldJoint>(nullptr).second;

	// Give the body a shape
	double floor_width = 5;
	double floor_height = 0.25;

	std::shared_ptr<BoxShape> box(
			new BoxShape(Eigen::Vector3d(floor_width, floor_width, floor_height)));
	box->setColor(dart::Color::Black());

	body->addVisualizationShape(box);
	body->addCollisionShape(box);

	// put the body into position
	Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
	tf.translation() = Eigen::Vector3d(0.0, 0.0, -floor_height / 2.0);
	body->getParentJoint()->setTransformFromParentBodyNode(tf);

	return floor;
}

SkeletonPtr createManipulator()
{
	dart::utils::DartLoader loader;
	SkeletonPtr manipulator = 
		loader.parseSkeleton(DART_DATA_PATH"/my_urdf/customized_wam_7dof_wam_bhand.urdf");
	manipulator->setName("manipulator");
	Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();

	manipulator->getJoint(0)->setTransformFromParentBodyNode(tf);

	// set initial position for each DOF
	manipulator->getDof(1)->setPosition(-100.0 * M_PI / 180.0);
	manipulator->getDof(3)->setPosition(180.0 * M_PI / 180.0);
	
	//set Joint position limit to make the manipulator not collapse into itself.
	for (size_t i = 0 ; i < manipulator->getNumJoints(); ++ i)
	{
		manipulator->getJoint(i)->setPositionLimitEnforced(true);
	}
	
	//manipulator->enableSelfCollision();

	return manipulator;
}


int main(int argc, char*argv[])
{
	SkeletonPtr floor = createFloor();
	SkeletonPtr manipulator = createManipulator();

	WorldPtr world = std::make_shared<World>();
	world->addSkeleton(floor);
	world->addSkeleton(manipulator);



	MyWindow window(world);

	std::cout << "Before simulation has started, you can ..." <<std::endl;
	std::cout << "The gravity is: [" << world->getGravity().transpose() << "]" << std::endl;
	std::cout << "spacebar: initialize the simulation." <<std::endl;

	glutInit(&argc, argv);
	window.initWindow(640,480,"Wam Arm & Barrett Hand by Yang Tian");
	glutMainLoop();
}
