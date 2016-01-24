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
					mHasEverRun = true;
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

SkeletonPtr createManipulator()
{
	dart::utils::DartLoader loader;
	SkeletonPtr manipulator = 
		loader.parseSkeleton(DART_DATA_PATH"/my_urdf/customized_wam_7dof_wam_bhand.urdf");
	manipulator->setName("manipulator");
	Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
	//tf.translation() = Eigen::Vector3d(0.000908563, -0.208275, -0.0256744);
	

	manipulator->getJoint(0)->setTransformFromParentBodyNode(tf);
	return manipulator;
}


int main(int argc, char*argv[])
{
	SkeletonPtr manipulator = createManipulator();

	WorldPtr world = std::make_shared<World>();
	world->addSkeleton(manipulator);

	MyWindow window(world);

	glutInit(&argc, argv);
	window.initWindow(640,480,"Wam Arm & Barrett Hand by Yang Tian");
	glutMainLoop();
}
