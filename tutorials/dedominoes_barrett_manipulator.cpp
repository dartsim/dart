#include "dart/dart.h"

using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::math;
using namespace dart::gui;

double push_duration = 1000;
double push_force = 8.0;

double floor_width = 5;
double floor_height = 0.1;

double domino_height = 0.3;
double domino_width = 0.4 * domino_height;
double domino_depth = domino_width / 5.0;

double domino_density = 2.6e3;
double domino_mass = domino_density * domino_height * domino_width * domino_depth;
double endeffector_offset = 0;

class Controller
{
public:
	Controller(const SkeletonPtr& manipulator, const SkeletonPtr& domino)
		: mManipulator(manipulator)
	{
		// Setup controller for static positions maintenance
		mQDesired = mManipulator->getPositions();

		// Setup information needed for operational space controller

		for (size_t i =0; i<mManipulator->getNumBodyNodes() ; i++)
		{
			std::cout<<"the "<<i<<"th BodyNode is: "<<mManipulator->getBodyNode(i)->getName()<<std::endl;
		}

		mEndEffector = mManipulator->getBodyNode(mManipulator->getNumBodyNodes()-1);
		
		mOffset = endeffector_offset * Eigen::Vector3d::UnitX();
		
		mTarget = std::make_shared<SimpleFrame>(Frame::World(), "target");

		// compute the transform needed to get from the center of the domino to the top of the domino
		Eigen::Isometry3d target_offset(Eigen::Isometry3d::Identity());
		target_offset.translation() = domino_height / 2.0 * Eigen::Vector3d::UnitZ();

	    // rotate the target's coordinate frame to make sure that it lines up with the end effector's reference frame
		target_offset.linear() = mEndEffector->getTransform(domino->getBodyNode(0)).linear();

		mTarget->setTransform(target_offset, domino->getBodyNode(0));


		mKpPD = 200.0;
		mKdPD = 20.0;

		mKpOS = 5.0;
		mKdOS = 0.01;
	}

	void setPDForces()
	{
		Eigen::VectorXd current_coordinates = mManipulator->getPositions();
		Eigen::VectorXd current_velocities = mManipulator->getVelocities();
	
		//stable PD controller
		//current_coordinates += current_velocities * mManipulator->getTimeStep();

		Eigen::VectorXd current_coordinate_error = mQDesired - current_coordinates;
		Eigen::VectorXd current_velocities_error = - current_velocities;

		const Eigen::MatrixXd& M= mManipulator->getMassMatrix();

		// compensate for gravity and coriolis forces
		const Eigen::VectorXd& Cg = mManipulator->getCoriolisAndGravityForces();
		
		mForces = M * (mKpPD * current_coordinate_error + mKdPD * current_velocities_error) /*+ Cg*/;


		mManipulator->setForces(mForces);
	}
	void setOperationalSpaceForces()
	{
		// Mass Matrix is 15 by 15
		const Eigen::MatrixXd& M = mManipulator->getMassMatrix();

		// compute Jacobian and Derivative of Jacobian, and their pseudo-inverse
		// Jacobian is 6 by 7, which should be 6 by 15
		Jacobian J = mEndEffector->getWorldJacobian(mOffset);

		// ##########################################################################################
		// ################ Augument Jacobian matrix for Model with BHand ###########################
		// ##########################################################################################
		
		Eigen::MatrixXd tmp(6,8);
		tmp.setZero();

		Eigen::MatrixXd tmp_J(6,15);
		tmp_J << J, tmp;

		J = tmp_J;

		//std::cout<<"#########debug####Jacobian size####"<<std::endl;
		//std::cout<<J<<std::endl;  
		//std::cout<<"###################################"<<std::endl;
		// ##########################################################################################
		// ##########################################################################################
		// ##########################################################################################


		Eigen::MatrixXd pinv_J = J.transpose() * (J * J.transpose() + 0.0025 * Eigen::Matrix6d::Identity()).inverse();

		Jacobian dJ = mEndEffector->getJacobianClassicDeriv(mOffset);

		// ##########################################################################################
		// ############### Augument Derivative Jacobian Matrix for Model with BHand##################
		// ##########################################################################################
		Eigen::MatrixXd tmp_dJ(6,15);
		tmp_dJ << dJ , tmp;

		dJ = tmp_dJ;
		// ##########################################################################################
		// ##########################################################################################
		// ##########################################################################################

		Eigen::MatrixXd pinv_dJ = dJ.transpose() * (dJ * dJ.transpose() + 0.0025 * Eigen::Matrix6d::Identity()).inverse();

		Eigen::Vector6d e;
		e.tail<3>() = mTarget->getWorldTransform().translation() - mEndEffector->getWorldTransform() * mOffset;

		Eigen::AngleAxisd aa(mTarget->getTransform(mEndEffector).linear());
		e.head<3>() = aa.angle() * aa.axis();

		Eigen::Vector6d de = -mEndEffector->getSpatialVelocity(mOffset, mTarget.get(), Frame::World());

		const Eigen::VectorXd& Cg = mManipulator->getCoriolisAndGravityForces();
		
		// convert gain to matrix form
		Eigen::Matrix6d Kp = mKpOS * Eigen::Matrix6d::Identity();
		size_t dofs = mManipulator->getNumDofs();
		Eigen::MatrixXd Kd = mKdOS * Eigen::MatrixXd::Identity(dofs,dofs);

		// compute Joint forces needed to achieve desired end effector force
		Eigen::Vector6d fDesired = Eigen::Vector6d::Zero();
		fDesired[3] = push_force;
		Eigen::VectorXd f = J.transpose() * fDesired;

		Eigen::VectorXd dq = mManipulator->getVelocities();
		mForces = M * (pinv_J * Kp * de + pinv_dJ * Kp * e) - Kd * dq + Kd * pinv_J * Kp * e + f +Cg ;


		mManipulator->setForces(mForces);


	}
protected:
	SkeletonPtr mManipulator;
	
	//simpleframe inherit from frame
	SimpleFramePtr mTarget;

	BodyNodePtr mEndEffector;

	//desired joint positions when not applying the operational space controller
	Eigen::VectorXd mQDesired;

	Eigen::Vector3d mOffset;

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
		: mHasEverRun(false), mPushCountDown(0)
	{
		setWorld(world);
		mController = std::unique_ptr<Controller> (
			new Controller(world->getSkeleton("manipulator"), world->getSkeleton("domino")));
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
		else
		{
			switch(key)
			{
				case 'r':
					mPushCountDown = push_duration;
				break;
			}
		}
		SimWindow::keyboard(key, x, y);
	}

	
	void timeStepping() override
	{
		if (mPushCountDown > 0)
		{
			mController->setOperationalSpaceForces();
			--mPushCountDown;
		}
		else
		{
			mController->setPDForces();
		}
		SimWindow::timeStepping();
	}

	void drawSkels() override
	{
		// Makre sure lighting is turned on and that polygons get filled in
		glEnable(GL_LIGHTING);

		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

		SimWindow::drawSkels();
	}
protected:
	SkeletonPtr mDomino;
	SkeletonPtr mFloor;
	bool mHasEverRun;
	int mPushCountDown;
	std::unique_ptr<Controller> mController;
};

SkeletonPtr createFloor()
{
	
	SkeletonPtr floor = Skeleton::create("floor");
	
	// Give the floor a body
	BodyNodePtr body = floor->createJointAndBodyNodePair<WeldJoint>().second;

	// Give the body a shape
	std::shared_ptr<BoxShape> box = std::make_shared<BoxShape>(
			Eigen::Vector3d(floor_width, floor_width, floor_height));
	box->setColor(dart::Color::Fuschia(0.3));

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
	//SkeletonPtr manipulator = loader.parseSkeleton(DART_DATA_PATH"/my_urdf/wam7.urdf");
	SkeletonPtr manipulator = loader.parseSkeleton(DART_DATA_PATH"/my_urdf/customized_wam_7dof_wam_bhand.urdf");
	manipulator->setName("manipulator");
	Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();

	manipulator->getJoint(0)->setTransformFromParentBodyNode(tf);
	tf.translation() = Eigen::Vector3d(-0.85, 0, 0);
	manipulator->getJoint(0)->setTransformFromParentBodyNode(tf);

	// set initial position for each DOF
	manipulator->getDof(1)->setPosition(75.0 * M_PI / 180.0);
	manipulator->getDof(3)->setPosition(90.0 * M_PI / 180.0);
	manipulator->getDof(5)->setPosition(-75.0 * M_PI / 180.0);
	

	//set Joint position limit to make the manipulator not collapse into itself.
	for (size_t i = 0 ; i < manipulator->getNumJoints(); ++ i)
	{
		manipulator->getJoint(i)->setPositionLimitEnforced(true);
	}
	
	//manipulator->enableSelfCollision();
	std::cout << "Whether check self Collision:" << manipulator->isEnabledSelfCollisionCheck() << std::endl;
	std::cout << "Whether check adjacent Collision:" << manipulator->isEnabledAdjacentBodyCheck() << std::endl;
	

	return manipulator;
}

SkeletonPtr createDomino()
{
	
    SkeletonPtr domino = Skeleton::create("domino");

	BodyNodePtr body = domino->createJointAndBodyNodePair<FreeJoint>(nullptr).second;

	std::shared_ptr<BoxShape> box(
			new BoxShape(Eigen::Vector3d(domino_depth, domino_width, domino_height)));

	body->addVisualizationShape(box);
	body->addCollisionShape(box);
	dart::dynamics::Inertia inertia;
	inertia.setMass(domino_mass);
	inertia.setMoment(box->computeInertia(domino_mass));
	body->setInertia(inertia);
	
	domino->getDof("Joint_pos_z")->setPosition(domino_height / 2.0);

	//Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
	//tf.translation() = Eigen::Vector3d(0.8, 0.0, 0.0);
	//body->getParentJoint()->setTransformFromParentBodyNode(tf);
	return domino;

}

int main(int argc, char*argv[])
{
	SkeletonPtr floor = createFloor();
	SkeletonPtr manipulator = createManipulator();
	SkeletonPtr domino = createDomino();

	WorldPtr world = std::make_shared<World>();
	world->addSkeleton(floor);
	world->addSkeleton(manipulator);
	world->addSkeleton(domino);



	MyWindow window(world);

	std::cout << "Before simulation has started, you can ..." <<std::endl;
	std::cout << "The gravity is: [" << world->getGravity().transpose() << "]" << std::endl;
	std::cout << "spacebar: initialize the simulation." <<std::endl;

	glutInit(&argc, argv);
	window.initWindow(640,480,"Wam Arm & Barrett Hand by Yang Tian");
	glutMainLoop();
}
