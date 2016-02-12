/*************************************************************************
    > File Name: toy_example_MPC.cpp
    > Author: Yang Tian
    > Mail: tianyang9310@gmail.com 
    > Created Time: Sun Feb  7 21:14:13 2016
 ************************************************************************/
#include "dart/dart.h"
#include <iostream>
#include <time.h>

using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::math;
using namespace dart::gui;

const double transparency = 0.3;

const double floor_length = 0.4;
const double floor_height = 0.01;
const double wall_height = floor_length / 8.0;
const double wall_thickness = floor_height;
const double wall_length = floor_length;
const double obstacle_radius = 0.05;
const double obstacle_height = wall_height / 4.0;

const double cube_length = 0.005;

const double obstacle_2_wall = 2*wall_thickness;

class Controller
{
public:
	Controller(const SkeletonPtr& cube, dart::collision::CollisionDetector* detector, size_t default_Num_contact)
		:mCube(cube),mDetector(detector),mdefault_Num_contact(default_Num_contact)
	{
		mSpeed = 0.2;
		mAcceleration = 0.2;

		mAcceleration_random = 5;

		mCube->getJoint(0)->setActuatorType(Joint::SERVO);

		//mVelocity_old_in_set_Acc_fun = 0;
		//mVelocity_new_in_set_Acc_fun = 0;

		//mTimer = new dart::common::Timer("mTimer_for_int_Acc");
		//mTime_old = 0;
		//mTime_new = 0;
		//mTimer->start();

		std::srand((unsigned int)time(NULL));
	}
	void setCubeVelocity()
	{
		// set horizontal velocity;
		
		mCube->getJoint(0)->setActuatorType(Joint::SERVO);
		
		if (mDetector->getNumContacts() <= mdefault_Num_contact)
		{
			mCube->getDof(0)->setVelocity(mSpeed);
		}

		//int index1 = mCube->getDof(0)->getIndexInSkeleton();
		//mCube->setCommand(index1, mSpeed);
	}
	double setCubeAcceleration()
	{
		// set vertical acceleration

		// using servo actuator type (input velocity) integrate on velocity
		/*
		mTime_new = mTimer->getElapsedTime();

		randomizeAcceleration();

		mVelocity_new_in_set_Acc_fun = mVelocity_old_in_set_Acc_fun + mAcceleration * (mTime_new - mTime_old);

		mCube->getDof(1)->setVelocity(mVelocity_new_in_set_Acc_fun);

		mTime_old = mTime_new;
		mVelocity_old_in_set_Acc_fun = mVelocity_new_in_set_Acc_fun;
		*/

		// set Acceleration directly
		//std::cout<<mDetector->getNumContacts()<<std::endl;	
		if (mDetector->getNumContacts() <= mdefault_Num_contact)
		{
			randomizeAcceleration();
			mCube->getJoint(0)->setActuatorType(Joint::ACCELERATION);
			mCube->getDof(1)->setAcceleration(mAcceleration);
		}

		//int index1 = mCube->getDof(1)->getIndexInSkeleton();
		//mCube->setCommand(index1, mAcceleration);

		return mAcceleration;
	}

	void randomizeAcceleration()
	{
		mAcceleration = mAcceleration_random * (std::rand() / double(RAND_MAX) - 0.5)*2;
	}

protected:
	SkeletonPtr mCube;
	double mSpeed;
	double mAcceleration;

	//double mVelocity_old_in_set_Acc_fun;
	//double mVelocity_new_in_set_Acc_fun;

	//dart::common::Timer *mTimer;
	//double mTime_old;
	//double mTime_new;
	
	double mAcceleration_random;
	
	dart::collision::CollisionDetector* mDetector;
	size_t mdefault_Num_contact;
};

class MyWindow : public dart::gui::SimWindow
{
public:
	MyWindow(const WorldPtr& world)
	{
		setWorld(world);

		detector = mWorld->getConstraintSolver()->getCollisionDetector();
		detector->detectCollision(true, true);

		size_t default_Num_contact = detector->getNumContacts();
		mController = std::unique_ptr<Controller>(new Controller(mWorld->getSkeleton("cube"),detector, default_Num_contact));
	}

	void timeStepping() override
	{
		double desire_Acceleration;
		mController->setCubeVelocity();
		mController->setCubeAcceleration();

		SimWindow::timeStepping();
	}

	void drawSkels() override
	{
		glEnable(GL_LIGHTING);

		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

		SimWindow::drawSkels();
	}
protected:
	std::unique_ptr<Controller> mController;

	dart::collision::CollisionDetector* detector;
};

SkeletonPtr createFloor()
{
	SkeletonPtr floor = Skeleton::create("floor");

	// create a bodynode 
	BodyNodePtr body = floor->createJointAndBodyNodePair<WeldJoint>().second;

	// attach a shape
	std::shared_ptr<BoxShape> box = std::make_shared<BoxShape>(
			Eigen::Vector3d(floor_length, floor_length/2.0, floor_height));
	box->setColor(dart::Color::Gray(0.3));
	body->addVisualizationShape(box);
	body->addCollisionShape(box);

	// put the body into the right position
	Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
	tf.translation() = Eigen::Vector3d(0.0, 0.0, -floor_height / 2.0);
	body->getParentJoint()->setTransformFromParentBodyNode(tf);

	return floor;
}

SkeletonPtr createWall(int wall_index)
{
	int flag1, flag2;
	flag1 = 2* ((wall_index - 1) / 2) -1;
	flag2 = 2* ((wall_index - 1 ) % 2) -1;

	SkeletonPtr wall = Skeleton::create("wall"+std::to_string(wall_index));

	// create a bodynode
	BodyNodePtr body = wall->createJointAndBodyNodePair<WeldJoint>().second;

	// attach a shape
	if( flag1 == -1)
	{
		std::shared_ptr<BoxShape> box = std::make_shared<BoxShape>(
			Eigen::Vector3d(wall_thickness, wall_length/2.0, wall_height));
		box->setColor(dart::Color::Gray(transparency));
		body->addVisualizationShape(box);
		body->addCollisionShape(box);
	}
	else
	{
		std::shared_ptr<BoxShape> box = std::make_shared<BoxShape>(
			Eigen::Vector3d(wall_length - 2*wall_thickness, wall_thickness, wall_height));
		box->setColor(dart::Color::Fuschia(transparency));
		body->addVisualizationShape(box);
		body->addCollisionShape(box);
	}

	// put the body node into the right position
	Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
	if( flag1 == -1)
	{
		tf.translation() = Eigen::Vector3d(flag2 * (floor_length / 2.0 - wall_thickness /2.0), 0.0, wall_height / 2.0);
	}
	else
	{
		tf.translation() = Eigen::Vector3d(0.0, flag2 * (floor_length / 4.0 -wall_thickness /2.0), wall_height / 2.0);
	}
	body->getParentJoint()->setTransformFromParentBodyNode(tf);

	return wall;
}

SkeletonPtr createObstacle(int obstacle_index)
{
	// create a skeleton
	SkeletonPtr obstacle = Skeleton::create("obstacle"+std::to_string(obstacle_index));

	// create a bodynode
	BodyNodePtr body = obstacle->createJointAndBodyNodePair<WeldJoint>().second;

	// create the shape
	if (obstacle_index <2)
	{
		std::shared_ptr<CylinderShape> cylinder = std::make_shared<CylinderShape>(
				obstacle_radius, obstacle_height);
		cylinder->setColor(dart::Color::Red(transparency));
		body->addVisualizationShape(cylinder);
		body->addCollisionShape(cylinder);
	}
	else
	{
		std::shared_ptr<CylinderShape> cylinder = std::make_shared<CylinderShape>(
				obstacle_radius/2.0, obstacle_height);
		cylinder->setColor(dart::Color::Red(transparency));
		body->addVisualizationShape(cylinder);
		body->addCollisionShape(cylinder);
	}

	// put it in the position
	Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
	if (obstacle_index < 2)
	{
		tf.translation() = Eigen::Vector3d(0.0, -floor_length/4 + obstacle_radius +  obstacle_2_wall, obstacle_height/2.0);
	}
	else
	{
		tf.translation() = Eigen::Vector3d(((obstacle_index - 2.5)*2)*floor_length/4.0, floor_length/4 - obstacle_radius/2.0 - obstacle_2_wall , obstacle_height/2.0);
	}

	body->getParentJoint()->setTransformFromParentBodyNode(tf);

	return obstacle;
}

SkeletonPtr createCube()
{
	// create a Skeleton
	SkeletonPtr cube = Skeleton::create("cube");

	// create a BodyNode
	BodyNodePtr body = cube->createJointAndBodyNodePair<PlanarJoint>().second;

	// create a shape
	std::shared_ptr<BoxShape> box = std::make_shared<BoxShape>(
				Eigen::Vector3d(cube_length, cube_length, cube_length));
	box->setColor(dart::Color::Black(2*transparency));
	body->addVisualizationShape(box);
	body->addCollisionShape(box);
	
	// put it in the right position
	Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
	tf.translation() = Eigen::Vector3d(-floor_length/2.0 + obstacle_2_wall, 0.0, cube_length/2.0);
	body->getParentJoint()->setTransformFromParentBodyNode(tf);

	// output number of joint and DOF
	std::cout<<"The cube has "<<cube->getNumJoints()<<" joints"<<std::endl;
	std::cout<<"The cube has "<<cube->getNumDofs()<<" Dofs"<<std::endl;

	return cube;
}

int main(int argc, char* argv[])
{
	std::cout<<"This is a toy example for Model Predictive Control"<<std::endl;
	SkeletonPtr floor = createFloor();
	SkeletonPtr wall_1 = createWall(1);
	SkeletonPtr wall_2 = createWall(2);
	SkeletonPtr wall_3 = createWall(3);
	SkeletonPtr wall_4 = createWall(4);
	SkeletonPtr obstacle_1 = createObstacle(1);
	SkeletonPtr obstacle_2 = createObstacle(2);
	SkeletonPtr obstacle_3 = createObstacle(3);
	SkeletonPtr cube = createCube();

	WorldPtr world = std::make_shared<World>();
	world->addSkeleton(floor);
	world->addSkeleton(wall_1);
	world->addSkeleton(wall_2);
	world->addSkeleton(wall_3);
	world->addSkeleton(wall_4);
	world->addSkeleton(obstacle_1);
	world->addSkeleton(obstacle_2);
	world->addSkeleton(obstacle_3);
	world->addSkeleton(cube);

	MyWindow window(world);

	glutInit(&argc, argv);
	window.initWindow(640, 480, "A toy example for Model Predictive Control");
	glutMainLoop();
}
