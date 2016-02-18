/*************************************************************************
    > File Name: toy_example_MPC.cpp
    > Author: Yang Tian
    > Mail: tianyang9310@gmail.com 
    > Created Time: Sun Feb  7 21:14:13 2016
 ************************************************************************/
#include "dart/dart.h"
#include <iostream>
#include <time.h>
#include <vector>
#include <algorithm>

using namespace dart::common;
using namespace dart::utils;
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

const double default_density = 1000;

class Controller
{
public:
	Controller(const SkeletonPtr& cube, dart::collision::CollisionDetector* detector, size_t default_Num_contact, double time_step)
		:mCube(cube),mDetector(detector),mdefault_Num_contact(default_Num_contact), mTime_step_in_Acc_fun(time_step)
	{
		mSpeed = 0.2;
		mAcceleration = 0.5;

		mAcceleration_random = 10;

		mCube->getJoint(0)->setActuatorType(Joint::VELOCITY);

		mVelocity_old_in_set_Acc_fun = 0;
		mVelocity_new_in_set_Acc_fun = 0;

		std::srand((unsigned int)time(NULL));
	}
	void setCubeVelocity()
	{
		// set horizontal velocity;
		
		mCube->getJoint(0)->setActuatorType(Joint::VELOCITY);

		if (mDetector->getNumContacts() <= mdefault_Num_contact)
		{
			//mCube->getDof(0)->setVelocity(mSpeed);
			
			mCube->getDof(0)->setCommand(mSpeed);
		
		}
		else
		{
			//mCube->getDof(0)->setVelocity(0);

			mCube->getDof(0)->setCommand(0);

		}
	}

	double setCubeAcceleration()
	{
		// set vertical acceleration

		// using servo actuator type (input velocity) integrate on velocity
		
		mCube->getJoint(0)->setActuatorType(Joint::VELOCITY);

		if (mDetector->getNumContacts() <= mdefault_Num_contact)
		{
			randomizeAcceleration();
			mVelocity_new_in_set_Acc_fun = mVelocity_old_in_set_Acc_fun + mAcceleration * mTime_step_in_Acc_fun;

			//mCube->getDof(1)->setVelocity(mVelocity_new_in_set_Acc_fun);

			mCube->getDof(1)->setCommand(mVelocity_new_in_set_Acc_fun);

			mVelocity_old_in_set_Acc_fun = mVelocity_new_in_set_Acc_fun;
		}
		else
		{
			//mCube->getDof(1)->setVelocity(0);

			mCube->getDof(1)->setCommand(0);

		}

		// set Acceleration directly
		/*
		if (mDetector->getNumContacts() <= mdefault_Num_contact)
		{
			randomizeAcceleration();
			mCube->getJoint(0)->setActuatorType(Joint::ACCELERATION);
			mCube->getDof(1)->setAcceleration(mAcceleration);
		}
		*/

		return mAcceleration;
	}


	void setCubeAcceleration(double desire_Acceleration)
	{
		// set vertical acceleration based on desired acceleration passed into

		// using servo actuator type (input velocity) integrate on velocity
		
		mCube->getJoint(0)->setActuatorType(Joint::VELOCITY);

		if (mDetector->getNumContacts() <= mdefault_Num_contact)
		{
			mVelocity_new_in_set_Acc_fun = mVelocity_old_in_set_Acc_fun + desire_Acceleration * mTime_step_in_Acc_fun;

			//mCube->getDof(1)->setVelocity(mVelocity_new_in_set_Acc_fun);

			mCube->getDof(1)->setCommand(mVelocity_new_in_set_Acc_fun);

			mVelocity_old_in_set_Acc_fun = mVelocity_new_in_set_Acc_fun;
		}
		else
		{
			//mCube->getDof(1)->setVelocity(0);

			mCube->getDof(1)->setCommand(0);

		}
	}

	void randomizeAcceleration()
	{
		mAcceleration = mAcceleration_random * (std::rand() / double(RAND_MAX) - 0.5)*2;
	}

protected:
	SkeletonPtr mCube;
	double mSpeed;
	double mAcceleration;

	double mVelocity_old_in_set_Acc_fun;
	double mVelocity_new_in_set_Acc_fun;
	double mTime_step_in_Acc_fun;

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

		default_Num_contact = detector->getNumContacts();

		std::cout<<"Default number of contacts is "<<default_Num_contact<<std::endl;

		mController = std::unique_ptr<Controller>(new Controller(mWorld->getSkeleton("cube"),detector, default_Num_contact, mWorld->getTimeStep()));
	}

	double MyMPC()
	{
		int num_samples = 5;
		int plan_horizon = 1;
		double desire_Acceleration = 0;
		std::vector<WorldPtr> world_array(num_samples);
		std::vector<std::unique_ptr<Controller>> controller_array(num_samples);
		std::vector<double> acceleration_array(num_samples);
		std::vector<int> cost_array(num_samples);
		
		// save the world
		// have best_acceleration vector and cost vector
		// cost vector is based on the number of contacts.
		for (int i = 0; i<num_samples; i++)
		{
			// states of the skeleton will not be cloned?
			world_array[i] = mWorld->clone();
			controller_array[i] = std::unique_ptr<Controller>(new Controller(world_array[i]->getSkeleton("cube"), 
																			 world_array[i]->getConstraintSolver()->getCollisionDetector(),
																			 default_Num_contact,
																			 world_array[i]->getTimeStep()));
			controller_array[i]->setCubeVelocity();
			acceleration_array[i] = controller_array[i]->setCubeAcceleration();
			for (int j = 0; j<plan_horizon; j++)
			{
				world_array[i]->step();
				controller_array[i]->setCubeVelocity();
				controller_array[i]->setCubeAcceleration();
			}
			cost_array[i] = world_array[i]->getConstraintSolver()->getCollisionDetector()->getNumContacts();
		}
		
		desire_Acceleration = acceleration_array[std::distance(cost_array.begin(), std::min_element(cost_array.begin(), cost_array.end()))];
		
		// compute forward
		// find the best acceleration
		return desire_Acceleration;
	}

	void timeStepping() override
	{
		mController->setCubeVelocity();
		mController->setCubeAcceleration(MyMPC());

		if (detector->getNumContacts() != default_Num_contact)
		{
			std::cout<<detector->getNumContacts();	
		}

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

	size_t default_Num_contact;
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

	// set inertia
	dart::dynamics::Inertia inertia;
	inertia.setMass(default_density * box->getVolume());
	inertia.setMoment(box->computeInertia(inertia.getMass()));
	body->setInertia(inertia);

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
		
		// set inertia
		dart::dynamics::Inertia inertia;
		inertia.setMass(default_density * box->getVolume());
		inertia.setMoment(box->computeInertia(inertia.getMass()));
		body->setInertia(inertia);
	}
	else
	{
		std::shared_ptr<BoxShape> box = std::make_shared<BoxShape>(
			Eigen::Vector3d(wall_length - 2*wall_thickness, wall_thickness, wall_height));
		box->setColor(dart::Color::Fuschia(transparency));
		body->addVisualizationShape(box);
		body->addCollisionShape(box);

		// set inertia
		dart::dynamics::Inertia inertia;
		inertia.setMass(default_density * box->getVolume());
		inertia.setMoment(box->computeInertia(inertia.getMass()));
		body->setInertia(inertia);
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
		// set inertia
		dart::dynamics::Inertia inertia;
		inertia.setMass(2 * default_density * cylinder->getVolume());
		inertia.setMoment(cylinder->computeInertia(inertia.getMass()));
		body->setInertia(inertia);
	}
	else
	{
		std::shared_ptr<CylinderShape> cylinder = std::make_shared<CylinderShape>(
				obstacle_radius/2.0, obstacle_height);
		cylinder->setColor(dart::Color::Red(transparency));
		body->addVisualizationShape(cylinder);
		body->addCollisionShape(cylinder);
		// set inertia
		dart::dynamics::Inertia inertia;
		inertia.setMass(2 * default_density * cylinder->getVolume());
		inertia.setMoment(cylinder->computeInertia(inertia.getMass()));
		body->setInertia(inertia);
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
	

	// set inertia
	dart::dynamics::Inertia inertia;
	inertia.setMass(0.5 * default_density * box->getVolume());
	inertia.setMoment(box->computeInertia(inertia.getMass()));
	body->setInertia(inertia);

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
