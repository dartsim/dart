/**
 * @file TestHelper.h
 * @author Can Erdogan
 * @date Feb 03, 2013
 * @brief Contains the helper functions for the tests.
 */

#ifndef DART_UNITTESTS_TEST_HELPERS_H
#define DART_UNITTESTS_TEST_HELPERS_H

#include <boost/math/special_functions/fpclassify.hpp>
#include <Eigen/Dense>
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/constraint/ConstraintDynamics.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/GenCoord.h"
#include "dart/dynamics/Joint.h"
#include "dart/dynamics/WeldJoint.h"
#include "dart/dynamics/PrismaticJoint.h"
#include "dart/dynamics/RevoluteJoint.h"
#include "dart/dynamics/ScrewJoint.h"
#include "dart/dynamics/TranslationalJoint.h"
#include "dart/dynamics/BallJoint.h"
#include "dart/dynamics/FreeJoint.h"
#include "dart/dynamics/EulerJoint.h"
#include "dart/dynamics/UniversalJoint.h"
#include "dart/dynamics/BoxShape.h"

using namespace dart;
using namespace dynamics;
using namespace Eigen;

/// Function headers
enum TypeOfDOF
{
    DOF_X, DOF_Y, DOF_Z, DOF_ROLL, DOF_PITCH, DOF_YAW
};

/*******************************************************************************/
/// Returns true if the two matrices are equal within the given bound
template <class MATRIX>
bool equals(const Eigen::DenseBase<MATRIX>& A,
            const Eigen::DenseBase<MATRIX>& B, double tol = 1e-5)
{
    // Get the matrix sizes and sanity check the call
    const size_t n1 = A.cols(), m1 = A.rows();
    const size_t n2 = B.cols(), m2 = B.rows();
    if (m1 != m2 || n1 != n2)
        return false;

    // Check each index
    for (size_t i = 0; i < m1; i++)
    {
        for (size_t j = 0; j < n1; j++)
        {
            if (boost::math::isnan(A(i,j)) ^ boost::math::isnan(B(i,j)))
                return false;
            else if (fabs(A(i,j) - B(i,j)) > tol)
                return false;
        }
    }

    // If no problems, the two matrices are equal
    return true;
}

/******************************************************************************/
/// Add an end-effector to the last link of the given robot
void addEndEffector(Skeleton* robot, BodyNode* parent_node, Vector3d dim)
{
    // Create the end-effector node with a random dimension
    BodyNode* node = new BodyNode("ee");
    WeldJoint* joint = new WeldJoint("eeJoint");
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translate(Eigen::Vector3d(0.0, 0.0, dim(2)));
    joint->setTransformFromParentBodyNode(T);
    Shape* shape = new BoxShape(Vector3d(0.2, 0.2, 0.2));
    node->setLocalCOM(Vector3d(0.0, 0.0, 0.0));
    node->setMass(1.0);
    node->addVisualizationShape(shape);
    node->addCollisionShape(shape);
    node->setParentJoint(joint);
    parent_node->addChildBodyNode(node);
    robot->addBodyNode(node);
}

/******************************************************************************/
/// Add a DOF to a given joint
Joint* create1DOFJoint(double val, double min, double max, int type)
{
    // Create the transformation based on the type
    Joint* newJoint = NULL;
    if(type == DOF_X)
        newJoint = new PrismaticJoint(Eigen::Vector3d(1.0, 0.0, 0.0));
    else if(type == DOF_Y)
        newJoint = new PrismaticJoint(Eigen::Vector3d(0.0, 1.0, 0.0));
    else if(type == DOF_Z)
        newJoint = new PrismaticJoint(Eigen::Vector3d(0.0, 0.0, 1.0));
    else if(type == DOF_YAW)
        newJoint = new RevoluteJoint(Eigen::Vector3d(0.0, 0.0, 1.0));
    else if(type == DOF_PITCH)
        newJoint = new RevoluteJoint(Eigen::Vector3d(0.0, 1.0, 0.0));
    else if(type == DOF_ROLL)
        newJoint = new RevoluteJoint(Eigen::Vector3d(1.0, 0.0, 0.0));
    // Add the transformation to the joint, set the min/max values and set it to the skeleton
    newJoint->getGenCoord(0)->set_q(val);
    newJoint->getGenCoord(0)->set_qMin(min);
    newJoint->getGenCoord(0)->set_qMax(max);

    return newJoint;
}

/******************************************************************************/
/// Creates a two link manipulator with the given dimensions where the first
/// link rotates around z-axis and second rotates around x in the zero
/// configuration.
Skeleton* createTwoLinkRobot(Vector3d dim1, TypeOfDOF type1,
                             Vector3d dim2, TypeOfDOF type2,
                             bool finished = true)
{
    Skeleton* robot = new Skeleton();

    // Create the first link, the joint with the ground and its shape
    double mass = 1.0;
    BodyNode* node = new BodyNode("link1");
    Joint* joint = create1DOFJoint(0.0, -DART_PI, DART_PI, type1);
    joint->setName("joint1");
    Shape* shape = new BoxShape(dim1);
    node->setLocalCOM(Vector3d(0.0, 0.0, dim1(2)/2.0));
    node->addVisualizationShape(shape);
    node->addCollisionShape(shape);
    node->setMass(mass);
    node->setParentJoint(joint);
    robot->addBodyNode(node);

    // Create the second link, the joint with link1 and its shape
    BodyNode* parent_node = robot->getBodyNode("link1");
    node = new BodyNode("link2");
    joint = create1DOFJoint(0.0, -DART_PI, DART_PI, type2);
    joint->setName("joint2");
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translate(Eigen::Vector3d(0.0, 0.0, dim1(2)));
    joint->setTransformFromParentBodyNode(T);
    shape = new BoxShape(dim2);
    node->setLocalCOM(Vector3d(0.0, 0.0, dim2(2)/2.0));
    node->addVisualizationShape(shape);
    node->addCollisionShape(shape);
    node->setMass(mass);
    node->setParentJoint(joint);
    parent_node->addChildBodyNode(node);
    robot->addBodyNode(node);

    // If finished, initialize the skeleton
    if(finished)
    {
        addEndEffector(robot, node, dim2);
        robot->init();
    }
    return robot;
}

/******************************************************************************/
/// Creates a two link manipulator with the given dimensions where the first
/// link rotates around z-axis and second rotates around x in the zero
/// configuration.
Skeleton* createThreeLinkRobot(Vector3d dim1, TypeOfDOF type1,
                               Vector3d dim2, TypeOfDOF type2,
                               Vector3d dim3, TypeOfDOF type3,
                               bool finished = false)
{
    Skeleton* robot = new Skeleton();

    // Create the first link, the joint with the ground and its shape
    double mass = 1.0;
    BodyNode* node = new BodyNode("link1");
    Joint* joint = create1DOFJoint(0.0, -DART_PI, DART_PI, type1);
    joint->setName("joint1");
    Shape* shape = new BoxShape(dim1);
    node->setLocalCOM(Vector3d(0.0, 0.0, dim1(2)/2.0));
    node->addVisualizationShape(shape);
    node->addCollisionShape(shape);
    node->setMass(mass);
    node->setParentJoint(joint);
    robot->addBodyNode(node);

    // Create the second link, the joint with link1 and its shape
    BodyNode* parent_node = robot->getBodyNode("link1");
    node = new BodyNode("link2");
    joint = create1DOFJoint(0.0, -DART_PI, DART_PI, type2);
    joint->setName("joint2");
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translate(Eigen::Vector3d(0.0, 0.0, dim1(2)));
    joint->setTransformFromParentBodyNode(T);
    shape = new BoxShape(dim2);
    node->setLocalCOM(Vector3d(0.0, 0.0, dim2(2)/2.0));
    node->addVisualizationShape(shape);
    node->addCollisionShape(shape);
    node->setMass(mass);
    node->setParentJoint(joint);
    parent_node->addChildBodyNode(node);
    robot->addBodyNode(node);

    // Create the third link, the joint with link2 and its shape
    parent_node = robot->getBodyNode("link2");
    node = new BodyNode("link3");
    joint = create1DOFJoint(0.0, -DART_PI, DART_PI, type3);
    joint->setName("joint3");
    T = Eigen::Isometry3d::Identity();
    T.translate(Eigen::Vector3d(0.0, 0.0, dim1(2)));
    joint->setTransformFromParentBodyNode(T);
    shape = new BoxShape(dim3);
    node->setLocalCOM(Vector3d(0.0, 0.0, dim3(2)/2.0));
    node->addVisualizationShape(shape);
    node->addCollisionShape(shape);
    node->setMass(mass);
    node->setParentJoint(joint);
    parent_node->addChildBodyNode(node);
    robot->addBodyNode(node);

    // If finished, initialize the skeleton
    if(finished)
    {
        addEndEffector(robot, node, dim3);
        robot->init();
    }
    return robot;
}

/******************************************************************************/
/// Creates a N link manipulator with the given dimensions where the first link
/// rotates around z-axis and second rotates around x in the zero configuration.
Skeleton* createNLinkRobot(int _n, Vector3d dim, TypeOfDOF type,
                           bool finished = false)
{
    assert(_n > 0);

    Skeleton* robot = new Skeleton();
    robot->setSelfCollidable(false);
    double mass = 1.0;

    // Create the first link, the joint with the ground and its shape
    BodyNode* parent_node = NULL;
    BodyNode* node = new BodyNode("link1");
    Joint* joint = create1DOFJoint(0.0, -DART_PI, DART_PI, type);
    joint->setName("joint1");
    joint->setDampingCoefficient(0, 0.01);
    Shape* shape = new BoxShape(dim);
    node->setLocalCOM(Vector3d(0.0, 0.0, dim(2)/2.0));
    node->addVisualizationShape(shape);
    node->addCollisionShape(shape);
    node->setMass(mass);
    node->setParentJoint(joint);
    robot->addBodyNode(node);
    parent_node = node;

    // Create links iteratively
    for (int i = 1; i < _n; ++i)
    {
        std::ostringstream ssLink;
        std::ostringstream ssJoint;
        ssLink << "link" << i;
        ssLink << "joint" << i;
        node = new BodyNode(ssLink.str());
        joint = create1DOFJoint(0.0, -DART_PI, DART_PI, type);
        joint->setName(ssJoint.str());
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.translate(Eigen::Vector3d(0.0, 0.0, dim(2)));
        joint->setTransformFromParentBodyNode(T);
        joint->setDampingCoefficient(0, 0.01);
        shape = new BoxShape(dim);
        node->setLocalCOM(Vector3d(0.0, 0.0, dim(2)/2.0));
        node->addVisualizationShape(shape);
        node->addCollisionShape(shape);
        node->setMass(mass);
        node->setParentJoint(joint);
        parent_node->addChildBodyNode(node);
        robot->addBodyNode(node);
        parent_node = node;
    }

    // If finished, initialize the skeleton
    if(finished)
    {
        addEndEffector(robot, node, dim);
        robot->init();
    }
    return robot;
}

#endif // #ifndef DART_UNITTESTS_TEST_HELPERS_H
