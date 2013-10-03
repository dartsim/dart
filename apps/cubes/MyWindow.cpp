#include "MyWindow.h"
#include "math/Helpers.h"
#include "simulation/World.h"
#include "dynamics/BodyNode.h"
#include "dynamics/Skeleton.h"
#include "dynamics/FreeJoint.h"
#include "dynamics/BoxShape.h"

using namespace dart;
using namespace dynamics;

MyWindow::MyWindow(): SimWindow()
{
    mForce = Eigen::Vector3d::Zero();
}

MyWindow::~MyWindow()
{
}

void MyWindow::timeStepping()
{
    mWorld->getSkeleton(1)->getBodyNode(0)->addExtForce(Eigen::Vector3d(0.0, 0.0, 0.0), mForce);
    mWorld->step();
    mForce /= 2.0;
}

void MyWindow::drawSkels()
{
    glEnable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    SimWindow::drawSkels();
}


void MyWindow::keyboard(unsigned char key, int x, int y)
{
    switch(key){
    case ' ': // use space key to play or stop the motion
        mSimulating = !mSimulating;
        if(mSimulating) {
            mPlay = false;
            glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
        }
        break;
    case 'p': // playBack
        mPlay = !mPlay;
        if (mPlay) {
            mSimulating = false;
            glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
        }
        break;
    case '[': // step backward
        if (!mSimulating) {
            mPlayFrame--;
            if(mPlayFrame < 0)
                mPlayFrame = 0;
            glutPostRedisplay();
        }
        break;
    case ']': // step forwardward
        if (!mSimulating) {
            mPlayFrame++;
            if(mPlayFrame >= mBakedStates.size())
                mPlayFrame = 0;
            glutPostRedisplay();
        }
        break;
    case 'v': // show or hide markers
        mShowMarkers = !mShowMarkers;
        break;
    case '1': // upper right force
        mForce[0] = -500;
        break;
    case '2': // upper right force
        mForce[0] = 500;
        break;
    case '3': // upper right force
        mForce[2] = -500;
        break;
    case '4': // upper right force
        mForce[2] = 500;
        break;
    case 'q': // Spawn a cube
    case 'Q': // Spawn a cube
    {
        Eigen::Vector3d position = Eigen::Vector3d(math::random(-1.0, 1.0), math::random(-1.0, 1.0), math::random(0.5, 1.0));
        Eigen::Vector3d size = Eigen::Vector3d(math::random(0.01, 0.2), math::random(0.01, 0.2), math::random(0.01, 0.2));
        spawnCube(position, size);
        break;
    }
    case 'w': // Spawn a cube
    case 'W': // Spawn a cube
    {
        if (mWorld->getNumSkeletons() > 4)
            mWorld->removeSkeleton(mWorld->getSkeleton(4));
        break;
    }
    default:
        Win3D::keyboard(key,x,y);

    }
    glutPostRedisplay();
}

void MyWindow::spawnCube(const Eigen::Vector3d& _position,
                         const Eigen::Vector3d& _size,
                         double _mass)
{
    Skeleton*  newCubeSkeleton = new Skeleton();
    BodyNode*  newBodyNode     = new BodyNode("cube_link");
    FreeJoint* newFreeJoint    = new FreeJoint("cube_joint");
    BoxShape*  newBoxShape     = new BoxShape(_size);

    newBodyNode->addVisualizationShape(newBoxShape);
    newBodyNode->addCollisionShape(newBoxShape);
    newBodyNode->setMass(_mass);
    newBodyNode->setParentJoint(newFreeJoint);
    newFreeJoint->setTransformFromParentBodyNode(Eigen::Isometry3d(Eigen::Translation3d(_position)));
    newBoxShape->setColor(Eigen::Vector3d(math::random(0.0, 1.0), math::random(0.0, 1.0), math::random(0.0, 1.0)));
    newCubeSkeleton->addBodyNode(newBodyNode);
    mWorld->addSkeleton(newCubeSkeleton);
}

