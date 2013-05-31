#include "MyWindow.h"
#include "PositionConstraint.h"
#include "math/UtilsMath.h"
#include "utils/Timer.h"
#include "yui/GLFuncs.h"
#include "optimizer/ObjectiveBox.h"
#include "optimizer/Var.h"
#include "kinematics/Skeleton.h"
#include "kinematics/Dof.h"
#include "kinematics/BodyNode.h"

using namespace utils;
using namespace optimizer;
using namespace kinematics;

void MyWindow::initIK()
{
    // create variables
    for (int i = 0; i < mSkel->getNumDofs(); i++) {
        Var *v = new Var(mSkel->getDof(i)->getValue(), mSkel->getDof(i)->getMin(), mSkel->getDof(i)->getMax());
        mVariables.push_back(v);
    }

    // create objBox
    mObjBox = new ObjectiveBox(mVariables.size());

    // create constraints
    BodyNode *node = mSkel->getNode("fullbody1_h_heel_left");
    Vector3d offset(0, 0, 0);
    Vector3d target = dart_math::xformHom(node->getWorldTransform(), offset);
    PositionConstraint* pos = new PositionConstraint(mVariables, mSkel, node, offset, target);
    mObjBox->add(pos);

    node = mSkel->getNode("fullbody1_h_heel_right");
    target = dart_math::xformHom(node->getWorldTransform(), offset);
    pos = new PositionConstraint(mVariables, mSkel, node, offset, target);
    mObjBox->add(pos);
}

void MyWindow::solveIK()
{
    int numIter = 300;
    double alpha = 0.01;
    int nDof =  mSkel->getNumDofs();
    for (int i = 0; i < numIter; i++) {
        // update gradients
        mObjBox->evalObjGrad();
        // update pose
        VectorXd newPose(nDof);
        for (int j = 0; j < nDof; j++)
            newPose[j] = mSkel->getDof(j)->getValue() - alpha * mObjBox->mObjGrad[j];
        mSkel->setPose(newPose, true, true);
    }
}

void MyWindow::displayTimer(int _val)
{
    glutPostRedisplay();
    glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
}

void MyWindow::draw()
{
    glDisable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    mSkel->draw(mRI);
    
    mRI->setPenColor(Vector3d(0.8, 0.8, 0.2));
    for (int i = 0; i < mObjBox->getNumConstraints(); i++) {
        Vector3d p = ((PositionConstraint*)mObjBox->getConstraint(i))->getTarget();
        mRI->pushMatrix();
        glTranslated(p[0], p[1], p[2]);
        mRI->drawEllipsoid(Vector3d(0.02, 0.02, 0.02));
        mRI->popMatrix();
    }
    glEnable(GL_LIGHTING);
}

void MyWindow::keyboard(unsigned char key, int x, int y)
{
    Vector3d target1 = ((PositionConstraint*)mObjBox->getConstraint(0))->getTarget();
    Vector3d target2 = ((PositionConstraint*)mObjBox->getConstraint(1))->getTarget();
    switch(key){
    case ' ': // use space key to set new target positions
        target1[0] += 0.05;
        target2[0] -= 0.05;
        ((PositionConstraint*)mObjBox->getConstraint(0))->setTarget(target1);
        ((PositionConstraint*)mObjBox->getConstraint(1))->setTarget(target2);
        solveIK();
        break;
    default:
        Win3D::keyboard(key,x,y);
    }
    glutPostRedisplay();
}
