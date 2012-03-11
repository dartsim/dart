#include "MyWindow.h"
#include "yui/GLFuncs.h"
#include "dynamics/SkeletonDynamics.h"
#include "dynamics/BodyNodeDynamics.h"
#include "kinematics/FileInfoDof.h"
#include "utils/UtilsMath.h"
#include <cstdio>

using namespace std;
using namespace Eigen;
using namespace kinematics;
using namespace utils;
using namespace integration;
using namespace dynamics;

void MyWindow::initDyn()
{
    double pose[] = {1.1571946631272334027, 0.28296769490469175778, -0.86841303362685362544, -0.25627969123426669329, -0.79685538047236859072, -0.12734463791710903813, 0.14789072934042360452, -0.48935240234150112482, -0.30611976871154328306, -0.14817345106151691425, -0.24066512198813386214, -0.21809634746591596843, -0.011130368938586579844, -0.034110448084233024879, -0.4397398082290809862, -0.038024760469828035314, 0.029673622504407651257, -0.11456185499668754268, -0.56881125122878961609, -0.062291363717269003508, -0.17039243995921912744, -0.34058612440903029972, -0.74166613411407411594, 0.083218661053851505915}; 

    mDofs.resize(mModel->getNumDofs());
    mDofVels.resize(mModel->getNumDofs());

    for (unsigned int i = 0; i < mModel->getNumDofs(); i++) {
        mDofs[i] = pose[i] + random(-0.1, 0.1);
        mDofVels[i] = 0.0;
        mController->setDesiredDof(i, pose[i]);
    }
    mModel->setPose(mDofs,false,false);
}

void MyWindow::displayTimer(int _val)
{
    int numIter = mDisplayTimeout / (mTimeStep * 1000);

    for (int i = 0; i < numIter; i++) {
        mController->computeTorques(mDofs, mDofVels);
        mModel->setPose(mDofs, false, false);
        mModel->computeDynamics(mGravity, mDofVels, true);
        mIntegrator.integrate(this, mTimeStep);
        cout << "iter " << i + mFrame << endl;
    }

    mFrame += numIter;   
    glutPostRedisplay();
    if (mRunning)	
        glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
}

void MyWindow::draw()
{     
    mModel->draw(mRI);
    if (mShowMarker)
        mModel->drawMarkers(mRI);

    // display the frame count in 2D text
    char buff[64];
    sprintf(buff, "%d", mFrame);
    string frame(buff);
    glDisable(GL_LIGHTING);
    glColor3f(0.0, 0.0, 0.0);
    yui::drawStringOnScreen(0.02f, 0.02f, frame);
    glEnable(GL_LIGHTING);
}

void MyWindow::keyboard(unsigned char key, int x, int y)
{
    switch (key) {
    case ' ': // use space key to play or stop the motion
        mRunning = !mRunning;
        if (mRunning)
            glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
        break;
    case 'r': // reset the motion to the first frame
        mFrame = 0;
        break;
    case 'h': // show or hide markers
        mShowMarker = !mShowMarker;
        break;
    case 'q': // up force
        mForce[1] = 5000.0;
        static_cast<BodyNodeDynamics*>(mModel->getNode("fixedHand_wrist"))->addExtForce(Vector3d(0.1, 0.0, 0), mForce);
        mForce[1] = 0.0;
        cout << "up force on wrist" << endl;
        break;
    case 'a': // down force
        mForce[1] = -100.0;
        static_cast<BodyNodeDynamics*>(mModel->getNode("fixedHand_middleDIP"))->addExtForce(Vector3d(0.05, 0.0, 0), mForce);
        mForce[1] = 0.0;
        cout << "down force on middle finger" << endl;
        break;
    default:
        Win3D::keyboard(key, x, y);
    }
    glutPostRedisplay();
}

VectorXd MyWindow::getState() {
    VectorXd state(mDofs.size() + mDofVels.size());
    state.head(mDofs.size()) = mDofs;
    state.tail(mDofVels.size()) = mDofVels;
    return state;
}

VectorXd MyWindow::evalDeriv() {
    VectorXd deriv(mDofs.size() + mDofVels.size());
    VectorXd qddot = mModel->getMassMatrix().fullPivHouseholderQr().solve(-mModel->getCombinedVector() + mModel->getExternalForces() + mController->getTorques()); // the control force is scaled by mass matrix 
    cout << mController->getTorques() << endl;
    mModel->clampRotation(mDofs, mDofVels);
    deriv.tail(mDofVels.size()) = qddot; // set qddot (accelerations)
    deriv.head(mDofs.size()) = (mDofVels + (qddot * mTimeStep)); // set new velocities
    return deriv;
}

void MyWindow::setState(VectorXd newState) {
    mDofVels = newState.tail(mDofVels.size());
    mDofs = newState.head(mDofs.size());
}
