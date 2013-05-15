#include "MyWindow.h"
#include "simulation/World.h"

using namespace Eigen;

void MyWindow::timeStepping()
{
    VectorXd damping = computeDamping();
    mWorld->getSkeleton(0)->setInternalForces(damping);
    mWorld->step();
}

VectorXd MyWindow::computeDamping()
{
    int nDof = mWorld->getSkeleton(0)->getNumDofs();
    VectorXd damping = VectorXd::Zero(nDof);
    // add damping to each joint; twist-dof has smaller damping
    damping = -0.01 * mWorld->getSkeleton(0)->get_dq();
    for (int i = 0; i < nDof; i++)
        if (i % 3 == 1)
            damping[i] *= 0.1;
    return damping;
}
