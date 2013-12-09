#include "MyWindow.h"

#include "dynamics/Skeleton.h"
#include "simulation/World.h"

void MyWindow::timeStepping()
{
    Eigen::VectorXd damping = computeDamping();
    mWorld->getSkeleton(0)->setInternalForceVector(damping);
    mWorld->step();
}

Eigen::VectorXd MyWindow::computeDamping()
{
    int nDof = mWorld->getSkeleton(0)->getNumGenCoords();
    Eigen::VectorXd damping = Eigen::VectorXd::Zero(nDof);
    // add damping to each joint; twist-dof has smaller damping
    damping = -0.01 * mWorld->getSkeleton(0)->get_dq();
    for (int i = 0; i < nDof; i++)
        if (i % 3 == 1)
            damping[i] *= 0.1;
    return damping;
}
