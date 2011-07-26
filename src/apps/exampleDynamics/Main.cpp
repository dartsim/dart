#include "dynamics/BodyNodeDynamics.h"
#include "dynamics/SkeletonDynamics.h"
#include "model3d/FileInfoSkel.hpp"
#include "model3d/FileInfoDof.h"
#include "model3d/BodyNode.h"

#include "MyWindow.h"

#include <iostream>
#include <Eigen/dense>

using namespace std;
using namespace Eigen;
using namespace model3d;
using namespace dynamics;

#include "utils/UtilsMath.h"
#include "utils/Paths.h"

int main(int argc, char* argv[])
{
    const char* modelfile;
    const char* doffile;
    if(argc!=3){
		modelfile = GROUNDZERO_DATA_PATH"skel/YutingEuler.skel";
		doffile = GROUNDZERO_DATA_PATH"dof/RHand.dof";
    }else{
        modelfile = argv[1];
        doffile = argv[2];
    }
	
    model3d::FileInfoSkel<dynamics::SkeletonDynamics> skel;
    skel.loadFile(modelfile, model3d::SKEL);

    SkeletonDynamics *skelDyn = static_cast<SkeletonDynamics*>(skel.getSkel());

    model3d::FileInfoDof motion(skel.getSkel());
    motion.loadFile(doffile);

    // test the velocity computation using the two methods: inverse dynamics and regular
    VectorXd q = VectorXd::Zero(skel.getSkel()->getNumDofs());
    VectorXd qdot = VectorXd::Zero(skel.getSkel()->getNumDofs());
    for(int i=0; i<skel.getSkel()->getNumDofs(); i++){
        q[i] = utils::random(-1.0, 1.0);
        qdot[i] = utils::random(-5.0, 5.0);
    }
    // set the pose
    skel.getSkel()->setPose(q);
    // solve the inverse dynamics
    Vector3d gravity(0.0, -9.81, 0.0);
    skelDyn->inverseDynamicsLinear(gravity, &qdot);

    for(int i=0; i<skel.getSkel()->getNumNodes(); i++){
        BodyNodeDynamics *nodei = static_cast<BodyNodeDynamics*>(skelDyn->getNode(i));
        // compute velocities using regular method
        nodei->updateTransform();
        nodei->updateFirstDerivatives();
        nodei->evalJacLin();
        nodei->evalJacAng();
        Vector3d v1 = Vector3d::Zero();
        Vector3d w1 = Vector3d::Zero();
        for(int j=0; j<nodei->getNumDependantDofs(); j++){
            int dj = nodei->getDependantDof(j);
            for(int k=0; k<3; k++) {
                v1[k] += nodei->mJv(k, j)*qdot[dj];
                w1[k] += nodei->mJw(k, j)*qdot[dj];
            }
        }

        // compute velocities using inverse dynamics routine
        Vector3d v2 = nodei->mW.topLeftCorner(3,3)*nodei->mVBody;
        Vector3d w2 = nodei->mW.topLeftCorner(3,3)*nodei->mOmegaBody;

        cout<<"Node: "<<nodei->getName()<<endl;
        //cout<<"Angular Jacobian regular: \n"<<(nodei->mJw).rightCols(nodei->getParentJoint()->getNumDofsRot())<<endl;
        //MatrixXd WJw = nodei->mJwJoint;
        //if(nodei->getParentNode()) WJw = nodei->getParentNode()->mW.topLeftCorner(3,3)*WJw;
        //cout<<"Angular Jacobian InvDyn : \n"<<WJw<<endl;
        cout<<"Linear velocity regular: \n"<<v1<<endl;
        cout<<"Linear velocity InvDyn : \n"<<v2<<endl;
        cout<<endl;
        cout<<"Angular velocity regular: \n"<<w1<<endl;
        cout<<"Angular velocity InvDyn : \n"<<w2<<endl;
        cout<<endl;
        getchar();
    }
    exit(1);


    MyWindow window(motion);
    window.computeMax();

    glutInit(&argc, argv);
    window.initWindow(640, 480, modelfile);
    glutMainLoop();

    return 0;
}
