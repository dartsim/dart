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
#include "utils/Timer.h"

int main(int argc, char* argv[])
{
    const char* skelfilename;
    const char* doffilename;
    if(argc!=3){
		//skelfilename = GROUNDZERO_DATA_PATH"skel/YutingEuler.skel";
		skelfilename = GROUNDZERO_DATA_PATH"skel/Yuting.vsk";
		doffilename = GROUNDZERO_DATA_PATH"dof/RHand.dof";
    }else{
        skelfilename = argv[1];
        doffilename = argv[2];
    }
	
    model3d::FileInfoSkel<dynamics::SkeletonDynamics> skelFile;
    skelFile.loadFile(skelfilename, model3d::VSK);

    SkeletonDynamics *skelDyn = static_cast<SkeletonDynamics*>(skelFile.getSkel());

    model3d::FileInfoDof motion(skelFile.getSkel());
    //motion.loadFile(doffilename);

    // test the velocity computation using the two methods: linear inverse dynamics and non-recursive
    VectorXd q = VectorXd::Zero(skelFile.getSkel()->getNumDofs());
    VectorXd qdot = VectorXd::Zero(skelFile.getSkel()->getNumDofs());
    for(int i=0; i<skelFile.getSkel()->getNumDofs(); i++){
        q[i] = utils::random(-1.0, 1.0);
        qdot[i] = utils::random(-5.0, 5.0);
    }
    // set the pose
    skelFile.getSkel()->setPose(q);
    // solve the inverse dynamics
    Vector3d gravity(0.0, -9.81, 0.0);

    // test the velocities computed by the two methods
    skelDyn->computeInverseDynamicsLinear(gravity, &qdot);
    for(int i=0; i<skelFile.getSkel()->getNumNodes(); i++){
        BodyNodeDynamics *nodei = static_cast<BodyNodeDynamics*>(skelDyn->getNode(i));
        // compute velocities using non-recursive method
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
        Vector3d v2 = nodei->mW.topLeftCorner(3,3)*nodei->mVelBody;
        Vector3d w2 = nodei->mW.topLeftCorner(3,3)*nodei->mOmegaBody;

        cout<<"Node: "<<nodei->getName()<<endl;
        //cout<<"Angular Jacobian non-recursive: \n"<<(nodei->mJw).rightCols(nodei->getParentJoint()->getNumDofsRot())<<endl;
        //MatrixXd WJw = nodei->mJwJoint;
        //if(nodei->getParentNode()) WJw = nodei->getParentNode()->mW.topLeftCorner(3,3)*WJw;
        //cout<<"Angular Jacobian InvDyn : \n"<<WJw<<endl;
        cout<<"Linear velocity nonrec: \n"<<v1<<endl;
        cout<<"Linear velocity InvDyn : \n"<<v2<<endl;
        cout<<endl;
        cout<<"Angular velocity nonrece: \n"<<w1<<endl;
        cout<<"Angular velocity InvDyn : \n"<<w2<<endl;
        cout<<endl;

        getchar();
    }

    // test the Jwdot using finite differences
    double dt = 1.0e-6;
    VectorXd origq = q;
    VectorXd newq = q + qdot*dt;
    for(int i=0; i<skelFile.getSkel()->getNumNodes(); i++){
        BodyNodeDynamics *nodei = static_cast<BodyNodeDynamics*>(skelDyn->getNode(i));

        skelFile.getSkel()->setPose(origq);
        skelDyn->computeInverseDynamicsLinear(gravity, &qdot);
        Matrix3d Ri = nodei->mW.topLeftCorner(3,3);

        MatrixXd JwOrig = nodei->mJwJoint;
        MatrixXd JwDotOrig = nodei->mJwDotJoint;
        Vector3d wOrig = Ri*nodei->mOmegaBody;
        Vector3d wDotOrig = Ri*nodei->mOmegaDotBody;
        Vector3d vOrig = Ri*nodei->mVelBody;
        Vector3d vDotOrig = Ri*nodei->mVelDotBody + gravity;

        skelFile.getSkel()->setPose(newq);
        skelDyn->computeInverseDynamicsLinear(gravity, &qdot);
        Matrix3d Rinew = nodei->mW.topLeftCorner(3,3);

        MatrixXd JwNew = nodei->mJwJoint;
        MatrixXd JwDotApprox = (JwNew-JwOrig)/dt;
        Vector3d wNew = Rinew*nodei->mOmegaBody;
        Vector3d wDotApprox = (wNew-wOrig)/dt;
        Vector3d vNew = Rinew*nodei->mVelBody;
        Vector3d vDotApprox = (vNew-vOrig)/dt;

        cout<<"JwDot: \n"<<JwDotOrig<<endl;
        cout<<"JwDot approx: \n"<<JwDotApprox<<endl;
        cout<<"wDot: \n"<<wDotOrig<<endl;
        cout<<"wDot approx: \n"<<wDotApprox<<endl;
        cout<<"vDot: \n"<<vDotOrig<<endl;
        cout<<"vDot approx: \n"<<vDotApprox<<endl;

        cout<<endl;
        getchar();
    }

    // test the dynamics: coriolis+gravity term
    VectorXd Cginvdyn = skelDyn->computeInverseDynamicsLinear(gravity, &qdot, NULL, true);
    skelDyn->computeDynamics(gravity, qdot, false); // compute dynamics by not using inverse dynamics
    //cout<<"C+g term inverse dynamics: "<<Cginvdyn<<endl;
    //cout<<"C+g term nonrec dynamics: "<<skelDyn->mCg<<endl;
    cout<<"Difference cg term: \n"<<Cginvdyn - skelDyn->mCg<<endl;

    // test the mass matrix
    skelDyn->computeDynamics(gravity, qdot, true); // compute dynamics by using inverse dynamics
    MatrixXd Minvdyn = skelDyn->mM;
    skelDyn->computeDynamics(gravity, qdot, false); // compute dynamics by using non-recursive dynamics
    MatrixXd Mnonrec = skelDyn->mM;
    cout<<"Difference\n"<<Minvdyn-Mnonrec<<endl;
    
    cout<<"\n\n";
    utils::Timer timer("dynamics");
    timer.startTimer();
    int N = 10000;
    for(int i=0; i<N; i++){
        skelFile.getSkel()->setPose(q, false, false);
        skelDyn->computeDynamics(gravity, qdot, true); // compute dynamics by using inverse dynamics
    }
    timer.stopTimer();
    cout<<"Dynamics using linear inverse dynamics: "<<timer.lastElapsed()/N<<endl;

    timer.startTimer();
    for(int i=0; i<N; i++){
        skelFile.getSkel()->setPose(q, false, false);
        skelDyn->computeDynamics(gravity, qdot, false); // compute dynamics by using non-recursive dynamics
    }
    timer.stopTimer();
    cout<<"Dynamics using non-recursive dynamics: "<<timer.lastElapsed()/N<<endl;

    timer.startTimer();
    for(int i=0; i<N; i++){
        MatrixXd Minv = skelDyn->mM.inverse();
    }
    timer.stopTimer();
    cout<<"Inverse M: "<<timer.lastElapsed()/N<<endl;

    exit(0);


    MyWindow window(motion);
    window.computeMax();

    glutInit(&argc, argv);
    window.initWindow(640, 480, skelfilename);
    glutMainLoop();

    return 0;
}
