#ifndef UNITTESTS_TESTDYNAMICS_H
#define UNITTESTS_TESTDYNAMICS_H

#include "dynamics/BodyNodeDynamics.h"
#include "dynamics/SkeletonDynamics.h"
#include "model3d/FileInfoSkel.hpp"
#include "model3d/FileInfoDof.h"
#include "model3d/BodyNode.h"
#include "utils/Paths.h"
#include <iostream>
#include <Eigen/dense>


TEST(DYNAMICS, COMPARE_VELOCITIES) {
    using namespace std;
    using namespace Eigen;
    using namespace model3d;
    using namespace dynamics;

    const char* skelfilename = GROUNDZERO_DATA_PATH"skel/Yuting.vsk";
    model3d::FileInfoSkel<dynamics::SkeletonDynamics> skelFile;
    bool loadModelResult = skelFile.loadFile(skelfilename, model3d::VSK);
    ASSERT_TRUE(loadModelResult);

    SkeletonDynamics *skelDyn = static_cast<SkeletonDynamics*>(skelFile.getSkel());
    EXPECT_TRUE(skelDyn != NULL);

    const double TOLERANCE_EXACT = 1.0e-10;
   

    // test the velocity computation using the two methods: inverse dynamics and regular
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
        EXPECT_TRUE(nodei != NULL);
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
        Vector3d v2 = nodei->mW.topLeftCorner(3,3)*nodei->mVelBody;
        Vector3d w2 = nodei->mW.topLeftCorner(3,3)*nodei->mOmegaBody;

        for(int k=0; k<3; k++) {
            EXPECT_NEAR(v1[k], v2[k], TOLERANCE_EXACT);
            EXPECT_NEAR(w1[k], w2[k], TOLERANCE_EXACT);
        }

        // Angular Jacobian regular
        MatrixXd Jw_regular = (nodei->mJw).rightCols(nodei->getParentJoint()->getNumDofsRot());
        // Angular jacobian inverse dynamics
        MatrixXd Jw_invdyn = nodei->mJwJoint;
        if(nodei->getParentNode()) Jw_invdyn = nodei->getParentNode()->mW.topLeftCorner(3,3)*Jw_invdyn;
        ASSERT_TRUE(Jw_regular.rows()==Jw_invdyn.rows());
        ASSERT_TRUE(Jw_regular.cols()==Jw_invdyn.cols());
        for(int ki=0; ki<Jw_regular.rows(); ki++){
            for(int kj=0; kj<Jw_regular.cols(); kj++){
                EXPECT_NEAR(Jw_regular(ki,kj), Jw_invdyn(ki,kj), TOLERANCE_EXACT);
            }
        }
    }
}
TEST(DYNAMICS, FINITEDIFF_ACCELERATIONS_INVERSEDYNAMICS) {
    using namespace std;
    using namespace Eigen;
    using namespace model3d;
    using namespace dynamics;

    const char* skelfilename = GROUNDZERO_DATA_PATH"skel/Yuting.vsk";
    model3d::FileInfoSkel<dynamics::SkeletonDynamics> skelFile;
    bool loadModelResult = skelFile.loadFile(skelfilename, model3d::VSK);
    ASSERT_TRUE(loadModelResult);

    SkeletonDynamics *skelDyn = static_cast<SkeletonDynamics*>(skelFile.getSkel());
    EXPECT_TRUE(skelDyn != NULL);

    // test the velocity computation using the two methods: inverse dynamics and regular
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
    // test the Jwdot using finite differences
    double TOLERANCE_APPROX = 1.0e-3;
    double dt = 1.0e-6;
    VectorXd origq = q;
    VectorXd newq = q + qdot*dt;
    for(int i=0; i<skelFile.getSkel()->getNumNodes(); i++){
        BodyNodeDynamics *nodei = static_cast<BodyNodeDynamics*>(skelDyn->getNode(i));
        EXPECT_TRUE(nodei != NULL);

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

        for(int ki=0; ki<JwDotOrig.rows(); ki++){
            EXPECT_NEAR(vDotOrig[ki], vDotApprox[ki], TOLERANCE_APPROX);
            EXPECT_NEAR(wDotOrig[ki], wDotApprox[ki], TOLERANCE_APPROX);
            for(int kj=0; kj<JwDotOrig.cols(); kj++){
                EXPECT_NEAR(JwDotOrig(ki,kj), JwDotApprox(ki,kj), TOLERANCE_APPROX);
            }
        }

    }
}

TEST(DYNAMICS, COMPARE_CORIOLIS) {
    using namespace std;
    using namespace Eigen;
    using namespace model3d;
    using namespace dynamics;

    const char* skelfilename = GROUNDZERO_DATA_PATH"skel/Yuting.vsk";
    model3d::FileInfoSkel<dynamics::SkeletonDynamics> skelFile;
    bool loadModelResult = skelFile.loadFile(skelfilename, model3d::VSK);
    ASSERT_TRUE(loadModelResult);

    SkeletonDynamics *skelDyn = static_cast<SkeletonDynamics*>(skelFile.getSkel());
    EXPECT_TRUE(skelDyn != NULL);

    const double TOLERANCE_EXACT = 1.0e-10;

    // test the velocity computation using the two methods: inverse dynamics and regular
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

    // test/compare the dynamics result for both methods
    VectorXd Cginvdyn = skelDyn->computeInverseDynamicsLinear(gravity, &qdot);
    skelDyn->computeDynamics(gravity, qdot, false); // compute dynamics by not using inverse dynamics

    for(int ki=0; ki<Cginvdyn.size(); ki++){
        EXPECT_NEAR(Cginvdyn[ki], skelDyn->mCg[ki], TOLERANCE_EXACT);
    }
}

TEST(DYNAMICS, COMPARE_MASS) {
    using namespace std;
    using namespace Eigen;
    using namespace model3d;
    using namespace dynamics;

    const char* skelfilename = GROUNDZERO_DATA_PATH"skel/Yuting.vsk";
    model3d::FileInfoSkel<dynamics::SkeletonDynamics> skelFile;
    bool loadModelResult = skelFile.loadFile(skelfilename, model3d::VSK);
    ASSERT_TRUE(loadModelResult);

    SkeletonDynamics *skelDyn = static_cast<SkeletonDynamics*>(skelFile.getSkel());
    EXPECT_TRUE(skelDyn != NULL);

    const double TOLERANCE_EXACT = 1.0e-10;

    // test the velocity computation using the two methods: inverse dynamics and regular
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

    // test/compare the dynamics result for both methods
    // test the mass matrix
    skelDyn->computeDynamics(gravity, qdot, true); // compute dynamics by using inverse dynamics
    MatrixXd Minvdyn = skelDyn->mM;
    skelDyn->computeDynamics(gravity, qdot, false); // compute dynamics by NOT using inverse dynamics: use regular dynamics
    MatrixXd Mregular = skelDyn->mM;

    for(int ki=0; ki<Minvdyn.rows(); ki++){
        for(int kj=0; kj<Minvdyn.cols(); kj++){
            EXPECT_NEAR(Minvdyn(ki,kj), Mregular(ki,kj), TOLERANCE_EXACT);
        }
    }
}


#endif // #ifndef UNITTESTS_TESTDYNAMICS_H

