/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Geoorgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>
#include <gtest/gtest.h>
#include "kinematics/FileInfoSkel.hpp"
#include "kinematics/Skeleton.h"
#include "kinematics/BodyNode.h"
#include "kinematics/FileInfoDof.h"
#include "kinematics/FileInfoC3D.h"
#include "kinematics/TrfmRotateExpmap.h"
#include "utils/Paths.h"
#include "math/UtilsMath.h"

using namespace std;

/* ********************************************************************************************* */
TEST(KINEMATICS, VSK_LOADER) {
    using namespace Eigen;
    using namespace kinematics;
  
    FileInfoSkel<Skeleton> modelFile;
    modelFile.loadFile(DART_DATA_PATH"skel/Yuting.vsk");
    Skeleton* skel = modelFile.getSkel();

    EXPECT_TRUE(skel != NULL);

    EXPECT_EQ(skel->getNumDofs(), 66);
    EXPECT_EQ(skel->getNumNodes(), 30);
    EXPECT_EQ(skel->getNumMarkers(), 53);
}

/* ********************************************************************************************* */
TEST(KINEMATICS, C3D_LOADER) {
    using namespace Eigen;
    using namespace kinematics;
  
    FileInfoC3D c3dFile;
    bool result = c3dFile.loadFile(DART_DATA_PATH"c3d/squat.c3d");
    ASSERT_TRUE(result);

    EXPECT_EQ(c3dFile.getNumMarkers(), 53);
    EXPECT_EQ(c3dFile.getNumFrames(), 2539);
}

/* ********************************************************************************************* */
TEST(KINEMATICS, TRANS_AND_DERIV) {
    using namespace Eigen;
    using namespace kinematics;
  
    FileInfoSkel<Skeleton> modelFile;
    bool loadModelResult = modelFile.loadFile(DART_DATA_PATH"skel/SehoonVSK3.vsk");
    ASSERT_TRUE(loadModelResult);
    
    Skeleton* skel = modelFile.getSkel();
    EXPECT_TRUE(skel != NULL);
    /* LOG(INFO) << "# Dofs = " << skel->getNumDofs(); */
    /* LOG(INFO) << "# Nodes  = " << skel->getNumNodes(); */


    FileInfoDof dofFile(skel);
    bool loadDofResult = dofFile.loadFile(DART_DATA_PATH"dof/init_Tpose.dof");
    ASSERT_TRUE(loadDofResult);
    /* LOG(INFO) << "# frames = " << dofFile.getNumFrames(); */

    VectorXd pose = dofFile.getPoseAtFrame(0);
    skel->setPose(pose, true, true);

    const int nodeIndex = 1;
    BodyNode* node = skel->getNode(nodeIndex);
    EXPECT_TRUE(node != NULL);

    const double TOLERANCE = 0.000001;

    // Check the one global transform matrix
    const Matrix4d& W = skel->getNode(20)->getWorldTransform();
    Matrix4d W_truth;
    W_truth << -0.110662, 0.991044, 0.074734, 0.642841
        , -0.987564, -0.118099, 0.103775, 0.327496
        , 0.111672, -0.0623207, 0.991789, -0.12855
        , 0, 0, 0, 1;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            EXPECT_NEAR(W(i, j), W_truth(i, j), TOLERANCE);
        }
    }

    // Check the derivative of one global transform matrix
    // const Matrix4d& Wq = skel->getNode(10)->mWq.at(4);
    const Matrix4d& Wq = skel->getNode(10)->getDerivWorldTransform(4);
    Matrix4d Wq_truth;
    Wq_truth << 0.121382, 0.413015, 0.902378, 0.161838
        ,-0.0175714, 0.00698451, 0.0149899, 0.00571836
        ,-0.992336, 0.0556535, 0.107702, 0.175059
        ,0, 0, 0, 0;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            EXPECT_NEAR(Wq(i, j), Wq_truth(i, j), TOLERANCE);
        }
    }


    // Check Jacobians
    BodyNode *nodecheck = skel->getNode(23);

    // Linear Jacobian
    const MatrixXd Jv = nodecheck->getJacobianLinear();
    MatrixXd Jv_truth = MatrixXd::Zero(Jv.rows(), Jv.cols());
    Jv_truth<<1, 0, 0, 0.013694, -0.0194371, -0.422612, 0.0157281, -0.00961473, -0.421925, 0.0026645, -0.0048767, -0.179914, -0.0013539, -0.00132969, -0.00885535, 
        0, 1, 0, 0.0321939, 0.00226492, -0.135736, 0.0330525, 0.00462613, -0.135448, 0.0181491, 0.00758086, -0.122187, 0.00532694, 0.0049221, -0.128917,
        0, 0, 1, 0.423948, 0.130694, 0.0166546, 0.42642, 0.118176, 0.0221309, 0.180296, 0.120584, 0.0105059, 0.0838259, 0.081304, 0.00719314;
    for (int i = 0; i < Jv.rows(); i++) {
        for (int j = 0; j < Jv.cols(); j++) {
            EXPECT_NEAR(Jv(i, j), Jv_truth(i, j), TOLERANCE);
        }
    }

    // Angular Jacobian
    const MatrixXd Jwbody = nodecheck->getWorldTransform().topLeftCorner<3,3>().transpose()
        * nodecheck->getJacobianAngular();
    MatrixXd Jwbody_truth = MatrixXd::Zero(Jwbody.rows(), Jwbody.cols());
    Jwbody_truth << 0, 0, 0, 0.0818662, 0.996527, 0.000504051, 0.110263, 0.993552, 0.0249018, 0.0772274, 0.995683, 0.0423836, 0.648386, 0.628838, 0.0338389,
        0, 0, 0, -0.995079, 0.082001, -0.0518665, -0.992054, 0.111479, -0.0554717, -0.996033, 0.078601, -0.0313861, -0.629201, 0.649145, -0.00994729,
        0, 0, 0, -0.0518265, 0.00391001, 0.998406, -0.0579139, -0.0187244, 0.998021, -0.0343359, -0.0398718, 0.998564, -0.0262454, -0.0235626, 0.999159;

    for (int i = 0; i < Jwbody.rows(); i++) {
        for (int j = 0; j < Jwbody.cols(); j++) {
            EXPECT_NEAR(Jwbody(i, j), Jwbody_truth(i, j), TOLERANCE);
        }
    }
}

/* ********************************************************************************************* */
int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
/* ********************************************************************************************* */
