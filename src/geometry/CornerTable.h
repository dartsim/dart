/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sumit Jain <sumit@cc.gatech.edu>
 * Date: 07/21/2011
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

#ifndef GEOMETRY_CORNERTABLE_H
#define GEOMETRY_CORNERTABLE_H

#include <Eigen/Dense>
#include <vector>
using namespace std;

namespace geometry{
    class Mesh3DTriangle;

    class CornerTable{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Mesh3DTriangle *mMesh;
        int mNumCorners;

        vector<int> mIndOppCorners;	///< gives the opposite corner
        vector<int> mIndCorners;    ///< gives the corner index given the vertex (size = nverts, each ith position gives one incident corner)

        CornerTable(Mesh3DTriangle *_mesh3d);
        virtual ~CornerTable(){}

        void computeCornerOpposites();
        void checkConsistentOrientation();
        void computeVertexToCornerLookup();
        void initCornerTable(){
            computeCornerOpposites();
            checkConsistentOrientation();
            computeVertexToCornerLookup();
        }

        // compute neighbors of a vertex/corner
        bool isCornerBoundaryVertex(int sc);
        vector<int> verticesAroundCorner(int sc);
        bool isVertexBoundaryVertex(int v) {
            return isCornerBoundaryVertex(mIndCorners[v]);
        }
        vector<int> verticesAroundVertex(int v) {
            return verticesAroundCorner(mIndCorners[v]);
        }

    protected:
 
        ///< basic operators: see Cornertable notes
        int t(int c);
        int v(int c);
        int n(int c);
        int o(int c);

        int p(int c);
        int r(int c);
        int l(int c);
        int sr(int c);
        int sl(int c);
        int s(int c);   ///< swing operator
        Eigen::Vector3d gc(int _c);
        Eigen::Vector3d gv(int _v);
        bool b(int c);

    };
}   // namespace geometry

#endif  //GEOMETRY_CORNERTABLE_H

