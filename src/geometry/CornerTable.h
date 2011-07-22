/*
    RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
    All rights reserved.

    Author	Sumit Jain
    Date	07/21/2011
*/

#ifndef GEOMETRY_CORNERTABLE_H
#define GEOMETRY_CORNERTABLE_H

#include <Eigen/dense>
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
        virtual CornerTable::~CornerTable(){}

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

