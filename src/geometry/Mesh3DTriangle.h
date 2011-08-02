/*
    RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
    All rights reserved.

    Author	Sumit Jain
    Date	07/21/2011
*/

#ifndef GEOMETRY_MESH3DTRIANGLE_H
#define GEOMETRY_MESH3DTRIANGLE_H

#include "Mesh3DGen.h"

namespace geometry {
    class CornerTable;

    class Mesh3DTriangle: public Mesh3DGen{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        CornerTable *mCornerTable;

        Mesh3DTriangle();
        Mesh3DTriangle(const char *_file, MeshFormat _format);
        virtual ~Mesh3DTriangle(){}

        // read and write obj files
        bool readMesh(const char *_file, Mesh3DGen::MeshFormat _format);
        bool writeMesh(const char *_file, Mesh3DGen::MeshFormat _format);

        double computeVolume();
        void computeVolDer(Eigen::VectorXd &_der);

        bool readCorners(const char *_file);
        bool writeCorners(const char *_file);
        inline Eigen::Vector3d getVertex(int _vi){return Eigen::Vector3d(mVertexPos[3*_vi+0], mVertexPos[3*_vi+1], mVertexPos[3*_vi+2]);}
        inline void setVertex(int _vi, Eigen::Vector3d &_vp){
            mVertexPos[3*_vi+0] = _vp[0];
            mVertexPos[3*_vi+1] = _vp[1];
            mVertexPos[3*_vi+2] = _vp[2];
        }
        inline void setVertexVel(int _vi, Eigen::Vector3d &_vel){
            mVertexVel[3*_vi+0] = _vel[0];
            mVertexVel[3*_vi+1] = _vel[1];
            mVertexVel[3*_vi+2] = _vel[2];
        }

        inline void getVertexIndicesOfTriangle(const int _triIndex, int &_v1, int &_v2, int &_v3){
            _v1 = mFaces[3*_triIndex+0];
            _v2 = mFaces[3*_triIndex+1];
            _v3 = mFaces[3*_triIndex+2];
        }

        virtual void draw(const Eigen::Vector4d& _color, bool _drawWireFrame, bool _drawSmooth=true);

        static vector<double> getBarycentricCoords(const Eigen::Vector3d& _p, const Eigen::Vector3d& _v1, const Eigen::Vector3d& _v2, const Eigen::Vector3d& _v3);
        // projects the point onto triangle (v1,v2,v3) along normal n and return (a1,a2,a3)
        static vector<double> getBarycentricCoords(const Eigen::Vector3d& _p, const Eigen::Vector3d& _v1, const Eigen::Vector3d& _v2, const Eigen::Vector3d& _v3, const Eigen::Vector3d& _n, Eigen::Vector3d &_pproj){
            // barycentric coords
            _pproj = _p - _n.dot(_p-_v1) *_n;	// ASSUME normalized normal
            return getBarycentricCoords(_pproj, _v1, _v2, _v3);
        }

        static vector<pair<int, int> > getVertexCorres(Mesh3DTriangle *_mesh1, Mesh3DTriangle *_mesh2);
        static void writeCorres(const char *_file, Mesh3DTriangle *_m1, Mesh3DTriangle *_m2, const vector<pair<int, int> > &_corres);
        static void readCorres(const char *_file, vector<pair<int, int> > &_corres);
    };

} // namespace geometry

#endif  // GEOMETRY_MESH3DTRIANGLE_H

