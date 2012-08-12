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

#ifndef GEOMETRY_MESH3DTRIANGLE_H
#define GEOMETRY_MESH3DTRIANGLE_H

#include "Mesh3D.h"

namespace geometry {
    class CornerTable;

    class Mesh3DTriangle: public Mesh3D{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        CornerTable *mCornerTable;

        Mesh3DTriangle();
        Mesh3DTriangle(const char *_file, MeshFormat _format);
        virtual ~Mesh3DTriangle(){}

        // read and write obj files
        bool readMesh(const char *_file, Mesh3D::MeshFormat _format);
        bool writeMesh(const char *_file, Mesh3D::MeshFormat _format);

        double computeVolume();
        void computeVolDer(Eigen::VectorXd &_der);

        bool readCorners(const char *_file);
        bool writeCorners(const char *_file);
        inline Eigen::Vector3d getVertex(int _vi) const {return Eigen::Vector3d(mVertexPos[3*_vi+0], mVertexPos[3*_vi+1], mVertexPos[3*_vi+2]);}
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

        virtual void draw(const Eigen::Vector4d& _color, bool _drawWireFrame, bool _drawSmooth=true) const;

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

