/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s):
 * Date:
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

#include "ShapeMesh.h"
#include "renderer/RenderInterface.h"
#include "geometry/Mesh3D.h"
#include <iostream>

using namespace Eigen;
using namespace geometry;
using namespace std;

namespace kinematics {

    ShapeMesh::ShapeMesh(Vector3d _dim, double _mass, Mesh3D *_mesh){
        mType = P_MESH;
        mDim = _dim;
        mMass = _mass;
        mMeshData = _mesh;
        initMeshes();
        if (mDim != Vector3d::Zero())
            computeVolume();
        if (mMass != 0){
            computeMassTensor();
            computeInertiaFromMassTensor();
        }
    }

    void ShapeMesh::draw(renderer::RenderInterface* _ri, const Vector4d& _color, bool _useDefaultColor) const {
        if (!_ri)
            return;
        if (!_useDefaultColor)
            _ri->setPenColor(_color);
        else
            _ri->setPenColor(mColor);
        _ri->pushMatrix();
        _ri->drawMesh(mDim, mVizMesh);
        _ri->popMatrix();
    }

    void ShapeMesh::computeMassTensor() {
        // use bounding box to represent the mesh
        double max_X = -1e7;
        double max_Y = -1e7;
        double max_Z = -1e7;
        double min_X = 1e7;
        double min_Y = 1e7;
        double min_Z = 1e7;

        for (unsigned int i = 0; i < mMeshData->mNumVertices; i++) {
            if (mMeshData->mVertexPos[3 * i] > max_X)
                max_X = mMeshData->mVertexPos[3 * i];
            if (mMeshData->mVertexPos[3 * i] < min_X)
                min_X = mMeshData->mVertexPos[3 * i];
            if (mMeshData->mVertexPos[3 * i + 1] > max_Y)
                max_Y = mMeshData->mVertexPos[3 * i + 1];
            if (mMeshData->mVertexPos[3 * i + 1] < min_Y)
                min_Y = mMeshData->mVertexPos[3 * i + 1];
            if (mMeshData->mVertexPos[3 * i + 2] > max_Z)
                max_Z = mMeshData->mVertexPos[3 * i + 2];
            if (mMeshData->mVertexPos[3 * i + 2] < min_Z)
                min_Z = mMeshData->mVertexPos[3 * i + 2];
        }
        double l = mDim[0] * (max_X - min_X);
        double h = mDim[1] * (max_Y - min_Y);
        double w = mDim[2] * (max_Z - min_Z);


        mMassTensor(0, 0) = l * l / 12;
        mMassTensor(1, 1) = h * h / 12;
        mMassTensor(2, 2) = w * w / 12;
        mMassTensor(3, 3) = 1;
        mMassTensor *= mMass;
    }

    void ShapeMesh::computeVolume() {
        mVolume = mDim(0) * mDim(1) * mDim(2); // a * b * c
    }

    void ShapeMesh::initMeshes() {
    }
} // namespace kinematics
