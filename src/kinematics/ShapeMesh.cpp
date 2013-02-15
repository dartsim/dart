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
#include <iostream>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/cimport.h>

using namespace Eigen;
using namespace std;

namespace kinematics {

    ShapeMesh::ShapeMesh(Vector3d _dim, double _mass, const aiScene *_mesh){
        mType = P_MESH;
        mDim = _dim;
        mMass = _mass;
        mMeshData = _mesh;
        initMeshes();
        if (mDim != Vector3d::Zero())
            computeVolume();
        if (mMass != 0){
            computeInertia();
        }

        listIndex = 0;
        mVizList = 0;
        mColList = 0;
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

    void ShapeMesh::computeInertia() {
        // use bounding box to represent the mesh
        double max_X = -1e7;
        double max_Y = -1e7;
        double max_Z = -1e7;
        double min_X = 1e7;
        double min_Y = 1e7;
        double min_Z = 1e7;

        for(unsigned int i = 0; i < mMeshData->mNumMeshes; i++) {
            for (unsigned int j = 0; j < mMeshData->mMeshes[i]->mNumVertices; j++) {
                if (mMeshData->mMeshes[i]->mVertices[j].x > max_X)
                    max_X = mMeshData->mMeshes[i]->mVertices[j].x;
                if (mMeshData->mMeshes[i]->mVertices[j].x < min_X)
                    min_X = mMeshData->mMeshes[i]->mVertices[j].x;
                if (mMeshData->mMeshes[i]->mVertices[j].y > max_Y)
                    max_Y = mMeshData->mMeshes[i]->mVertices[j].y;
                if (mMeshData->mMeshes[i]->mVertices[j].y < min_Y)
                    min_Y = mMeshData->mMeshes[i]->mVertices[j].y;
                if (mMeshData->mMeshes[i]->mVertices[j].z > max_Z)
                    max_Z = mMeshData->mMeshes[i]->mVertices[j].z;
                if (mMeshData->mMeshes[i]->mVertices[j].z < min_Z)
                    min_Z = mMeshData->mMeshes[i]->mVertices[j].z;
            }
        }
        double l = mDim[0] * (max_X - min_X);
        double h = mDim[1] * (max_Y - min_Y);
        double w = mDim[2] * (max_Z - min_Z);


        mInertia = Matrix3d::Zero();
        mInertia(0, 0) = mMass / 12.0 * (h * h + w * w);
        mInertia(1, 1) = mMass / 12.0 * (l * l + w * w);
        mInertia(2, 2) = mMass / 12.0 * (l * l + h * h);
    }

    void ShapeMesh::computeVolume() {
        mVolume = mDim(0) * mDim(1) * mDim(2); // a * b * c
    }

    void ShapeMesh::initMeshes() {
    }

    const aiScene* ShapeMesh::loadMesh(const string& fileName) {
        aiPropertyStore* propertyStore = aiCreatePropertyStore();
        aiSetImportPropertyInteger(propertyStore, AI_CONFIG_PP_SBP_REMOVE, aiPrimitiveType_POINT | aiPrimitiveType_LINE ); // remove points and lines
        const aiScene* scene = aiImportFileExWithProperties(fileName.c_str(), aiProcess_GenNormals             |
                                                                              aiProcess_Triangulate            |
                                                                              aiProcess_JoinIdenticalVertices  |
                                                                              aiProcess_SortByPType            |
                                                                              aiProcess_PreTransformVertices   |
                                                                              aiProcess_OptimizeMeshes,
                                                            NULL, propertyStore);
        aiReleasePropertyStore(propertyStore);
        return scene;
    }
} // namespace kinematics
