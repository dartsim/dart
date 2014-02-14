/*
 * Copyright (c) 2011-2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Chen Tang
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
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

#ifndef DART_COLLISION_FCL_MESH_COLLISIONSHAPES_H_
#define DART_COLLISION_FCL_MESH_COLLISIONSHAPES_H_

#include <cmath>
#include <iostream>

#include <assimp/scene.h>

#include "fcl/BVH/BVH_model.h"

namespace dart {
namespace collision {

template<class BV>
fcl::BVHModel<BV>* createMesh(float _scaleX, float _scaleY, float _scaleZ,
                              const aiScene *_mesh,
                              const fcl::Transform3f& _transform) {
  assert(_mesh);
  fcl::BVHModel<BV>* model = new fcl::BVHModel<BV>;
  model->beginModel();

  for (unsigned int i = 0; i < _mesh->mNumMeshes; i++) {
    for (unsigned int j = 0; j < _mesh->mMeshes[i]->mNumFaces; j++) {
      fcl::Vec3f vertices[3];
      for (unsigned int k = 0; k < 3; k++) {
        const aiVector3D& vertex
            = _mesh->mMeshes[i]->mVertices[
                  _mesh->mMeshes[i]->mFaces[j].mIndices[k]];
        vertices[k] = fcl::Vec3f(vertex.x * _scaleX,
                                 vertex.y * _scaleY,
                                 vertex.z * _scaleZ);
        vertices[k] = _transform.transform(vertices[k]);
      }
      model->addTriangle(vertices[0], vertices[1], vertices[2]);
    }
  }

  model->endModel();
  return model;
}

template<class BV>
fcl::BVHModel<BV>* createEllipsoid(float _sizeX, float _sizeY, float _sizeZ,
                                   const fcl::Transform3f& _transform) {
  float v[59][3] = {
    {0, 0, 0},
    {0.135299, -0.461940, -0.135299},
    {0.000000, -0.461940, -0.191342},
    {-0.135299, -0.461940, -0.135299},
    {-0.191342, -0.461940, 0.000000},
    {-0.135299, -0.461940, 0.135299},
    {0.000000, -0.461940, 0.191342},
    {0.135299, -0.461940, 0.135299},
    {0.191342, -0.461940, 0.000000},
    {0.250000, -0.353553, -0.250000},
    {0.000000, -0.353553, -0.353553},
    {-0.250000, -0.353553, -0.250000},
    {-0.353553, -0.353553, 0.000000},
    {-0.250000, -0.353553, 0.250000},
    {0.000000, -0.353553, 0.353553},
    {0.250000, -0.353553, 0.250000},
    {0.353553, -0.353553, 0.000000},
    {0.326641, -0.191342, -0.326641},
    {0.000000, -0.191342, -0.461940},
    {-0.326641, -0.191342, -0.326641},
    {-0.461940, -0.191342, 0.000000},
    {-0.326641, -0.191342, 0.326641},
    {0.000000, -0.191342, 0.461940},
    {0.326641, -0.191342, 0.326641},
    {0.461940, -0.191342, 0.000000},
    {0.353553, 0.000000, -0.353553},
    {0.000000, 0.000000, -0.500000},
    {-0.353553, 0.000000, -0.353553},
    {-0.500000, 0.000000, 0.000000},
    {-0.353553, 0.000000, 0.353553},
    {0.000000, 0.000000, 0.500000},
    {0.353553, 0.000000, 0.353553},
    {0.500000, 0.000000, 0.000000},
    {0.326641, 0.191342, -0.326641},
    {0.000000, 0.191342, -0.461940},
    {-0.326641, 0.191342, -0.326641},
    {-0.461940, 0.191342, 0.000000},
    {-0.326641, 0.191342, 0.326641},
    {0.000000, 0.191342, 0.461940},
    {0.326641, 0.191342, 0.326641},
    {0.461940, 0.191342, 0.000000},
    {0.250000, 0.353553, -0.250000},
    {0.000000, 0.353553, -0.353553},
    {-0.250000, 0.353553, -0.250000},
    {-0.353553, 0.353553, 0.000000},
    {-0.250000, 0.353553, 0.250000},
    {0.000000, 0.353553, 0.353553},
    {0.250000, 0.353553, 0.250000},
    {0.353553, 0.353553, 0.000000},
    {0.135299, 0.461940, -0.135299},
    {0.000000, 0.461940, -0.191342},
    {-0.135299, 0.461940, -0.135299},
    {-0.191342, 0.461940, 0.000000},
    {-0.135299, 0.461940, 0.135299},
    {0.000000, 0.461940, 0.191342},
    {0.135299, 0.461940, 0.135299},
    {0.191342, 0.461940, 0.000000},
    {0.000000, -0.500000, 0.000000},
    {0.000000, 0.500000, 0.000000}
  };
  int f[112][3] = {
    {1, 2, 9},
    {9, 2, 10},
    {2, 3, 10},
    {10, 3, 11},
    {3, 4, 11},
    {11, 4, 12},
    {4, 5, 12},
    {12, 5, 13},
    {5, 6, 13},
    {13, 6, 14},
    {6, 7, 14},
    {14, 7, 15},
    {7, 8, 15},
    {15, 8, 16},
    {8, 1, 16},
    {16, 1, 9},
    {9, 10, 17},
    {17, 10, 18},
    {10, 11, 18},
    {18, 11, 19},
    {11, 12, 19},
    {19, 12, 20},
    {12, 13, 20},
    {20, 13, 21},
    {13, 14, 21},
    {21, 14, 22},
    {14, 15, 22},
    {22, 15, 23},
    {15, 16, 23},
    {23, 16, 24},
    {16, 9, 24},
    {24, 9, 17},
    {17, 18, 25},
    {25, 18, 26},
    {18, 19, 26},
    {26, 19, 27},
    {19, 20, 27},
    {27, 20, 28},
    {20, 21, 28},
    {28, 21, 29},
    {21, 22, 29},
    {29, 22, 30},
    {22, 23, 30},
    {30, 23, 31},
    {23, 24, 31},
    {31, 24, 32},
    {24, 17, 32},
    {32, 17, 25},
    {25, 26, 33},
    {33, 26, 34},
    {26, 27, 34},
    {34, 27, 35},
    {27, 28, 35},
    {35, 28, 36},
    {28, 29, 36},
    {36, 29, 37},
    {29, 30, 37},
    {37, 30, 38},
    {30, 31, 38},
    {38, 31, 39},
    {31, 32, 39},
    {39, 32, 40},
    {32, 25, 40},
    {40, 25, 33},
    {33, 34, 41},
    {41, 34, 42},
    {34, 35, 42},
    {42, 35, 43},
    {35, 36, 43},
    {43, 36, 44},
    {36, 37, 44},
    {44, 37, 45},
    {37, 38, 45},
    {45, 38, 46},
    {38, 39, 46},
    {46, 39, 47},
    {39, 40, 47},
    {47, 40, 48},
    {40, 33, 48},
    {48, 33, 41},
    {41, 42, 49},
    {49, 42, 50},
    {42, 43, 50},
    {50, 43, 51},
    {43, 44, 51},
    {51, 44, 52},
    {44, 45, 52},
    {52, 45, 53},
    {45, 46, 53},
    {53, 46, 54},
    {46, 47, 54},
    {54, 47, 55},
    {47, 48, 55},
    {55, 48, 56},
    {48, 41, 56},
    {56, 41, 49},
    {2, 1, 57},
    {3, 2, 57},
    {4, 3, 57},
    {5, 4, 57},
    {6, 5, 57},
    {7, 6, 57},
    {8, 7, 57},
    {1, 8, 57},
    {49, 50, 58},
    {50, 51, 58},
    {51, 52, 58},
    {52, 53, 58},
    {53, 54, 58},
    {54, 55, 58},
    {55, 56, 58},
    {56, 49, 58}
  };

  fcl::BVHModel<BV>* model = new fcl::BVHModel<BV>;
  fcl::Vec3f p1, p2, p3;
  model->beginModel();

  for (int i = 0; i < 112; i++) {
    p1 = fcl::Vec3f(v[f[i][0]][0] * _sizeX,
                    v[f[i][0]][1] * _sizeY,
                    v[f[i][0]][2] * _sizeZ);
    p2 = fcl::Vec3f(v[f[i][1]][0] * _sizeX,
                    v[f[i][1]][1] * _sizeY,
                    v[f[i][1]][2] * _sizeZ);
    p3 = fcl::Vec3f(v[f[i][2]][0] * _sizeX,
                    v[f[i][2]][1] * _sizeY,
                    v[f[i][2]][2] * _sizeZ);
    p1 = _transform.transform(p1);
    p2 = _transform.transform(p2);
    p3 = _transform.transform(p3);
    model->addTriangle(p1, p2, p3);
  }
  model->endModel();
  return model;
}

// Create a cube mesh for collision detection
template<class BV>
fcl::BVHModel<BV>* createCube(float _sizeX, float _sizeY, float _sizeZ,
                              const fcl::Transform3f& _transform) {
  float n[6][3] = {
    {-1.0, 0.0, 0.0},
    {0.0, 1.0, 0.0},
    {1.0, 0.0, 0.0},
    {0.0, -1.0, 0.0},
    {0.0, 0.0, 1.0},
    {0.0, 0.0, -1.0}
  };
  int faces[6][4] = {
    {0, 1, 2, 3},
    {3, 2, 6, 7},
    {7, 6, 5, 4},
    {4, 5, 1, 0},
    {5, 6, 2, 1},
    {7, 4, 0, 3}
  };
  float v[8][3];

  v[0][0] = v[1][0] = v[2][0] = v[3][0] = -_sizeX / 2;
  v[4][0] = v[5][0] = v[6][0] = v[7][0] = _sizeX / 2;
  v[0][1] = v[1][1] = v[4][1] = v[5][1] = -_sizeY / 2;
  v[2][1] = v[3][1] = v[6][1] = v[7][1] = _sizeY / 2;
  v[0][2] = v[3][2] = v[4][2] = v[7][2] = -_sizeZ / 2;
  v[1][2] = v[2][2] = v[5][2] = v[6][2] = _sizeZ / 2;

  fcl::BVHModel<BV>* model = new fcl::BVHModel<BV>;
  fcl::Vec3f p1, p2, p3;
  model->beginModel();

  for (int i = 0; i < 6; i++) {
    p1 = fcl::Vec3f(v[faces[i][0]][0], v[faces[i][0]][1], v[faces[i][0]][2]);
    p2 = fcl::Vec3f(v[faces[i][1]][0], v[faces[i][1]][1], v[faces[i][1]][2]);
    p3 = fcl::Vec3f(v[faces[i][2]][0], v[faces[i][2]][1], v[faces[i][2]][2]);
    p1 = _transform.transform(p1);
    p2 = _transform.transform(p2);
    p3 = _transform.transform(p3);
    model->addTriangle(p1, p2, p3);
    p1 = fcl::Vec3f(v[faces[i][0]][0], v[faces[i][0]][1], v[faces[i][0]][2]);
    p2 = fcl::Vec3f(v[faces[i][2]][0], v[faces[i][2]][1], v[faces[i][2]][2]);
    p3 = fcl::Vec3f(v[faces[i][3]][0], v[faces[i][3]][1], v[faces[i][3]][2]);
    p1 = _transform.transform(p1);
    p2 = _transform.transform(p2);
    p3 = _transform.transform(p3);
    model->addTriangle(p1, p2, p3);
  }
  model->endModel();
  return model;
}

template<class BV>
fcl::BVHModel<BV>* createCylinder(double _baseRadius, double _topRadius,
                                  double _height, int _slices, int _stacks,
                                  const fcl::Transform3f& _transform) {
  const int CACHE_SIZE = 240;

  int i, j;
  float sinCache[CACHE_SIZE];
  float cosCache[CACHE_SIZE];
  float angle;
  float zBase;
  float zLow, zHigh;
  float sintemp, costemp;
  float deltaRadius;
  float radiusLow, radiusHigh;

  if (_slices >= CACHE_SIZE) _slices = CACHE_SIZE-1;

  if (_slices < 2 || _stacks < 1 || _baseRadius < 0.0 || _topRadius < 0.0 ||
      _height < 0.0) {
    return NULL;
  }

  /* Center at CoM */
  zBase = -_height/2;

  /* Compute delta */
  deltaRadius = _baseRadius - _topRadius;

  /* Cache is the vertex locations cache */
  for (i = 0; i < _slices; i++) {
    angle = 2 * M_PI * i / _slices;
    sinCache[i] = sin(angle);
    cosCache[i] = cos(angle);
  }

  sinCache[_slices] = sinCache[0];
  cosCache[_slices] = cosCache[0];

  fcl::BVHModel<BV>* model = new fcl::BVHModel<BV>;
  fcl::Vec3f p1, p2, p3, p4;

  model->beginModel();

  /* Base of cylinder */
  sintemp = sinCache[0];
  costemp = cosCache[0];
  radiusLow = _baseRadius;
  zLow = zBase;
  p1 = fcl::Vec3f(radiusLow * sintemp, radiusLow * costemp, zLow);
  p1 = _transform.transform(p1);
  for (i = 1; i < _slices; i++) {
    p2 = fcl::Vec3f(radiusLow * sinCache[i], radiusLow * cosCache[i], zLow);
    p3 = fcl::Vec3f(radiusLow * sinCache[i+1], radiusLow * cosCache[i+1], zLow);
    p2 = _transform.transform(p2);
    p3 = _transform.transform(p3);
    model->addTriangle(p1, p2, p3);

    Eigen::Vector3d v(radiusLow * sinCache[i], radiusLow * cosCache[i], zLow);
  }

  /* Body of cylinder */
  for (i = 0; i < _slices; i++) {
    for (j = 0; j < _stacks; j++) {
      zLow = j * _height / _stacks + zBase;
      zHigh = (j + 1) * _height / _stacks + zBase;
      radiusLow = _baseRadius
                  - deltaRadius * (static_cast<float>(j) / _stacks);
      radiusHigh = _baseRadius
                   - deltaRadius * (static_cast<float>(j + 1) / _stacks);

      p1 = fcl::Vec3f(radiusLow * sinCache[i], radiusLow * cosCache[i],
                      zLow);
      p2 = fcl::Vec3f(radiusLow * sinCache[i+1], radiusLow * cosCache[i+1],
                      zLow);
      p3 = fcl::Vec3f(radiusHigh * sinCache[i], radiusHigh * cosCache[i],
                      zHigh);
      p4 = fcl::Vec3f(radiusHigh * sinCache[i+1], radiusHigh * cosCache[i+1],
                      zHigh);
      p1 = _transform.transform(p1);
      p2 = _transform.transform(p2);
      p3 = _transform.transform(p3);
      p4 = _transform.transform(p4);

      model->addTriangle(p1, p2, p3);
      model->addTriangle(p2, p3, p4);
    }
  }

  /* Top of cylinder */
  sintemp = sinCache[0];
  costemp = cosCache[0];
  radiusLow = _topRadius;
  zLow = zBase + _height;
  p1 = fcl::Vec3f(radiusLow * sintemp, radiusLow * costemp, zLow);
  p1 = _transform.transform(p1);
  for (i = 1; i < _slices; i++) {
    p2 = fcl::Vec3f(radiusLow * sinCache[i], radiusLow * cosCache[i], zLow);
    p3 = fcl::Vec3f(radiusLow * sinCache[i+1], radiusLow * cosCache[i+1], zLow);
    p2 = _transform.transform(p2);
    p3 = _transform.transform(p3);
    model->addTriangle(p1, p2, p3);
  }

  model->endModel();
  return model;
}

}  // namespace collision
}  // namespace dart

#endif  // DART_COLLISION_FCL_MESH_COLLISIONSHAPES_H_
