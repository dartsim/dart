/*
 * Copyright (c) 2011-2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>,
 *            Jeongseok Lee <jslee02@gmail.com>
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

#include "dart/dynamics/Shape.h"

#define PRIMITIVE_MAGIC_NUMBER 1000

namespace dart {
namespace dynamics {

Shape::Shape(ShapeType _type)
  : mType(_type),
    mBoundingBoxDim(0, 0, 0),
    mVolume(0),
    mID(mCounter++),
    mColor(0.5, 0.5, 1.0),
    mTransform(Eigen::Isometry3d::Identity()) {
}

Shape::~Shape() {
}

void Shape::setColor(const Eigen::Vector3d& _color) {
  mColor = _color;
}

const Eigen::Vector3d& Shape::getColor() const {
  return mColor;
}

const Eigen::Vector3d& Shape::getBoundingBoxDim() const {
  return mBoundingBoxDim;
}

void Shape::setLocalTransform(const Eigen::Isometry3d& _Transform) {
  mTransform = _Transform;
}

const Eigen::Isometry3d& Shape::getLocalTransform() const {
  return mTransform;
}

void Shape::setOffset(const Eigen::Vector3d& _offset) {
  mTransform.translation() = _offset;
}

Eigen::Vector3d Shape::getOffset() const {
  return mTransform.translation();
}

double Shape::getVolume() const {
  return mVolume;
}

int Shape::getID() const {
  return mID;
}

Shape::ShapeType Shape::getShapeType() const {
  return mType;
}

int Shape::mCounter = PRIMITIVE_MAGIC_NUMBER;

}  // namespace dynamics
}  // namespace dart
