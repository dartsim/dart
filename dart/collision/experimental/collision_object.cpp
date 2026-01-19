/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#include <dart/collision/experimental/collision_object.hpp>

namespace dart::collision::experimental {

std::size_t CollisionObject::nextId_ = 0;

CollisionObject::CollisionObject(
    std::shared_ptr<Shape> shape, const Eigen::Isometry3d& transform)
    : shape_(std::move(shape)), transform_(transform), id_(nextId_++)
{
}

const Shape* CollisionObject::getShape() const
{
  return shape_.get();
}

std::shared_ptr<Shape> CollisionObject::getShapePtr() const
{
  return shape_;
}

const Eigen::Isometry3d& CollisionObject::getTransform() const
{
  return transform_;
}

void CollisionObject::setTransform(const Eigen::Isometry3d& transform)
{
  transform_ = transform;
}

Aabb CollisionObject::computeAabb() const
{
  if (!shape_) {
    return Aabb();
  }

  Aabb localAabb = shape_->computeLocalAabb();
  return Aabb::transformed(localAabb, transform_);
}

std::size_t CollisionObject::getId() const
{
  return id_;
}

void CollisionObject::setUserData(void* data)
{
  userData_ = data;
}

void* CollisionObject::getUserData() const
{
  return userData_;
}

}
