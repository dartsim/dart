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

#include <dart/common/Castable.hpp>

#include <gtest/gtest.h>

#include <memory>

using namespace dart::common;

namespace {

class Shape : public Castable<Shape>
{
public:
  virtual ~Shape() = default;
  [[nodiscard]] virtual std::string_view getType() const = 0;
};

class Sphere : public Shape
{
public:
  DART_STRING_TYPE(Sphere);

  explicit Sphere(double radius) : mRadius(radius) {}

  double getRadius() const
  {
    return mRadius;
  }

private:
  double mRadius;
};

class Box : public Shape
{
public:
  DART_STRING_TYPE(Box);

  Box(double width, double height, double depth)
    : mWidth(width), mHeight(height), mDepth(depth)
  {
  }

  double getWidth() const
  {
    return mWidth;
  }
  double getHeight() const
  {
    return mHeight;
  }
  double getDepth() const
  {
    return mDepth;
  }

private:
  double mWidth;
  double mHeight;
  double mDepth;
};

class Cylinder : public Shape
{
public:
  DART_STRING_TYPE(Cylinder);

  Cylinder(double radius, double height) : mRadius(radius), mHeight(height) {}

  double getRadius() const
  {
    return mRadius;
  }
  double getHeight() const
  {
    return mHeight;
  }

private:
  double mRadius;
  double mHeight;
};

} // namespace

//==============================================================================
TEST(Castable, IsReturnsCorrectType)
{
  Sphere sphere(1.0);
  Box box(1.0, 2.0, 3.0);
  Cylinder cylinder(0.5, 2.0);

  Shape& sphereRef = sphere;
  Shape& boxRef = box;
  Shape& cylinderRef = cylinder;

  EXPECT_TRUE(sphereRef.is<Sphere>());
  EXPECT_FALSE(sphereRef.is<Box>());
  EXPECT_FALSE(sphereRef.is<Cylinder>());

  EXPECT_FALSE(boxRef.is<Sphere>());
  EXPECT_TRUE(boxRef.is<Box>());
  EXPECT_FALSE(boxRef.is<Cylinder>());

  EXPECT_FALSE(cylinderRef.is<Sphere>());
  EXPECT_FALSE(cylinderRef.is<Box>());
  EXPECT_TRUE(cylinderRef.is<Cylinder>());
}

//==============================================================================
TEST(Castable, AsReturnsPointerOrNull)
{
  Sphere sphere(2.5);
  Box box(1.0, 2.0, 3.0);

  Shape& sphereRef = sphere;
  Shape& boxRef = box;

  Sphere* spherePtr = sphereRef.as<Sphere>();
  Box* boxFromSpherePtr = sphereRef.as<Box>();

  EXPECT_NE(spherePtr, nullptr);
  EXPECT_EQ(boxFromSpherePtr, nullptr);
  EXPECT_DOUBLE_EQ(spherePtr->getRadius(), 2.5);

  Box* boxPtr = boxRef.as<Box>();
  Sphere* sphereFromBoxPtr = boxRef.as<Sphere>();

  EXPECT_NE(boxPtr, nullptr);
  EXPECT_EQ(sphereFromBoxPtr, nullptr);
  EXPECT_DOUBLE_EQ(boxPtr->getWidth(), 1.0);
}

//==============================================================================
TEST(Castable, AsConstReturnsConstPointerOrNull)
{
  const Sphere sphere(3.0);
  const Box box(4.0, 5.0, 6.0);

  const Shape& sphereRef = sphere;
  const Shape& boxRef = box;

  const Sphere* spherePtr = sphereRef.as<Sphere>();
  const Box* boxFromSpherePtr = sphereRef.as<Box>();

  EXPECT_NE(spherePtr, nullptr);
  EXPECT_EQ(boxFromSpherePtr, nullptr);
  EXPECT_DOUBLE_EQ(spherePtr->getRadius(), 3.0);

  const Box* boxPtr = boxRef.as<Box>();
  const Sphere* sphereFromBoxPtr = boxRef.as<Sphere>();

  EXPECT_NE(boxPtr, nullptr);
  EXPECT_EQ(sphereFromBoxPtr, nullptr);
  EXPECT_DOUBLE_EQ(boxPtr->getHeight(), 5.0);
}

//==============================================================================
TEST(Castable, AsRefReturnsReference)
{
  Sphere sphere(1.5);
  Shape& shapeRef = sphere;

  Sphere& sphereRef = shapeRef.asRef<Sphere>();
  EXPECT_DOUBLE_EQ(sphereRef.getRadius(), 1.5);
}

//==============================================================================
TEST(Castable, AsRefConstReturnsConstReference)
{
  const Cylinder cylinder(0.75, 3.0);
  const Shape& shapeRef = cylinder;

  const Cylinder& cylinderRef = shapeRef.asRef<Cylinder>();
  EXPECT_DOUBLE_EQ(cylinderRef.getRadius(), 0.75);
  EXPECT_DOUBLE_EQ(cylinderRef.getHeight(), 3.0);
}

//==============================================================================
TEST(Castable, GetStaticTypeReturnsCorrectString)
{
  EXPECT_EQ(Sphere::getStaticType(), "Sphere");
  EXPECT_EQ(Box::getStaticType(), "Box");
  EXPECT_EQ(Cylinder::getStaticType(), "Cylinder");
}

//==============================================================================
TEST(Castable, GetTypeReturnsCorrectString)
{
  Sphere sphere(1.0);
  Box box(1.0, 1.0, 1.0);
  Cylinder cylinder(1.0, 1.0);

  EXPECT_EQ(sphere.getType(), "Sphere");
  EXPECT_EQ(box.getType(), "Box");
  EXPECT_EQ(cylinder.getType(), "Cylinder");

  Shape& sphereRef = sphere;
  EXPECT_EQ(sphereRef.getType(), "Sphere");
}

//==============================================================================
TEST(Castable, WorksWithUniquePtr)
{
  std::unique_ptr<Shape> shape = std::make_unique<Sphere>(5.0);

  EXPECT_TRUE(shape->is<Sphere>());
  EXPECT_FALSE(shape->is<Box>());

  Sphere* sphere = shape->as<Sphere>();
  EXPECT_NE(sphere, nullptr);
  EXPECT_DOUBLE_EQ(sphere->getRadius(), 5.0);
}

//==============================================================================
TEST(Castable, WorksWithSharedPtr)
{
  std::shared_ptr<Shape> shape = std::make_shared<Box>(2.0, 3.0, 4.0);

  EXPECT_FALSE(shape->is<Sphere>());
  EXPECT_TRUE(shape->is<Box>());

  Box* box = shape->as<Box>();
  EXPECT_NE(box, nullptr);
  EXPECT_DOUBLE_EQ(box->getWidth(), 2.0);
  EXPECT_DOUBLE_EQ(box->getHeight(), 3.0);
  EXPECT_DOUBLE_EQ(box->getDepth(), 4.0);
}

//==============================================================================
TEST(Castable, AsCanModifyThroughPointer)
{
  Sphere sphere(1.0);
  Shape& shapeRef = sphere;

  Sphere* ptr = shapeRef.as<Sphere>();
  ASSERT_NE(ptr, nullptr);

  EXPECT_DOUBLE_EQ(ptr->getRadius(), 1.0);
  EXPECT_DOUBLE_EQ(sphere.getRadius(), 1.0);
}
