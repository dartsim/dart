/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/CapsuleShape.hpp>
#include <dart/dynamics/ConeShape.hpp>
#include <dart/dynamics/CylinderShape.hpp>
#include <dart/dynamics/EllipsoidShape.hpp>
#include <dart/dynamics/PyramidShape.hpp>

#include <gtest/gtest.h>

#include <limits>
#include <vector>

#include <cmath>

using namespace dart::dynamics;

namespace {

//==============================================================================
// The set of invalid scalar dimensions every validated setter must reject.
const std::vector<double>& invalidScalars()
{
  static const std::vector<double> values{
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity(),
      0.0,
      -1.0};
  return values;
}

} // namespace

//==============================================================================
TEST(ShapeDimensionValidation, BoxShapeRejectsInvalidSize)
{
  auto box = std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 2.0, 3.0));
  const Eigen::Vector3d original = box->getSize();

  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  for (const double bad : invalidScalars()) {
    // Bad value in each component (one at a time) must be rejected.
    box->setSize(Eigen::Vector3d(bad, 2.0, 3.0));
    EXPECT_EQ(box->getSize(), original);

    box->setSize(Eigen::Vector3d(1.0, bad, 3.0));
    EXPECT_EQ(box->getSize(), original);

    box->setSize(Eigen::Vector3d(1.0, 2.0, bad));
    EXPECT_EQ(box->getSize(), original);
  }

  // Fully invalid vectors are rejected as well.
  box->setSize(Eigen::Vector3d(nan, nan, nan));
  EXPECT_EQ(box->getSize(), original);

  box->setSize(Eigen::Vector3d(-inf, -inf, -inf));
  EXPECT_EQ(box->getSize(), original);
}

//==============================================================================
TEST(ShapeDimensionValidation, CylinderShapeRejectsInvalidRadiusAndHeight)
{
  auto cylinder = std::make_shared<CylinderShape>(1.0, 2.0);
  const double originalRadius = cylinder->getRadius();
  const double originalHeight = cylinder->getHeight();

  for (const double bad : invalidScalars()) {
    cylinder->setRadius(bad);
    EXPECT_DOUBLE_EQ(cylinder->getRadius(), originalRadius);

    cylinder->setHeight(bad);
    EXPECT_DOUBLE_EQ(cylinder->getHeight(), originalHeight);
  }
}

//==============================================================================
TEST(ShapeDimensionValidation, CapsuleShapeRejectsInvalidRadiusAndHeight)
{
  auto capsule = std::make_shared<CapsuleShape>(1.0, 2.0);
  const double originalRadius = capsule->getRadius();
  const double originalHeight = capsule->getHeight();

  for (const double bad : invalidScalars()) {
    capsule->setRadius(bad);
    EXPECT_DOUBLE_EQ(capsule->getRadius(), originalRadius);

    capsule->setHeight(bad);
    EXPECT_DOUBLE_EQ(capsule->getHeight(), originalHeight);
  }
}

//==============================================================================
TEST(ShapeDimensionValidation, EllipsoidShapeRejectsInvalidDiametersAndRadii)
{
  auto ellipsoid
      = std::make_shared<EllipsoidShape>(Eigen::Vector3d(1.0, 2.0, 3.0));
  const Eigen::Vector3d original = ellipsoid->getDiameters();

  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  for (const double bad : invalidScalars()) {
    ellipsoid->setDiameters(Eigen::Vector3d(bad, 2.0, 3.0));
    EXPECT_EQ(ellipsoid->getDiameters(), original);

    ellipsoid->setDiameters(Eigen::Vector3d(1.0, bad, 3.0));
    EXPECT_EQ(ellipsoid->getDiameters(), original);

    ellipsoid->setDiameters(Eigen::Vector3d(1.0, 2.0, bad));
    EXPECT_EQ(ellipsoid->getDiameters(), original);

    // setRadii routes through setDiameters, so it must reject the same.
    ellipsoid->setRadii(Eigen::Vector3d(bad, 1.0, 1.0));
    EXPECT_EQ(ellipsoid->getDiameters(), original);
  }

  ellipsoid->setDiameters(Eigen::Vector3d(nan, nan, nan));
  EXPECT_EQ(ellipsoid->getDiameters(), original);

  ellipsoid->setDiameters(Eigen::Vector3d(-inf, -inf, -inf));
  EXPECT_EQ(ellipsoid->getDiameters(), original);
}

//==============================================================================
TEST(ShapeDimensionValidation, ConeShapeRejectsInvalidRadiusAndHeight)
{
  auto cone = std::make_shared<ConeShape>(1.0, 2.0);
  const double originalRadius = cone->getRadius();
  const double originalHeight = cone->getHeight();

  for (const double bad : invalidScalars()) {
    cone->setRadius(bad);
    EXPECT_DOUBLE_EQ(cone->getRadius(), originalRadius);

    cone->setHeight(bad);
    EXPECT_DOUBLE_EQ(cone->getHeight(), originalHeight);
  }
}

//==============================================================================
TEST(ShapeDimensionValidation, PyramidShapeRejectsInvalidDimensions)
{
  auto pyramid = std::make_shared<PyramidShape>(1.0, 2.0, 3.0);
  const double originalWidth = pyramid->getBaseWidth();
  const double originalDepth = pyramid->getBaseDepth();
  const double originalHeight = pyramid->getHeight();

  for (const double bad : invalidScalars()) {
    pyramid->setBaseWidth(bad);
    EXPECT_DOUBLE_EQ(pyramid->getBaseWidth(), originalWidth);

    pyramid->setBaseDepth(bad);
    EXPECT_DOUBLE_EQ(pyramid->getBaseDepth(), originalDepth);

    pyramid->setHeight(bad);
    EXPECT_DOUBLE_EQ(pyramid->getHeight(), originalHeight);
  }
}
