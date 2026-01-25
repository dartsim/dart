/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
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

// Unit tests for XmlHelpers parsing functions
// See: https://github.com/dartsim/dart/issues/2423 (toVector* parsing)
// See: https://github.com/dartsim/dart/issues/2425 (getValue* null pointer)

#include <dart/utils/XmlHelpers.hpp>

#include <gtest/gtest.h>
#include <tinyxml2.h>

#include <stdexcept>
#include <string>

#include <cmath>

using namespace dart::utils;

//==============================================================================
// Tests for valid input (round-trip parsing)
//==============================================================================

TEST(XmlHelpers, ValidVector2dParsing)
{
  const std::string input = "1.5 2.5";
  const Eigen::Vector2d result = toVector2d(input);
  EXPECT_DOUBLE_EQ(result[0], 1.5);
  EXPECT_DOUBLE_EQ(result[1], 2.5);
}

TEST(XmlHelpers, ValidVector3dParsing)
{
  const std::string input = "1.0 2.0 3.0";
  const Eigen::Vector3d result = toVector3d(input);
  EXPECT_DOUBLE_EQ(result[0], 1.0);
  EXPECT_DOUBLE_EQ(result[1], 2.0);
  EXPECT_DOUBLE_EQ(result[2], 3.0);
}

TEST(XmlHelpers, ValidVector6dParsing)
{
  const std::string input = "1.0 2.0 3.0 4.0 5.0 6.0";
  const Eigen::Vector6d result = toVector6d(input);
  EXPECT_DOUBLE_EQ(result[0], 1.0);
  EXPECT_DOUBLE_EQ(result[1], 2.0);
  EXPECT_DOUBLE_EQ(result[2], 3.0);
  EXPECT_DOUBLE_EQ(result[3], 4.0);
  EXPECT_DOUBLE_EQ(result[4], 5.0);
  EXPECT_DOUBLE_EQ(result[5], 6.0);
}

//==============================================================================
// Tests for invalid input - Issue #2423
// These tests verify that exceptions are properly thrown when parsing fails
//==============================================================================

TEST(XmlHelpers, InvalidVector2dThrowsException)
{
  // Test with invalid second component
  const std::string input = "1.0 invalid";
  EXPECT_THROW(toVector2d(input), std::exception);
}

TEST(XmlHelpers, InvalidVector2dFirstComponentThrowsException)
{
  // Test with invalid first component
  const std::string input = "not_a_number 2.0";
  EXPECT_THROW(toVector2d(input), std::exception);
}

TEST(XmlHelpers, InvalidVector2iThrowsException)
{
  const std::string input = "1 invalid";
  EXPECT_THROW(toVector2i(input), std::exception);
}

TEST(XmlHelpers, InvalidVector3dThrowsException)
{
  const std::string input = "1.0 2.0 invalid";
  EXPECT_THROW(toVector3d(input), std::exception);
}

TEST(XmlHelpers, InvalidVector3iThrowsException)
{
  const std::string input = "1 2 invalid";
  EXPECT_THROW(toVector3i(input), std::exception);
}

TEST(XmlHelpers, InvalidVector4dThrowsException)
{
  const std::string input = "1.0 2.0 3.0 invalid";
  EXPECT_THROW(toVector4d(input), std::exception);
}

TEST(XmlHelpers, InvalidVector6dThrowsException)
{
  const std::string input = "1.0 2.0 3.0 4.0 5.0 invalid";
  EXPECT_THROW(toVector6d(input), std::exception);
}

TEST(XmlHelpers, InvalidVectorXdThrowsException)
{
  const std::string input = "1.0 2.0 invalid 4.0";
  EXPECT_THROW(toVectorXd(input), std::exception);
}

TEST(XmlHelpers, InvalidIsometry3dThrowsException)
{
  // Isometry3d expects 6 values: x, y, z, roll, pitch, yaw
  const std::string input = "1.0 2.0 3.0 0.0 0.0 invalid";
  EXPECT_THROW(toIsometry3d(input), std::exception);
}

TEST(XmlHelpers, InvalidIsometry3dWithExtrinsicRotationThrowsException)
{
  const std::string input = "1.0 2.0 3.0 invalid 0.0 0.0";
  EXPECT_THROW(toIsometry3dWithExtrinsicRotation(input), std::exception);
}

//==============================================================================
// Test that valid Isometry3d parsing still works
//==============================================================================

TEST(XmlHelpers, ValidIsometry3dParsing)
{
  const std::string input = "1.0 2.0 3.0 0.0 0.0 0.0";
  const Eigen::Isometry3d result = toIsometry3d(input);
  EXPECT_DOUBLE_EQ(result.translation()[0], 1.0);
  EXPECT_DOUBLE_EQ(result.translation()[1], 2.0);
  EXPECT_DOUBLE_EQ(result.translation()[2], 3.0);
}

//==============================================================================
// Test scalar parsing functions
//==============================================================================

TEST(XmlHelpers, ValidDoubleParsing)
{
  EXPECT_DOUBLE_EQ(toDouble("3.14159"), 3.14159);
  EXPECT_DOUBLE_EQ(toDouble("-2.5"), -2.5);
  EXPECT_DOUBLE_EQ(toDouble("0"), 0.0);
}

TEST(XmlHelpers, InvalidDoubleThrowsException)
{
  EXPECT_THROW(toDouble("not_a_number"), std::exception);
}

TEST(XmlHelpers, ValidIntParsing)
{
  EXPECT_EQ(toInt("42"), 42);
  EXPECT_EQ(toInt("-10"), -10);
  EXPECT_EQ(toInt("0"), 0);
}

TEST(XmlHelpers, InvalidIntThrowsException)
{
  EXPECT_THROW(toInt("not_a_number"), std::exception);
}

TEST(XmlHelpers, ValidBoolParsing)
{
  EXPECT_TRUE(toBool("true"));
  EXPECT_TRUE(toBool("TRUE"));
  EXPECT_TRUE(toBool("1"));
  EXPECT_FALSE(toBool("false"));
  EXPECT_FALSE(toBool("FALSE"));
  EXPECT_FALSE(toBool("0"));
}

//==============================================================================
// Tests for getValue* functions with missing child element - Issue #2425
// These tests verify that proper exceptions are thrown instead of SIGSEGV
//==============================================================================

TEST(XmlHelpers, GetValueBoolWithExistingChild)
{
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLElement* root = doc.NewElement("root");
  doc.InsertEndChild(root);

  tinyxml2::XMLElement* child = doc.NewElement("flag");
  child->SetText("true");
  root->InsertEndChild(child);

  EXPECT_TRUE(getValueBool(root, "flag"));
}

TEST(XmlHelpers, GetValueBoolWithMissingChildThrows)
{
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLElement* root = doc.NewElement("root");
  doc.InsertEndChild(root);

  tinyxml2::XMLElement* child = doc.NewElement("existing_child");
  child->SetText("true");
  root->InsertEndChild(child);

  EXPECT_THROW(getValueBool(root, "missing_child"), std::runtime_error);
}

TEST(XmlHelpers, GetValueStringWithMissingChildThrows)
{
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLElement* root = doc.NewElement("root");
  doc.InsertEndChild(root);

  EXPECT_THROW(getValueString(root, "missing"), std::runtime_error);
}

TEST(XmlHelpers, GetValueIntWithMissingChildThrows)
{
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLElement* root = doc.NewElement("root");
  doc.InsertEndChild(root);

  EXPECT_THROW(getValueInt(root, "missing"), std::runtime_error);
}

TEST(XmlHelpers, GetValueDoubleWithMissingChildThrows)
{
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLElement* root = doc.NewElement("root");
  doc.InsertEndChild(root);

  EXPECT_THROW(getValueDouble(root, "missing"), std::runtime_error);
}

TEST(XmlHelpers, GetValueVector3dWithMissingChildThrows)
{
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLElement* root = doc.NewElement("root");
  doc.InsertEndChild(root);

  EXPECT_THROW(getValueVector3d(root, "missing"), std::runtime_error);
}

TEST(XmlHelpers, GetValueWithEmptyElementThrows)
{
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLElement* root = doc.NewElement("root");
  doc.InsertEndChild(root);

  tinyxml2::XMLElement* child = doc.NewElement("empty");
  root->InsertEndChild(child);

  EXPECT_THROW(getValueString(root, "empty"), std::runtime_error);
}

TEST(XmlHelpers, GetValueIntWithValidChild)
{
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLElement* root = doc.NewElement("root");
  doc.InsertEndChild(root);

  tinyxml2::XMLElement* child = doc.NewElement("count");
  child->SetText("42");
  root->InsertEndChild(child);

  EXPECT_EQ(getValueInt(root, "count"), 42);
}

TEST(XmlHelpers, GetValueDoubleWithValidChild)
{
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLElement* root = doc.NewElement("root");
  doc.InsertEndChild(root);

  tinyxml2::XMLElement* child = doc.NewElement("value");
  child->SetText("3.14159");
  root->InsertEndChild(child);

  EXPECT_DOUBLE_EQ(getValueDouble(root, "value"), 3.14159);
}

TEST(XmlHelpers, GetValueVector3dWithValidChild)
{
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLElement* root = doc.NewElement("root");
  doc.InsertEndChild(root);

  tinyxml2::XMLElement* child = doc.NewElement("pos");
  child->SetText("1.0 2.0 3.0");
  root->InsertEndChild(child);

  const Eigen::Vector3d result = getValueVector3d(root, "pos");
  EXPECT_DOUBLE_EQ(result[0], 1.0);
  EXPECT_DOUBLE_EQ(result[1], 2.0);
  EXPECT_DOUBLE_EQ(result[2], 3.0);
}

TEST(XmlHelpers, HasElementReturnsTrueForExisting)
{
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLElement* root = doc.NewElement("root");
  doc.InsertEndChild(root);

  tinyxml2::XMLElement* child = doc.NewElement("child");
  root->InsertEndChild(child);

  EXPECT_TRUE(hasElement(root, "child"));
}

TEST(XmlHelpers, HasElementReturnsFalseForMissing)
{
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLElement* root = doc.NewElement("root");
  doc.InsertEndChild(root);

  EXPECT_FALSE(hasElement(root, "missing"));
}

TEST(XmlHelpers, GetValueUIntWithMissingChildThrows)
{
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLElement* root = doc.NewElement("root");
  doc.InsertEndChild(root);

  EXPECT_THROW(getValueUInt(root, "missing"), std::runtime_error);
}

TEST(XmlHelpers, GetValueFloatWithMissingChildThrows)
{
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLElement* root = doc.NewElement("root");
  doc.InsertEndChild(root);

  EXPECT_THROW(getValueFloat(root, "missing"), std::runtime_error);
}

TEST(XmlHelpers, GetValueCharWithMissingChildThrows)
{
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLElement* root = doc.NewElement("root");
  doc.InsertEndChild(root);

  EXPECT_THROW(getValueChar(root, "missing"), std::runtime_error);
}

TEST(XmlHelpers, GetValueVector2dWithMissingChildThrows)
{
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLElement* root = doc.NewElement("root");
  doc.InsertEndChild(root);

  EXPECT_THROW(getValueVector2d(root, "missing"), std::runtime_error);
}

TEST(XmlHelpers, GetValueVector6dWithMissingChildThrows)
{
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLElement* root = doc.NewElement("root");
  doc.InsertEndChild(root);

  EXPECT_THROW(getValueVector6d(root, "missing"), std::runtime_error);
}

TEST(XmlHelpers, GetValueVectorXdWithMissingChildThrows)
{
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLElement* root = doc.NewElement("root");
  doc.InsertEndChild(root);

  EXPECT_THROW(getValueVectorXd(root, "missing"), std::runtime_error);
}

TEST(XmlHelpers, GetValueIsometry3dWithMissingChildThrows)
{
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLElement* root = doc.NewElement("root");
  doc.InsertEndChild(root);

  EXPECT_THROW(getValueIsometry3d(root, "missing"), std::runtime_error);
}

//==============================================================================
// Additional valid-case tests for comprehensive coverage
//==============================================================================

TEST(XmlHelpers, GetValueStringWithValidChild)
{
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLElement* root = doc.NewElement("root");
  doc.InsertEndChild(root);

  tinyxml2::XMLElement* child = doc.NewElement("name");
  child->SetText("hello world");
  root->InsertEndChild(child);

  EXPECT_EQ(getValueString(root, "name"), "hello world");
}

TEST(XmlHelpers, GetValueUIntWithValidChild)
{
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLElement* root = doc.NewElement("root");
  doc.InsertEndChild(root);

  tinyxml2::XMLElement* child = doc.NewElement("count");
  child->SetText("42");
  root->InsertEndChild(child);

  EXPECT_EQ(getValueUInt(root, "count"), 42u);
}

TEST(XmlHelpers, GetValueFloatWithValidChild)
{
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLElement* root = doc.NewElement("root");
  doc.InsertEndChild(root);

  tinyxml2::XMLElement* child = doc.NewElement("value");
  child->SetText("3.14");
  root->InsertEndChild(child);

  EXPECT_FLOAT_EQ(getValueFloat(root, "value"), 3.14f);
}

TEST(XmlHelpers, GetValueCharWithValidChild)
{
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLElement* root = doc.NewElement("root");
  doc.InsertEndChild(root);

  tinyxml2::XMLElement* child = doc.NewElement("letter");
  child->SetText("A");
  root->InsertEndChild(child);

  EXPECT_EQ(getValueChar(root, "letter"), 'A');
}

TEST(XmlHelpers, GetValueVector2dWithValidChild)
{
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLElement* root = doc.NewElement("root");
  doc.InsertEndChild(root);

  tinyxml2::XMLElement* child = doc.NewElement("pos");
  child->SetText("1.5 2.5");
  root->InsertEndChild(child);

  const Eigen::Vector2d result = getValueVector2d(root, "pos");
  EXPECT_DOUBLE_EQ(result[0], 1.5);
  EXPECT_DOUBLE_EQ(result[1], 2.5);
}

TEST(XmlHelpers, GetValueVector3iWithMissingChildThrows)
{
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLElement* root = doc.NewElement("root");
  doc.InsertEndChild(root);

  EXPECT_THROW(getValueVector3i(root, "missing"), std::runtime_error);
}

TEST(XmlHelpers, GetValueVector3iWithValidChild)
{
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLElement* root = doc.NewElement("root");
  doc.InsertEndChild(root);

  tinyxml2::XMLElement* child = doc.NewElement("pos");
  child->SetText("1 2 3");
  root->InsertEndChild(child);

  const Eigen::Vector3i result = getValueVector3i(root, "pos");
  EXPECT_EQ(result[0], 1);
  EXPECT_EQ(result[1], 2);
  EXPECT_EQ(result[2], 3);
}

TEST(XmlHelpers, GetValueVector6dWithValidChild)
{
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLElement* root = doc.NewElement("root");
  doc.InsertEndChild(root);

  tinyxml2::XMLElement* child = doc.NewElement("vec");
  child->SetText("1.0 2.0 3.0 4.0 5.0 6.0");
  root->InsertEndChild(child);

  const Eigen::Vector6d result = getValueVector6d(root, "vec");
  EXPECT_DOUBLE_EQ(result[0], 1.0);
  EXPECT_DOUBLE_EQ(result[1], 2.0);
  EXPECT_DOUBLE_EQ(result[2], 3.0);
  EXPECT_DOUBLE_EQ(result[3], 4.0);
  EXPECT_DOUBLE_EQ(result[4], 5.0);
  EXPECT_DOUBLE_EQ(result[5], 6.0);
}

TEST(XmlHelpers, GetValueVectorXdWithValidChild)
{
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLElement* root = doc.NewElement("root");
  doc.InsertEndChild(root);

  tinyxml2::XMLElement* child = doc.NewElement("vec");
  child->SetText("1.0 2.0 3.0 4.0");
  root->InsertEndChild(child);

  const Eigen::VectorXd result = getValueVectorXd(root, "vec");
  ASSERT_EQ(result.size(), 4);
  EXPECT_DOUBLE_EQ(result[0], 1.0);
  EXPECT_DOUBLE_EQ(result[1], 2.0);
  EXPECT_DOUBLE_EQ(result[2], 3.0);
  EXPECT_DOUBLE_EQ(result[3], 4.0);
}

TEST(XmlHelpers, GetValueIsometry3dWithValidChild)
{
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLElement* root = doc.NewElement("root");
  doc.InsertEndChild(root);

  tinyxml2::XMLElement* child = doc.NewElement("transform");
  child->SetText("1.0 2.0 3.0 0.0 0.0 0.0");
  root->InsertEndChild(child);

  const Eigen::Isometry3d result = getValueIsometry3d(root, "transform");
  EXPECT_DOUBLE_EQ(result.translation()[0], 1.0);
  EXPECT_DOUBLE_EQ(result.translation()[1], 2.0);
  EXPECT_DOUBLE_EQ(result.translation()[2], 3.0);
}

TEST(XmlHelpers, GetValueIsometry3dWithExtrinsicRotationMissingChildThrows)
{
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLElement* root = doc.NewElement("root");
  doc.InsertEndChild(root);

  EXPECT_THROW(
      getValueIsometry3dWithExtrinsicRotation(root, "missing"),
      std::runtime_error);
}

TEST(XmlHelpers, GetValueIsometry3dWithExtrinsicRotationValidChild)
{
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLElement* root = doc.NewElement("root");
  doc.InsertEndChild(root);

  tinyxml2::XMLElement* child = doc.NewElement("transform");
  child->SetText("1.0 2.0 3.0 0.0 0.0 0.0");
  root->InsertEndChild(child);

  const Eigen::Isometry3d result
      = getValueIsometry3dWithExtrinsicRotation(root, "transform");
  EXPECT_DOUBLE_EQ(result.translation()[0], 1.0);
  EXPECT_DOUBLE_EQ(result.translation()[1], 2.0);
  EXPECT_DOUBLE_EQ(result.translation()[2], 3.0);
}

TEST(XmlHelpers, GetValueBoolFalseValue)
{
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLElement* root = doc.NewElement("root");
  doc.InsertEndChild(root);

  tinyxml2::XMLElement* child = doc.NewElement("flag");
  child->SetText("false");
  root->InsertEndChild(child);

  EXPECT_FALSE(getValueBool(root, "flag"));
}

TEST(XmlHelpers, GetValueBoolNumericValues)
{
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLElement* root = doc.NewElement("root");
  doc.InsertEndChild(root);

  tinyxml2::XMLElement* trueChild = doc.NewElement("trueFlag");
  trueChild->SetText("1");
  root->InsertEndChild(trueChild);

  tinyxml2::XMLElement* falseChild = doc.NewElement("falseFlag");
  falseChild->SetText("0");
  root->InsertEndChild(falseChild);

  EXPECT_TRUE(getValueBool(root, "trueFlag"));
  EXPECT_FALSE(getValueBool(root, "falseFlag"));
}
