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

#include "dart/io/xml_helpers.hpp"

#include "dart/common/local_resource_retriever.hpp"
#include "dart/common/logging.hpp"
#include "dart/common/macros.hpp"
#include "dart/math/geometry.hpp"

#include <fmt/format.h>

#include <iostream>
#include <ranges>
#include <stdexcept>
#include <vector>

namespace dart {
namespace io {

namespace {

bool warnIfNullAttributeElement(
    const tinyxml2::XMLElement* element,
    std::string_view attributeName,
    std::string_view typeName,
    std::string_view fallback)
{
  if (element != nullptr) {
    return false;
  }

  DART_WARN(
      "[getAttribute] Error in parsing {} type attribute [{}] of a null "
      "element. Returning {}.",
      typeName,
      attributeName,
      fallback);
  return true;
}

} // namespace

//==============================================================================
std::string toString(bool v)
{
  return fmt::format("{}", v);
}

//==============================================================================
std::string toString(int v)
{
  return std::to_string(v);
}

//==============================================================================
std::string toString(unsigned int v)
{
  return std::to_string(v);
}

//==============================================================================
std::string toString(float v)
{
  return std::to_string(v);
}

//==============================================================================
std::string toString(double v)
{
  return std::to_string(v);
}

//==============================================================================
std::string toString(char v)
{
  return fmt::format("{}", v);
}

//==============================================================================
bool toBool(std::string_view str)
{
  const std::string upper = common::toUpper(std::string(str));
  if (upper == "TRUE" || str == "1") {
    return true;
  } else if (upper == "FALSE" || str == "0") {
    return false;
  } else {
    DART_ERROR("value [{}] is not a valid boolean type. Retuning false.", str);
    return false;
  }
}

//==============================================================================
int toInt(std::string_view str)
{
  return std::stoi(std::string(str));
}

//==============================================================================
unsigned int toUInt(std::string_view str)
{
  return static_cast<unsigned int>(std::stoul(std::string(str)));
}

//==============================================================================
float toFloat(std::string_view str)
{
  return std::stof(std::string(str));
}

//==============================================================================
double toDouble(std::string_view str)
{
  return std::stod(std::string(str));
}

//==============================================================================
char toChar(std::string_view str)
{
  if (str.empty()) {
    DART_ERROR("");
    return 0;
  }

  DART_ERROR_IF(str.size() != 1, "");

  return str[0];
}

namespace {

template <typename Vector>
void assignDoublePieces(
    Vector& vector,
    const std::vector<std::string>& pieces,
    std::string_view scalarType,
    std::string_view vectorType)
{
  for (const auto i : std::views::iota(std::size_t{0}, pieces.size())) {
    if (pieces[i] != "") {
      try {
        vector[static_cast<Eigen::Index>(i)] = toDouble(pieces[i]);
      } catch (std::exception& e) {
        DART_ERROR(
            "value [{}] is not a valid {} for {}[{}]: {}",
            pieces[i],
            scalarType,
            vectorType,
            i,
            e.what());
        throw;
      }
    }
  }
}

template <typename Vector>
Vector toFixedVector(
    std::string_view str,
    std::size_t expectedSize,
    std::string_view scalarType,
    std::string_view vectorType)
{
  Vector ret;

  const std::vector<std::string> pieces = common::split(common::trim(str));
  DART_ASSERT(pieces.size() == expectedSize);
  (void)expectedSize;

  assignDoublePieces(ret, pieces, scalarType, vectorType);
  return ret;
}

} // namespace

//==============================================================================
Eigen::Vector2d toVector2d(std::string_view str)
{
  return toFixedVector<Eigen::Vector2d>(str, 2, "double", "Eigen::Vector2d");
}

//==============================================================================
Eigen::Vector2i toVector2i(std::string_view str)
{
  return toFixedVector<Eigen::Vector2i>(str, 2, "double", "Eigen::Vector2i");
}

//==============================================================================
Eigen::Vector3d toVector3d(std::string_view str)
{
  return toFixedVector<Eigen::Vector3d>(str, 3, "double", "Eigen::Vector3d");
}

//==============================================================================
Eigen::Vector3i toVector3i(std::string_view str)
{
  return toFixedVector<Eigen::Vector3i>(str, 3, "int", "Eigen::Vector3i");
}

//==============================================================================
Eigen::Vector4d toVector4d(std::string_view str)
{
  return toFixedVector<Eigen::Vector4d>(str, 4, "double", "Eigen::Vector4d");
}

//==============================================================================
Eigen::Vector6d toVector6d(std::string_view str)
{
  return toFixedVector<Eigen::Vector6d>(str, 6, "double", "Eigen::Vector6d");
}

//==============================================================================
Eigen::VectorXd toVectorXd(std::string_view str)
{
  const std::vector<std::string> pieces = common::split(common::trim(str));
  DART_ASSERT(!pieces.empty());

  Eigen::VectorXd ret(pieces.size());
  assignDoublePieces(ret, pieces, "double", "Eigen::VectorXd");
  return ret;
}

//==============================================================================
Eigen::Isometry3d toIsometry3d(std::string_view str)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  Eigen::Vector6d elements = Eigen::Vector6d::Zero();
  const std::vector<std::string> pieces = common::split(common::trim(str));
  DART_ASSERT(pieces.size() == 6);
  assignDoublePieces(elements, pieces, "double", "SE3");

  T.linear() = math::eulerXYZToMatrix(elements.tail<3>());
  T.translation() = elements.head<3>();
  return T;
}

//==============================================================================
Eigen::Isometry3d toIsometry3dWithExtrinsicRotation(std::string_view str)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  Eigen::Vector6d elements = Eigen::Vector6d::Zero();
  const std::vector<std::string> pieces = common::split(common::trim(str));
  DART_ASSERT(pieces.size() == 6);
  assignDoublePieces(elements, pieces, "double", "SE3");

  Eigen::Vector3d reverseEulerAngles(
      elements.tail<3>()[2], elements.tail<3>()[1], elements.tail<3>()[0]);

  T.linear() = math::eulerZYXToMatrix(reverseEulerAngles);
  T.translation() = elements.head<3>();
  return T;
}

//==============================================================================
// Helper function to safely get text from a child element
// Throws std::runtime_error if the child element is missing or has no text
static std::string getChildElementText(
    const tinyxml2::XMLElement* parentElement,
    const std::string& childName,
    std::string_view parentDescription = "")
{
  const tinyxml2::XMLElement* childElement
      = parentElement->FirstChildElement(childName.c_str());
  if (childElement == nullptr) {
    const std::string parentName
        = parentDescription.empty()
              ? (parentElement->Name() ? parentElement->Name() : "unknown")
              : std::string(parentDescription);
    const auto msg = fmt::format(
        "Child element [{}] not found in parent element [{}]",
        childName,
        parentName);
    DART_ERROR("{}", msg);
    throw std::runtime_error(msg);
  }

  const char* text = childElement->GetText();
  if (text == nullptr) {
    const std::string parentName
        = parentDescription.empty()
              ? (parentElement->Name() ? parentElement->Name() : "unknown")
              : std::string(parentDescription);
    const auto msg = fmt::format(
        "Child element [{}] in parent [{}] has no text content",
        childName,
        parentName);
    DART_ERROR("{}", msg);
    throw std::runtime_error(msg);
  }

  return std::string(text);
}

//==============================================================================
std::string getValueString(
    const tinyxml2::XMLElement* parentElement, std::string_view name)
{
  DART_ASSERT(parentElement != nullptr);
  DART_ASSERT(!name.empty());

  return getChildElementText(parentElement, std::string(name));
}

//==============================================================================
bool getValueBool(
    const tinyxml2::XMLElement* parentElement, std::string_view name)
{
  DART_ASSERT(parentElement != nullptr);
  DART_ASSERT(!name.empty());

  const std::string str = getChildElementText(parentElement, std::string(name));

  if (common::toUpper(str) == "TRUE" || str == "1") {
    return true;
  } else if (common::toUpper(str) == "FALSE" || str == "0") {
    return false;
  } else {
    DART_ERROR("value [{}] is not a valid boolean type. Returning false.", str);
    DART_ASSERT(0);
    return false;
  }
}

//==============================================================================
int getValueInt(
    const tinyxml2::XMLElement* parentElement, std::string_view name)
{
  DART_ASSERT(parentElement != nullptr);
  DART_ASSERT(!name.empty());

  return toInt(getChildElementText(parentElement, std::string(name)));
}

//==============================================================================
unsigned int getValueUInt(
    const tinyxml2::XMLElement* parentElement, std::string_view name)
{
  DART_ASSERT(parentElement != nullptr);
  DART_ASSERT(!name.empty());

  return toUInt(getChildElementText(parentElement, std::string(name)));
}

//==============================================================================
float getValueFloat(
    const tinyxml2::XMLElement* parentElement, std::string_view name)
{
  DART_ASSERT(parentElement != nullptr);
  DART_ASSERT(!name.empty());

  return toFloat(getChildElementText(parentElement, std::string(name)));
}

//==============================================================================
double getValueDouble(
    const tinyxml2::XMLElement* parentElement, std::string_view name)
{
  DART_ASSERT(parentElement != nullptr);
  DART_ASSERT(!name.empty());

  return toDouble(getChildElementText(parentElement, std::string(name)));
}

//==============================================================================
char getValueChar(
    const tinyxml2::XMLElement* parentElement, std::string_view name)
{
  DART_ASSERT(parentElement != nullptr);
  DART_ASSERT(!name.empty());

  return toChar(getChildElementText(parentElement, std::string(name)));
}

//==============================================================================
Eigen::Vector2d getValueVector2d(
    const tinyxml2::XMLElement* parentElement, std::string_view name)
{
  DART_ASSERT(parentElement != nullptr);
  DART_ASSERT(!name.empty());

  return toVector2d(getChildElementText(parentElement, std::string(name)));
}

//==============================================================================
Eigen::Vector3d getValueVector3d(
    const tinyxml2::XMLElement* parentElement, std::string_view name)
{
  DART_ASSERT(parentElement != nullptr);
  DART_ASSERT(!name.empty());

  return toVector3d(getChildElementText(parentElement, std::string(name)));
}

//==============================================================================
Eigen::Vector3i getValueVector3i(
    const tinyxml2::XMLElement* parentElement, std::string_view name)
{
  DART_ASSERT(parentElement != nullptr);
  DART_ASSERT(!name.empty());

  return toVector3i(getChildElementText(parentElement, std::string(name)));
}

//==============================================================================
Eigen::Vector6d getValueVector6d(
    const tinyxml2::XMLElement* parentElement, std::string_view name)
{
  DART_ASSERT(parentElement != nullptr);
  DART_ASSERT(!name.empty());

  return toVector6d(getChildElementText(parentElement, std::string(name)));
}

//==============================================================================
Eigen::VectorXd getValueVectorXd(
    const tinyxml2::XMLElement* parentElement, std::string_view name)
{
  DART_ASSERT(parentElement != nullptr);
  DART_ASSERT(!name.empty());

  return toVectorXd(getChildElementText(parentElement, std::string(name)));
}

//==============================================================================
Eigen::Vector3d getValueVec3(
    const tinyxml2::XMLElement* parentElement, std::string_view name)
{
  DART_ASSERT(parentElement != nullptr);
  DART_ASSERT(!name.empty());

  return toVector3d(getChildElementText(parentElement, std::string(name)));
}

//==============================================================================
Eigen::Isometry3d getValueIsometry3d(
    const tinyxml2::XMLElement* parentElement, std::string_view name)
{
  DART_ASSERT(parentElement != nullptr);
  DART_ASSERT(!name.empty());

  return toIsometry3d(getChildElementText(parentElement, std::string(name)));
}

//==============================================================================
Eigen::Isometry3d getValueIsometry3dWithExtrinsicRotation(
    const tinyxml2::XMLElement* parentElement, std::string_view name)
{
  DART_ASSERT(parentElement != nullptr);
  DART_ASSERT(!name.empty());

  return toIsometry3dWithExtrinsicRotation(
      getChildElementText(parentElement, std::string(name)));
}

//==============================================================================
bool hasElement(
    const tinyxml2::XMLElement* parentElement, std::string_view name)
{
  DART_ASSERT(parentElement != nullptr);
  DART_ASSERT(!name.empty());

  const std::string nameString(name);
  return parentElement->FirstChildElement(nameString.c_str()) == nullptr ? false
                                                                         : true;
}

//==============================================================================
const tinyxml2::XMLElement* getElement(
    const tinyxml2::XMLElement* parentElement, std::string_view name)
{
  DART_ASSERT(!name.empty());

  const std::string nameString(name);
  return parentElement->FirstChildElement(nameString.c_str());
}

//==============================================================================
tinyxml2::XMLElement* getElement(
    tinyxml2::XMLElement* parentElement, std::string_view name)
{
  DART_ASSERT(!name.empty());

  const std::string nameString(name);
  return parentElement->FirstChildElement(nameString.c_str());
}

//==============================================================================
std::string toString(tinyxml2::XMLError errorCode)
{
  switch (errorCode) {
    using enum tinyxml2::XMLError;
    case XML_SUCCESS:
      return "XML_SUCCESS";
    case XML_NO_ATTRIBUTE:
      return "XML_NO_ATTRIBUTE";
    case XML_WRONG_ATTRIBUTE_TYPE:
      return "XML_WRONG_ATTRIBUTE_TYPE";
    case XML_ERROR_FILE_NOT_FOUND:
      return "XML_ERROR_FILE_NOT_FOUND";
    case XML_ERROR_FILE_COULD_NOT_BE_OPENED:
      return "XML_ERROR_FILE_COULD_NOT_BE_OPENED";
    case XML_ERROR_FILE_READ_ERROR:
      return "XML_ERROR_FILE_READ_ERROR";
    case XML_ERROR_PARSING_ELEMENT:
      return "XML_ERROR_PARSING_ELEMENT";
    case XML_ERROR_PARSING_ATTRIBUTE:
      return "XML_ERROR_PARSING_ATTRIBUTE";
    case XML_ERROR_PARSING_TEXT:
      return "XML_ERROR_PARSING_TEXT";
    case XML_ERROR_PARSING_CDATA:
      return "XML_ERROR_PARSING_CDATA";
    case XML_ERROR_PARSING_COMMENT:
      return "XML_ERROR_PARSING_COMMENT";
    case XML_ERROR_PARSING_DECLARATION:
      return "XML_ERROR_PARSING_DECLARATION";
    case XML_ERROR_PARSING_UNKNOWN:
      return "XML_ERROR_PARSING_UNKNOWN";
    case XML_ERROR_EMPTY_DOCUMENT:
      return "XML_ERROR_EMPTY_DOCUMENT";
    case XML_ERROR_MISMATCHED_ELEMENT:
      return "XML_ERROR_MISMATCHED_ELEMENT";
    case XML_ERROR_PARSING:
      return "XML_ERROR_PARSING";
    case XML_CAN_NOT_CONVERT_TEXT:
      return "XML_CAN_NOT_CONVERT_TEXT";
    case XML_NO_TEXT_NODE:
      return "XML_NO_TEXT_NODE";
    case XML_ERROR_COUNT:
      return "XML_ERROR_COUNT";
    default:
      return "Unknown error";
  }
}

//==============================================================================
void openXMLFile(
    tinyxml2::XMLDocument& doc,
    const common::Uri& uri,
    const common::ResourceRetrieverPtr& retrieverOrNullPtr)
{
  common::ResourceRetrieverPtr retriever;
  if (retrieverOrNullPtr) {
    retriever = retrieverOrNullPtr;
  } else {
    retriever = std::make_shared<common::LocalResourceRetriever>();
  }

  const auto content = retriever->readAll(uri);
  const auto result = doc.Parse(&content.front());
  if (result != tinyxml2::XML_SUCCESS) {
    DART_WARN(
        "[openXMLFile] Failed parsing XML: TinyXML2 returned error '{}'.",
        toString(result));
    throw std::runtime_error("Failed parsing XML.");
  }
}

//==============================================================================
bool readXmlFile(
    tinyxml2::XMLDocument& doc,
    const common::Uri& uri,
    const common::ResourceRetrieverPtr& retrieverOrNullPtr)
{
  common::ResourceRetrieverPtr retriever;
  if (retrieverOrNullPtr) {
    retriever = retrieverOrNullPtr;
  } else {
    retriever = std::make_shared<common::LocalResourceRetriever>();
  }

  const auto content = retriever->readAll(uri);
  const auto result = doc.Parse(&content.front());
  if (result != tinyxml2::XML_SUCCESS) {
    DART_WARN(
        "[readXmlFile] Failed parsing XML: TinyXML2 returned error '{}'.",
        toString(result));
    return false;
  }

  return true;
}

//==============================================================================
bool hasAttribute(const tinyxml2::XMLElement* element, const char* const name)
{
  if (element == nullptr || name == nullptr) {
    return false;
  }

  const char* const result = element->Attribute(name);
  return result != nullptr;
}

//==============================================================================
std::string getAttributeString(
    const tinyxml2::XMLElement* element, std::string_view attributeName)
{
  if (warnIfNullAttributeElement(
          element, attributeName, "string", "empty string")) {
    return std::string();
  }

  const std::string attributeNameString(attributeName);
  const char* const result = element->Attribute(attributeNameString.c_str());

  if (nullptr == result) {
    DART_WARN(
        "[getAttribute] Error in parsing string type attribute [{}] of an "
        "element [{}]. Returning empty string.",
        attributeNameString,
        element->Name());
    return std::string();
  }

  return std::string(result);
}

//==============================================================================
bool getAttributeBool(
    const tinyxml2::XMLElement* element, std::string_view attributeName)
{
  if (warnIfNullAttributeElement(element, attributeName, "bool", "false")) {
    return false;
  }

  bool val = false;
  const std::string attributeNameString(attributeName);
  const int result
      = element->QueryBoolAttribute(attributeNameString.c_str(), &val);

  if (result != tinyxml2::XML_SUCCESS) {
    DART_WARN(
        "[getAttribute] Error in parsing bool type attribute [{}] of an "
        "element [{}]. Returning false instead.",
        attributeNameString,
        element->Name());
    return false;
  }

  return val;
}

//==============================================================================
int getAttributeInt(
    const tinyxml2::XMLElement* element, std::string_view attributeName)
{
  if (warnIfNullAttributeElement(element, attributeName, "int", "zero")) {
    return 0;
  }

  int val = 0;
  const std::string attributeNameString(attributeName);
  const int result
      = element->QueryIntAttribute(attributeNameString.c_str(), &val);

  if (result != tinyxml2::XML_SUCCESS) {
    DART_WARN(
        "[getAttribute] Error in parsing int type attribute [{}] of an element "
        "[{}]. Returning zero instead.",
        attributeNameString,
        element->Name());
    return 0;
  }

  return val;
}

//==============================================================================
unsigned int getAttributeUInt(
    const tinyxml2::XMLElement* element, std::string_view attributeName)
{
  if (warnIfNullAttributeElement(
          element, attributeName, "unsigned int", "zero")) {
    return 0u;
  }

  unsigned int val = 0u;
  const std::string attributeNameString(attributeName);
  const int result
      = element->QueryUnsignedAttribute(attributeNameString.c_str(), &val);

  if (result != tinyxml2::XML_SUCCESS) {
    DART_WARN(
        "[getAttribute] Error in parsing unsigned int type attribute [{}] of "
        "an "
        "element [{}]. Returning zero instead.",
        attributeNameString,
        element->Name());
    return 0u;
  }

  return val;
}

//==============================================================================
float getAttributeFloat(
    const tinyxml2::XMLElement* element, std::string_view attributeName)
{
  if (warnIfNullAttributeElement(element, attributeName, "float", "zero")) {
    return 0.0f;
  }

  float val = 0.0f;
  const std::string attributeNameString(attributeName);
  const int result
      = element->QueryFloatAttribute(attributeNameString.c_str(), &val);

  if (result != tinyxml2::XML_SUCCESS) {
    DART_WARN(
        "[getAttribute] Error in parsing float type attribute [{}] of an "
        "element [{}]. Returning zero instead.",
        attributeNameString,
        element->Name());
    return 0.0f;
  }

  return val;
}

//==============================================================================
double getAttributeDouble(
    const tinyxml2::XMLElement* element, std::string_view attributeName)
{
  if (warnIfNullAttributeElement(element, attributeName, "double", "zero")) {
    return 0.0;
  }

  double val = 0.0;
  const std::string attributeNameString(attributeName);
  const int result
      = element->QueryDoubleAttribute(attributeNameString.c_str(), &val);

  if (result != tinyxml2::XML_SUCCESS) {
    DART_WARN(
        "[getAttribute] Error in parsing double type attribute [{}] of an "
        "element [{}]. Returning zero instead.",
        attributeNameString,
        element->Name());
    return 0.0;
  }

  return val;
}

//==============================================================================
char getAttributeChar(
    const tinyxml2::XMLElement* element, std::string_view attributeName)
{
  const std::string val = getAttributeString(element, attributeName);

  return toChar(val);
}

//==============================================================================
Eigen::Vector2i getAttributeVector2i(
    const tinyxml2::XMLElement* element, std::string_view attributeName)
{
  if (warnIfNullAttributeElement(
          element, attributeName, "Eigen::Vector2i", "zero vector")) {
    return Eigen::Vector2i::Zero();
  }

  const std::string val = getAttributeString(element, attributeName);

  return toVector2i(val);
}

//==============================================================================
Eigen::Vector2d getAttributeVector2d(
    const tinyxml2::XMLElement* element, std::string_view attributeName)
{
  if (warnIfNullAttributeElement(
          element, attributeName, "Eigen::Vector2d", "zero vector")) {
    return Eigen::Vector2d::Zero();
  }

  const std::string val = getAttributeString(element, attributeName);

  return toVector2d(val);
}

//==============================================================================
Eigen::Vector3d getAttributeVector3d(
    const tinyxml2::XMLElement* element, std::string_view attributeName)
{
  if (warnIfNullAttributeElement(
          element, attributeName, "Eigen::Vector3d", "zero vector")) {
    return Eigen::Vector3d::Zero();
  }

  const std::string val = getAttributeString(element, attributeName);

  return toVector3d(val);
}

//==============================================================================
Eigen::Vector4d getAttributeVector4d(
    const tinyxml2::XMLElement* element, std::string_view attributeName)
{
  if (warnIfNullAttributeElement(
          element, attributeName, "Eigen::Vector4d", "zero vector")) {
    return Eigen::Vector4d::Zero();
  }

  const std::string val = getAttributeString(element, attributeName);

  return toVector4d(val);
}

//==============================================================================
Eigen::Vector6d getAttributeVector6d(
    const tinyxml2::XMLElement* element, std::string_view attributeName)
{
  if (warnIfNullAttributeElement(
          element, attributeName, "Eigen::Vector6d", "zero vector")) {
    return Eigen::Vector6d::Zero();
  }

  const std::string val = getAttributeString(element, attributeName);

  return toVector6d(val);
}

//==============================================================================
Eigen::VectorXd getAttributeVectorXd(
    const tinyxml2::XMLElement* element, std::string_view attributeName)
{
  if (warnIfNullAttributeElement(
          element, attributeName, "Eigen::VectorXd", "empty vector")) {
    return Eigen::VectorXd();
  }

  const std::string val = getAttributeString(element, attributeName);

  return toVectorXd(val);
}

//==============================================================================
bool copyNode(tinyxml2::XMLNode* destParent, const tinyxml2::XMLNode& src)
{
  // Protect from evil
  if (destParent == nullptr) {
    return false;
  }

  // Get the document context where new memory will be allocated from
  tinyxml2::XMLDocument* doc = destParent->GetDocument();

  // Make the copy
  tinyxml2::XMLNode* copy = src.ShallowClone(doc);
  if (copy == nullptr) {
    return false;
  }

  // Add this child
  destParent->InsertEndChild(copy);

  // Add the grandkids
  for (const tinyxml2::XMLNode* node = src.FirstChild(); node != nullptr;
       node = node->NextSibling()) {
    if (!copyNode(copy, *node)) {
      return false;
    }
  }

  return true;
}

//==============================================================================
bool copyChildNodes(tinyxml2::XMLNode* destParent, const tinyxml2::XMLNode& src)
{
  for (const tinyxml2::XMLNode* node = src.FirstChild(); node != nullptr;
       node = node->NextSibling()) {
    if (!copyNode(destParent, *node)) {
      return false;
    }
  }

  return true;
}

} // namespace io
} // namespace dart
