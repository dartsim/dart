/*
 * Copyright (c) 2011-2023, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include "dart/io/XmlHelpers.hpp"

#include "dart/common/Console.hpp"
#include "dart/common/LocalResourceRetriever.hpp"
#include "dart/math/Geometry.hpp"

#include <fmt/format.h>

#include <iostream>
#include <vector>

namespace dart {
namespace io {

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
bool toBool(const std::string& str)
{
  if (common::toUpper(str) == "TRUE" || str == "1")
    return true;
  else if (common::toUpper(str) == "FALSE" || str == "0")
    return false;
  else {
    dterr << "value [" << str << "] is not a valid boolean type. "
          << "Retuning false." << std::endl;
    return false;
  }
}

//==============================================================================
int toInt(const std::string& str)
{
  return std::stoi(str);
}

//==============================================================================
unsigned int toUInt(const std::string& str)
{
  return static_cast<unsigned int>(std::stoul(str));
}

//==============================================================================
float toFloat(const std::string& str)
{
  return std::stof(str);
}

//==============================================================================
double toDouble(const std::string& str)
{
  return std::stod(str);
}

//==============================================================================
char toChar(const std::string& str)
{
  if (str.empty()) {
    DART_ERROR("");
    return 0;
  }

  if (str.size() != 1) {
    DART_ERROR("");
  }

  return str[0];
}

//==============================================================================
Eigen::Vector2d toVector2d(const std::string& str)
{
  Eigen::Vector2d ret;

  const std::vector<std::string> pieces = common::split(common::trim(str));
  assert(pieces.size() == 2);

  for (std::size_t i = 0; i < pieces.size(); ++i) {
    if (pieces[i] != "") {
      try {
        ret[i] = toDouble(pieces[i]);
      } catch (std::exception& e) {
        std::cerr << "value [" << pieces[i]
                  << "] is not a valid double for Eigen::Vector2d[" << i
                  << "]: " << e.what() << std::endl;
      }
    }
  }

  return ret;
}

//==============================================================================
Eigen::Vector2i toVector2i(const std::string& str)
{
  Eigen::Vector2i ret;

  const std::vector<std::string> pieces = common::split(common::trim(str));
  assert(pieces.size() == 2);

  for (std::size_t i = 0; i < pieces.size(); ++i) {
    if (pieces[i] != "") {
      try {
        ret[i] = toDouble(pieces[i]);
      } catch (std::exception& e) {
        std::cerr << "value [" << pieces[i]
                  << "] is not a valid double for Eigen::Vector2i[" << i
                  << "]: " << e.what() << std::endl;
      }
    }
  }

  return ret;
}

//==============================================================================
Eigen::Vector3d toVector3d(const std::string& str)
{
  Eigen::Vector3d ret;

  const std::vector<std::string> pieces = common::split(common::trim(str));
  assert(pieces.size() == 3);

  for (std::size_t i = 0; i < pieces.size(); ++i) {
    if (pieces[i] != "") {
      try {
        ret[i] = toDouble(pieces[i]);
      } catch (std::exception& e) {
        std::cerr << "value [" << pieces[i]
                  << "] is not a valid double for Eigen::Vector3d[" << i
                  << "]: " << e.what() << std::endl;
      }
    }
  }

  return ret;
}

//==============================================================================
Eigen::Vector3i toVector3i(const std::string& str)
{
  Eigen::Vector3i ret;

  const std::vector<std::string> pieces = common::split(common::trim(str));
  assert(pieces.size() == 3);

  for (std::size_t i = 0; i < pieces.size(); ++i) {
    if (pieces[i] != "") {
      try {
        ret[i] = toDouble(pieces[i]);
      } catch (std::exception& e) {
        std::cerr << "value [" << pieces[i]
                  << "] is not a valid int for Eigen::Vector3i[" << i
                  << "]: " << e.what() << std::endl;
      }
    }
  }

  return ret;
}

//==============================================================================
Eigen::Vector4d toVector4d(const std::string& str)
{
  Eigen::Vector4d ret;

  const std::vector<std::string> pieces = common::split(common::trim(str));
  assert(pieces.size() == 4);

  for (std::size_t i = 0; i < pieces.size(); ++i) {
    if (pieces[i] != "") {
      try {
        ret[i] = toDouble(pieces[i]);
      } catch (std::exception& e) {
        std::cerr << "value [" << pieces[i]
                  << "] is not a valid double for Eigen::Vector4d[" << i
                  << "]: " << e.what() << std::endl;
      }
    }
  }

  return ret;
}

//==============================================================================
Eigen::Vector6d toVector6d(const std::string& str)
{
  Eigen::Vector6d ret;

  const std::vector<std::string> pieces = common::split(common::trim(str));
  assert(pieces.size() == 6);

  for (std::size_t i = 0; i < pieces.size(); ++i) {
    if (pieces[i] != "") {
      try {
        ret[i] = toDouble(pieces[i]);
      } catch (std::exception& e) {
        std::cerr << "value [" << pieces[i]
                  << "] is not a valid double for Eigen::Vector6d[" << i
                  << "]: " << e.what() << std::endl;
      }
    }
  }

  return ret;
}

//==============================================================================
Eigen::VectorXd toVectorXd(const std::string& str)
{
  const std::vector<std::string> pieces = common::split(common::trim(str));
  assert(pieces.size() > 0);

  Eigen::VectorXd ret(pieces.size());

  for (std::size_t i = 0; i < pieces.size(); ++i) {
    if (pieces[i] != "") {
      try {
        ret[i] = toDouble(pieces[i]);
      } catch (std::exception& e) {
        std::cerr << "value [" << pieces[i]
                  << "] is not a valid double for Eigen::VectorXd[" << i
                  << "]: " << e.what() << std::endl;
      }
    }
  }

  return ret;
}

//==============================================================================
Eigen::Isometry3d toIsometry3d(const std::string& str)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  Eigen::Vector6d elements = Eigen::Vector6d::Zero();
  const std::vector<std::string> pieces = common::split(common::trim(str));
  assert(pieces.size() == 6);

  for (std::size_t i = 0; i < pieces.size(); ++i) {
    if (pieces[i] != "") {
      try {
        elements[i] = toDouble(pieces[i]);
      } catch (std::exception& e) {
        std::cerr << "value [" << pieces[i]
                  << "] is not a valid double for SE3[" << i
                  << "]: " << e.what() << std::endl;
      }
    }
  }

  T.linear() = math::eulerXYZToMatrix(elements.tail<3>());
  T.translation() = elements.head<3>();
  return T;
}

//==============================================================================
Eigen::Isometry3d toIsometry3dWithExtrinsicRotation(const std::string& str)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  Eigen::Vector6d elements = Eigen::Vector6d::Zero();
  const std::vector<std::string> pieces = common::split(common::trim(str));
  assert(pieces.size() == 6);

  for (std::size_t i = 0; i < pieces.size(); ++i) {
    if (pieces[i] != "") {
      try {
        elements[i] = toDouble(pieces[i]);
      } catch (std::exception& e) {
        std::cerr << "value [" << pieces[i]
                  << "] is not a valid double for SE3[" << i
                  << "]: " << e.what() << std::endl;
      }
    }
  }

  Eigen::Vector3d reverseEulerAngles(
      elements.tail<3>()[2], elements.tail<3>()[1], elements.tail<3>()[0]);

  T.linear() = math::eulerZYXToMatrix(reverseEulerAngles);
  T.translation() = elements.head<3>();
  return T;
}

//==============================================================================
std::string getValueString(
    const tinyxml2::XMLElement* parentElement, const std::string& name)
{
  assert(parentElement != nullptr);
  assert(!name.empty());

  std::string str = parentElement->FirstChildElement(name.c_str())->GetText();

  return str;
}

//==============================================================================
bool getValueBool(
    const tinyxml2::XMLElement* parentElement, const std::string& name)
{
  assert(parentElement != nullptr);
  assert(!name.empty());

  std::string str = parentElement->FirstChildElement(name.c_str())->GetText();

  if (common::toUpper(str) == "TRUE" || str == "1")
    return true;
  else if (common::toUpper(str) == "FALSE" || str == "0")
    return false;
  else {
    std::cerr << "value [" << str << "] is not a valid boolean type. "
              << "Returning false." << std::endl;
    assert(0);
    return false;
  }
}

//==============================================================================
int getValueInt(
    const tinyxml2::XMLElement* parentElement, const std::string& name)
{
  assert(parentElement != nullptr);
  assert(!name.empty());

  std::string str = parentElement->FirstChildElement(name.c_str())->GetText();

  return toInt(str);
}

//==============================================================================
unsigned int getValueUInt(
    const tinyxml2::XMLElement* parentElement, const std::string& name)
{
  assert(parentElement != nullptr);
  assert(!name.empty());

  std::string str = parentElement->FirstChildElement(name.c_str())->GetText();

  return toUInt(str);
}

//==============================================================================
float getValueFloat(
    const tinyxml2::XMLElement* parentElement, const std::string& name)
{
  assert(parentElement != nullptr);
  assert(!name.empty());

  std::string str = parentElement->FirstChildElement(name.c_str())->GetText();

  return toFloat(str);
}

//==============================================================================
double getValueDouble(
    const tinyxml2::XMLElement* parentElement, const std::string& name)
{
  assert(parentElement != nullptr);
  assert(!name.empty());

  std::string str = parentElement->FirstChildElement(name.c_str())->GetText();

  return toDouble(str);
}

//==============================================================================
char getValueChar(
    const tinyxml2::XMLElement* parentElement, const std::string& name)
{
  assert(parentElement != nullptr);
  assert(!name.empty());

  std::string str = parentElement->FirstChildElement(name.c_str())->GetText();

  return toChar(str);
}

//==============================================================================
Eigen::Vector2d getValueVector2d(
    const tinyxml2::XMLElement* parentElement, const std::string& name)
{
  assert(parentElement != nullptr);
  assert(!name.empty());

  std::string str = parentElement->FirstChildElement(name.c_str())->GetText();

  return toVector2d(str);
}

//==============================================================================
Eigen::Vector3d getValueVector3d(
    const tinyxml2::XMLElement* parentElement, const std::string& name)
{
  assert(parentElement != nullptr);
  assert(!name.empty());

  std::string str = parentElement->FirstChildElement(name.c_str())->GetText();

  return toVector3d(str);
}

//==============================================================================
Eigen::Vector3i getValueVector3i(
    const tinyxml2::XMLElement* parentElement, const std::string& name)
{
  assert(parentElement != nullptr);
  assert(!name.empty());

  std::string str = parentElement->FirstChildElement(name.c_str())->GetText();

  return toVector3i(str);
}

//==============================================================================
Eigen::Vector6d getValueVector6d(
    const tinyxml2::XMLElement* parentElement, const std::string& name)
{
  assert(parentElement != nullptr);
  assert(!name.empty());

  std::string str = parentElement->FirstChildElement(name.c_str())->GetText();

  return toVector6d(str);
}

//==============================================================================
Eigen::VectorXd getValueVectorXd(
    const tinyxml2::XMLElement* parentElement, const std::string& name)
{
  assert(parentElement != nullptr);
  assert(!name.empty());

  std::string str = parentElement->FirstChildElement(name.c_str())->GetText();

  return toVectorXd(str);
}

//==============================================================================
Eigen::Vector3d getValueVec3(
    const tinyxml2::XMLElement* parentElement, const std::string& name)
{
  assert(parentElement != nullptr);
  assert(!name.empty());

  std::string str = parentElement->FirstChildElement(name.c_str())->GetText();

  return toVector3d(str);
}

//==============================================================================
Eigen::Isometry3d getValueIsometry3d(
    const tinyxml2::XMLElement* parentElement, const std::string& name)
{
  assert(parentElement != nullptr);
  assert(!name.empty());

  std::string str = parentElement->FirstChildElement(name.c_str())->GetText();

  return toIsometry3d(str);
}

//==============================================================================
Eigen::Isometry3d getValueIsometry3dWithExtrinsicRotation(
    const tinyxml2::XMLElement* parentElement, const std::string& name)
{
  assert(parentElement != nullptr);
  assert(!name.empty());

  std::string str = parentElement->FirstChildElement(name.c_str())->GetText();

  return toIsometry3dWithExtrinsicRotation(str);
}

//==============================================================================
bool hasElement(
    const tinyxml2::XMLElement* parentElement, const std::string& name)
{
  assert(parentElement != nullptr);
  assert(!name.empty());

  return parentElement->FirstChildElement(name.c_str()) == nullptr ? false
                                                                   : true;
}

//==============================================================================
const tinyxml2::XMLElement* getElement(
    const tinyxml2::XMLElement* parentElement, const std::string& name)
{
  assert(!name.empty());

  return parentElement->FirstChildElement(name.c_str());
}

//==============================================================================
tinyxml2::XMLElement* getElement(
    tinyxml2::XMLElement* parentElement, const std::string& name)
{
  assert(!name.empty());

  return parentElement->FirstChildElement(name.c_str());
}

//==============================================================================
std::string toString(tinyxml2::XMLError errorCode)
{
  switch (errorCode) {
    case tinyxml2::XMLError::XML_SUCCESS:
      return "XML_SUCCESS";
    case tinyxml2::XMLError::XML_NO_ATTRIBUTE:
      return "XML_NO_ATTRIBUTE";
    case tinyxml2::XMLError::XML_WRONG_ATTRIBUTE_TYPE:
      return "XML_WRONG_ATTRIBUTE_TYPE";
    case tinyxml2::XMLError::XML_ERROR_FILE_NOT_FOUND:
      return "XML_ERROR_FILE_NOT_FOUND";
    case tinyxml2::XMLError::XML_ERROR_FILE_COULD_NOT_BE_OPENED:
      return "XML_ERROR_FILE_COULD_NOT_BE_OPENED";
    case tinyxml2::XMLError::XML_ERROR_FILE_READ_ERROR:
      return "XML_ERROR_FILE_READ_ERROR";
    case tinyxml2::XMLError::XML_ERROR_PARSING_ELEMENT:
      return "XML_ERROR_PARSING_ELEMENT";
    case tinyxml2::XMLError::XML_ERROR_PARSING_ATTRIBUTE:
      return "XML_ERROR_PARSING_ATTRIBUTE";
    case tinyxml2::XMLError::XML_ERROR_PARSING_TEXT:
      return "XML_ERROR_PARSING_TEXT";
    case tinyxml2::XMLError::XML_ERROR_PARSING_CDATA:
      return "XML_ERROR_PARSING_CDATA";
    case tinyxml2::XMLError::XML_ERROR_PARSING_COMMENT:
      return "XML_ERROR_PARSING_COMMENT";
    case tinyxml2::XMLError::XML_ERROR_PARSING_DECLARATION:
      return "XML_ERROR_PARSING_DECLARATION";
    case tinyxml2::XMLError::XML_ERROR_PARSING_UNKNOWN:
      return "XML_ERROR_PARSING_UNKNOWN";
    case tinyxml2::XMLError::XML_ERROR_EMPTY_DOCUMENT:
      return "XML_ERROR_EMPTY_DOCUMENT";
    case tinyxml2::XMLError::XML_ERROR_MISMATCHED_ELEMENT:
      return "XML_ERROR_MISMATCHED_ELEMENT";
    case tinyxml2::XMLError::XML_ERROR_PARSING:
      return "XML_ERROR_PARSING";
    case tinyxml2::XMLError::XML_CAN_NOT_CONVERT_TEXT:
      return "XML_CAN_NOT_CONVERT_TEXT";
    case tinyxml2::XMLError::XML_NO_TEXT_NODE:
      return "XML_NO_TEXT_NODE";
    case tinyxml2::XMLError::XML_ERROR_COUNT:
      return "XML_ERROR_COUNT";
    default:
      return "Unknow error";
  }
}

//==============================================================================
void openXMLFile(
    tinyxml2::XMLDocument& doc,
    const common::Uri& uri,
    const common::ResourceRetrieverPtr& retrieverOrNullPtr)
{
  common::ResourceRetrieverPtr retriever;
  if (retrieverOrNullPtr)
    retriever = retrieverOrNullPtr;
  else
    retriever = std::make_shared<common::LocalResourceRetriever>();

  const auto content = retriever->readAll(uri);
  const auto result = doc.Parse(&content.front());
  if (result != tinyxml2::XML_SUCCESS) {
    dtwarn << "[openXMLFile] Failed parsing XML: TinyXML2 returned error '"
           << toString(result) << "'.\n";
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
  if (retrieverOrNullPtr)
    retriever = retrieverOrNullPtr;
  else
    retriever = std::make_shared<common::LocalResourceRetriever>();

  const auto content = retriever->readAll(uri);
  const auto result = doc.Parse(&content.front());
  if (result != tinyxml2::XML_SUCCESS) {
    dtwarn << "[readXmlFile] Failed parsing XML: TinyXML2 returned error '"
           << toString(result) << "'.\n";
    return false;
  }

  return true;
}

//==============================================================================
bool hasAttribute(const tinyxml2::XMLElement* element, const char* const name)
{
  const char* const result = element->Attribute(name);
  return result != nullptr;
}

//==============================================================================
std::string getAttributeString(
    const tinyxml2::XMLElement* element, const std::string& attributeName)
{
  const char* const result = element->Attribute(attributeName.c_str());

  if (nullptr == result) {
    dtwarn << "[getAttribute] Error in parsing string type attribute ["
           << attributeName << "] of an element [" << element->Name()
           << "]. Returning empty string.\n";
    return std::string();
  }

  return std::string(result);
}

//==============================================================================
bool getAttributeBool(
    const tinyxml2::XMLElement* element, const std::string& attributeName)
{
  bool val = false;
  const int result = element->QueryBoolAttribute(attributeName.c_str(), &val);

  if (result != tinyxml2::XML_SUCCESS) {
    dtwarn << "[getAttribute] Error in parsing bool type attribute ["
           << attributeName << "] of an element [" << element->Name()
           << "]. Returning false instead.\n";
    return false;
  }

  return val;
}

//==============================================================================
int getAttributeInt(
    const tinyxml2::XMLElement* element, const std::string& attributeName)
{
  int val = 0;
  const int result = element->QueryIntAttribute(attributeName.c_str(), &val);

  if (result != tinyxml2::XML_SUCCESS) {
    dtwarn << "[getAttribute] Error in parsing int type attribute ["
           << attributeName << "] of an element [" << element->Name()
           << "]. Returning zero instead.\n";
    return 0;
  }

  return val;
}

//==============================================================================
unsigned int getAttributeUInt(
    const tinyxml2::XMLElement* element, const std::string& attributeName)
{
  unsigned int val = 0u;
  const int result
      = element->QueryUnsignedAttribute(attributeName.c_str(), &val);

  if (result != tinyxml2::XML_SUCCESS) {
    dtwarn << "[getAttribute] Error in parsing unsiged int type attribute ["
           << attributeName << "] of an element [" << element->Name()
           << "]. Returning zero instead.\n";
    return 0u;
  }

  return val;
}

//==============================================================================
float getAttributeFloat(
    const tinyxml2::XMLElement* element, const std::string& attributeName)
{
  float val = 0.0f;
  const int result = element->QueryFloatAttribute(attributeName.c_str(), &val);

  if (result != tinyxml2::XML_SUCCESS) {
    dtwarn << "[getAttribute] Error in parsing float type attribute ["
           << attributeName << "] of an element [" << element->Name()
           << "]. Returning zero instead.\n";
    return 0.0f;
  }

  return val;
}

//==============================================================================
double getAttributeDouble(
    const tinyxml2::XMLElement* element, const std::string& attributeName)
{
  double val = 0.0;
  const int result = element->QueryDoubleAttribute(attributeName.c_str(), &val);

  if (result != tinyxml2::XML_SUCCESS) {
    dtwarn << "[getAttribute] Error in parsing double type attribute ["
           << attributeName << "] of an element [" << element->Name()
           << "]. Returning zero instead.\n";
    return 0.0;
  }

  return val;
}

//==============================================================================
char getAttributeChar(
    const tinyxml2::XMLElement* element, const std::string& attributeName)
{
  const std::string val = getAttributeString(element, attributeName);

  return toChar(val);
}

//==============================================================================
Eigen::Vector2i getAttributeVector2i(
    const tinyxml2::XMLElement* element, const std::string& attributeName)
{
  const std::string val = getAttributeString(element, attributeName);

  return toVector2i(val);
}

//==============================================================================
Eigen::Vector2d getAttributeVector2d(
    const tinyxml2::XMLElement* element, const std::string& attributeName)
{
  const std::string val = getAttributeString(element, attributeName);

  return toVector2d(val);
}

//==============================================================================
Eigen::Vector3d getAttributeVector3d(
    const tinyxml2::XMLElement* element, const std::string& attributeName)
{
  const std::string val = getAttributeString(element, attributeName);

  return toVector3d(val);
}

//==============================================================================
Eigen::Vector4d getAttributeVector4d(
    const tinyxml2::XMLElement* element, const std::string& attributeName)
{
  const std::string val = getAttributeString(element, attributeName);

  return toVector4d(val);
}

//==============================================================================
Eigen::Vector6d getAttributeVector6d(
    const tinyxml2::XMLElement* element, const std::string& attributeName)
{
  const std::string val = getAttributeString(element, attributeName);

  return toVector6d(val);
}

//==============================================================================
Eigen::VectorXd getAttributeVectorXd(
    const tinyxml2::XMLElement* element, const std::string& attributeName)
{
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
