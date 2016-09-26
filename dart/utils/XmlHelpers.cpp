/*
 * Copyright (c) 2013-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2013-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
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

#include "dart/utils/XmlHelpers.hpp"

#include <iostream>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include "dart/common/Console.hpp"
#include "dart/math/Geometry.hpp"
#include "dart/common/LocalResourceRetriever.hpp"

namespace dart {
namespace utils {

//==============================================================================
std::string toString(bool v)
{
  return boost::lexical_cast<std::string>(v);
}

//==============================================================================
std::string toString(int v)
{
  return boost::lexical_cast<std::string>(v);
}

//==============================================================================
std::string toString(unsigned int v)
{
  return boost::lexical_cast<std::string>(v);
}

//==============================================================================
std::string toString(float v)
{
  return boost::lexical_cast<std::string>(v);
}

//==============================================================================
std::string toString(double v)
{
  return boost::lexical_cast<std::string>(v);
}

//==============================================================================
std::string toString(char v)
{
  return boost::lexical_cast<std::string>(v);
}

//==============================================================================
std::string toString(const Eigen::Vector2d& v)
{
  return boost::lexical_cast<std::string>(v.transpose());
}

//==============================================================================
std::string toString(const Eigen::Vector3d& v)
{
  return boost::lexical_cast<std::string>(v.transpose());
}

//==============================================================================
std::string toString(const Eigen::Vector3i& v)
{
  return boost::lexical_cast<std::string>(v.transpose());
}

//==============================================================================
std::string toString(const Eigen::Vector6d& v)
{
  return boost::lexical_cast<std::string>(v.transpose());
}

//==============================================================================
std::string toString(const Eigen::VectorXd& v)
{
  return boost::lexical_cast<std::string>(v.transpose());
}

//==============================================================================
std::string toString(const Eigen::Isometry3d& v)
{
  std::ostringstream ostr;
  ostr.precision(6);

  Eigen::Vector3d xyz = math::matrixToEulerXYZ(v.linear());

  ostr << v.translation()(0) << " "
       << v.translation()(1) << " "
       << v.translation()(2) << " ";
  ostr << xyz[0] << " " << xyz[1] << " " << xyz[2];

  return ostr.str();
}

//==============================================================================
bool toBool(const std::string& str)
{
  if (boost::to_upper_copy(str) == "TRUE" || str == "1")
    return true;
  else if (boost::to_upper_copy(str) == "FALSE" || str == "0")
    return false;
  else
  {
    dterr << "value ["
          << str
          << "] is not a valid boolean type. "
          << "Retuning false."
          << std::endl;
    return false;
  }
}

//==============================================================================
int toInt(const std::string& str)
{
  return boost::lexical_cast<int>(str);
}

//==============================================================================
unsigned int toUInt(const std::string& str)
{
  return boost::lexical_cast<unsigned int>(str);
}

//==============================================================================
float toFloat(const std::string& str)
{
  return boost::lexical_cast<float>(str);
}

//==============================================================================
double toDouble(const std::string& str)
{
  return boost::lexical_cast<double>(str);
}
//==============================================================================
char toChar(const std::string& str)
{
  return boost::lexical_cast<char>(str);
}

//==============================================================================
Eigen::Vector2d toVector2d(const std::string& str)
{
  Eigen::Vector2d ret;

  std::vector<std::string> pieces;
  std::string trimedStr = boost::trim_copy(str);
  boost::split(pieces, trimedStr, boost::is_any_of(" "),
               boost::token_compress_on);
  assert(pieces.size() == 2);

  for (std::size_t i = 0; i < pieces.size(); ++i)
  {
    if (pieces[i] != "")
    {
      try
      {
        ret(i) = boost::lexical_cast<double>(pieces[i].c_str());
      }
      catch (boost::bad_lexical_cast& e)
      {
        std::cerr << "value ["
                  << pieces[i]
                     << "] is not a valid double for Eigen::Vector2d["
                     << i
                     << std::endl;
      }
    }
  }

  return ret;
}

//==============================================================================
Eigen::Vector3d toVector3d(const std::string& str)
{
  Eigen::Vector3d ret;

  std::vector<std::string> pieces;
  std::string trimedStr = boost::trim_copy(str);
  boost::split(pieces, trimedStr, boost::is_any_of(" "),
               boost::token_compress_on);
  assert(pieces.size() == 3);

  for (std::size_t i = 0; i < pieces.size(); ++i)
  {
    if (pieces[i] != "")
    {
      try
      {
        ret(i) = boost::lexical_cast<double>(pieces[i].c_str());
      }
      catch(boost::bad_lexical_cast& e)
      {
        std::cerr << "value ["
                  << pieces[i]
                     << "] is not a valid double for Eigen::Vector3d["
                     << i
                     << "]"
                     << std::endl;
      }
    }
  }

  return ret;
}

//==============================================================================
Eigen::Vector3i toVector3i(const std::string& str)
{
  Eigen::Vector3i ret;

  std::vector<std::string> pieces;
  std::string trimedStr = boost::trim_copy(str);
  boost::split(pieces, trimedStr, boost::is_any_of(" "),
               boost::token_compress_on);
  assert(pieces.size() == 3);

  for (std::size_t i = 0; i < pieces.size(); ++i)
  {
    if (pieces[i] != "")
    {
      try
      {
        ret(i) = boost::lexical_cast<int>(pieces[i].c_str());
      }
      catch(boost::bad_lexical_cast& e)
      {
        std::cerr << "value ["
                  << pieces[i]
                     << "] is not a valid int for Eigen::Vector3i["
                     << i
                     << "]"
                     << std::endl;
      }
    }
  }

  return ret;
}

//==============================================================================
Eigen::Vector6d toVector6d(const std::string& str)
{
  Eigen::Vector6d ret;

  std::vector<std::string> pieces;
  std::string trimedStr = boost::trim_copy(str);
  boost::split(pieces, trimedStr, boost::is_any_of(" "),
               boost::token_compress_on);
  assert(pieces.size() == 6);

  for (std::size_t i = 0; i < pieces.size(); ++i)
  {
    if (pieces[i] != "")
    {
      try
      {
        ret(i) = boost::lexical_cast<double>(pieces[i].c_str());
      }
      catch(boost::bad_lexical_cast& e)
      {
        std::cerr << "value ["
                  << pieces[i]
                     << "] is not a valid double for Eigen::Vector6d["
                     << i
                     << "]"
                     << std::endl;
      }
    }
  }

  return ret;
}

//==============================================================================
Eigen::VectorXd toVectorXd(const std::string& str)
{
  std::vector<std::string> pieces;
  std::string trimedStr = boost::trim_copy(str);
  boost::split(pieces, trimedStr, boost::is_any_of(" "),
               boost::token_compress_on);
  assert(pieces.size() > 0);

  Eigen::VectorXd ret(pieces.size());

  for (std::size_t i = 0; i < pieces.size(); ++i)
  {
    if (pieces[i] != "")
    {
      try
      {
        ret(i) = boost::lexical_cast<double>(pieces[i].c_str());
      }
      catch(boost::bad_lexical_cast& e)
      {
        std::cerr << "value ["
                  << pieces[i]
                     << "] is not a valid double for Eigen::VectorXd["
                     << i
                     << "]"
                     << std::endl;
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
  std::vector<std::string> pieces;
  std::string trimedStr = boost::trim_copy(str);
  boost::split(pieces, trimedStr, boost::is_any_of(" "),
               boost::token_compress_on);
  assert(pieces.size() == 6);

  for (std::size_t i = 0; i < pieces.size(); ++i)
  {
    if (pieces[i] != "")
    {
      try
      {
        elements(i) = boost::lexical_cast<double>(pieces[i].c_str());
      }
      catch(boost::bad_lexical_cast& e)
      {
        std::cerr << "value ["
                  << pieces[i]
                     << "] is not a valid double for SE3["
                     << i
                     << "]"
                     << std::endl;
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
  std::vector<std::string> pieces;
  std::string trimedStr = boost::trim_copy(str);
  boost::split(pieces, trimedStr, boost::is_any_of(" "),
               boost::token_compress_on);
  assert(pieces.size() == 6);

  for (std::size_t i = 0; i < pieces.size(); ++i)
  {
    if (pieces[i] != "")
    {
      try
      {
        elements(i) = boost::lexical_cast<double>(pieces[i].c_str());
      }
      catch(boost::bad_lexical_cast& e)
      {
        std::cerr << "value ["
                  << pieces[i]
                     << "] is not a valid double for SE3["
                     << i
                     << "]"
                     << std::endl;
      }
    }
  }

  Eigen::Vector3d reverseEulerAngles(
        elements.tail<3>()[2],
      elements.tail<3>()[1],
      elements.tail<3>()[0]);

  T.linear() = math::eulerZYXToMatrix(reverseEulerAngles);
  T.translation() = elements.head<3>();
  return T;
}

//==============================================================================
std::string getValueString(const tinyxml2::XMLElement* parentElement,
                           const std::string& name)
{
  assert(parentElement != nullptr);
  assert(!name.empty());

  std::string str = parentElement->FirstChildElement(name.c_str())->GetText();

  return str;
}

//==============================================================================
bool getValueBool(const tinyxml2::XMLElement* parentElement,
                  const std::string& name)
{
  assert(parentElement != nullptr);
  assert(!name.empty());

  std::string str = parentElement->FirstChildElement(name.c_str())->GetText();

  if (boost::to_upper_copy(str) == "TRUE" || str == "1")
    return true;
  else if (boost::to_upper_copy(str) == "FALSE" || str == "0")
    return false;
  else
  {
    std::cerr << "value ["
              << str
              << "] is not a valid boolean type. "
              << "Returning false."
              << std::endl;
    assert(0);
    return false;
  }
}

//==============================================================================
int getValueInt(const tinyxml2::XMLElement* parentElement,
                const std::string& name)
{
  assert(parentElement != nullptr);
  assert(!name.empty());

  std::string str = parentElement->FirstChildElement(name.c_str())->GetText();

  return toInt(str);
}

//==============================================================================
unsigned int getValueUInt(const tinyxml2::XMLElement* parentElement,
                          const std::string& name)
{
  assert(parentElement != nullptr);
  assert(!name.empty());

  std::string str = parentElement->FirstChildElement(name.c_str())->GetText();

  return toUInt(str);
}

//==============================================================================
float getValueFloat(const tinyxml2::XMLElement* parentElement,
                    const std::string& name)
{
  assert(parentElement != nullptr);
  assert(!name.empty());

  std::string str = parentElement->FirstChildElement(name.c_str())->GetText();

  return toFloat(str);
}

//==============================================================================
double getValueDouble(const tinyxml2::XMLElement* parentElement,
                      const std::string& name)
{
  assert(parentElement != nullptr);
  assert(!name.empty());

  std::string str = parentElement->FirstChildElement(name.c_str())->GetText();

  return toDouble(str);
}

//==============================================================================
char getValueChar(const tinyxml2::XMLElement* parentElement,
                  const std::string& name)
{
  assert(parentElement != nullptr);
  assert(!name.empty());

  std::string str = parentElement->FirstChildElement(name.c_str())->GetText();

  return toChar(str);
}

//==============================================================================
Eigen::Vector2d getValueVector2d(const tinyxml2::XMLElement* parentElement,
                                 const std::string& name)
{
  assert(parentElement != nullptr);
  assert(!name.empty());

  std::string str = parentElement->FirstChildElement(name.c_str())->GetText();

  return toVector2d(str);
}

//==============================================================================
Eigen::Vector3d getValueVector3d(const tinyxml2::XMLElement* parentElement,
                                 const std::string& name)
{
  assert(parentElement != nullptr);
  assert(!name.empty());

  std::string str = parentElement->FirstChildElement(name.c_str())->GetText();

  return toVector3d(str);
}

//==============================================================================
Eigen::Vector3i getValueVector3i(const tinyxml2::XMLElement* parentElement,
                                 const std::string& name)
{
  assert(parentElement != nullptr);
  assert(!name.empty());

  std::string str = parentElement->FirstChildElement(name.c_str())->GetText();

  return toVector3i(str);
}

//==============================================================================
Eigen::Vector6d getValueVector6d(const tinyxml2::XMLElement* parentElement,
                                 const std::string& name)
{
  assert(parentElement != nullptr);
  assert(!name.empty());

  std::string str = parentElement->FirstChildElement(name.c_str())->GetText();

  return toVector6d(str);
}

//==============================================================================
Eigen::VectorXd getValueVectorXd(const tinyxml2::XMLElement* parentElement,
                                 const std::string& name)
{
  assert(parentElement != nullptr);
  assert(!name.empty());

  std::string str = parentElement->FirstChildElement(name.c_str())->GetText();

  return toVectorXd(str);
}

//==============================================================================
Eigen::Vector3d getValueVec3(const tinyxml2::XMLElement* parentElement,
                             const std::string& name)
{
  assert(parentElement != nullptr);
  assert(!name.empty());

  std::string str = parentElement->FirstChildElement(name.c_str())->GetText();

  return toVector3d(str);
}

//==============================================================================
Eigen::Isometry3d getValueIsometry3d(const tinyxml2::XMLElement* parentElement,
                                     const std::string& name)
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
bool hasElement(const tinyxml2::XMLElement* parentElement,
                const std::string& name)
{
  assert(parentElement != nullptr);
  assert(!name.empty());

  return parentElement->FirstChildElement(name.c_str())
      == nullptr ? false : true;
}

//==============================================================================
const tinyxml2::XMLElement* getElement(
    const tinyxml2::XMLElement* parentElement,
    const std::string& name)
{
  assert(!name.empty());

  return parentElement->FirstChildElement(name.c_str());
}

//==============================================================================
tinyxml2::XMLElement* getElement(tinyxml2::XMLElement* parentElement,
                                 const std::string& name)
{
  assert(!name.empty());

  return parentElement->FirstChildElement(name.c_str());
}

//==============================================================================
void openXMLFile(tinyxml2::XMLDocument& doc,
                 const common::Uri& uri,
                 const common::ResourceRetrieverPtr& retrieverOrNullPtr)
{
  common::ResourceRetrieverPtr retriever;
  if(retrieverOrNullPtr)
    retriever = retrieverOrNullPtr;
  else
    retriever = std::make_shared<common::LocalResourceRetriever>();

  const common::ResourcePtr resource = retriever->retrieve(uri);
  if(!resource)
  {
    dtwarn << "[openXMLFile] Failed opening URI '"
           << uri.toString() << "'.\n";
    throw std::runtime_error("Failed opening URI.");
  }

  // C++11 guarantees that std::string has contiguous storage.
  const std::size_t size = resource->getSize();
  std::string content;
  content.resize(size);
  if(resource->read(&content.front(), size, 1) != 1)
  {
    dtwarn << "[openXMLFile] Failed reading from URI '"
           << uri.toString() << "'.\n";
    throw std::runtime_error("Failed reading from URI.");
  }

  int const result = doc.Parse(&content.front());
  if(result != tinyxml2::XML_SUCCESS)
  {
    dtwarn << "[openXMLFile] Failed parsing XML: TinyXML2 returned error"
              " code " << result << ".\n";
    throw std::runtime_error("Failed parsing XML.");
  }
}

//==============================================================================
bool hasAttribute(const tinyxml2::XMLElement* element, const char* const name)
{
  const char* const result = element->Attribute(name);
  return result != 0;
}

//==============================================================================
std::string getAttribute(tinyxml2::XMLElement * element,
                         const char* const name)
{
  return getAttributeString(element, name);
}

//==============================================================================
void getAttribute(tinyxml2::XMLElement* element, const char* const name,
                  double* d)
{
  *d = getAttributeDouble(element, name);
}

//==============================================================================
std::string getAttributeString(const tinyxml2::XMLElement* element,
                               const std::string& attributeName)
{
  const char* const result = element->Attribute(attributeName.c_str());

  if (nullptr == result)
  {
    dtwarn << "[getAttribute] Error in parsing string type attribute ["
           << attributeName << "] of an element [" << element->Name()
           << "]. Returning empty string.\n";
    return std::string();
  }

  return std::string(result);
}

//==============================================================================
bool getAttributeBool(const tinyxml2::XMLElement* element,
                      const std::string& attributeName)
{
  bool val = false;
  const int result = element->QueryBoolAttribute(attributeName.c_str(),
                                                 &val);

  if (result != tinyxml2::XML_SUCCESS)
  {
    dtwarn << "[getAttribute] Error in parsing bool type attribute ["
           << attributeName << "] of an element [" << element->Name()
           << "]. Returning false instead.\n";
    return false;
  }

  return val;
}

//==============================================================================
int getAttributeInt(const tinyxml2::XMLElement* element,
                    const std::string& attributeName)
{
  int val = 0;
  const int result = element->QueryIntAttribute(attributeName.c_str(), &val);

  if (result != tinyxml2::XML_SUCCESS)
  {
    dtwarn << "[getAttribute] Error in parsing int type attribute ["
           << attributeName << "] of an element [" << element->Name()
           << "]. Returning zero instead.\n";
    return 0;
  }

  return val;
}

//==============================================================================
unsigned int getAttributeUInt(const tinyxml2::XMLElement* element,
                              const std::string& attributeName)
{
  unsigned int val = 0u;
  const int result = element->QueryUnsignedAttribute(attributeName.c_str(),
                                                     &val);

  if (result != tinyxml2::XML_SUCCESS)
  {
    dtwarn << "[getAttribute] Error in parsing unsiged int type attribute ["
           << attributeName << "] of an element [" << element->Name()
           << "]. Returning zero instead.\n";
    return 0u;
  }

  return val;
}

//==============================================================================
float getAttributeFloat(const tinyxml2::XMLElement* element,
                        const std::string& attributeName)
{
  float val = 0.0f;
  const int result = element->QueryFloatAttribute(attributeName.c_str(),
                                                  &val);

  if (result != tinyxml2::XML_SUCCESS)
  {
    dtwarn << "[getAttribute] Error in parsing float type attribute ["
           << attributeName << "] of an element [" << element->Name()
           << "]. Returning zero instead.\n";
    return 0.0f;
  }

  return val;
}

//==============================================================================
double getAttributeDouble(const tinyxml2::XMLElement* element,
                          const std::string& attributeName)
{
  double val = 0.0;
  const int result = element->QueryDoubleAttribute(attributeName.c_str(),
                                                   &val);

  if (result != tinyxml2::XML_SUCCESS)
  {
    dtwarn << "[getAttribute] Error in parsing double type attribute ["
           << attributeName << "] of an element [" << element->Name()
           << "]. Returning zero instead.\n";
    return 0.0;
  }

  return val;
}

//==============================================================================
char getAttributeChar(const tinyxml2::XMLElement* element,
                      const std::string& attributeName)
{
  const std::string val = getAttributeString(element, attributeName);

  return toChar(val);
}

//==============================================================================
Eigen::Vector2d getAttributeVector2d(const tinyxml2::XMLElement* element,
                                     const std::string& attributeName)
{
  const std::string val = getAttributeString(element, attributeName);

  return toVector2d(val);
}

//==============================================================================
Eigen::Vector3d getAttributeVector3d(const tinyxml2::XMLElement* element,
                                     const std::string& attributeName)
{
  const std::string val = getAttributeString(element, attributeName);

  return toVector3d(val);
}

//==============================================================================
Eigen::Vector6d getAttributeVector6d(const tinyxml2::XMLElement* element,
                                     const std::string& attributeName)
{
  const std::string val = getAttributeString(element, attributeName);

  return toVector6d(val);
}

//==============================================================================
Eigen::VectorXd getAttributeVectorXd(const tinyxml2::XMLElement* element,
                                     const std::string& attributeName)
{
  const std::string val = getAttributeString(element, attributeName);

  return toVectorXd(val);
}

} // namespace utils
} // namespace dart
