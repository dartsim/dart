/*
 * Copyright (c) 2013-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
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

#include "dart/utils/Parser.h"

#include <iostream>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include "dart/common/Console.h"
#include "dart/math/Geometry.h"
#include "dart/common/LocalResourceRetriever.h"

namespace dart {
namespace utils {

std::string toString(bool _v)
{
    return boost::lexical_cast<std::string>(_v);
}

std::string toString(int _v)
{
    return boost::lexical_cast<std::string>(_v);
}

std::string toString(unsigned int _v)
{
    return boost::lexical_cast<std::string>(_v);
}

std::string toString(float _v)
{
    //if (std::isfinite(_v))
        return boost::lexical_cast<std::string>(_v);
    //else
    //    return std::string("0");
}

std::string toString(double _v)
{
    //if (std::isfinite(_v))
        return boost::lexical_cast<std::string>(_v);
    //else
    //    return std::string("0");
}

std::string toString(char _v)
{
    return boost::lexical_cast<std::string>(_v);
}

//==============================================================================
std::string toString(const Eigen::Vector2d& _v)
{
  return boost::lexical_cast<std::string>(_v.transpose());
}

//==============================================================================
std::string toString(const Eigen::Vector3d& _v)
{
  return boost::lexical_cast<std::string>(_v.transpose());
}

//==============================================================================
std::string toString(const Eigen::Vector3i& _v)
{
  return boost::lexical_cast<std::string>(_v.transpose());
}

//==============================================================================
std::string toString(const Eigen::Vector6d& _v)
{
  return boost::lexical_cast<std::string>(_v.transpose());
}

//==============================================================================
std::string toString(const Eigen::VectorXd& _v)
{
  return boost::lexical_cast<std::string>(_v.transpose());
}

std::string toString(const Eigen::Isometry3d& _v)
{
    std::ostringstream ostr;
    ostr.precision(6);

    Eigen::Vector3d xyz = math::matrixToEulerXYZ(_v.linear());

    ostr << _v.translation()(0) << " "
         << _v.translation()(1) << " "
         << _v.translation()(2) << " ";
    ostr << xyz[0] << " " << xyz[1] << " " << xyz[2];

    return ostr.str();
}

bool toBool(const std::string& _str)
{
    if (boost::to_upper_copy(_str) == "TRUE" || _str == "1")
        return true;
    else if (boost::to_upper_copy(_str) == "FALSE" || _str == "0")
        return false;
    else
    {
        dterr << "value ["
              << _str
              << "] is not a valid boolean type. "
              << "Retuning false."
              << std::endl;
        return false;
    }
}

int toInt(const std::string& _str)
{
    return boost::lexical_cast<int>(_str);
}

unsigned int toUInt(const std::string& _str)
{
    return boost::lexical_cast<unsigned int>(_str);
}

float toFloat(const std::string& _str)
{
    return boost::lexical_cast<float>(_str);
}

double toDouble(const std::string& _str)
{
    return boost::lexical_cast<double>(_str);
}

char toChar(const std::string& _str)
{
    return boost::lexical_cast<char>(_str);
}

Eigen::Vector2d toVector2d(const std::string& _str)
{
    Eigen::Vector2d ret;

    std::vector<std::string> pieces;
    std::string trimedStr = boost::trim_copy(_str);
    boost::split(pieces, trimedStr, boost::is_any_of(" "), boost::token_compress_on);
    assert(pieces.size() == 2);

    for (size_t i = 0; i < pieces.size(); ++i)
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
                          << "] is not a valid double for Eigen::Vector2d["
                          << i
                          << std::endl;
            }
        }
    }

    return ret;
}

Eigen::Vector3d toVector3d(const std::string& _str)
{
    Eigen::Vector3d ret;

    std::vector<std::string> pieces;
    std::string trimedStr = boost::trim_copy(_str);
    boost::split(pieces, trimedStr, boost::is_any_of(" "), boost::token_compress_on);
    assert(pieces.size() == 3);

    for (size_t i = 0; i < pieces.size(); ++i)
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

Eigen::Vector3i toVector3i(const std::string& _str)
{
  Eigen::Vector3i ret;

  std::vector<std::string> pieces;
  std::string trimedStr = boost::trim_copy(_str);
  boost::split(pieces, trimedStr, boost::is_any_of(" "), boost::token_compress_on);
  assert(pieces.size() == 3);

  for (size_t i = 0; i < pieces.size(); ++i)
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

Eigen::Vector6d toVector6d(const std::string& _str)
{
    Eigen::Vector6d ret;

    std::vector<std::string> pieces;
    std::string trimedStr = boost::trim_copy(_str);
    boost::split(pieces, trimedStr, boost::is_any_of(" "), boost::token_compress_on);
    assert(pieces.size() == 6);

    for (size_t i = 0; i < pieces.size(); ++i)
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
Eigen::VectorXd toVectorXd(const std::string& _str)
{
  std::vector<std::string> pieces;
  std::string trimedStr = boost::trim_copy(_str);
  boost::split(pieces, trimedStr, boost::is_any_of(" "),
               boost::token_compress_on);
  assert(pieces.size() > 0);

  Eigen::VectorXd ret(pieces.size());

  for (size_t i = 0; i < pieces.size(); ++i)
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

Eigen::Isometry3d toIsometry3d(const std::string& _str)
{
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    Eigen::Vector6d elements = Eigen::Vector6d::Zero();
    std::vector<std::string> pieces;
    std::string trimedStr = boost::trim_copy(_str);
    boost::split(pieces, trimedStr, boost::is_any_of(" "), boost::token_compress_on);
    assert(pieces.size() == 6);

    for (size_t i = 0; i < pieces.size(); ++i)
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

Eigen::Isometry3d toIsometry3dWithExtrinsicRotation(const std::string& _str)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  Eigen::Vector6d elements = Eigen::Vector6d::Zero();
  std::vector<std::string> pieces;
  std::string trimedStr = boost::trim_copy(_str);
  boost::split(pieces, trimedStr, boost::is_any_of(" "), boost::token_compress_on);
  assert(pieces.size() == 6);

  for (size_t i = 0; i < pieces.size(); ++i)
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

std::string getValueString(tinyxml2::XMLElement* _parentElement, const std::string& _name)
{
    assert(_parentElement != nullptr);
    assert(!_name.empty());

    std::string str = _parentElement->FirstChildElement(_name.c_str())->GetText();

    return str;
}

bool getValueBool(tinyxml2::XMLElement* _parentElement, const std::string& _name)
{
    assert(_parentElement != nullptr);
    assert(!_name.empty());

    std::string str = _parentElement->FirstChildElement(_name.c_str())->GetText();

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

int getValueInt(tinyxml2::XMLElement* _parentElement, const std::string& _name)
{
    assert(_parentElement != nullptr);
    assert(!_name.empty());

    std::string str = _parentElement->FirstChildElement(_name.c_str())->GetText();

    return toInt(str);
}

unsigned int getValueUInt(tinyxml2::XMLElement* _parentElement, const std::string& _name)
{
    assert(_parentElement != nullptr);
    assert(!_name.empty());

    std::string str = _parentElement->FirstChildElement(_name.c_str())->GetText();

    return toUInt(str);
}

float getValueFloat(tinyxml2::XMLElement* _parentElement, const std::string& _name)
{
    assert(_parentElement != nullptr);
    assert(!_name.empty());

    std::string str = _parentElement->FirstChildElement(_name.c_str())->GetText();

    return toFloat(str);
}

double getValueDouble(tinyxml2::XMLElement* _parentElement, const std::string& _name)
{
    assert(_parentElement != nullptr);
    assert(!_name.empty());

    std::string str = _parentElement->FirstChildElement(_name.c_str())->GetText();

    return toDouble(str);
}

char getValueChar(tinyxml2::XMLElement* _parentElement, const std::string& _name)
{
    assert(_parentElement != nullptr);
    assert(!_name.empty());

    std::string str = _parentElement->FirstChildElement(_name.c_str())->GetText();

    return toChar(str);
}

Eigen::Vector2d getValueVector2d(tinyxml2::XMLElement* _parentElement, const std::string& _name)
{
    assert(_parentElement != nullptr);
    assert(!_name.empty());

    std::string str = _parentElement->FirstChildElement(_name.c_str())->GetText();

    return toVector2d(str);
}

Eigen::Vector3d getValueVector3d(tinyxml2::XMLElement* _parentElement, const std::string& _name)
{
    assert(_parentElement != nullptr);
    assert(!_name.empty());

    std::string str = _parentElement->FirstChildElement(_name.c_str())->GetText();

    return toVector3d(str);
}

Eigen::Vector3i getValueVector3i(tinyxml2::XMLElement* _parentElement, const std::string& _name)
{
  assert(_parentElement != nullptr);
  assert(!_name.empty());

  std::string str = _parentElement->FirstChildElement(_name.c_str())->GetText();

  return toVector3i(str);
}

Eigen::Vector6d getValueVector6d(tinyxml2::XMLElement* _parentElement, const std::string& _name)
{
    assert(_parentElement != nullptr);
    assert(!_name.empty());

    std::string str = _parentElement->FirstChildElement(_name.c_str())->GetText();

    return toVector6d(str);
}

//==============================================================================
Eigen::VectorXd getValueVectorXd(tinyxml2::XMLElement* _parentElement,
                                 const std::string& _name)
{
  assert(_parentElement != nullptr);
  assert(!_name.empty());

  std::string str = _parentElement->FirstChildElement(_name.c_str())->GetText();

  return toVectorXd(str);
}

Eigen::Vector3d getValueVec3(tinyxml2::XMLElement* _parentElement, const std::string& _name)
{
    assert(_parentElement != nullptr);
    assert(!_name.empty());

    std::string str = _parentElement->FirstChildElement(_name.c_str())->GetText();

    return toVector3d(str);
}

Eigen::Isometry3d getValueIsometry3d(tinyxml2::XMLElement* _parentElement, const std::string& _name)
{
    assert(_parentElement != nullptr);
    assert(!_name.empty());

    std::string str = _parentElement->FirstChildElement(_name.c_str())->GetText();

    return toIsometry3d(str);
}

Eigen::Isometry3d getValueIsometry3dWithExtrinsicRotation(tinyxml2::XMLElement* _parentElement, const std::string& _name)
{
    assert(_parentElement != nullptr);
    assert(!_name.empty());

    std::string str = _parentElement->FirstChildElement(_name.c_str())->GetText();

    return toIsometry3dWithExtrinsicRotation(str);
}

bool hasElement(tinyxml2::XMLElement* _parentElement, const std::string& _name)
{
    assert(_parentElement != nullptr);
    assert(!_name.empty());

    return _parentElement->FirstChildElement(_name.c_str()) == nullptr ? false : true;
}

tinyxml2::XMLElement* getElement(tinyxml2::XMLElement* _parentElement,
                                 const std::string& _name)
{
    assert(!_name.empty());

    return _parentElement->FirstChildElement(_name.c_str());
}

//==============================================================================
void openXMLFile(
    tinyxml2::XMLDocument& doc, const common::Uri& uri,
    const common::ResourceRetrieverPtr& _retriever)
{
  common::ResourceRetrieverPtr retriever;
  if(_retriever)
    retriever = _retriever;
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
  const size_t size = resource->getSize();
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

bool hasAttribute(tinyxml2::XMLElement* element, const char* const name)
{
  const char* const result = element->Attribute(name);
  return result != 0;
}

std::string getAttribute(tinyxml2::XMLElement * element,
                                const char* const name)
{
    const char* const result = element->Attribute(name);
    if( result == 0 )
    {
        std::ostringstream oss;
        oss << "Missing attribute " << name << " on " << element->Name();
        throw std::runtime_error(oss.str());
    }
    return std::string(result);
}

void getAttribute(tinyxml2::XMLElement* element,
                         const char* const name,
                         double* d)
{
    int result = element->QueryDoubleAttribute(name, d);
    if( result != tinyxml2::XML_SUCCESS )
    {
        std::ostringstream oss;
        oss << "Error parsing double attribute " << name << " on " << element->Name();
        throw std::runtime_error(oss.str());
    }
}

ElementEnumerator::ElementEnumerator(tinyxml2::XMLElement* _parent,
                                     const std::string& _name)
    : m_name(_name),
      m_parent(_parent),
      m_current(nullptr)
{
}

ElementEnumerator::~ElementEnumerator()
{
}

bool ElementEnumerator::valid() const
{
    return m_current != nullptr;
}

bool ElementEnumerator::next()
{
    if(!m_parent)
        return false;

    if(m_current)
        m_current = m_current->NextSiblingElement(m_name.c_str());
    else
        m_current = m_parent->FirstChildElement(m_name.c_str());

    if(!valid())
        m_parent = nullptr;

    return valid();
}

bool ElementEnumerator::operator==(const ElementEnumerator& _rhs) const
{
    // If they point at the same node, then the names must match
    return (this->m_parent == _rhs.m_parent) &&
           (this->m_current == _rhs.m_current) &&
           (this->m_current != 0 || (this->m_name == _rhs.m_name));
}

ElementEnumerator& ElementEnumerator::operator=(const ElementEnumerator& _rhs)
{
    this->m_name = _rhs.m_name;
    this->m_parent = _rhs.m_parent;
    this->m_current = _rhs.m_current;
  return *this;
}

} // namespace utils
} // namespace dart
