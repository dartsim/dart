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

#ifndef DART_UTILS_XMLHELPERS_HPP_
#define DART_UTILS_XMLHELPERS_HPP_

#include <dart/io/Export.hpp>

#include <dart/math/Geometry.hpp>
#include <dart/math/MathTypes.hpp>

#include <dart/common/Console.hpp>
#include <dart/common/Logging.hpp>
#include <dart/common/ResourceRetriever.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tinyxml2.h>

#include <string>

namespace dart {
namespace io {

DART_IO_API std::string toString(bool v);
DART_IO_API std::string toString(int v);
DART_IO_API std::string toString(unsigned int v);
DART_IO_API std::string toString(float v);
DART_IO_API std::string toString(double v);
DART_IO_API std::string toString(char v);
template <typename S, int N>
std::string toString(const Eigen::Matrix<S, N, 1>& v);
template <typename S>
std::string toString(
    const Eigen::Transform<S, 3, Eigen::Isometry>& v,
    const std::string& rotationType = "intrinsic");

DART_IO_API bool toBool(const std::string& str);
DART_IO_API int toInt(const std::string& str);
DART_IO_API unsigned int toUInt(const std::string& str);
DART_IO_API float toFloat(const std::string& str);
DART_IO_API double toDouble(const std::string& str);
DART_IO_API char toChar(const std::string& str);
DART_IO_API Eigen::Vector2d toVector2d(const std::string& str);
DART_IO_API Eigen::Vector2i toVector2i(const std::string& str);
DART_IO_API Eigen::Vector3d toVector3d(const std::string& str);
DART_IO_API Eigen::Vector3i toVector3i(const std::string& str);
DART_IO_API Eigen::Vector4d toVector4d(const std::string& str);
DART_IO_API Eigen::Vector6d toVector6d(const std::string& str);
DART_IO_API Eigen::VectorXd toVectorXd(const std::string& str);
template <std::size_t N>
Eigen::Matrix<double, N, 1> toVectorNd(const std::string& str);
// TODO: The definition of str is not clear for transform (see: #250)
DART_IO_API Eigen::Isometry3d toIsometry3d(const std::string& str);
DART_IO_API Eigen::Isometry3d toIsometry3dWithExtrinsicRotation(
    const std::string& str);

DART_IO_API std::string getValueString(
    const tinyxml2::XMLElement* parentElement, const std::string& name);
DART_IO_API bool getValueBool(
    const tinyxml2::XMLElement* parentElement, const std::string& name);
DART_IO_API int getValueInt(
    const tinyxml2::XMLElement* parentElement, const std::string& name);
DART_IO_API unsigned int getValueUInt(
    const tinyxml2::XMLElement* parentElement, const std::string& name);
DART_IO_API float getValueFloat(
    const tinyxml2::XMLElement* parentElement, const std::string& name);
DART_IO_API double getValueDouble(
    const tinyxml2::XMLElement* parentElement, const std::string& name);
DART_IO_API char getValueChar(
    const tinyxml2::XMLElement* parentElement, const std::string& name);
DART_IO_API Eigen::Vector2d getValueVector2d(
    const tinyxml2::XMLElement* parentElement, const std::string& name);
DART_IO_API Eigen::Vector3d getValueVector3d(
    const tinyxml2::XMLElement* parentElement, const std::string& name);
DART_IO_API Eigen::Vector3i getValueVector3i(
    const tinyxml2::XMLElement* parentElement, const std::string& name);
DART_IO_API Eigen::Vector6d getValueVector6d(
    const tinyxml2::XMLElement* parentElement, const std::string& name);
DART_IO_API Eigen::VectorXd getValueVectorXd(
    const tinyxml2::XMLElement* parentElement, const std::string& name);
DART_IO_API Eigen::Isometry3d getValueIsometry3d(
    const tinyxml2::XMLElement* parentElement, const std::string& name);
DART_IO_API Eigen::Isometry3d getValueIsometry3dWithExtrinsicRotation(
    const tinyxml2::XMLElement* parentElement, const std::string& name);

// TODO(JS): Deprecate
DART_IO_API void openXMLFile(
    tinyxml2::XMLDocument& doc,
    const common::Uri& uri,
    const common::ResourceRetrieverPtr& retriever = nullptr);

DART_IO_API bool readXmlFile(
    tinyxml2::XMLDocument& doc,
    const common::Uri& uri,
    const common::ResourceRetrieverPtr& retrieverOrNullPtr = nullptr);

DART_IO_API bool hasElement(
    const tinyxml2::XMLElement* parentElement, const std::string& name);

DART_IO_API const tinyxml2::XMLElement* getElement(
    const tinyxml2::XMLElement* parentElement, const std::string& name);

DART_IO_API tinyxml2::XMLElement* getElement(
    tinyxml2::XMLElement* parentElement, const std::string& name);

DART_IO_API bool hasAttribute(
    const tinyxml2::XMLElement* element, const char* const name);

DART_IO_API std::string getAttributeString(
    const tinyxml2::XMLElement* element, const std::string& attributeName);
DART_IO_API bool getAttributeBool(
    const tinyxml2::XMLElement* element, const std::string& attributeName);
DART_IO_API int getAttributeInt(
    const tinyxml2::XMLElement* element, const std::string& attributeName);
DART_IO_API unsigned int getAttributeUInt(
    const tinyxml2::XMLElement* element, const std::string& attributeName);
DART_IO_API float getAttributeFloat(
    const tinyxml2::XMLElement* element, const std::string& attributeName);
DART_IO_API double getAttributeDouble(
    const tinyxml2::XMLElement* element, const std::string& attributeName);
DART_IO_API char getAttributeChar(
    const tinyxml2::XMLElement* element, const std::string& attributeName);
DART_IO_API Eigen::Vector2i getAttributeVector2i(
    const tinyxml2::XMLElement* element, const std::string& attributeName);
DART_IO_API Eigen::Vector2d getAttributeVector2d(
    const tinyxml2::XMLElement* element, const std::string& attributeName);
DART_IO_API Eigen::Vector3d getAttributeVector3d(
    const tinyxml2::XMLElement* element, const std::string& attributeName);
DART_IO_API Eigen::Vector4d getAttributeVector4d(
    const tinyxml2::XMLElement* element, const std::string& attributeName);
DART_IO_API Eigen::Vector6d getAttributeVector6d(
    const tinyxml2::XMLElement* element, const std::string& attributeName);
DART_IO_API Eigen::VectorXd getAttributeVectorXd(
    const tinyxml2::XMLElement* element, const std::string& attributeName);
template <std::size_t N>
Eigen::Matrix<double, N, 1> getAttributeVectorNd(
    const tinyxml2::XMLElement* element, const std::string& attributeName);

/// TemplatedElementEnumerator is a convenience class to help visiting all the
/// child elements of given parent element. This class is templated to cover
/// const and non-const tinyxml2::XMLElement types.
template <typename ElementType>
class TemplatedElementEnumerator
{
protected:
  using ElementPtr = ElementType*;
  using ElementRef = ElementType&;

public:
  /// Constructor that takes parent element and
  TemplatedElementEnumerator(
      ElementPtr parentElement, const std::string& childElementName);

  /// Destructor
  ~TemplatedElementEnumerator();

  /// Set the current element to the next sibling element or to the first child
  /// element of given parent element if it exists; returns success
  bool next();

  /// Get the current element
  ElementPtr get() const;

  /// Dereference operator
  ElementPtr operator->() const;

  /// Dereference operator
  ElementRef operator*() const;

  /// Equality operator
  bool operator==(const TemplatedElementEnumerator<ElementType>& rhs) const;

  /// Assignment operator
  TemplatedElementEnumerator<ElementType>& operator=(
      const TemplatedElementEnumerator<ElementType>& rhs);

private:
  /// Returns true if the current element is valid (not a nullptr)
  bool valid() const;

private:
  /// Parent element
  ElementPtr mParentElement;

  /// Child element name
  std::string mChildElementName;

  /// Currently visiting child element
  ElementPtr mCurrentElement;
};

// ElementEnumerator is for iterating elements for
using ElementEnumerator = TemplatedElementEnumerator<tinyxml2::XMLElement>;
using ConstElementEnumerator
    = TemplatedElementEnumerator<const tinyxml2::XMLElement>;

DART_IO_API bool copyNode(
    tinyxml2::XMLNode* destParent, const tinyxml2::XMLNode& src);

DART_IO_API bool copyChildNodes(
    tinyxml2::XMLNode* destParent, const tinyxml2::XMLNode& src);

} // namespace io
} // namespace dart

#include <dart/io/detail/XmlHelpers-impl.hpp>

#endif // #ifndef DART_UTILS_XMLHELPERS_HPP_
