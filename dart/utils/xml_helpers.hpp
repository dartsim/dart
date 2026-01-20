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

#ifndef DART_UTILS_XMLHELPERS_HPP_
#define DART_UTILS_XMLHELPERS_HPP_

#include <dart/utils/export.hpp>

#include <dart/math/geometry.hpp>
#include <dart/math/math_types.hpp>

#include <dart/common/logging.hpp>
#include <dart/common/resource_retriever.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tinyxml2.h>

#include <string>
#include <string_view>

namespace dart {
namespace utils {

DART_UTILS_API std::string toString(bool v);
DART_UTILS_API std::string toString(int v);
DART_UTILS_API std::string toString(unsigned int v);
DART_UTILS_API std::string toString(float v);
DART_UTILS_API std::string toString(double v);
DART_UTILS_API std::string toString(char v);
template <typename S, int N>
std::string toString(const Eigen::Matrix<S, N, 1>& v);
template <typename S>
std::string toString(
    const Eigen::Transform<S, 3, Eigen::Isometry>& v,
    std::string_view rotationType = "intrinsic");

DART_UTILS_API bool toBool(std::string_view str);
DART_UTILS_API int toInt(std::string_view str);
DART_UTILS_API unsigned int toUInt(std::string_view str);
DART_UTILS_API float toFloat(std::string_view str);
DART_UTILS_API double toDouble(std::string_view str);
DART_UTILS_API char toChar(std::string_view str);
DART_UTILS_API Eigen::Vector2d toVector2d(std::string_view str);
DART_UTILS_API Eigen::Vector2i toVector2i(std::string_view str);
DART_UTILS_API Eigen::Vector3d toVector3d(std::string_view str);
DART_UTILS_API Eigen::Vector3i toVector3i(std::string_view str);
DART_UTILS_API Eigen::Vector4d toVector4d(std::string_view str);
DART_UTILS_API Eigen::Vector6d toVector6d(std::string_view str);
DART_UTILS_API Eigen::VectorXd toVectorXd(std::string_view str);
template <std::size_t N>
Eigen::Matrix<double, N, 1> toVectorNd(std::string_view str);
// TODO: The definition of str is not clear for transform (see: #250)
DART_UTILS_API Eigen::Isometry3d toIsometry3d(std::string_view str);
DART_UTILS_API Eigen::Isometry3d toIsometry3dWithExtrinsicRotation(
    std::string_view str);

DART_UTILS_API std::string getValueString(
    const tinyxml2::XMLElement* parentElement, std::string_view name);
DART_UTILS_API bool getValueBool(
    const tinyxml2::XMLElement* parentElement, std::string_view name);
DART_UTILS_API int getValueInt(
    const tinyxml2::XMLElement* parentElement, std::string_view name);
DART_UTILS_API unsigned int getValueUInt(
    const tinyxml2::XMLElement* parentElement, std::string_view name);
DART_UTILS_API float getValueFloat(
    const tinyxml2::XMLElement* parentElement, std::string_view name);
DART_UTILS_API double getValueDouble(
    const tinyxml2::XMLElement* parentElement, std::string_view name);
DART_UTILS_API char getValueChar(
    const tinyxml2::XMLElement* parentElement, std::string_view name);
DART_UTILS_API Eigen::Vector2d getValueVector2d(
    const tinyxml2::XMLElement* parentElement, std::string_view name);
DART_UTILS_API Eigen::Vector3d getValueVector3d(
    const tinyxml2::XMLElement* parentElement, std::string_view name);
DART_UTILS_API Eigen::Vector3i getValueVector3i(
    const tinyxml2::XMLElement* parentElement, std::string_view name);
DART_UTILS_API Eigen::Vector6d getValueVector6d(
    const tinyxml2::XMLElement* parentElement, std::string_view name);
DART_UTILS_API Eigen::VectorXd getValueVectorXd(
    const tinyxml2::XMLElement* parentElement, std::string_view name);
DART_UTILS_API Eigen::Isometry3d getValueIsometry3d(
    const tinyxml2::XMLElement* parentElement, std::string_view name);
DART_UTILS_API Eigen::Isometry3d getValueIsometry3dWithExtrinsicRotation(
    const tinyxml2::XMLElement* parentElement, std::string_view name);

// TODO(JS): Deprecate
DART_UTILS_API void openXMLFile(
    tinyxml2::XMLDocument& doc,
    const common::Uri& uri,
    const common::ResourceRetrieverPtr& retriever = nullptr);

DART_UTILS_API bool readXmlFile(
    tinyxml2::XMLDocument& doc,
    const common::Uri& uri,
    const common::ResourceRetrieverPtr& retrieverOrNullPtr = nullptr);

DART_UTILS_API bool hasElement(
    const tinyxml2::XMLElement* parentElement, std::string_view name);

DART_UTILS_API const tinyxml2::XMLElement* getElement(
    const tinyxml2::XMLElement* parentElement, std::string_view name);

DART_UTILS_API tinyxml2::XMLElement* getElement(
    tinyxml2::XMLElement* parentElement, std::string_view name);

DART_UTILS_API bool hasAttribute(
    const tinyxml2::XMLElement* element, const char* const name);

DART_UTILS_API std::string getAttributeString(
    const tinyxml2::XMLElement* element, std::string_view attributeName);
DART_UTILS_API bool getAttributeBool(
    const tinyxml2::XMLElement* element, std::string_view attributeName);
DART_UTILS_API int getAttributeInt(
    const tinyxml2::XMLElement* element, std::string_view attributeName);
DART_UTILS_API unsigned int getAttributeUInt(
    const tinyxml2::XMLElement* element, std::string_view attributeName);
DART_UTILS_API float getAttributeFloat(
    const tinyxml2::XMLElement* element, std::string_view attributeName);
DART_UTILS_API double getAttributeDouble(
    const tinyxml2::XMLElement* element, std::string_view attributeName);
DART_UTILS_API char getAttributeChar(
    const tinyxml2::XMLElement* element, std::string_view attributeName);
DART_UTILS_API Eigen::Vector2i getAttributeVector2i(
    const tinyxml2::XMLElement* element, std::string_view attributeName);
DART_UTILS_API Eigen::Vector2d getAttributeVector2d(
    const tinyxml2::XMLElement* element, std::string_view attributeName);
DART_UTILS_API Eigen::Vector3d getAttributeVector3d(
    const tinyxml2::XMLElement* element, std::string_view attributeName);
DART_UTILS_API Eigen::Vector4d getAttributeVector4d(
    const tinyxml2::XMLElement* element, std::string_view attributeName);
DART_UTILS_API Eigen::Vector6d getAttributeVector6d(
    const tinyxml2::XMLElement* element, std::string_view attributeName);
DART_UTILS_API Eigen::VectorXd getAttributeVectorXd(
    const tinyxml2::XMLElement* element, std::string_view attributeName);
template <std::size_t N>
Eigen::Matrix<double, N, 1> getAttributeVectorNd(
    const tinyxml2::XMLElement* element, std::string_view attributeName);

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
      ElementPtr parentElement, std::string_view childElementName);

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

bool copyNode(tinyxml2::XMLNode* destParent, const tinyxml2::XMLNode& src);

bool copyChildNodes(
    tinyxml2::XMLNode* destParent, const tinyxml2::XMLNode& src);

} // namespace utils
} // namespace dart

#include <dart/utils/detail/xml_helpers-impl.hpp>

#endif // #ifndef DART_UTILS_XMLHELPERS_HPP_
