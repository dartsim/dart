/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#pragma once

#include <string>

#include "dart/math/SmartPointer.hpp"

namespace dart {
namespace math {

class Geometry
{
public:
  /// DataVariance can be used by renderers to determine whether it should
  /// expect data for this geometry to change during each update.
  enum DataVariance
  {
    STATIC = 0, /// No data will ever change
    DYNAMIC_TRANSFORM
    = 1 << 1, /// The relative transform of the Geometry might change
    DYNAMIC_PRIMITIVE = 1 << 2, /// The primitive properties (such as x/y/z
                                /// scaling) of the geometry might change
    DYNAMIC_COLOR
    = 1 << 3, /// The coloring or textures of the geometry might change
    DYNAMIC_VERTICES
    = 1 << 4, /// Vertex positions of a mesh might change (this does not include
              /// adding or removing vertices) (this enum is not relevant for
              /// primitive geometries)
    DYNAMIC_ELEMENTS
    = 1 << 5, /// The number of elements and/or arrangement of elements might
              /// change (this includes adding and removing vertices)  (this
              /// enum is not relevant for primitive geometries)
    DYNAMIC = 0xFF /// All data is subject to changing
  };

  /// Default constructor.
  Geometry();

  /// Destructor.
  virtual ~Geometry();

  /// Returns a string representing the geometry type
  /// \sa is()
  virtual const std::string& getType() const
  {
    static const std::string empty;
    return empty;
  }
  // TODO(JS): Make this a pure virtual function

  /// Returns true if the types of this Geometry and the template parameter (a
  /// geometry class) are identical. This function is a syntactic sugar, which
  /// is identical to: (getType() == GeometryType::getStaticType()).
  ///
  /// Example code:
  /// \code
  /// if (geometry->is<Sphere>())
  ///   std::cout << "The geometry type is sphere!\n";
  /// \endcode
  ///
  /// \sa getType()
  template <typename GeometryType>
  bool is() const;

  template <typename GeometryType>
  const GeometryType* as() const;

  /// Sets the name of this geometry.
  void setName(const std::string& name);

  /// Returns the name of this geometry.
  const std::string& getName() const;

private:
  /// Name of this geometry.
  std::string mName;
};

} // namespace math
} // namespace dart

#include "dart/math/geometry/detail/Geometry-impl.hpp"
