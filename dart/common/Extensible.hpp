/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <mxgrey@gatech.edu>
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

#ifndef DART_COMMON_EXTENSIBLE_HPP_
#define DART_COMMON_EXTENSIBLE_HPP_

#include <memory>
#include <vector>

namespace dart {
namespace common {

/// Extensible is a CRTP base class that provides an interface for easily
/// creating data structures that are meant to be extended. Ordinary copying
/// does not work with extended data structures, because information from the
/// derived classes will typically be lost during a copy. The Extensible class
/// provides an interface for creating a new copy of an extended data structure,
/// as well as copying information between two extended data structures of the
/// same type.
template <class T>
class Extensible
{
public:

  /// Default constructor
  Extensible() = default;

  /// Virtual destructor
  virtual ~Extensible() = default;

  /// Do not copy this class directly, use clone() or copy() instead
  Extensible(const Extensible& doNotCopy) = delete;

  /// Do not copy this class directly, use clone() or copy() instead
  Extensible& operator=(const Extensible& doNotCopy) = delete;

  /// Implement this function to allow your Extensible type to be copied safely.
  virtual std::unique_ptr<T> clone() const = 0;

  /// Copy the contents of anotherExtensible into this one.
  virtual void copy(const T& anotherExtensible) = 0;
};

/// The ExtensibleMixer class is used to easily create an Extensible (such as
/// Node::State) which simply takes an existing class (Mixin) and creates an
/// Extensible that wraps it. This creates all the appropriate copy, move, and
/// clone members, allowing you to follow the Rule Of Zero. You can also
/// construct an instance in the exact same way that you would construct a Mixin
/// instance.
template <class T, class Mixin>
class ExtensibleMixer : public T, public Mixin
{
public:

  /// Default constructor. Uses the default constructor of Mixin
  ExtensibleMixer();

  /// Templated constructor. Uses whichever Mixin constructor is able to match
  /// the arguments.
  template <typename ... Args>
  ExtensibleMixer(Args&&... args);

  /// Constructs using a Mixin instance
  ExtensibleMixer(const Mixin& mixin);

  /// Constructs using a Mixin rvalue
  ExtensibleMixer(Mixin&& mixin);

  /// Copy constructor
  ExtensibleMixer(const ExtensibleMixer<T, Mixin>& other);

  /// Move constructor
  ExtensibleMixer(ExtensibleMixer<T, Mixin>&& other);

  /// Copy assignment operator that uses a Mixin instance
  ExtensibleMixer& operator=(const Mixin& mixin);

  /// Move assignment operator that uses a Mixin rvalue
  ExtensibleMixer& operator=(Mixin&& mixin);

  /// Copy assignment operator
  ExtensibleMixer& operator=(const ExtensibleMixer& other);

  /// Move assignment operator
  ExtensibleMixer& operator=(ExtensibleMixer&& other);

  // Documentation inherited
  std::unique_ptr<T> clone() const override final;

  // Documentation inherited
  void copy(const T& other) override final;

};

/// MapHolder is a templated wrapper class that is used to allow maps of
/// Addon::State and Addon::Properties to be handled in a semantically
/// palatable way.
template <typename MapType>
class ExtensibleMapHolder final
{
public:

  /// Default constructor
  ExtensibleMapHolder() = default;

  /// Copy constructor
  ExtensibleMapHolder(const ExtensibleMapHolder& otherStates);

  /// Move constructor
  ExtensibleMapHolder(ExtensibleMapHolder&& otherStates);

  /// Map-based constructor
  ExtensibleMapHolder(const MapType& otherMap);

  /// Map-based move constructor
  ExtensibleMapHolder(MapType&& otherMap);

  /// Assignment operator
  ExtensibleMapHolder& operator=(const ExtensibleMapHolder& otherStates);

  /// Move assignment operator
  ExtensibleMapHolder& operator=(ExtensibleMapHolder&& otherStates);

  /// Map-based assignment operator
  ExtensibleMapHolder& operator=(const MapType& otherMap);

  /// Map-based move assignment operator
  ExtensibleMapHolder& operator=(MapType&& otherMap);

  /// Get the map that is being held
  MapType& getMap();

  /// Get the map that is being held
  const MapType& getMap() const;

private:

  /// A map containing the collection of States for the Addon
  MapType mMap;
};

/// The ExtensibleVector type wraps a std::vector of an Extensible type allowing
/// it to be handled by an ExtensibleMapHolder
template <typename T>
class ExtensibleVector final
{
public:

  /// Default constructor
  ExtensibleVector() = default;

  /// Construct from a regular vector
  ExtensibleVector(const std::vector<T>& regularVector);

  /// Construct from a regular vector using move semantics
  ExtensibleVector(std::vector<T>&& regularVector);

  /// Do not copy this class directly, use clone() or copy() instead
  ExtensibleVector(const ExtensibleVector& doNotCopy) = delete;

  /// Do not copy this class directly, use clone() or copy() instead
  ExtensibleVector& operator=(const ExtensibleVector& doNotCopy) = delete;

  /// Create a copy of this ExtensibleVector's contents
  std::unique_ptr< ExtensibleVector<T> > clone() const;

  /// Copy the contents of another extensible vector into this one.
  void copy(const ExtensibleVector<T>& anotherVector);

  /// Get a reference to the std::vector that this class is wrapping
  const std::vector<T>& getVector() const;

private:

  /// The std::vector that this class is wrapping
  std::vector<T> mVector;
};

} // namespace common
} // namespace dart

#include "dart/common/detail/Extensible.hpp"

#endif // DART_COMMON_EXTENSIBLE_HPP_
