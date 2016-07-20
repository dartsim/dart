/*
 * Copyright (c) 2015-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2015-2016, Humanoid Lab, Georgia Tech Research Corporation
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

#ifndef DART_COMMON_CLONEABLE_HPP_
#define DART_COMMON_CLONEABLE_HPP_

#include <memory>
#include <vector>

namespace dart {
namespace common {

//==============================================================================
/// Cloneable is a CRTP base class that provides an interface for easily
/// creating data structures that are meant to be extended. Ordinary copying
/// does not work with extended data structures, because information from the
/// derived classes will typically be lost during a copy. The Cloneable class
/// provides an interface for creating a new copy of an extended data structure,
/// as well as copying information between two extended data structures of the
/// same type.
template <class T>
class Cloneable
{
public:

  /// Default constructor
  Cloneable() = default;

  /// Virtual destructor
  virtual ~Cloneable() = default;

  /// Do not copy this class directly, use clone() or copy() instead
  Cloneable(const Cloneable& doNotCopy) = delete;

  /// Do not copy this class directly, use clone() or copy() instead
  Cloneable& operator=(const Cloneable& doNotCopy) = delete;

  /// Implement this function to allow your Cloneable type to be copied safely.
  virtual std::unique_ptr<T> clone() const = 0;

  /// Copy the contents of anotherCloneable into this one.
  virtual void copy(const T& anotherCloneable) = 0;
};

//==============================================================================
/// The MakeCloneable class is used to easily create an Cloneable (such as
/// Node::State) which simply takes an existing class (Mixin) and creates an
/// Cloneable that wraps it. This creates all the appropriate copy, move, and
/// clone members, allowing you to follow the Rule Of Zero. You can also
/// construct an instance in the exact same way that you would construct a Mixin
/// instance.
template <class Base, class Mixin>
class MakeCloneable : public Base, public Mixin
{
public:

  using Data = Mixin;

  /// Default constructor. Uses the default constructor of Mixin
  MakeCloneable();

  /// Templated constructor. Uses whichever Mixin constructor is able to match
  /// the arguments.
  template <typename... Args>
  MakeCloneable(Args&&... args);

  /// Constructs using a Mixin instance
  MakeCloneable(const Mixin& mixin);

  /// Constructs using a Mixin rvalue
  MakeCloneable(Mixin&& mixin);

  /// Copy constructor
  MakeCloneable(const MakeCloneable<Base, Mixin>& other);

  /// Move constructor
  MakeCloneable(MakeCloneable<Base, Mixin>&& other);

  /// Copy assignment operator that uses a Mixin instance
  MakeCloneable& operator=(const Mixin& mixin);

  /// Move assignment operator that uses a Mixin rvalue
  MakeCloneable& operator=(Mixin&& mixin);

  /// Copy assignment operator
  MakeCloneable& operator=(const MakeCloneable& other);

  /// Move assignment operator
  MakeCloneable& operator=(MakeCloneable&& other);

  // Documentation inherited
  std::unique_ptr<Base> clone() const override final;

  // Documentation inherited
  void copy(const Base& other) override final;

};

//==============================================================================
template <class Base, class OwnerT, class DataT,
          void (*setData)(OwnerT*, const DataT&),
          DataT (*getData)(const OwnerT*)>
class ProxyCloneable : public Base
{
public:

  using Data = DataT;
  using Owner = OwnerT;

  /// Default constructor. Constructs a default version of Data.
  ProxyCloneable();

  /// Constructor that ties this instance to an Owner.
  ProxyCloneable(OwnerT* owner);

  /// Constructor that ties this instance to an Owner and then sets its data
  /// using the remaining arguments.
  template <typename... Args>
  ProxyCloneable(OwnerT* owner, Args&&... args);

  /// Templated constructor. Uses whichever Data constructor is able to match
  /// the arguments.
  template <typename... Args>
  ProxyCloneable(Args&&... args);

  /// Copy constructor
  ProxyCloneable(const ProxyCloneable& other);

  /// Move constructor
  ProxyCloneable(ProxyCloneable&& other);

  /// Copy assignment operator that uses a Data instance
  ProxyCloneable& operator=(const Data& data);

  /// Move assignment operator that uses a Data instance
  ProxyCloneable& operator=(Data&& other);

  /// Copy assignment operator
  ProxyCloneable& operator=(const ProxyCloneable& other);

  /// Move assignment operator
  ProxyCloneable& operator=(ProxyCloneable&& other);

  /// Set the Data of this ProxyCloneable
  void set(const Data& data);

  /// Set the Data of this ProxyCloneable
  void set(Data&& data);

  /// Set the Data of this ProxyCloneable based on another
  void set(const ProxyCloneable& other);

  /// Set the Data of this ProxyCloneable based on another
  void set(ProxyCloneable&& other);

  /// Get the Data of this ProxyCloneable
  Data get() const;

  /// Get the Owner of this ProxyCloneable
  OwnerT* getOwner();

  /// Get the Owner of this ProxyCloneable
  const OwnerT* getOwner() const;

  // Documentation inherited
  std::unique_ptr<Base> clone() const override final;

  // Documentation inherited
  void copy(const Base& other) override final;

protected:

  /// The ProxyCloneable will not store any data in mData if it has an Owner.
  OwnerT* mOwner;

  /// The ProxyCloneable will hold data in this field if it does not have an
  /// owner
  std::unique_ptr<Data> mData;

};

//==============================================================================
/// MapHolder is a templated wrapper class that is used to allow maps of
/// Aspect::State and Aspect::Properties to be handled in a semantically
/// palatable way.
template <typename MapType>
class CloneableMap
{
public:

  /// Default constructor
  CloneableMap() = default;

  /// Copy constructor
  CloneableMap(const CloneableMap& otherStates);

  /// Move constructor
  CloneableMap(CloneableMap&& otherStates);

  /// Map-based constructor
  CloneableMap(const MapType& otherMap);

  /// Map-based move constructor
  CloneableMap(MapType&& otherMap);

  /// Assignment operator
  CloneableMap& operator=(const CloneableMap& otherStates);

  /// Move assignment operator
  CloneableMap& operator=(CloneableMap&& otherStates);

  /// Map-based assignment operator
  CloneableMap& operator=(const MapType& otherMap);

  /// Map-based move assignment operator
  CloneableMap& operator=(MapType&& otherMap);

  /// Copy the contents of another cloneable map into this one. If merge is set
  /// to false, then any fields in this map which are not present in the other
  /// map will be erased; otherwise they will be kept. Setting merge to false
  /// will make the contents of this map an exact duplicate of the other map.
  void copy(const CloneableMap& otherMap, bool merge=false);

  /// Copy the contents of a map into this one. If merge is set to false, then
  /// any fields in this map which are not present in the other map will be
  /// erased; otherwise they will be kept. Setting merge to false will make the
  /// contents of this map an exact duplicate of the other map.
  void copy(const MapType& otherMap, bool merge=false);

  /// Merge the contents of another cloneable map into this one. If there are
  /// any entries which both maps have, then the contents of otherMap will take
  /// precedence. This is the same as calling copy(otherMap, true).
  void merge(const CloneableMap& otherMap);

  /// Merge the contents of another map into this one. If there are any entries
  /// which both maps have, then the contents of otherMap will take precedence.
  /// This is the same as calling copy(otherMap, true).
  void merge(const MapType& otherMap);

  /// Get the map that is being held
  MapType& getMap();

  /// Get the map that is being held
  const MapType& getMap() const;

protected:

  /// A map containing the collection of States for the Aspect
  MapType mMap;
};

//==============================================================================
/// The CloneableVector type wraps a std::vector of an Cloneable type allowing
/// it to be handled by an CloneableMapHolder
template <typename T>
class CloneableVector
{
public:

  /// Default constructor
  CloneableVector() = default;

  /// Construct from a regular vector
  CloneableVector(const std::vector<T>& regularVector);

  /// Construct from a regular vector using move semantics
  CloneableVector(std::vector<T>&& regularVector);

  /// Construct this vector empty and then perform copy(other)
  CloneableVector(const CloneableVector& other);

  /// Call copy(other) on this vector
  CloneableVector& operator=(const CloneableVector& other);

  /// Create a copy of this CloneableVector's contents
  std::unique_ptr< CloneableVector<T> > clone() const;

  /// Copy the contents of another cloneable vector into this one.
  void copy(const CloneableVector<T>& anotherVector);

  /// Get a reference to the std::vector that this class is wrapping
  std::vector<T>& getVector();

  /// Get a reference to the std::vector that this class is wrapping
  const std::vector<T>& getVector() const;

private:

  /// The std::vector that this class is wrapping
  std::vector<T> mVector;
};

} // namespace common
} // namespace dart

#include "dart/common/detail/Cloneable.hpp"

#endif // DART_COMMON_CLONEABLE_HPP_
