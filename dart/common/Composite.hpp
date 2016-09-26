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

#ifndef DART_COMMON_COMPOSITE_HPP_
#define DART_COMMON_COMPOSITE_HPP_

#include "dart/common/detail/CompositeData.hpp"

namespace dart {
namespace common {

/// Composite is a base class that should be virtually inherited by any class
/// that wants to be able to manage Aspects.
///
/// The base Composite class is completely agnostic to what kind of Aspects it
/// is given. Aspects are stored in a std::map, so access to its Aspects happens
/// on average in log(N) time. Most often, a class that accepts Aspects will have
/// certain Aspect types that it will need to access frequently, and it would be
/// beneficial to have constant-time access to those Aspect types. To get
/// constant-time access to specific Aspect types, you can use the templated
/// class SpecializedForAspect.
class Composite
{
public:

  using State = detail::CompositeState;
  using Properties = detail::CompositeProperties;

  using AspectMap = std::map< std::type_index, std::unique_ptr<Aspect> >;
  using RequiredAspectSet = std::unordered_set<std::type_index>;

  template <typename... Aspects>
  using MakeState = detail::MakeCompositeState<Aspects...>;

  template <typename... Aspects>
  using MakeProperties = detail::MakeCompositeProperties<Aspects...>;

  /// Virtual destructor
  virtual ~Composite() = default;

  /// Default constructor
  Composite() = default;

  /// It is currently unsafe to copy an Composite
  // TODO(MXG): Consider making this safe by cloning Aspects into the new copy
  Composite(const Composite&) = delete;

  /// It is currently unsafe to move an Composite
  Composite(Composite&&) = delete;

  /// It is currently unsafe to copy an Composite
  Composite& operator=(const Composite&) = delete;

  /// It is currently unsafe to move an Composite
  Composite& operator=(Composite&&) = delete;

  /// Check if this Composite currently has a certain type of Aspect
  template <class T>
  bool has() const;

  /// Get a certain type of Aspect from this Composite
  template <class T>
  T* get();

  /// Get a certain type of Aspect from this Composite
  template <class T>
  const T* get() const;

  /// Make a clone of the aspect and place the clone into this Composite. If
  /// an Aspect of the same type already exists in this Composite, the
  /// existing Aspect will be destroyed.
  template <class T>
  void set(const T* aspect);

  /// Use move semantics to place aspect into this Composite. If an Aspect of
  /// the same type already exists in this Composite, the existing Aspect will
  /// be destroyed.
  template <class T>
  void set(std::unique_ptr<T>&& aspect);

  /// Construct an Aspect inside of this Composite
  template <class T, typename ...Args>
  T* createAspect(Args&&... args);

  /// Remove an Aspect from this Composite.
  template <class T>
  void removeAspect();

  /// Remove an Aspect from this Composite, but return its unique_ptr instead
  /// of letting it be deleted. This allows you to safely use move semantics to
  /// transfer an Aspect between two Composites.
  template <class T>
  std::unique_ptr<T> releaseAspect();

  /// Check if this Composite is specialized for a specific type of Aspect.
  ///
  /// By default, this simply returns false.
  template <class T>
  static constexpr bool isSpecializedFor();

  /// Check if this Composite requires this specific type of Aspect
  template <class T>
  bool requiresAspect() const;

  /// Set the states of the aspects in this Composite based on the given
  /// Composite::State. The states of any Aspect types that do not exist
  /// within this composite will be ignored.
  void setCompositeState(const State& newStates);

  /// Get the states of the aspects inside of this Composite
  State getCompositeState() const;

  /// Fill outgoingStates with the states of the aspects inside this Composite
  void copyCompositeStateTo(State& outgoingStates) const;

  /// Set the properties of the aspects in this Composite based on the given
  /// Composite::Properties. The properties of any Aspect types that do not
  /// exist within this composite will be ignored.
  void setCompositeProperties(const Properties& newProperties);

  /// Get the properties of the aspects inside of this Composite
  Properties getCompositeProperties() const;

  /// Fill outgoingProperties with the properties of the aspects inside this
  /// Composite
  void copyCompositePropertiesTo(Properties& outgoingProperties) const;

  /// Give this Composite a copy of each Aspect from otherComposite
  void duplicateAspects(const Composite* fromComposite);

  /// Make the Aspects of this Composite match the Aspects of otherComposite. Any
  /// Aspects in this Composite which do not exist in otherComposite will be
  /// erased.
  void matchAspects(const Composite* otherComposite);

protected:

  /// Add this Aspect to the Composite. This allows derived Composite types to
  /// call the protected Aspect::setComposite function.
  void addToComposite(Aspect* aspect);

  /// Remove this Aspect from the Composite. This allows derived Composite types
  /// to call the protected Aspect::loseComposite function.
  void removeFromComposite(Aspect* aspect);

  /// Non-templated version of set(const T*)
  void _set(std::type_index type_idx, const Aspect* aspect);

  /// Non-templated version of set(std::unqiue_ptr<T>&&)
  void _set(std::type_index type_idx, std::unique_ptr<Aspect> aspect);

  /// A map that relates the type of Aspect to its pointer
  AspectMap mAspectMap;

  /// A set containing type information for Aspects which are not allowed to
  /// leave this composite.
  RequiredAspectSet mRequiredAspects;
};

//==============================================================================
/// Attach an arbitrary number of Aspects to the specified Composite type.
// TODO(MXG): Consider putting this functionality into the Composite class
// itself. Also consider allowing the user to specify arguments for the
// constructors of the Aspects.
template <class T>
void createAspects(T* /*comp*/);

//==============================================================================
template <class T, class NextAspect, class... Aspects>
void createAspects(T* comp);

} // namespace common
} // namespace dart

#include "dart/common/detail/Composite.hpp"

#endif // DART_COMMON_COMPOSITE_HPP_
