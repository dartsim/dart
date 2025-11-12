/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#pragma once

#include <dart8/common/type_list.hpp>
#include <dart8/ecs/entity_object_base.hpp>

namespace dart8 {

//==============================================================================
// Component access wrapper types for self-documenting class declarations
//==============================================================================

/// Tag component wrapper - marks empty tag-only components
/// Tag components are used for ECS filtering but have no data
template <typename... Tags>
struct TagComps
{
  using type = TypeList<Tags...>;
};

/// Wrapper for components that are only read (never modified)
/// Multiple readers can access these in parallel safely
template <typename... Comps>
struct ReadOnlyComps
{
  using type = TypeList<Comps...>;
};

/// Wrapper for components that are only written to (never read)
/// Rare case - usually for output-only data
template <typename... Comps>
struct WriteOnlyComps
{
  using type = TypeList<Comps...>;
};

/// Wrapper for components that are both read and written
/// Requires exclusive access - cannot run in parallel with other access
template <typename... Comps>
struct ReadWriteComps
{
  using type = TypeList<Comps...>;
};

//==============================================================================
/// EntityObjectWith - Template for compile-time component access validation
///
/// This template adds compile-time validated component access on top of the
/// non-template EntityObject base. It uses virtual inheritance to enable the
/// diamond inheritance pattern (DART 6's SpecializedForAspect pattern).
///
/// Usage Pattern:
/// --------------
/// @code
/// // Simple class with validation
/// class Frame : public EntityObjectWith<
///     TagComps<comps::FrameTag>,
///     ReadOnlyComps<>,
///     WriteOnlyComps<>,
///     ReadWriteComps<comps::FrameKinematics>>
/// {
///     void doWork() {
///         // Validated - FrameKinematics in ReadWrite list
///         auto& kin = getMutable<comps::FrameKinematics>();
///
///         // Compile error - FrameTag only in Tag list, not ReadWrite
///         auto& tag = getMutable<comps::FrameTag>();
///     }
/// };
///
/// // Derived class adds its own component validation
/// class FreeFrame
///     : public Frame,  // Inherit Frame's API
///       public EntityObjectWith<  // Add FreeFrame's validation
///           TagComps<comps::FreeFrameTag>,
///           ReadOnlyComps<>,
///           WriteOnlyComps<>,
///           ReadWriteComps<comps::FreeFrameProperties>>
/// {
///     // Can access both Frame and FreeFrame components!
///     // Virtual inheritance ensures single EntityObject storage
/// };
/// @endcode
///
/// Component Access Rules:
/// -----------------------
/// - TagComps: Tag components (empty structs for ECS filtering)
/// - ReadOnlyComps: Can only use getReadOnly<T>()
/// - WriteOnlyComps: Can only use getMutable<T>()
/// - ReadWriteComps: Can use both getReadOnly<T>() and getMutable<T>()
///
/// @see EntityObject, Frame, FreeFrame, FixedFrame
template <
    typename Tags = TagComps<>,
    typename ReadOnly = ReadOnlyComps<>,
    typename WriteOnly = WriteOnlyComps<>,
    typename ReadWrite = ReadWriteComps<>>
class EntityObjectWith : public virtual EntityObject
{
public:
  // Extract component lists from wrapper types
  using TagList = typename Tags::type;
  using ReadOnlyList = typename ReadOnly::type;
  using WriteOnlyList = typename WriteOnly::type;
  using ReadWriteList = typename ReadWrite::type;

  /// Default constructor
  EntityObjectWith() = default;

  /// Virtual destructor
  virtual ~EntityObjectWith() = default;

  /// Get read-only access to a component
  ///
  /// Component must be in ReadOnlyList or ReadWriteList.
  /// Compile-time error if component not declared.
  ///
  /// @tparam Component Component type to access
  /// @return Const reference to component
  template <typename Component>
  const Component& getReadOnly() const;

  /// Get mutable access to a component
  ///
  /// Component must be in WriteOnlyList or ReadWriteList.
  /// Compile-time error if component not declared.
  ///
  /// @tparam Component Component type to access
  /// @return Mutable reference to component
  template <typename Component>
  Component& getMutable();

  /// Try to get read-only access to a component (may not exist)
  ///
  /// Component must be in ReadOnlyList or ReadWriteList.
  /// Returns nullptr if component doesn't exist on entity.
  ///
  /// @tparam Component Component type to access
  /// @return Pointer to component, or nullptr if not found
  template <typename Component>
  const Component* tryGetReadOnly() const;

  /// Try to get mutable access to a component (may not exist)
  ///
  /// Component must be in WriteOnlyList or ReadWriteList.
  /// Returns nullptr if component doesn't exist on entity.
  ///
  /// @tparam Component Component type to access
  /// @return Pointer to component, or nullptr if not found
  template <typename Component>
  Component* tryGetMutable();

  /// Get mutable access to cache/lazy-evaluation component from const method
  ///
  /// This is specifically for cache fields in const methods that use lazy
  /// evaluation. The const-ness is logical (the observable state doesn't
  /// change), but the cache needs physical mutation.
  ///
  /// **Use cases:**
  /// - Transform caching: `getTransform() const` updates cached world transform
  /// - Jacobian caching: Computed on-demand, stored for reuse
  /// - Any memoization pattern where cache is implementation detail
  ///
  /// **Serialization note:**
  /// Components accessed via getCacheMutable() should typically be:
  /// - Excluded from serialization (cache can be recomputed)
  /// - Or marked with special metadata for serialization code
  ///
  /// @tparam Component The component type (must be in ReadWriteComps<>)
  /// @return Pointer to mutable component or nullptr if not present
  ///
  /// Example:
  /// @code
  /// const Eigen::Isometry3d& Frame::getTransform() const {
  ///   auto* cache = getCacheMutable<comps::FrameKinematics>();
  ///   if (cache->needTransformUpdate) {
  ///     cache->worldTransform = computeTransform(); // Update cache
  ///     cache->needTransformUpdate = false;
  ///   }
  ///   return cache->worldTransform;
  /// }
  /// @endcode
  template <typename Component>
  Component* getCacheMutable() const;
};

} // namespace dart8

#include <dart8/ecs/entity_object_with_impl.hpp>
