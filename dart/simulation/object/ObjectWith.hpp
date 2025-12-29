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

#ifndef DART_SIMULATION_OBJECT_OBJECTWITH_HPP_
#define DART_SIMULATION_OBJECT_OBJECTWITH_HPP_

#include <dart/simulation/object/Object.hpp>
#include <dart/simulation/object/TypeList.hpp>

namespace dart::simulation::object {

template <typename... Tags>
struct TagComps
{
  using type = TypeList<Tags...>;
};

template <typename... Comps>
struct ReadOnlyComps
{
  using type = TypeList<Comps...>;
};

template <typename... Comps>
struct WriteOnlyComps
{
  using type = TypeList<Comps...>;
};

template <typename... Comps>
struct ReadWriteComps
{
  using type = TypeList<Comps...>;
};

template <
    typename Tags = TagComps<>,
    typename ReadOnly = ReadOnlyComps<>,
    typename WriteOnly = WriteOnlyComps<>,
    typename ReadWrite = ReadWriteComps<>>
class ObjectWith : public virtual Object
{
public:
  using TagList = typename Tags::type;
  using ReadOnlyList = typename ReadOnly::type;
  using WriteOnlyList = typename WriteOnly::type;
  using ReadWriteList = typename ReadWrite::type;

  ObjectWith() = default;
  virtual ~ObjectWith() = default;

  template <typename Component>
  const Component& getReadOnly() const;

  template <typename Component>
  Component& getMutable();

  template <typename Component>
  const Component* tryGetReadOnly() const;

  template <typename Component>
  Component* tryGetMutable();

  template <typename Component>
  Component* getCacheMutable() const;
};

} // namespace dart::simulation::object

#include <dart/simulation/object/ObjectWith-impl.hpp>

#endif // DART_SIMULATION_OBJECT_OBJECTWITH_HPP_
