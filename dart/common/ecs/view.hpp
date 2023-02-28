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

#pragma once

#include <dart/common/Fwd.hpp>
#include <dart/common/TemplateUtils.hpp>

#include <type_traits>

#include <cstdint>

namespace dart::common {

template <typename EntityManagerT, typename... Components>
class View
{
public:
  using This = View<EntityManagerT, Components...>;
  using EntityManagerType = EntityManagerT;
  using EntityType = typename EntityManagerType::EntityType;
  using iterator = typename std::vector<
      typename EntityManagerType::EntityDataType>::iterator;
  using const_iterator = typename std::vector<
      typename EntityManagerType::EntityDataType>::const_iterator;

  View(EntityManagerType& em) : m_em(em)
  {
    // Empty
  }

  iterator begin()
  {
    return m_entities.begin();
  }

  iterator end()
  {
    return m_entities.end();
  }

  const_iterator begin() const
  {
    return m_entities.cbegin();
  }

  const_iterator end() const
  {
    return m_entities.cend();
  }

  const_iterator cbegin() const
  {
    return m_entities.cbegin();
  }

  const_iterator cend() const
  {
    return m_entities.cend();
  }

  void update()
  {
    m_entities.clear();
  }

  template <typename Func>
  void each(Func&& /*func*/)
  {
    // using traits = FunctionTraits<std::decay_t<Func>>;
    // DART_INFO("# of args: {}", traits::num_args);
    // using FuncType = std::decay_t<Func>;
    //using Arg1Type = typename FunctionTraits<FuncType>::template arg<0>::type;
    // using Arg2Type = typename FunctionTraits<FuncType>::template arg<1>::type;
  }

private:
  EntityManagerType& m_em;
  std::vector<typename EntityManagerType::EntityDataType> m_entities;
};

} // namespace dart::common

//==============================================================================
// Implementation
//==============================================================================

namespace dart::common {

//==============================================================================

} // namespace dart::common
