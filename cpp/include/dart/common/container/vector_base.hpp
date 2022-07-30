/*
 * Copyright (c) 2011-2022, The DART development contributors:
 * https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include "dart/common/container/vector.hpp"
#include "dart/common/memory_allocator/memory_manager.hpp"

namespace dart::common {

template <typename BaseT>
class ArrayForBasePtr
{
public:
  using BasePtr = BaseT*;
  using ConstBasePtr = const BaseT*;

  ArrayForBasePtr() = default;

  virtual ~ArrayForBasePtr() = default;

  virtual void push_back(BasePtr pointer) = 0;

  virtual void erase(BasePtr pointer) = 0;

  virtual int size() const = 0;

  virtual bool empty() const = 0;

  virtual BasePtr get(int index) = 0;

  virtual ConstBasePtr get(int index) const = 0;

  virtual void clear() = 0;
};

template <typename BaseT, typename DerivedT>
class ArrayForDerivedPtr final : public ArrayForBasePtr<BaseT>
{
public:
  using Base = ArrayForBasePtr<BaseT>;

  using BasePtr = typename Base::BasePtr;
  using ConstBasePtr = typename Base::ConstBasePtr;

  using DerivedPtr = DerivedT*;
  using ConstDerivedPtr = const DerivedT*;

  ArrayForDerivedPtr(MemoryAllocator& allocator = MemoryAllocator::GetDefault())
    : m_data(allocator)
  {
    static_assert(
        std::is_base_of_v<BaseT, DerivedT>,
        "BaseT is not the base type of DerivedT");
  }

  void push_back(BasePtr pointer) override
  {
    if (auto casted = dynamic_cast<DerivedPtr>(pointer)) {
      push_back_derived(casted);
    } else {
      DART_DEBUG(
          "Cannot erase a pointer {} because the type is not {}.",
          typeid(DerivedT).name());
    }
  }

  void push_back_derived(DerivedPtr pointer)
  {
    m_data.push_back(pointer);
  }

  void erase(BasePtr pointer) override
  {
    if (auto casted = dynamic_cast<DerivedPtr>(pointer)) {
      erase_derived(casted);
    } else {
      DART_DEBUG(
          "Cannot erase a pointer {} because the type is not {}.",
          reinterpret_cast<void*>(pointer),
          typeid(DerivedT).name());
    }
  }

  void erase_derived(DerivedPtr pointer)
  {
    common::erase(m_data, pointer);
  }

  int size() const override
  {
    return m_data.size();
  }

  bool empty() const override
  {
    return m_data.empty();
  }

  BasePtr get(int index) override
  {
    return get_derived(index);
  }

  ConstBasePtr get(int index) const override
  {
    return get_derived(index);
  }

  DerivedPtr get_derived(int index)
  {
    return m_data[index];
  }

  ConstDerivedPtr get_derived(int index) const
  {
    return m_data[index];
  }

  void clear() override
  {
    m_data.clear();
  }

private:
  common::vector<DerivedPtr> m_data;
};

} // namespace dart::common
