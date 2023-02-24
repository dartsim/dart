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

#include <tuple>
#include <vector>

namespace dart::common {

/// Template class representing a structure of arrays (SoA)
///
/// @tparam Ts Variadic template parameter pack representing the types of the
/// arrays
template <typename... Ts>
struct SoA
{
  /// Tuple containing the arrays of data
  std::tuple<std::vector<Ts>...> data;

  /// Checks if all arrays are empty
  ///
  /// @return True if all arrays are empty, false otherwise
  [[nodiscard]] bool isEmpty() const;

  /// Returns the size of the arrays
  ///
  /// @return Size of the arrays
  [[nodiscard]] size_t getSize() const;

  /// Returns a const reference to the array at the specified index
  ///
  /// @tparam Index Index of the array
  /// @return Const reference to the array at the specified index
  template <size_t Index>
  [[nodiscard]] const auto& getArray() const;

  /// Returns a reference to the array at the specified index
  ///
  /// @tparam Index Index of the array
  /// @return Reference to the array at the specified index
  template <size_t Index>
  [[nodiscard]] auto& getArray();

  /// Returns a const reference to the array of the specified type
  ///
  /// @tparam T Type of the array
  /// @return Const reference to the array of the specified type
  template <typename T>
  [[nodiscard]] const auto& getArray() const;

  /// Returns a reference to the array of the specified type
  ///
  /// @tparam T Type of the array
  /// @return Reference to the array of the specified type
  template <typename T>
  [[nodiscard]] auto& getArray();

  /// Returns a const reference to the element at the specified index in the
  /// array at the specified index
  ///
  /// @tparam Index Index of the array
  /// @param i Index of the element
  /// @return Const reference to the element at the specified index in the array
  /// at the specified index
  template <size_t Index>
  [[nodiscard]] const auto& get(size_t i) const;

  /// Returns a reference to the element at the specified index in the array at
  /// the specified index
  ///
  /// @tparam Index Index of the array
  /// @param i Index of the element
  /// @return Reference to the element at the specified index in the array at
  /// the specified index
  template <size_t Index>
  [[nodiscard]] auto& get(size_t i);

  /// Returns a const reference to the element at the specified index in the
  /// array of the specified type
  ///
  /// @tparam T Type of the array
  /// @param i Index of the element
  /// @return Const reference to the element at the specified index in the array
  /// of the specified type
  template <typename T>
  [[nodiscard]] const T& get(size_t i) const;

  /// Returns a reference to the element at the specified index in the array of
  /// the specified type
  ///
  /// @tparam T Type of the array
  /// @param i Index of the element
  /// @return Reference to the element at the specified index in the array of
  /// the specified type
  template <typename T>
  [[nodiscard]] T& get(size_t i);

  /// Increases the size of all arrays
  ///
  /// @param N Number of elements to increase the size by
  void increaseSizeAll(size_t N);

  /// Swaps the elements at the specified indices in all arrays
  ///
  /// @param i Index of the first element
  /// @param j Index of the second element
  void swapElementsAll(size_t i, size_t j);

private:
  template <typename T>
  static constexpr size_t getIndex();

  template <typename T>
  [[nodiscard]] const std::vector<T>& dataFor() const;

  template <typename T>
  [[nodiscard]] std::vector<T>& dataFor();

  template <typename T, size_t... Is>
  void increaseSizeImpl(
      std::vector<T>& v, size_t N, std::index_sequence<Is...>);

  template <size_t... Is>
  void increaseSizeAllImpl(std::index_sequence<Is...>, size_t N);

  template <typename T, size_t... Is>
  void swapElementsImpl(
      std::vector<T>& v, size_t i, size_t j, std::index_sequence<Is...>);
};

} // namespace dart::common

//==============================================================================
// Implementation
//==============================================================================

#include <dart/common/Macros.hpp>

namespace dart::common {

//==============================================================================
template <typename... Ts>
bool SoA<Ts...>::isEmpty() const
{
  return std::get<0>(data).empty();
}

//==============================================================================
template <typename... Ts>
size_t SoA<Ts...>::getSize() const
{
  return std::get<0>(data).size();
}

//==============================================================================
template <typename... Ts>
template <size_t Index>
const auto& SoA<Ts...>::getArray() const
{
  return std::get<Index>(data);
}

//==============================================================================
template <typename... Ts>
template <size_t Index>
auto& SoA<Ts...>::getArray()
{
  return std::get<Index>(data);
}

//==============================================================================
template <typename... Ts>
template <typename T>
const auto& SoA<Ts...>::getArray() const
{
  return std::get<std::vector<T>>(data);
}

//==============================================================================
template <typename... Ts>
template <typename T>
auto& SoA<Ts...>::getArray()
{
  return std::get<std::vector<T>>(data);
}

//==============================================================================
template <typename... Ts>
template <size_t Index>
const auto& SoA<Ts...>::get(size_t i) const
{
  return std::get<Index>(data)[i];
}

//==============================================================================
template <typename... Ts>
template <size_t Index>
auto& SoA<Ts...>::get(size_t i)
{
  return std::get<Index>(data)[i];
}

//==============================================================================
template <typename... Ts>
template <typename T>
const T& SoA<Ts...>::get(size_t i) const
{
  return std::get<std::vector<T>>(data)[i];
}

//==============================================================================
template <typename... Ts>
template <typename T>
T& SoA<Ts...>::get(size_t i)
{
  return std::get<std::vector<T>>(data)[i];
}

//==============================================================================
template <typename... Ts>
void SoA<Ts...>::increaseSizeAll(size_t N)
{
  increaseSizeAllImpl(std::make_index_sequence<sizeof...(Ts)>{}, N);
}

//==============================================================================
template <typename... Ts>
void SoA<Ts...>::swapElementsAll(size_t i, size_t j)
{
  (swapElementsImpl(
       std::get<getIndex<Ts>()>(data),
       i,
       j,
       std::make_index_sequence<sizeof...(Ts)>{}),
   ...);
}

//==============================================================================
template <typename... Ts>
template <typename T>
constexpr size_t SoA<Ts...>::getIndex()
{
  static_assert((std::is_same_v<Ts, T> || ...), "Type not in SoA");
  size_t index = 0;
  ((std::is_same_v<Ts, T> ? true : (++index)), ...);
  return index;
}

//==============================================================================
template <typename... Ts>
template <typename T>
const std::vector<T>& SoA<Ts...>::dataFor() const
{
  return std::get<std::vector<T>>(data);
}

//==============================================================================
template <typename... Ts>
template <typename T>
std::vector<T>& SoA<Ts...>::dataFor()
{
  return std::get<std::vector<T>>(data);
}

//==============================================================================
template <typename... Ts>
template <typename T, size_t... Is>
void SoA<Ts...>::increaseSizeImpl(
    std::vector<T>& v, size_t N, std::index_sequence<Is...>)
{
  v.resize(v.size() + N);
}

//==============================================================================
template <typename... Ts>
template <size_t... Is>
void SoA<Ts...>::increaseSizeAllImpl(std::index_sequence<Is...>, size_t N)
{
  (increaseSizeImpl(
       std::get<Is>(data), N, std::make_index_sequence<sizeof...(Ts)>{}),
   ...);
}

//==============================================================================
template <typename... Ts>
template <typename T, size_t... Is>
void SoA<Ts...>::swapElementsImpl(
    std::vector<T>& v, size_t i, size_t j, std::index_sequence<Is...>)
{
  (void)v;
  ((std::swap(std::get<Is>(data)[i], std::get<Is>(data)[j])), ...);
}

} // namespace dart::common
