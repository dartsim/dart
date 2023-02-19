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
#include <type_traits>

namespace dart::common::detail {

/// Returns -1 if the given value is negative, and 1 otherwise.
///
/// @param t The input value.
/// @return -1 if the input value is negative, and 1 otherwise.
template <typename T>
int sign(T t)
{
  return (t < T(0)) ? -1 : 1;
}

/// An iterator that yields a sequence of constant values of a given type.
template <typename T>
class ConstRangeIterator
{
public:
  using value_type = const T;
  using difference_type = std::ptrdiff_t;
  using reference = const T&;
  using pointer = const T*;
  using iterator_category = std::input_iterator_tag;

  /// Constructs an iterator with the given current value and step size.
  explicit ConstRangeIterator(T current, T step)
    : m_current(current), m_step(step)
  {
    /// Empty
  }

  /// Returns true if this iterator is equal to the other iterator, false
  /// otherwise.
  [[nodiscard]] bool operator==(const ConstRangeIterator& other) const
  {
    return m_current == other.m_current;
  }

  /// Returns true if this iterator is not equal to the other iterator, false
  /// otherwise.
  [[nodiscard]] bool operator!=(const ConstRangeIterator& other) const
  {
    return !(*this == other);
  }

  /// Returns a reference to the current value of this iterator.
  [[nodiscard]] reference operator*() const
  {
    return m_current;
  }

  /// Advances this iterator by one step and returns a reference to this
  /// iterator.
  ConstRangeIterator& operator++()
  {
    m_current += m_step;
    return *this;
  }

private:
  T m_current;
  T m_step;
};

/// A proxy object that generates a sequence of constant values of a given type.
template <typename T>
class RangeImpl
{
public:
  using size_type = std::size_t;
  using iterator = ConstRangeIterator<T>;
  using const_iterator = ConstRangeIterator<T>;
  using reference = typename std::add_const<T>::type&;
  using const_reference = typename std::add_const<T>::type&;

  /// Constructs a sequence of values starting from start, ending at end
  /// (exclusive), and stepping by step.
  explicit RangeImpl(T start, T end, T step)
    : m_start(start), m_end(end), m_step(step)
  {
    // Empty
  }

  /// Returns an iterator pointing to the first element of this sequence.
  [[nodiscard]] ConstRangeIterator<T> begin() const
  {
    return ConstRangeIterator<T>(m_start, m_step);
  }

  /// Returns an iterator pointing to the end of this sequence.
  [[nodiscard]] ConstRangeIterator<T> end() const
  {
    const T last(m_end - sign(m_step));
    return ConstRangeIterator<T>(last - (last - m_start) % m_step + m_step, 0);
  }

private:
  const T m_start;
  const T m_end;
  const T m_step;
};

/// Provides type traits for containers.
template <typename Container>
struct Traits
{
  /// Alias for the container's size type.
  using size_type = typename Container::size_type;

  /// Alias for the container's reference type, const or non-const
  /// depending on the container's constness.
  using reference = typename std::conditional<
      std::is_const<Container>::value,
      typename Container::const_reference,
      typename Container::reference>::type;

  /// Alias for the container's iterator type, const or non-const
  /// depending on the container's constness.
  using iterator = typename std::conditional<
      std::is_const<Container>::value,
      typename Container::const_iterator,
      typename Container::iterator>::type;
};

/// An iterator that enumerates the elements in a container.
template <typename Container>
struct EnumerateIterator
{
public:
  /// Alias for the pair type of the element index and element reference.
  using reference = std::pair<size_t, typename Traits<Container>::reference>;

  /// Input iterator tag.
  using iterator_category = std::input_iterator_tag;

  /// Value type of the iterator.
  using value_type = reference;

  /// Difference type of the iterator.
  using difference_type = size_t;

  /// Pointer to the value type of the iterator.
  using pointer = value_type*;

  /// Constructs an EnumerateIterator from an iterator and a start index.
  explicit EnumerateIterator(
      typename Traits<Container>::iterator it, size_t start = 0)
    : m_it(it), m_cnt(start)
  {
    // Empty
  }

  /// Compares the iterator with another iterator for equality.
  [[nodiscard]] bool operator==(const EnumerateIterator& other) const
  {
    return m_it == other.m_it;
  }

  /// Compares the iterator with another iterator for inequality.
  [[nodiscard]] bool operator!=(const EnumerateIterator& other) const
  {
    return !(*this == other);
  }

  /// Advances the iterator to the next element and updates the element
  /// index.
  EnumerateIterator& operator++()
  {
    ++m_it;
    ++m_cnt;
    return *this;
  }

  /// Dereferences the iterator and returns a reference to the enumerated
  /// element.
  [[nodiscard]] reference operator*() const
  {
    return reference(m_cnt, *m_it);
  }

private:
  typename Traits<Container>::iterator
      m_it;     ///< The underlying container iterator.
  size_t m_cnt; ///< The current element index.
};

/// A proxy for enumerating the elements in a container.
template <typename Container>
struct EnumerateImpl
{
private:
  using type = typename std::remove_reference<Container>::type;

public:
  /// Alias for the size type of the container.
  using size_type = size_t;

  /// Alias for the enumerated iterator type of the container.
  using iterator = EnumerateIterator<type>;

  /// Alias for the const enumerated iterator type of the container.
  using const_iterator = EnumerateIterator<typename std::add_const<type>::type>;

  /// Alias for the reference type of the container, const or non-const
  /// depending on the container's constness.
  using reference = type&;

  /// Alias for the const reference type of the container.
  using const_reference = typename std::add_const<type>::type&;

  /// Construct an EnumerateProxy from a container and a starting index.
  ///
  /// @param[in] data The container to be enumerated.
  /// @param[in] start The starting index for the enumeration.
  template <typename T>
  explicit EnumerateImpl(T&& data, size_t start = 0)
    : m_container(std::forward<T>(data)), m_start(start)
  {
    // Empty
  }

  /// Returns an iterator to the first enumerated element of the container.
  ///
  /// @return An iterator to the first enumerated element.
  [[nodiscard]] EnumerateIterator<type> begin()
  {
    return EnumerateIterator<type>(m_container.begin(), m_start);
  }

  /// Returns an iterator to the end of the enumeration.
  ///
  /// @return An iterator to the end of the enumeration.
  [[nodiscard]] EnumerateIterator<type> end()
  {
    return EnumerateIterator<type>(m_container.end(), -1);
  }

private:
  typename std::conditional<
      std::is_rvalue_reference<Container&&>::value,
      type,
      type&>::type m_container; ///< The container to be enumerated.
  size_t m_start;               ///< The starting index for the enumeration.
};

} // namespace dart::common::detail
