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

#include <dart/common/Memory.hpp>

#include <vector>

#include <cassert>
#include <cstddef>

namespace dart::common {

/// A helper class for iterating over a range of indices.
///
/// This class is useful for iterating over a range of indices in a for loop.
/// For example, the following code iterates over the indices 0, 1, 2, and 3:
///
/// @code
///   for (auto i : Range(4))
///   {
///     // Do something with i
///   }
/// @endcode
class Range
{
public:
  /// An iterator that iterates over the range of integers.
  class Iterator
  {
  public:
    /// Constructs an iterator for the given index.
    /// @param index The starting index for the iterator.
    Iterator(size_t index) : m_index(index)
    {
      // Empty
    }

    /// Returns the current value of the iterator.
    /// @return The current value of the iterator.
    size_t operator*() const
    {
      return m_index;
    }

    /// Advances the iterator to the next value in the range.
    /// @return A reference to the iterator after it has been advanced.
    Iterator& operator++()
    {
      ++m_index;
      return *this;
    }

    /// Compares two iterators for inequality.
    /// @param other The other iterator to compare against.
    /// @return True if the two iterators are not equal, false otherwise.
    bool operator!=(const Iterator& other) const
    {
      return m_index != other.m_index;
    }

  private:
    size_t m_index;
  };

  /// Constructs a range of integer values from 0 to end-1.
  /// @param end The end of the range (exclusive).
  Range(size_t end) : m_end(end)
  {
    // Empty
  }

  /// Returns an iterator to the beginning of the range.
  /// @return An iterator to the beginning of the range.
  auto begin() const
  {
    return Iterator(0);
  }

  /// Returns an iterator to the end of the range.
  /// @return An iterator to the end of the range.
  auto end() const
  {
    return Iterator(m_end);
  }

private:
  const size_t m_end;
};

/// Returns an object from a vector if the index is valid, otherwise return
/// nullptr.
///
/// @param[in] index Index of the object to be returned.
/// @param[in] vec Vector of objects.
/// @return Object at the given index if the index is valid, otherwise return
/// nullptr.
template <typename T>
T getVectorObjectIfAvailable(std::size_t index, const std::vector<T>& vec);

} // namespace dart::common

//==============================================================================
// Implementation
//==============================================================================

namespace dart::common {

template <typename T>
T getVectorObjectIfAvailable(std::size_t index, const std::vector<T>& vec)
{
  assert(index < vec.size());
  if (index < vec.size())
    return vec[index];

  return nullptr;
}

} // namespace dart::common
