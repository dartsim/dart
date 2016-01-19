/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael Koval <mkoval@cs.cmu.edu>
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

#ifndef KIDO_COMMON_RESOURCE_HPP_
#define KIDO_COMMON_RESOURCE_HPP_

#include <cstddef>
#include <memory>

namespace kido {
namespace common {

/// \brief Resource provides file-like access to a resource loaded from URI.
///
/// It is expected that each \a ResourceRetriever will provide a concrete /
/// instantiation of the Resource class. This interface exposes an similar API
/// to that of the the standard C file manipulation functions.
class Resource
{
public:
  /// \brief Position to seek relative to.
  enum SeekType {
    SEEKTYPE_CUR, ///< Current position.
    SEEKTYPE_END, ///< End of file.
    SEEKTYPE_SET  ///< Begining of file.
  };

  virtual ~Resource() = default;

  /// \brief Return the size of the resource, in bytes.
  virtual size_t getSize() = 0;

  /// \brief Return the current value of the position indicator.
  /// \note This method has the same API as the standard ftell function.
  virtual size_t tell() = 0;

  /// \brief Set the position indicator to a new position.
  /// \param[in] _offset Offset, in bytes, relative to _origin.
  /// \param[in] _origin Position used as the reference of _offset.
  /// \note This method has the same API as the standard fseek function.
  virtual bool seek(ptrdiff_t _offset, SeekType _origin) = 0;

  /// \brief Read _count element, each of size _size, into _buffer.
  /// \param[out] _buffer Pointer to a block of memory with a size of at least
  ///                     (_size * _count) bytes.
  /// \param[in] _size Size, in bytes, of each element.
  /// \param[in] _count Number of elements, each of _size bytes.
  /// \note This method has the same API as the standard fread function.
  virtual size_t read(void *_buffer, size_t _size, size_t _count) = 0; 
};

using ResourcePtr = std::shared_ptr<Resource>;

} // namespace common
} // namespace kido

#endif // ifndef KIDO_COMMON_RESOURCE_HPP_
