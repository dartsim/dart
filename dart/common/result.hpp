/*
 * Copyright (c) 2011, The DART development contributors
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

#include <dart/common/exception.hpp>

#include <optional>
#include <source_location>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>

namespace dart::common {

/// Error information carrying message and source location.
struct Error
{
  std::string message;
  std::source_location location;

  Error(
      std::string msg,
      std::source_location loc = std::source_location::current())
    : message(std::move(msg)), location(loc)
  {
  }
};

namespace detail {

struct OkTag
{
};
struct ErrTag
{
};

} // namespace detail

/// Result type for operations that may fail with error information.
///
/// Use Result when:
/// - Failure is expected and should be handled by caller
/// - You want to return error context without exceptions
/// - The operation may fail for recoverable reasons
///
/// Use exceptions (DART_THROW) when:
/// - Failure indicates programmer error (null pointer, out of range)
/// - Failure is unexpected and typically not recoverable
template <typename T, typename E = Error>
class [[nodiscard]] Result
{
public:
  using ValueType = T;
  using ErrorType = E;

  template <typename U>
  Result(detail::OkTag, U&& value)
    : m_storage(std::in_place_index<0>, std::forward<U>(value))
  {
  }

  template <typename U>
  Result(detail::ErrTag, U&& error)
    : m_storage(std::in_place_index<1>, std::forward<U>(error))
  {
  }

  template <typename U = T>
  static Result ok(U&& value)
  {
    return Result(detail::OkTag{}, std::forward<U>(value));
  }

  template <typename U = E>
  static Result err(U&& error)
  {
    return Result(detail::ErrTag{}, std::forward<U>(error));
  }

  [[nodiscard]] bool isOk() const noexcept
  {
    return m_storage.index() == 0;
  }

  [[nodiscard]] bool isErr() const noexcept
  {
    return m_storage.index() == 1;
  }

  [[nodiscard]] explicit operator bool() const noexcept
  {
    return isOk();
  }

  [[nodiscard]] T& value() &
  {
    if (isErr()) {
      DART_THROW_T(
          InvalidOperationException,
          "Attempted to access value of Result in error state");
    }
    return std::get<0>(m_storage);
  }

  [[nodiscard]] const T& value() const&
  {
    if (isErr()) {
      DART_THROW_T(
          InvalidOperationException,
          "Attempted to access value of Result in error state");
    }
    return std::get<0>(m_storage);
  }

  [[nodiscard]] T&& value() &&
  {
    if (isErr()) {
      DART_THROW_T(
          InvalidOperationException,
          "Attempted to access value of Result in error state");
    }
    return std::move(std::get<0>(m_storage));
  }

  [[nodiscard]] E& error() &
  {
    if (isOk()) {
      DART_THROW_T(
          InvalidOperationException,
          "Attempted to access error of Result in ok state");
    }
    return std::get<1>(m_storage);
  }

  [[nodiscard]] const E& error() const&
  {
    if (isOk()) {
      DART_THROW_T(
          InvalidOperationException,
          "Attempted to access error of Result in ok state");
    }
    return std::get<1>(m_storage);
  }

  [[nodiscard]] T valueOr(T defaultValue) const&
  {
    if (isOk()) {
      return std::get<0>(m_storage);
    }
    return defaultValue;
  }

  [[nodiscard]] T valueOr(T defaultValue) &&
  {
    if (isOk()) {
      return std::move(std::get<0>(m_storage));
    }
    return defaultValue;
  }

  [[nodiscard]] T* operator->()
  {
    return &value();
  }

  [[nodiscard]] const T* operator->() const
  {
    return &value();
  }

  [[nodiscard]] T& operator*() &
  {
    return value();
  }

  [[nodiscard]] const T& operator*() const&
  {
    return value();
  }

  [[nodiscard]] T&& operator*() &&
  {
    return std::move(*this).value();
  }

  template <typename F>
  [[nodiscard]] auto map(
      F&& f) const& -> Result<decltype(f(std::declval<const T&>())), E>
  {
    using U = decltype(f(std::declval<const T&>()));
    if (isOk()) {
      return Result<U, E>::ok(f(std::get<0>(m_storage)));
    }
    return Result<U, E>::err(std::get<1>(m_storage));
  }

  template <typename F>
  [[nodiscard]] auto map(F&& f) && -> Result<decltype(f(std::declval<T>())), E>
  {
    using U = decltype(f(std::declval<T>()));
    if (isOk()) {
      return Result<U, E>::ok(f(std::move(std::get<0>(m_storage))));
    }
    return Result<U, E>::err(std::move(std::get<1>(m_storage)));
  }

  template <typename F>
  [[nodiscard]] auto andThen(
      F&& f) const& -> decltype(f(std::declval<const T&>()))
  {
    using ResultU = decltype(f(std::declval<const T&>()));
    if (isOk()) {
      return f(std::get<0>(m_storage));
    }
    return ResultU::err(std::get<1>(m_storage));
  }

  template <typename F>
  [[nodiscard]] auto andThen(F&& f) && -> decltype(f(std::declval<T>()))
  {
    using ResultU = decltype(f(std::declval<T>()));
    if (isOk()) {
      return f(std::move(std::get<0>(m_storage)));
    }
    return ResultU::err(std::move(std::get<1>(m_storage)));
  }

private:
  std::variant<T, E> m_storage;
};

/// Void specialization for operations that succeed/fail without a value.
template <typename E>
class [[nodiscard]] Result<void, E>
{
public:
  using ValueType = void;
  using ErrorType = E;

  Result() : m_error(std::nullopt) {}

  template <typename U>
  explicit Result(U&& error) : m_error(std::forward<U>(error))
  {
  }

  static Result ok()
  {
    return Result();
  }

  template <typename U = E>
  static Result err(U&& error)
  {
    return Result(std::forward<U>(error));
  }

  [[nodiscard]] bool isOk() const noexcept
  {
    return !m_error.has_value();
  }

  [[nodiscard]] bool isErr() const noexcept
  {
    return m_error.has_value();
  }

  [[nodiscard]] explicit operator bool() const noexcept
  {
    return isOk();
  }

  [[nodiscard]] E& error() &
  {
    if (isOk()) {
      DART_THROW_T(
          InvalidOperationException,
          "Attempted to access error of Result in ok state");
    }
    return *m_error;
  }

  [[nodiscard]] const E& error() const&
  {
    if (isOk()) {
      DART_THROW_T(
          InvalidOperationException,
          "Attempted to access error of Result in ok state");
    }
    return *m_error;
  }

private:
  std::optional<E> m_error;
};

} // namespace dart::common
