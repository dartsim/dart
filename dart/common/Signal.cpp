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

#include "dart/common/Signal.hpp"

namespace dart {
namespace common {

//==============================================================================
Connection::Connection()
{
  // Do nothing
}

//==============================================================================
Connection::Connection(const Connection& other)
  : mWeakConnectionBody(other.mWeakConnectionBody)
{
  // Do nothing
}

//==============================================================================
Connection::Connection(Connection&& other)
  : mWeakConnectionBody(std::move(other.mWeakConnectionBody))
{
  // Do nothing
}

//==============================================================================
Connection& Connection::operator=(const Connection& other)
{
  mWeakConnectionBody = other.mWeakConnectionBody;
  return *this;
}

//==============================================================================
Connection& Connection::operator=(Connection&& other)
{
  mWeakConnectionBody = std::move(other.mWeakConnectionBody);
  return *this;
}

//==============================================================================
Connection::Connection(
    const std::weak_ptr<signal::detail::ConnectionBodyBase>& connectionBody)
  : mWeakConnectionBody(connectionBody)
{
  // Do nothing
}

//==============================================================================
Connection::Connection(
    std::weak_ptr<signal::detail::ConnectionBodyBase>&& connectionBody)
  : mWeakConnectionBody(std::move(connectionBody))
{
  // Do nothing
}

//==============================================================================
Connection::~Connection()
{
  // Do nothing
}

//==============================================================================
bool Connection::isConnected() const
{
  return !mWeakConnectionBody.expired();
}

//==============================================================================
void Connection::disconnect() const
{
  if (auto connectionBody = mWeakConnectionBody.lock())
    connectionBody->disconnect();
}

//==============================================================================
ScopedConnection::ScopedConnection(const Connection& other) : Connection(other)
{
  // Do nothing
}

//==============================================================================
ScopedConnection::ScopedConnection(Connection&& other)
  : Connection(std::move(other))
{
  // Do nothing
}

//==============================================================================
ScopedConnection::~ScopedConnection()
{
  disconnect();
}

} // namespace common
} // namespace dart
