/*
 * Copyright (c) 2015-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2015-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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
Connection::Connection(const Connection& _other)
  : mWeakConnectionBody(_other.mWeakConnectionBody)
{
  // Do nothing
}

//==============================================================================
Connection::Connection(Connection&& _other)
  : mWeakConnectionBody(std::move(_other.mWeakConnectionBody))
{
  // Do nothing
}

//==============================================================================
Connection& Connection::operator=(const Connection& _other)
{
  mWeakConnectionBody = _other.mWeakConnectionBody;
  return *this;
}

//==============================================================================
Connection& Connection::operator=(Connection&& _other)
{
  mWeakConnectionBody = std::move(_other.mWeakConnectionBody);
  return *this;
}

//==============================================================================
Connection::Connection(
    const std::weak_ptr<signal::detail::ConnectionBodyBase>& _connectionBody)
  : mWeakConnectionBody(_connectionBody)
{
  // Do nothing
}

//==============================================================================
Connection::Connection(
    std::weak_ptr<signal::detail::ConnectionBodyBase>&& _connectionBody)
  : mWeakConnectionBody(std::move(_connectionBody))
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
  std::shared_ptr<signal::detail::ConnectionBodyBase>
      connectionBody(mWeakConnectionBody.lock());

  if (nullptr == connectionBody)
    return false;

  return connectionBody->isConnected();
}

//==============================================================================
void Connection::disconnect() const
{
  std::shared_ptr<signal::detail::ConnectionBodyBase>
      connectionBody(mWeakConnectionBody.lock());

  if (nullptr == connectionBody)
    return;

  connectionBody->disconnect();
}

//==============================================================================
ScopedConnection::ScopedConnection(const Connection& _other)
  : Connection(_other)
{
  // Do nothing
}

//==============================================================================
ScopedConnection::ScopedConnection(Connection&& _other)
  : Connection(std::move(_other))
{
  // Do nothing
}

//==============================================================================
ScopedConnection::~ScopedConnection()
{
  disconnect();
}

}  // namespace common
}  // namespace dart

