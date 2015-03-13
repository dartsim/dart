/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
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

#include "dart/common/Signal.h"
#include <boost/signals2/connection.hpp>
namespace dart {
namespace common {

//==============================================================================
Connection::Connection() :
  mIsConnected(false),
  mSignal(nullptr),
  mId(0)
{
}

//==============================================================================
Connection::Connection(const Connection& _other)
  : mIsConnected(_other.mIsConnected),
    mSignal(_other.mSignal),
    mId(_other.mId)
{
}

//==============================================================================
Connection::Connection(SignalBase* _signal, size_t _id)
  : mIsConnected(true),
    mSignal(_signal),
    mId(_id)
{
}

//==============================================================================
Connection::~Connection()
{
}

//==============================================================================
size_t Connection::getId() const
{
  return mId;
}

//==============================================================================
bool Connection::isConnected() const
{
  if (nullptr == mSignal)
    return false;

  return mIsConnected;
}

//==============================================================================
void Connection::disconnect()
{
  if (!isConnected())
    return;

  mSignal->disconnect(mId);
  mIsConnected = false;
}

//==============================================================================
ScopedConnection::ScopedConnection(const Connection& _other)
  : Connection(_other)
{
}

//==============================================================================
ScopedConnection::~ScopedConnection()
{
  disconnect();
}

//==============================================================================
SignalBase::SignalBase()
{
}

//==============================================================================
SignalBase::~SignalBase()
{
}

//==============================================================================
void SignalBase::disconnect(const Connection& _connection)
{
  disconnect(_connection.getId());
}

}  // namespace common
}  // namespace dart

