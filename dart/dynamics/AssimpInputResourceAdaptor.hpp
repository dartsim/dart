/*
 * Copyright (c) 2011-2019, The DART development contributors
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

#ifndef DART_DYNAMICS_ASSIMPINPUTRESOURCEADAPTOR_HPP_
#define DART_DYNAMICS_ASSIMPINPUTRESOURCEADAPTOR_HPP_

#include <assimp/cfileio.h>
#include <assimp/IOStream.hpp>
#include <assimp/IOSystem.hpp>
#include "dart/common/Resource.hpp"
#include "dart/common/ResourceRetriever.hpp"

namespace dart {
namespace dynamics {

class AssimpInputResourceRetrieverAdaptor : public Assimp::IOSystem
{
public:
  explicit AssimpInputResourceRetrieverAdaptor(
    const common::ResourceRetrieverPtr& _resourceRetriever);
  virtual ~AssimpInputResourceRetrieverAdaptor();

  /// \brief Tests for the existence of a file at the given path. 
  /// 
  /// \param pFile Path to the file
  /// \return true if there is a file with this path, else false.
  bool Exists(const char* pFile) const override;

  ///  \brief Returns the system specific directory separator
  ///
  /// \return  System specific directory separator
  char getOsSeparator() const override;

  /// \brief Open a new file with a given path.
  ///
  /// When the access to the file is finished, call Close() to release
  /// all associated resources (or the virtual dtor of the IOStream).
  ///
  /// \param pFile Path to the file
  /// \param pMode Desired file I/O mode. Required are: "wb", "w", "wt",
  ///        "rb", "r", "rt".
  /// \return New IOStream interface allowing the lib to access
  ///        the underlying file. 
  Assimp::IOStream* Open(const char* pFile, const char* pMode = "rb") override;

  /// \brief Closes the given file and releases all resources 
  ///   associated with it.
  /// \param pFile The file instance previously created by Open().
  void Close(Assimp::IOStream* pFile) override;

private:
  common::ResourceRetrieverPtr mResourceRetriever;
};

class AssimpInputResourceAdaptor : public Assimp::IOStream
{
public:
  explicit AssimpInputResourceAdaptor(const common::ResourcePtr& _resource);
  virtual ~AssimpInputResourceAdaptor();

  /// \brief Read from the file
  ///
  /// See fread() for more details
  std::size_t Read(void* pvBuffer, std::size_t pSize, std::size_t pCount) override;

  /// \brief Not implemented. This is a read-only stream.
  std::size_t Write(const void* pvBuffer, std::size_t pSize, std::size_t pCount) override;

  /// \brief Set the read/write cursor of the file
  ///
  /// Note that the offset is _negative_ for aiOrigin_END.
  /// See fseek() for more details
  aiReturn Seek(std::size_t pOffset, aiOrigin pOrigin) override;

  /// \brief Get the current position of the read/write cursor
  ///
  /// See ftell() for more details
  std::size_t Tell() const override;

  /// \brief Returns filesize
  ///
  /// Returns the filesize.
  std::size_t FileSize() const override;

  /// \brief Not implemented. This is a read-only stream.
  void Flush() override;

private:
  common::ResourcePtr mResource;
};

aiFileIO createFileIO(Assimp::IOSystem* adaptor);

} // namespace dynamics
} // namespace dart

#endif // ifndef DART_DYNAMICS_ASSIMPINPUTRESOURCEADAPTOR_HPP_
