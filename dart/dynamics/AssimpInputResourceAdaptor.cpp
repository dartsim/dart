#include <cassert>
#include <iostream>
#include <assimp/IOStream.hpp>
#include "dart/common/Console.h"
#include "AssimpInputResourceAdaptor.h"

namespace kido {
namespace dynamics {

/*
 * AssimpInputResourceRetrieverWrapper
 */

//==============================================================================
AssimpInputResourceRetrieverAdaptor::AssimpInputResourceRetrieverAdaptor(
      const common::ResourceRetrieverPtr& _resourceRetriever)
  : mResourceRetriever(_resourceRetriever)
{
  // do nothing
}

//==============================================================================
AssimpInputResourceRetrieverAdaptor::~AssimpInputResourceRetrieverAdaptor()
{
  // do nothing
}

//==============================================================================
bool AssimpInputResourceRetrieverAdaptor::Exists(const char* pFile) const
{
  return mResourceRetriever->exists(pFile);
}

//==============================================================================
char AssimpInputResourceRetrieverAdaptor::getOsSeparator() const
{
  // URIs always use forward slash as a delimeter.
  return '/';
}

//==============================================================================
Assimp::IOStream* AssimpInputResourceRetrieverAdaptor::Open(
  const char* pFile, const char* pMode)
{
  // TODO: How do we support text mode?
  if(pMode != std::string("r") && pMode != std::string("rb")
                               && pMode != std::string("rt"))
  {
    dtwarn << "[AssimpInputResourceRetrieverAdaptor::Open] Unsupported mode '"
           << pMode << "'. Only 'r', 'rb', and 'rt' are supported.\n";
    return nullptr;
  }

  if(const common::ResourcePtr resource = mResourceRetriever->retrieve(pFile))
    return new AssimpInputResourceAdaptor(resource);
  else
    return nullptr;
}

//==============================================================================
void AssimpInputResourceRetrieverAdaptor::Close(Assimp::IOStream* pFile)
{
  if(pFile)
    delete pFile;
}


/*
 * AssimpInputResourceAdaptor
 */

//==============================================================================
AssimpInputResourceAdaptor::AssimpInputResourceAdaptor(
      const common::ResourcePtr& _resource)
  : mResource(_resource)
{
  assert(_resource);
}

//==============================================================================
AssimpInputResourceAdaptor::~AssimpInputResourceAdaptor()
{
  // do nothing
}

//==============================================================================
size_t AssimpInputResourceAdaptor::Read(
  void* pvBuffer, size_t psize, size_t pCount)
{
  return mResource->read(pvBuffer, psize, pCount);
}

//==============================================================================
size_t AssimpInputResourceAdaptor::Write(
  const void* pvBuffer, size_t pSize, size_t pCount)
{
  dtwarn << "[AssimpInputResourceAdaptor::Write] Write is not implemented."
            " This is a read-only stream.\n";
  return 0;
}

//==============================================================================
aiReturn AssimpInputResourceAdaptor::Seek(size_t pOffset, aiOrigin pOrigin)
{
  using common::Resource;

  Resource::SeekType origin;
  switch(pOrigin)
  {
  case aiOrigin_CUR:
    origin = Resource::SEEKTYPE_CUR;
    break;

  case aiOrigin_END:
    origin = Resource::SEEKTYPE_END;
    break;

  case aiOrigin_SET:
    origin = Resource::SEEKTYPE_SET;
    break;

  default:
    dtwarn << "[AssimpInputResourceAdaptor::Seek] Invalid origin. Expected"
              " aiOrigin_CUR, aiOrigin_END, or aiOrigin_SET.\n";
    return aiReturn_FAILURE;
  }

  if(mResource->seek(pOffset, origin))
    return aiReturn_SUCCESS;
  else
    return aiReturn_FAILURE;
}

//==============================================================================
size_t AssimpInputResourceAdaptor::Tell() const
{
  return mResource->tell();
}

//==============================================================================
size_t AssimpInputResourceAdaptor::FileSize() const
{
  return mResource->getSize();
}

//==============================================================================
void AssimpInputResourceAdaptor::Flush()
{
  dtwarn << "[AssimpInputResourceAdaptor::Flush] Flush is not implemented."
            " This is a read-only stream.\n";
}


/*
 * C API
 */
namespace {

//==============================================================================
inline Assimp::IOSystem *getIOSystem(aiFileIO *_io)
{
  return reinterpret_cast<Assimp::IOSystem *>(_io->UserData);
}

//==============================================================================
inline Assimp::IOStream *getIOStream(aiFile *_file)
{
  return reinterpret_cast<Assimp::IOStream *>(_file->UserData);
}

//==============================================================================
void fileFlushProc(aiFile *_file)
{
  getIOStream(_file)->Flush();
}

//==============================================================================
size_t fileReadProc(aiFile *_file, char *_buffer, size_t _size, size_t _count)
{
  return getIOStream(_file)->Read(_buffer, _size, _count);
}

//==============================================================================
aiReturn fileSeekProc(aiFile *_file, size_t _offset, aiOrigin _origin)
{
  return getIOStream(_file)->Seek(_offset, _origin);
}

//==============================================================================
size_t fileSizeProc(aiFile *_file)
{
  return getIOStream(_file)->FileSize();
}

//==============================================================================
size_t fileTellProc(aiFile *_file)
{
  return getIOStream(_file)->Tell();
}

//==============================================================================
size_t fileWriteProc(
  aiFile *_file, const char *_buffer, size_t _size, size_t _count)
{
  return getIOStream(_file)->Write(_buffer, _size, _count);
}

//==============================================================================
aiFile *fileOpenProc(aiFileIO* _io, const char* _path, const char* _mode)
{
  Assimp::IOStream* stream = getIOSystem(_io)->Open(_path, _mode);
  if(!stream)
    return nullptr;

  aiFile* out = new aiFile;
  out->FileSizeProc = &fileSizeProc;
  out->FlushProc = &fileFlushProc;
  out->ReadProc = &fileReadProc;
  out->SeekProc = &fileSeekProc;
  out->TellProc = &fileTellProc;
  out->WriteProc = &fileWriteProc;
  out->UserData = reinterpret_cast<char*>(stream);
  return out;
}

//==============================================================================
void fileCloseProc(aiFileIO *_io, aiFile *_file)
{
  getIOSystem(_io)->Close(getIOStream(_file));
}

}

//==============================================================================
aiFileIO createFileIO(Assimp::IOSystem* _system)
{
  aiFileIO out;
  out.OpenProc = &fileOpenProc;
  out.CloseProc = &fileCloseProc;
  out.UserData = reinterpret_cast<char*>(_system);
  return out;
}

} // namespace dynamics
} // namespace kido
