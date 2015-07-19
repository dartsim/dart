#include <cstring>
#include <iostream>
#include "LocalResource.h"
#include "dart/common/Console.h"

namespace dart {
namespace utils {

LocalResource::LocalResource(const std::string& _path)
  : mFile(std::fopen(_path.c_str(), "rb"))
{
  if(!mFile)
  {
    dtwarn << "[LocalResource::constructor] Failed opening file '" << _path
          << "' for reading: " << std::strerror(errno) << "\n";
  }
}

LocalResource::~LocalResource()
{
  if (!mFile)
    return;

  if (std::fclose(mFile) == EOF)
  {
    dtwarn << "[LocalResource::destructor] Failed closing file.\n";
  }
}

bool LocalResource::isGood() const
{
  return !!mFile;
}

size_t LocalResource::getFileSize()
{
  if(!mFile)
    return 0;

  const long offset = std::ftell(mFile);
  if(offset == -1L)
  {
    dtwarn << "[LocalResource::getFileSize] Unable to compute file size: Failed"
             " getting current offset: " << std::strerror(errno) << "\n";
    return 0;
  }

  // The SEEK_END option is not required by the C standard. However, it is
  // required by POSIX.
  // TODO: Does this work on Windows?
  if(std::fseek(mFile, 0, SEEK_END) || std::ferror(mFile))
  {
    dtwarn << "[LocalResource::getFileSize] Unable to compute file size: Failed"
             " seeking to the end of the file.\n";
    return 0;
  }

  const long size = std::ftell(mFile);
  if(size == -1L)
  {
    dtwarn << "[LocalResource::getFileSize] Unable to compute file size: Failed"
             " getting end of file offset: " << std::strerror(errno) << "\n";
    return 0;
  }

  if(std::fseek(mFile, offset, SEEK_SET) || std::ferror(mFile))
  {
    dtwarn << "[LocalResource::getFileSize] Unable to compute file size: Failed"
             " seeking to the current position.\n";
    return 0;
  }

  return size;
}

size_t LocalResource::tell()
{
  if(!mFile)
    return 0;
  
  const long offset = std::ftell(mFile);
  if (offset == -1L)
  {
    dtwarn << "[LocalResource::tell] Failed"
             " seeking to the current position.\n";
  }

  // We return -1 to match the beahvior of DefaultIoStream in Assimp.
  return offset;
}

bool LocalResource::seek(ptrdiff_t _offset, SeekType _mode)
{
  int origin;
  switch(_mode)
  {
  case Resource::SEEKTYPE_CUR:
    origin = SEEK_CUR;
    break;

  case Resource::SEEKTYPE_END:
    origin = SEEK_END;
    break;

  case Resource::SEEKTYPE_SET:
    origin = SEEK_SET;
    break;

  default:
    dtwarn << "[LocalResource::Seek] Invalid origin. Expected"
             " SEEKTYPE_CUR, SEEKTYPE_END, or SEEKTYPE_SET.\n";
    return false;
  }

  if (!std::fseek(mFile, _offset, origin) && !std::ferror(mFile))
    return true;
  else
  {
    dtwarn << "[LocalResource::seek] Seeking failed.\n";
    return false;
  }
}

size_t LocalResource::read(void *_buffer, size_t _size, size_t _count)
{
  if (!mFile)
    return 0;

  const size_t result = std::fread(_buffer, _size, _count, mFile);
  if (std::ferror(mFile)) 
  {
    dtwarn << "[LocalResource::tell] Failed reading file.\n";
  }
  return result;
}

} // namespace utils
} // namespace dart

