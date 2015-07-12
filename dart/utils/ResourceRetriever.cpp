#include <cassert>
#include "ResourceRetriever.h"

namespace dart {
namespace utils {

MemoryResource::MemoryResource()
  : mData(nullptr),
    mSize(0),
    mIsOwner(false)
{
}

MemoryResource::MemoryResource(uint8_t* _data, size_t _size, bool _isOwner)
  : mData(_data),
    mSize(_size),
    mIsOwner(_isOwner)
{
  assert(_size == 0 || _data != nullptr);
}

MemoryResource::~MemoryResource()
{
  if (mData && mIsOwner)
    delete mData;
}

uint8_t* MemoryResource::getData()
{
  return mData;
}

const uint8_t* MemoryResource::getData() const
{
  return mData;
}

size_t MemoryResource::getSize() const
{
  return mSize;
}

}  // namespace utils
}  // namespace dart
