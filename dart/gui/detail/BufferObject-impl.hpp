#include "dart/gui/VertexBufferObject.hpp"

#include <cassert>
#ifndef NDEBUG
#include "dart/common/Console.hpp"
#endif
#include "dart/gui/glfw/LoadGlfw.hpp"

namespace dart {
namespace gui {

//==============================================================================
template <GLenum Buffer, typename BufferData>
BufferObject<Buffer, BufferData>::BufferObject() : mId{0u}
{
  static_assert(
      BufferType == GL_ARRAY_BUFFER || BufferType == GL_ELEMENT_ARRAY_BUFFER,
      "Invalid buffer type.");

#ifndef NDEBUG
  mIsBound = false;
  mAllocated = false;
#endif
}

//==============================================================================
template <GLenum Buffer, typename BufferData>
void BufferObject<Buffer, BufferData>::initialize()
{
#ifndef NDEBUG
  if (mAllocated)
  {
    dtdbg << "This should not happen. Please report this as a bug!\n";
  }
#endif

  glGenBuffers(1, &mId);
  assert(mId != 0u);

#ifndef NDEBUG
  mAllocated = true;
#endif
}

//==============================================================================
template <GLenum Buffer, typename BufferData>
void BufferObject<Buffer, BufferData>::release()
{
#ifndef NDEBUG
  if (!mAllocated)
  {
    dtdbg << "This should not happen. Please report this as a bug!\n";
  }
#endif

  glDeleteFramebuffers(1, &mId);
}

//==============================================================================
template <GLenum Buffer, typename BufferData>
void BufferObject<Buffer, BufferData>::copyData(
    std::size_t size, const void* data, GLenum usage)
{
  assert(
      usage == GL_STATIC_DRAW || usage == GL_DYNAMIC_DRAW
      || usage == GL_STREAM_DRAW);

  bind();

  glBufferData(BufferType, size, data, usage);

  unbind();
}

//==============================================================================
template <GLenum Buffer, typename BufferData>
void BufferObject<Buffer, BufferData>::upload(GLenum usage)
{
  assert(
      usage == GL_STATIC_DRAW || usage == GL_DYNAMIC_DRAW
      || usage == GL_STREAM_DRAW);

  bind();

  const std::size_t dataInBytes = sizeof(BufferData) * mData.size();
  glBufferData(BufferType, dataInBytes, mData.data(), usage);
}

//==============================================================================
template <GLenum Buffer, typename BufferData>
void BufferObject<Buffer, BufferData>::bind()
{
#ifndef NDEBUG
  if (mIsBound)
  {
    dtdbg << "This should not happen. Please report this as a bug!\n";
  }
#endif

  assert(mId != 0);
  glBindBuffer(BufferType, mId);

#ifndef NDEBUG
  mIsBound = true;
#endif
}

//==============================================================================
template <GLenum Buffer, typename BufferData>
void BufferObject<Buffer, BufferData>::unbind()
{
#ifndef NDEBUG
  if (!mIsBound)
  {
    dtdbg << "This should not happen. Please report this as a bug!\n";
  }
#endif

  assert(mId != 0);
  glBindBuffer(BufferType, 0);

#ifndef NDEBUG
  mIsBound = false;
#endif
}

} // namespace gui
} // namespace dart
