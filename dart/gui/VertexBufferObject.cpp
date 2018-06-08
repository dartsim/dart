#include "dart/gui/VertexBufferObject.hpp"

#include <cassert>
#include "dart/gui/glfw/LoadGlfw.hpp"

namespace dart {
namespace gui {

//==============================================================================
VertextBufferObject::VertextBufferObject(GLenum targetBufferType)
  : mVertexBufferID{0u}, mTargetBufferType{targetBufferType}, mIsBound{false}
{
  glGenBuffers(1, &mVertexBufferID);

  assert(mVertexBufferID != 0u);
  assert(
      mTargetBufferType == GL_ARRAY_BUFFER
      || mTargetBufferType == GL_ELEMENT_ARRAY_BUFFER);
}

//==============================================================================
VertextBufferObject::~VertextBufferObject()
{
  glDeleteFramebuffers(1, &mVertexBufferID);
}

//==============================================================================
void VertextBufferObject::copyData(const void* data, GLenum usage)
{
  assert(
      usage == GL_STATIC_DRAW || usage == GL_DYNAMIC_DRAW
      || usage == GL_STREAM_DRAW);

  if (!mIsBound)
    bind();

  glBufferData(mTargetBufferType, sizeof(data), data, usage);

  unbind();
}

//==============================================================================
void VertextBufferObject::bind()
{
  assert(mVertexBufferID != 0);

  glBindBuffer(mTargetBufferType, mVertexBufferID);
}

//==============================================================================
void VertextBufferObject::unbind()
{
  assert(mVertexBufferID != 0);

  glBindBuffer(mTargetBufferType, 0);
}

} // namespace gui
} // namespace dart
