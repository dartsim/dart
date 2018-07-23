#ifndef DART_GUI_DETAIL_VERTEXARRAYOBJECT_IMPL_HPP_
#define DART_GUI_DETAIL_VERTEXARRAYOBJECT_IMPL_HPP_

#include "dart/gui/VertexArrayObject.hpp"

#include <cassert>
#include "dart/common/Console.hpp"
#include "dart/common/Memory.hpp"
#include "dart/gui/glfw/LoadGlfw.hpp"

namespace dart {
namespace gui {

//==============================================================================
template <typename VertexT, typename IndexT>
VertexArrayObject<VertexT, IndexT>::VertexArrayObject() : mVertexArrayId{0u}
{
  // Do nothing
}

//==============================================================================
template <typename VertexT, typename IndexT>
void VertexArrayObject<VertexT, IndexT>::initialize()
{
  glGenVertexArrays(1, &mVertexArrayId);
  assert(mVertexArrayId != 0);

  bind();

  // Must be called after bind() is called
  mVBO.initialize();

  mIBO.initialize();
}

//==============================================================================
template <typename VertexT, typename IndexT>
void VertexArrayObject<VertexT, IndexT>::release()
{
  mVBO.release();
  mIBO.release();

  glDeleteVertexArrays(1, &mVertexArrayId);
}

//==============================================================================
template <typename VertexT, typename IndexT>
void VertexArrayObject<VertexT, IndexT>::bind()
{
  assert(mVertexArrayId != 0);
  glBindVertexArray(mVertexArrayId);
}

//==============================================================================
template <typename VertexT, typename IndexT>
void VertexArrayObject<VertexT, IndexT>::unbind()
{
  assert(mVertexArrayId != 0);
  glBindVertexArray(0);
}

//==============================================================================
template <typename VertexT, typename IndexT>
void VertexArrayObject<VertexT, IndexT>::upload()
{
  if (mVBO.size() == 0u || mIBO.size() == 0u)
    return;

  mVBO.upload();
  mIBO.upload();
}

//==============================================================================
template <typename VertexT, typename IndexT>
void VertexArrayObject<VertexT, IndexT>::draw(GLenum mode)
{
  bind();
  glDrawElements(mode, mIBO.size(), GL_UNSIGNED_INT, 0);
}

} // namespace gui
} // namespace dart

#endif // DART_GUI_DETAIL_VERTEXARRAYOBJECT_IMPL_HPP_
