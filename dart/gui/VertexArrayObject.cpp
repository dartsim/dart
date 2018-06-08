#include "dart/gui/VertexArrayObject.hpp"

#include <cassert>
#include "dart/common/Console.hpp"
#include "dart/gui/glfw/LoadGlfw.hpp"

namespace dart {
namespace gui {

//==============================================================================
VertexArrayObject::VertexArrayObject() : mVertexArrayId{0u}
{
  glGenVertexArrays(1, &mVertexArrayId);
  assert(mVertexArrayId != 0);
}

//==============================================================================
VertexArrayObject::~VertexArrayObject()
{
  if (mVertexArrayId != 0)
  {
    glDeleteVertexArrays(1, &mVertexArrayId);
    mVertexArrayId = 0;
  }
}

//==============================================================================
void VertexArrayObject::bind()
{
  assert(mVertexArrayId != 0);

  glBindVertexArray(mVertexArrayId);
}

//==============================================================================
void VertexArrayObject::unbind()
{
  assert(mVertexArrayId != 0);

  glBindVertexArray(0);
}

} // namespace gui
} // namespace dart
