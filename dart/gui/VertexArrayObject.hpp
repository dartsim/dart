#ifndef DART_GUI_GLFW_VERTEXARRAYOBJECT_HPP_
#define DART_GUI_GLFW_VERTEXARRAYOBJECT_HPP_

#include "dart/common/SmartPointer.hpp"

#include "dart/gui/LoadOpengl.hpp"

namespace dart {
namespace gui {

// Vertex array can't be shared across contexts even though you passed a context
// that you want to share when creating new context. So VBO needs to be created
// per context always.
class VertexArrayObject
{
public:
  /// Constructor
  VertexArrayObject();

  /// Destructor
  ~VertexArrayObject();

  /// Binds the VAO
  void bind();

  /// Unbinds the VAO
  void unbind();

protected:
  /// Unique ID of the Vertex Array Object
  GLuint mVertexArrayId;
};

} // namespace gui
} // namespace dart

#endif // DART_GUI_GLFW_VERTEXARRAYOBJECT_HPP_
