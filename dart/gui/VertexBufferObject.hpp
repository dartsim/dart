#ifndef DART_GUI_GLFW_VERTEXBUFFEROBJECT_HPP_
#define DART_GUI_GLFW_VERTEXBUFFEROBJECT_HPP_

#include "dart/common/SmartPointer.hpp"

#include "dart/gui/LoadOpengl.hpp"

namespace dart {
namespace gui {

/// A wrapper class of OpenGL vertex buffer objects (VBO) that can store a large
/// number of vertices in the GPU's memory.
class VertextBufferObject final
{
public:
  /// Constructor
  ///
  /// \param[in] targetData OpenGL buffer type, which should be one of
  /// followings:
  /// - GL_ARRAY_BUFFER: only points
  /// - GL_ELEMENT_ARRAY_BUFFER: points and indices
  VertextBufferObject(GLenum targetBufferType = GL_ARRAY_BUFFER);

  /// Destructor
  ~VertextBufferObject();

  /// \param[in] usage A parameter specifies how we want the graphics card to
  /// manage the given data. It can take 3 forms:
  /// - GL_STATIC_DRAW : The data will most likely not change at all or very
  ///   rarely.
  /// - GL_DYNAMIC_DRAW: the data is likely to change a lot.
  /// - GL_STREAM_DRAW : the data will change every time it is drawn.
  void copyData(const void* data, GLenum usage = GL_STATIC_DRAW);

protected:
  void bind();
  void unbind();

  /// A unique ID associated with this VBO where the id is assigned using
  /// glGenBuffers function.
  GLuint mVertexBufferID;

  /// OpenGL buffer type.
  GLenum mTargetBufferType;

private:
  bool mIsBound;
};

} // namespace gui
} // namespace dart

#endif // DART_GUI_GLFW_VERTEXBUFFEROBJECT_HPP_
