#ifndef DART_GUI_GLFW_VERTEXARRAYOBJECT_HPP_
#define DART_GUI_GLFW_VERTEXARRAYOBJECT_HPP_

#include "dart/common/SmartPointer.hpp"

#include "dart/gui/IndexBufferObject.hpp"
#include "dart/gui/LoadOpengl.hpp"
#include "dart/gui/VertexBufferObject.hpp"

namespace dart {
namespace gui {

// Vertex array can't be shared across contexts even though you passed a context
// that you want to share when creating new context. So VBO needs to be created
// per context always.
template <typename VertexT, typename IndexT>
struct VertexArrayObject final
{
public:
  using VertexBufferObjectType = VertexBufferObject<VertexT>;
  using IndexBufferObjectType = IndexBufferObject<IndexT>;

  /// Constructor
  VertexArrayObject();

  /// Destructor
  ~VertexArrayObject() = default;

  void initialize();

  void release();

  /// Binds the VAO
  void bind();

  /// Unbinds the VAO
  void unbind();

  void upload();

  void draw(GLenum mode = GL_TRIANGLES);
  // TODO(JS): temp

  /// Unique ID of the Vertex Array Object
  GLuint mVertexArrayId;

  VertexBufferObject<VertexT> mVBO;
  IndexBufferObject<IndexT> mIBO;

#ifndef NDEBUG
  bool mIsAllocated;
#endif
};

} // namespace gui
} // namespace dart

#include "dart/gui/detail/VertexArrayObject-impl.hpp"

#endif // DART_GUI_GLFW_VERTEXARRAYOBJECT_HPP_
