#ifndef DART_GUI_BUFFEROBJECT_HPP_
#define DART_GUI_BUFFEROBJECT_HPP_

#include <vector>

#include "dart/common/SmartPointer.hpp"
#include "dart/gui/LoadOpengl.hpp"

namespace dart {
namespace gui {

/// A wrapper class of OpenGL vertex buffer objects (VBO) that can store a large
/// number of vertices in the GPU's memory.
/// \tparam OpenGL buffer type, which should be one of followings:
/// - GL_ARRAY_BUFFER: only points
/// - GL_ELEMENT_ARRAY_BUFFER: points and indices
template <GLenum Buffer_, typename BufferDataT>
struct BufferObject
{
  static constexpr GLenum BufferType = Buffer_;
  // static constexpr GLenum UsatePattern = UsatePattern_;

  using BufferData = BufferDataT;

  /// Constructor
  BufferObject();

  /// Destructor
  ~BufferObject() = default;

  /// Allocates this buffer object
  void initialize();

  /// Deallocates this buffer object
  void release();

  /// \param[in] usage A parameter specifies how we want the graphics card to
  /// manage the given data. It can take 3 forms:
  /// - GL_STATIC_DRAW : The data will most likely not change at all or very
  ///   rarely.
  /// - GL_DYNAMIC_DRAW: the data is likely to change a lot.
  /// - GL_STREAM_DRAW : the data will change every time it is drawn.
  void copyData(std::size_t size, const void* data, GLenum usage = GL_STATIC_DRAW);

  void upload(GLenum usage = GL_STATIC_DRAW);

  // TODO(JS): Remove one of copyData and upload() once decide whether keep data
  // in this class or not.

  /// Binds this VBO
  void bind();

  /// Unbinds this VBO
  void unbind();

  std::size_t size() const
  {
    return mData.size();
  }

  bool empty() const
  {
    return mData.empty();
  }

  /// A unique ID associated with this VBO where the id is assigned using
  /// glGenBuffers function.
  GLuint mId;

  std::vector<BufferDataT> mData;

#ifndef NDEBUG // Debug mode
  /// Whether this VBO is bounded
  bool mIsBound;

  /// Whether allocated
  bool mAllocated;
#endif
};

} // namespace gui
} // namespace dart

#include "dart/gui/detail/BufferObject-impl.hpp"

#endif // DART_GUI_BUFFEROBJECT_HPP_
