#ifndef DART_GUI_INDEXBUFFEROBJECT_HPP_
#define DART_GUI_INDEXBUFFEROBJECT_HPP_

#include <vector>

#include "dart/gui/BufferObject.hpp"

namespace dart {
namespace gui {

template <typename IndexT>
struct IndexBufferObject : BufferObject<GL_ELEMENT_ARRAY_BUFFER, IndexT>
{
  using IndexType = IndexT;

  /// Constructor
  IndexBufferObject();
};

} // namespace gui
} // namespace dart

#include "dart/gui/detail/IndexBufferObject-impl.hpp"

#endif // DART_GUI_INDEXBUFFEROBJECT_HPP_
