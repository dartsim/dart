#ifndef DART_GUI_GLFW_VERTEXSHADER_HPP_
#define DART_GUI_GLFW_VERTEXSHADER_HPP_

#include <string>

#include "dart/gui/Shader.hpp"

namespace dart {
namespace gui {

class VertexShader : public Shader
{
public:
  VertexShader(
      const common::Uri& shaderUri = "",
      common::ResourceRetriever* retriever = nullptr);
  // TODO: remove the default parameters
  // TODO: change to URI

  virtual ~VertexShader() = default;
};

} // namespace gui
} // namespace dart

#endif // DART_GUI_GLFW_VERTEXSHADER_HPP_
