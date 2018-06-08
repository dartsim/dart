#ifndef DART_GUI_GLFW_FRAGMENTSHADER_HPP_
#define DART_GUI_GLFW_FRAGMENTSHADER_HPP_

#include <string>

#include "dart/gui/Shader.hpp"

namespace dart {
namespace gui {

class FragmentShader : public Shader
{
public:
  FragmentShader(const common::Uri& shaderUri, common::ResourceRetriever* retriever = nullptr);
  // TODO: remove the default parameters
  // TODO: change to URI

  virtual ~FragmentShader() = default;
};

} // namespace gui
} // namespace dart

#endif // DART_GUI_GLFW_FRAGMENTSHADER_HPP_
