#include "dart/gui/FragmentShader.hpp"

namespace dart {
namespace gui {

//==============================================================================
FragmentShader::FragmentShader(
    const common::Uri& shaderUri, common::ResourceRetriever* retriever)
  : Shader(GL_FRAGMENT_SHADER, shaderUri, retriever)
{
  // Do nothing
}

} // namespace gui
} // namespace dart
