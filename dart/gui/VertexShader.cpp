#include "dart/gui/VertexShader.hpp"

namespace dart {
namespace gui {

//==============================================================================
VertexShader::VertexShader(
    const common::Uri& shaderUri, common::ResourceRetriever* retriever)
  : Shader(GL_VERTEX_SHADER, shaderUri, retriever)
{
  // Do nothing
}

} // namespace gui
} // namespace dart
