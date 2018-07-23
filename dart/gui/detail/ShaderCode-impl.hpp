#ifndef DART_GUI_DETIAL_SHADERCODE_IMPL_HPP_
#define DART_GUI_DETIAL_SHADERCODE_IMPL_HPP_

#include "dart/gui/ShaderCode.hpp"

#include <cassert>
#include <vector>
#include "dart/common/Console.hpp"
#include "dart/common/LocalResourceRetriever.hpp"

namespace dart {
namespace gui {

//==============================================================================
template <GLuint ShaderType>
ShaderCode<ShaderType>::ShaderCode(const char* shaderString)
  : ShaderCode(std::string(shaderString))
{
  // Do nothing
}

//==============================================================================
template <GLuint ShaderType>
ShaderCode<ShaderType>::ShaderCode(const std::string& shaderString)
  : mCode(shaderString)
{
  // Do nothing
}

//==============================================================================
template <GLuint ShaderType>
ShaderCode<ShaderType>::ShaderCode(
    const common::Uri& shaderUri, common::ResourceRetriever* retriever)
{
  std::unique_ptr<common::ResourceRetriever> localRetriever;
  if (!retriever)
  {
    localRetriever.reset(new common::LocalResourceRetriever());
    retriever = localRetriever.get();
  }

  mCode = retriever->readAll(shaderUri);
}

//==============================================================================
template <GLuint ShaderType>
const char* ShaderCode<ShaderType>::getCString() const
{
  return getString().c_str();
}

//==============================================================================
template <GLuint ShaderType>
const std::string& ShaderCode<ShaderType>::getString() const
{
  return mCode;
}

} // namespace gui
} // namespace dart

#endif // DART_GUI_DETIAL_SHADERCODE_IMPL_HPP_
