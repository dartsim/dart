#include "dart/gui/Program.hpp"

#include <fstream>
#include <vector>
#include "dart/common/Console.hpp"

#define DART_MAX_UNIFORM_NAME_LENGTH 1024

namespace dart {
namespace gui {

//==============================================================================
Program::Program(
    const VertexShaderCode& vertexShader,
    const FragmentShaderCode& fragmentShader)
  : mProgramId{0u}
{
  mProgramId = glCreateProgram();
  assert(mProgramId);

  const auto vertexShaderId = compileShader(vertexShader);
  const auto fragmentShaderId = compileShader(fragmentShader);

  glAttachShader(mProgramId, vertexShaderId);
  glAttachShader(mProgramId, fragmentShaderId);

  glDeleteShader(vertexShaderId);
  glDeleteShader(fragmentShaderId);

  glLinkProgram(mProgramId);

  GLint isLinked;
  glGetProgramiv(mProgramId, GL_LINK_STATUS, &isLinked);

  if (!isLinked)
  {
    GLsizei logLength;
    glGetProgramiv(mProgramId, GL_INFO_LOG_LENGTH, &logLength);
    std::vector<GLchar> strLog(static_cast<std::size_t>(logLength));
    glGetProgramInfoLog(mProgramId, logLength, nullptr, strLog.data());
    dterr << "Linker Error in vertex shader or in fragment shader: "
          << strLog.data() << "\n";
    assert(false);
  }

}

//==============================================================================
void Program::bind()
{
  assert(mProgramId != 0);
  glUseProgram(mProgramId);
}

//==============================================================================
void Program::unbind()
{
  assert(mProgramId != 0);
  glUseProgram(0);
}

//==============================================================================
GLint Program::getUniformLocation(
    const std::string& variableName, bool errorIfMissing) const
{
  assert(mProgramId != 0);
  const auto location = glGetUniformLocation(mProgramId, variableName.c_str());
  if (location == -1 && errorIfMissing)
  {
    dterr << "Error in vertex shader or in fragment shader: "
          << "No Uniform variable : " << variableName << "\n";
    throw std::logic_error("Error in Shader");
  }

  return location;
}

//==============================================================================
GLint Program::getAttribLocation(
    const std::string& variableName, bool errorIfMissing) const
{
  assert(mProgramId != 0);
  const auto location = glGetAttribLocation(mProgramId, variableName.c_str());
  if (location == -1 && errorIfMissing)
  {
    dterr << "Error in vertex shader or in fragment shader: "
          << "No Uniform variable : " << variableName;
    throw std::logic_error("Error in Shader");
  }

  return location;
}

//==============================================================================
void Program::setFloatUniform(
    const std::string& variableName, float value, bool errorIfMissing) const
{
  assert(mProgramId != 0);
  const GLint location = getUniformLocation(variableName, errorIfMissing);
  if (location != -1)
  {
    glUniform1f(location, value);
  }
}

//==============================================================================
void Program::setIntUniform(
    const std::string& variableName, int value, bool errorIfMissing) const
{
  assert(mProgramId != 0);
  const GLint location = getUniformLocation(variableName, errorIfMissing);
  if (location != -1)
  {
    glUniform1i(location, value);
  }
}

//==============================================================================
void Program::setVector2Uniform(
    const std::string& variableName,
    const Eigen::Vector2f& v,
    bool errorIfMissing) const
{
  assert(mProgramId != 0);
  const GLint location = getUniformLocation(variableName, errorIfMissing);
  if (location != -1)
    glUniform2fv(location, 1, v.data());
}

//==============================================================================
void Program::setVector3Uniform(
    const std::string& variableName,
    const Eigen::Vector3f& v,
    bool errorIfMissing) const
{
  assert(mProgramId != 0);

  const GLint location = getUniformLocation(variableName, errorIfMissing);
  if (location != -1)
    glUniform3fv(location, 1, v.data());
}

//==============================================================================
void Program::setVector4Uniform(
    const std::string& variableName,
    const Eigen::Vector4f& v,
    bool errorIfMissing) const
{
  assert(mProgramId != 0);

  const GLint location = getUniformLocation(variableName, errorIfMissing);
  if (location != -1)
    glUniform4fv(location, 1, v.data());
}

//==============================================================================
void Program::setMatrix3x3Uniform(
    const std::string& variableName,
    const float* matrix,
    bool transpose,
    bool errorIfMissing) const
{
  assert(mProgramId != 0);
  const GLint location = getUniformLocation(variableName, errorIfMissing);
  if (location != -1)
    glUniformMatrix3fv(location, 1, transpose, matrix);
}

//==============================================================================
void Program::setMatrix3x3Uniform(
    const std::string& variableName,
    const Eigen::Matrix3f& matrix,
    bool errorIfMissing) const
{
  setMatrix3x3Uniform(variableName, matrix.data(), false, errorIfMissing);
  // TODO: add test if matrix.data() order is right
}

//==============================================================================
void Program::setMatrix4x4Uniform(
    const std::string& variableName,
    const float* matrix,
    bool transpose,
    bool errorIfMissing) const
{
  assert(mProgramId != 0);
  const GLint location = getUniformLocation(variableName, errorIfMissing);
  if (location != -1)
    glUniformMatrix4fv(location, 1, transpose, matrix);
}

//==============================================================================
void Program::setMatrix4x4Uniform(
    const std::string& variableName,
    const Eigen::Matrix4f& matrix,
    bool errorIfMissing) const
{
  setMatrix4x4Uniform(variableName, matrix.data(), false, errorIfMissing);
  // TODO: add test if matrix.data() order is right
  // TODO: add transposed version
}

//==============================================================================
void Program::setMatrix4x4Uniform(
    const std::string& variableName,
    const Eigen::Affine3f& matrix,
    bool errorIfMissing) const
{
  setMatrix4x4Uniform(variableName, matrix.data(), false, errorIfMissing);
  // TODO: add test if matrix.data() order is right
  // TODO: add transposed version
}

//==============================================================================
void Program::setMatrix4x4Uniform(
    const std::string& variableName,
    const Eigen::Isometry3f& matrix,
    bool errorIfMissing) const
{
  setMatrix4x4Uniform(variableName, matrix.data(), false, errorIfMissing);
  // TODO: add test if matrix.data() order is right
  // TODO: add transposed version
}

//==============================================================================
GLint Program::getUniform(const std::string& name) const
{
  auto it = mUniformLocationMap.find(name);

  if (it == mUniformLocationMap.end())
  {
    // TODO(JS): Error handling
  }

  return it->second;
}

//==============================================================================
GLint Program::getAttribute(const std::string& name) const
{
  auto it = mAttributeLocationMap.find(name);

  if (it == mAttributeLocationMap.end())
  {
    // TODO(JS): Error handling
  }

  return it->second;
}

//==============================================================================
void Program::generateUniformLists()
{
  GLint numUniforms;
  glGetProgramiv(mProgramId, GL_ACTIVE_UNIFORMS, &numUniforms);

  std::vector<GLchar> nameData(DART_MAX_UNIFORM_NAME_LENGTH);
  for (GLuint i = 0u; i < static_cast<GLuint>(numUniforms); ++i)
  {
    GLsizei nameLength = 0;
    GLint arraySize = 0;
    GLenum type = 0;
    glGetActiveUniform(
        mProgramId,
        i,
        static_cast<GLsizei>(nameData.size()),
        &nameLength,
        &arraySize,
        &type,
        &nameData[0]);
    addUniform(
        std::string(nameData.data(), static_cast<std::size_t>(nameLength)));
  }
}

//==============================================================================
GLint Program::addUniform(const std::string& name)
{
  const auto it = mUniformLocationMap.find(name);
  if (it != mUniformLocationMap.end())
    return it->second;

  const GLint location = glGetUniformLocation(mProgramId, name.c_str());

  if (mUniformLocationMap[name] == -1)
  {
    dterr << "Uniform '" << name << "' does not exist in shader\n";
    assert(false);
  }

  mUniformLocationMap[name] = location;

  return location;
}

//==============================================================================
GLint Program::addAttribute(const std::string& name)
{
  const auto it = mAttributeLocationMap.find(name);
  if (it != mAttributeLocationMap.end())
    return it->second;

  const GLint location = glGetAttribLocation(mProgramId, name.c_str());

  if (mAttributeLocationMap[name] == -1)
  {
    dterr << "Attribute '" << name << "' does not exist in shader\n";
    assert(false);
  }

  mAttributeLocationMap[name] = location;

  return location;
}

} // namespace gui
} // namespace dart
