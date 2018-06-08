#include "dart/gui/FragmentShader.hpp"

#include <fstream>
#include "dart/common/Console.hpp"

namespace dart {
namespace gui {

//==============================================================================
FragmentShader::FragmentShader(const std::string& fragmentShaderFilepath)
  : mFragmentShaderFilepath(fragmentShaderFilepath)
{
  assert(!fragmentShaderFilepath.empty());

  // Set the shader filenames
  mFragmentShaderFilepath = fragmentShaderFilepath;

  std::ifstream fileFragmentShader;
  fileFragmentShader.open(fragmentShaderFilepath.c_str(), std::ios::binary);

  if (!fileFragmentShader.is_open())
  {
    dterr << "Error : Impossible to open the fragment shader file "
          << fragmentShaderFilepath;
    assert(false);
    return;
  }

  // Get the size of the file
  fileFragmentShader.seekg(0, std::ios::end);
  uint fileSize = (uint)(fileFragmentShader.tellg());
  assert(fileSize != 0);

  // Read the file
  fileFragmentShader.seekg(std::ios::beg);
  char* bufferFragmentShader = new char[fileSize + 1];
  fileFragmentShader.read(bufferFragmentShader, fileSize);
  fileFragmentShader.close();
  bufferFragmentShader[fileSize] = '\0';

  // Create the OpenGL fragment shader and compile it
  mFragmentShaderID = glCreateShader(GL_FRAGMENT_SHADER);
  assert(mFragmentShaderID != 0);
  glShaderSource(
      mFragmentShaderID, 1, (const char**)(&bufferFragmentShader), NULL);
  glCompileShader(mFragmentShaderID);
  delete[] bufferFragmentShader;

  // Get the compilation information
  int compiled;
  glGetShaderiv(mFragmentShaderID, GL_COMPILE_STATUS, &compiled);

  // If the compilation failed
  if (compiled == 0)
  {

    // Get the log of the compilation
    int lengthLog;
    glGetShaderiv(mFragmentShaderID, GL_INFO_LOG_LENGTH, &lengthLog);
    char* str = new char[lengthLog];
    glGetShaderInfoLog(mFragmentShaderID, lengthLog, NULL, str);

    // Display the log of the compilation
    dterr << "Fragment Shader Error (in " << fragmentShaderFilepath
          << ") : " << str;
    delete[] str;
    assert(false);
    return;
  }
}

//==============================================================================
FragmentShader::~FragmentShader()
{
  glDeleteShader(mFragmentShaderID);
}

//==============================================================================
GLuint FragmentShader::getId() const
{
  return mFragmentShaderID;
}

} // namespace gui
} // namespace dart
