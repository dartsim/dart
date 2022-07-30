/*
 * Copyright (c) 2011-2022, The DART development contributors:
 * https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "dart/gui/image.hpp"

#include "dart/common/logging.hpp"
#include "dart/common/macro.hpp"

namespace dart::gui {

//==============================================================================
int get_bytes_per_channel(int type)
{
  if (type == GL_BYTE || type == GL_UNSIGNED_BYTE) {
    return 1;
  } else if (type == GL_SHORT || type == GL_UNSIGNED_SHORT) {
    return 2;
  }

  DART_FATAL("Unsupported pixel data type {}.", type);
  return 0;
}

//==============================================================================
int get_bytes_per_pixel(int format, int type)
{
  const int bytes_per_channel = get_bytes_per_channel(type);

  if (format == GL_RGB) {
    return bytes_per_channel * 3;
  } else if (format == GL_RGBA) {
    return bytes_per_channel * 4;
  }

  DART_FATAL("Unsupported pixel format {}.", format);
  return 0;
}

//==============================================================================
struct Image::Implementation
{
  int width;
  int height;
  int pixel_format;
  int pixel_type;
  std::string data;
  osg::ref_ptr<osg::Image> osg_image;

  Implementation()
  {
    // Do nothing
  }
};

//==============================================================================
Image::Image() : m_impl(std::make_unique<Implementation>())
{
  m_impl->osg_image = new osg::Image();
}

//==============================================================================
Image::~Image()
{
  // Do nothing
}

//==============================================================================
void Image::set_data(
    const unsigned char* data, int width, int height, int format, int type)
{
  m_impl->width = width;
  m_impl->height = height;
  m_impl->pixel_format = format;
  m_impl->pixel_type = type;
  m_impl->data.resize(width * height * get_bytes_per_pixel(format, type));
  std::memcpy(m_impl->data.data(), data, m_impl->data.size());
}

//==============================================================================
void Image::set_from_osg_image(const osg::Image& image)
{
  m_impl->osg_image
      = static_cast<osg::Image*>(image.clone(osg::CopyOp::DEEP_COPY_ALL));
}

} // namespace dart::gui
