/*
    Copyright (c) 2013, Taiga Nomi and the respective contributors
    All rights reserved.

    Use of this source code is governed by a BSD-style license that can be found
    in the LICENSE file.
*/
#pragma once

#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

#include "tiny_dnn/util/util.h"

namespace tiny_dnn {
namespace detail {

struct mnist_header {
  uint32_t magic_number;
  uint32_t num_items;
  uint32_t num_rows;
  uint32_t num_cols;
};

inline void parse_mnist_header(std::ifstream &ifs, mnist_header &header) {
  ifs.read(reinterpret_cast<char *>(&header.magic_number), 4);
  ifs.read(reinterpret_cast<char *>(&header.num_items), 4);
  ifs.read(reinterpret_cast<char *>(&header.num_rows), 4);
  ifs.read(reinterpret_cast<char *>(&header.num_cols), 4);

  if (is_little_endian()) {
    reverse_endian(&header.magic_number);
    reverse_endian(&header.num_items);
    reverse_endian(&header.num_rows);
    reverse_endian(&header.num_cols);
  }

  if (header.magic_number != 0x00000803 || header.num_items <= 0)
    throw nn_error("MNIST label-file format error");
  if (ifs.fail() || ifs.bad()) throw nn_error("file error");
}

inline void parse_mnist_image(std::ifstream &ifs,
                              const mnist_header &header,
                              float_t scale_min,
                              float_t scale_max,
                              int x_padding,
                              int y_padding,
                              vec_t &dst) {
  const int width  = header.num_cols + 2 * x_padding;
  const int height = header.num_rows + 2 * y_padding;

  std::vector<uint8_t> image_vec(header.num_rows * header.num_cols);

  ifs.read(reinterpret_cast<char *>(&image_vec[0]),
           header.num_rows * header.num_cols);

  dst.resize(width * height, scale_min);

  for (uint32_t y = 0; y < header.num_rows; y++)
    for (uint32_t x = 0; x < header.num_cols; x++)
      dst[width * (y + y_padding) + x + x_padding] =
        (image_vec[y * header.num_cols + x] / float_t(255)) *
          (scale_max - scale_min) +
        scale_min;
}

}  // namespace detail

/**
 * parse MNIST database format labels with rescaling/resizing
 * http://yann.lecun.com/exdb/mnist/
 *
 * @param label_file [in]  filename of database (i.e.train-labels-idx1-ubyte)
 * @param labels     [out] parsed label data
 **/
inline void parse_mnist_labels(const std::string &label_file,
                               std::vector<label_t> *labels) {
  std::ifstream ifs(label_file.c_str(), std::ios::in | std::ios::binary);

  if (ifs.bad() || ifs.fail())
    throw nn_error("failed to open file:" + label_file);

  uint32_t magic_number, num_items;

  ifs.read(reinterpret_cast<char *>(&magic_number), 4);
  ifs.read(reinterpret_cast<char *>(&num_items), 4);

  if (is_little_endian()) {  // MNIST data is big-endian format
    reverse_endian(&magic_number);
    reverse_endian(&num_items);
  }

  if (magic_number != 0x00000801 || num_items <= 0)
    throw nn_error("MNIST label-file format error");

  labels->resize(num_items);
  for (uint32_t i = 0; i < num_items; i++) {
    uint8_t label;
    ifs.read(reinterpret_cast<char *>(&label), 1);
    (*labels)[i] = static_cast<label_t>(label);
  }
}

/**
 * parse MNIST database format images with rescaling/resizing
 * http://yann.lecun.com/exdb/mnist/
 * - if original image size is WxH, output size is
 *(W+2*x_padding)x(H+2*y_padding)
 * - extra padding pixels are filled with scale_min
 *
 * @param image_file [in]  filename of database (i.e.train-images-idx3-ubyte)
 * @param images     [out] parsed image data
 * @param scale_min  [in]  min-value of output
 * @param scale_max  [in]  max-value of output
 * @param x_padding  [in]  adding border width (left,right)
 * @param y_padding  [in]  adding border width (top,bottom)
 *
 * [example]
 * scale_min=-1.0, scale_max=1.0, x_padding=1, y_padding=0
 *
 * [input]       [output]
 *  64  64  64   -1.0 -0.5 -0.5 -0.5 -1.0
 * 128 128 128   -1.0  0.0  0.0  0.0 -1.0
 * 255 255 255   -1.0  1.0  1.0  1.0 -1.0
 *
 **/
inline void parse_mnist_images(const std::string &image_file,
                               std::vector<vec_t> *images,
                               float_t scale_min,
                               float_t scale_max,
                               int x_padding,
                               int y_padding) {
  if (x_padding < 0 || y_padding < 0)
    throw nn_error("padding size must not be negative");
  if (scale_min >= scale_max)
    throw nn_error("scale_max must be greater than scale_min");

  std::ifstream ifs(image_file.c_str(), std::ios::in | std::ios::binary);

  if (ifs.bad() || ifs.fail())
    throw nn_error("failed to open file:" + image_file);

  detail::mnist_header header;

  detail::parse_mnist_header(ifs, header);

  images->resize(header.num_items);
  for (uint32_t i = 0; i < header.num_items; i++) {
    vec_t image;
    detail::parse_mnist_image(ifs, header, scale_min, scale_max, x_padding,
                              y_padding, image);
    (*images)[i] = image;
  }
}

}  // namespace tiny_dnn
