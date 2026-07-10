#include "gui/offscreen.hpp"

#include "dart/gui/debug.hpp"
#include "dart/gui/offscreen.hpp"
#include "dart/gui/viewer.hpp"

#include <Python.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <cstddef>
#include <cstdint>

namespace nb = nanobind;

namespace dart::python_nb {
namespace {

struct BufferMetadata
{
  Py_ssize_t shape[3];
  Py_ssize_t strides[3];
};

int renderedImageGetBuffer(PyObject* obj, Py_buffer* view, int flags)
{
  if (view == nullptr) {
    PyErr_SetString(PyExc_BufferError, "null Py_buffer requested");
    return -1;
  }

  auto* image = nb::inst_ptr<dart::gui::RenderedImage>(nb::handle(obj));
  if (image == nullptr) {
    PyErr_SetString(PyExc_BufferError, "invalid RenderedImage buffer owner");
    return -1;
  }
  if (image->channels != 4) {
    PyErr_SetString(
        PyExc_BufferError, "RenderedImage exposes only 4-channel RGBA buffers");
    return -1;
  }
  const std::size_t expectedSize = static_cast<std::size_t>(image->width)
                                   * static_cast<std::size_t>(image->height)
                                   * static_cast<std::size_t>(image->channels);
  if (image->pixels.size() != expectedSize) {
    PyErr_SetString(
        PyExc_BufferError, "RenderedImage pixel storage has inconsistent size");
    return -1;
  }

  auto* metadata = new BufferMetadata;
  metadata->shape[0] = static_cast<Py_ssize_t>(image->height);
  metadata->shape[1] = static_cast<Py_ssize_t>(image->width);
  metadata->shape[2] = static_cast<Py_ssize_t>(image->channels);
  metadata->strides[0]
      = static_cast<Py_ssize_t>(image->width * image->channels);
  metadata->strides[1] = static_cast<Py_ssize_t>(image->channels);
  metadata->strides[2] = 1;

  Py_INCREF(obj);
  view->obj = obj;
  view->buf = image->pixels.empty() ? nullptr : image->pixels.data();
  view->len = static_cast<Py_ssize_t>(image->pixels.size());
  view->readonly = 0;
  view->itemsize = 1;
  view->format = (flags & PyBUF_FORMAT) ? const_cast<char*>("B") : nullptr;
  view->ndim = 3;
  view->shape = metadata->shape;
  view->strides = metadata->strides;
  view->suboffsets = nullptr;
  view->internal = metadata;
  return 0;
}

void renderedImageReleaseBuffer(PyObject*, Py_buffer* view)
{
  delete static_cast<BufferMetadata*>(view->internal);
  view->internal = nullptr;
}

void appendU8(std::vector<std::uint8_t>& out, std::uint8_t value)
{
  out.push_back(value);
}

void appendBigU32(std::vector<std::uint8_t>& out, std::uint32_t value)
{
  out.push_back(static_cast<std::uint8_t>((value >> 24) & 0xffu));
  out.push_back(static_cast<std::uint8_t>((value >> 16) & 0xffu));
  out.push_back(static_cast<std::uint8_t>((value >> 8) & 0xffu));
  out.push_back(static_cast<std::uint8_t>(value & 0xffu));
}

void appendLittleU16(std::vector<std::uint8_t>& out, std::uint16_t value)
{
  out.push_back(static_cast<std::uint8_t>(value & 0xffu));
  out.push_back(static_cast<std::uint8_t>((value >> 8) & 0xffu));
}

std::uint32_t crc32(const std::uint8_t* data, std::size_t size)
{
  std::uint32_t crc = 0xffffffffu;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      const std::uint32_t mask = 0u - (crc & 1u);
      crc = (crc >> 1) ^ (0xedb88320u & mask);
    }
  }
  return crc ^ 0xffffffffu;
}

std::uint32_t adler32(const std::vector<std::uint8_t>& data)
{
  constexpr std::uint32_t mod = 65521u;
  std::uint32_t a = 1u;
  std::uint32_t b = 0u;
  for (std::uint8_t value : data) {
    a = (a + value) % mod;
    b = (b + a) % mod;
  }
  return (b << 16) | a;
}

void appendChunk(
    std::vector<std::uint8_t>& png,
    const char type[4],
    const std::vector<std::uint8_t>& data)
{
  appendBigU32(png, static_cast<std::uint32_t>(data.size()));
  const std::size_t chunkStart = png.size();
  png.insert(png.end(), type, type + 4);
  png.insert(png.end(), data.begin(), data.end());
  appendBigU32(png, crc32(png.data() + chunkStart, png.size() - chunkStart));
}

std::vector<std::uint8_t> zlibStore(const std::vector<std::uint8_t>& raw)
{
  std::vector<std::uint8_t> zlib;
  zlib.reserve(raw.size() + raw.size() / 65535u * 5u + 16u);
  appendU8(zlib, 0x78u);
  appendU8(zlib, 0x01u);

  std::size_t offset = 0;
  while (offset < raw.size() || raw.empty()) {
    const std::size_t remaining = raw.size() - offset;
    const std::size_t blockSize = std::min<std::size_t>(remaining, 65535u);
    const bool finalBlock = offset + blockSize >= raw.size();
    appendU8(zlib, finalBlock ? 0x01u : 0x00u);
    appendLittleU16(zlib, static_cast<std::uint16_t>(blockSize));
    appendLittleU16(
        zlib,
        static_cast<std::uint16_t>(~static_cast<std::uint16_t>(blockSize)));
    zlib.insert(
        zlib.end(), raw.begin() + offset, raw.begin() + offset + blockSize);
    offset += blockSize;
    if (finalBlock) {
      break;
    }
  }
  appendBigU32(zlib, adler32(raw));
  return zlib;
}

std::vector<std::uint8_t> encodePng(const dart::gui::RenderedImage& image)
{
  if (image.channels != 4) {
    throw std::invalid_argument(
        "RenderedImage PNG encoding requires RGBA data");
  }
  const std::size_t rowBytes
      = static_cast<std::size_t>(image.width) * image.channels;
  const std::size_t expectedSize
      = rowBytes * static_cast<std::size_t>(image.height);
  if (image.pixels.size() != expectedSize) {
    throw std::invalid_argument(
        "RenderedImage pixel storage has inconsistent size");
  }

  std::vector<std::uint8_t> png{
      0x89u, 0x50u, 0x4eu, 0x47u, 0x0du, 0x0au, 0x1au, 0x0au};

  std::vector<std::uint8_t> ihdr;
  ihdr.reserve(13u);
  appendBigU32(ihdr, image.width);
  appendBigU32(ihdr, image.height);
  appendU8(ihdr, 8u);
  appendU8(ihdr, 6u);
  appendU8(ihdr, 0u);
  appendU8(ihdr, 0u);
  appendU8(ihdr, 0u);
  appendChunk(png, "IHDR", ihdr);

  std::vector<std::uint8_t> raw;
  raw.reserve((rowBytes + 1u) * static_cast<std::size_t>(image.height));
  for (std::uint32_t y = 0; y < image.height; ++y) {
    appendU8(raw, 0u);
    const auto rowStart
        = image.pixels.begin() + static_cast<std::ptrdiff_t>(rowBytes * y);
    raw.insert(
        raw.end(), rowStart, rowStart + static_cast<std::ptrdiff_t>(rowBytes));
  }

  appendChunk(png, "IDAT", zlibStore(raw));
  appendChunk(png, "IEND", {});
  return png;
}

dart::gui::OffscreenRenderOptions makeOptions(
    int width,
    int height,
    const std::string& renderBackend,
    int warmupFrames,
    bool highFidelity)
{
  dart::gui::OffscreenRenderOptions options;
  options.width = width;
  options.height = height;
  options.renderBackend = renderBackend;
  options.warmupFrames = warmupFrames;
  options.highFidelity = highFidelity;
  return options;
}

} // namespace

void defGuiOffscreen(nb::module_& m)
{
  using dart::gui::OffscreenRenderer;
  using dart::gui::OffscreenRenderOptions;
  using dart::gui::OrbitCamera;
  using dart::gui::RenderableDescriptor;
  using dart::gui::RenderedImage;

  static PyType_Slot renderedImageSlots[]
      = {{Py_bf_getbuffer, reinterpret_cast<void*>(renderedImageGetBuffer)},
         {Py_bf_releasebuffer,
          reinterpret_cast<void*>(renderedImageReleaseBuffer)},
         {0, nullptr}};

  nb::class_<RenderedImage>(
      m, "RenderedImage", nb::type_slots(renderedImageSlots))
      .def_ro("width", &RenderedImage::width)
      .def_ro("height", &RenderedImage::height)
      .def_ro("channels", &RenderedImage::channels)
      .def_prop_ro(
          "pixels",
          [](const RenderedImage& image) {
            return nb::bytes(
                reinterpret_cast<const char*>(image.pixels.data()),
                image.pixels.size());
          })
      .def("png_bytes", [](const RenderedImage& image) {
        const std::vector<std::uint8_t> png = encodePng(image);
        return nb::bytes(reinterpret_cast<const char*>(png.data()), png.size());
      });

  nb::class_<OffscreenRenderOptions>(m, "OffscreenRenderOptions")
      .def(nb::init<>())
      .def_rw("width", &OffscreenRenderOptions::width)
      .def_rw("height", &OffscreenRenderOptions::height)
      .def_rw("render_backend", &OffscreenRenderOptions::renderBackend)
      .def_rw("warmup_frames", &OffscreenRenderOptions::warmupFrames)
      .def_rw("high_fidelity", &OffscreenRenderOptions::highFidelity);

  nb::class_<OffscreenRenderer>(m, "OffscreenRenderer")
      .def(
          nb::new_([](int width,
                      int height,
                      const std::string& renderBackend,
                      int warmupFrames,
                      bool highFidelity) {
            return new OffscreenRenderer(makeOptions(
                width, height, renderBackend, warmupFrames, highFidelity));
          }),
          nb::arg("width") = 640,
          nb::arg("height") = 480,
          nb::arg("render_backend") = "",
          nb::arg("warmup_frames") = 2,
          nb::arg("high_fidelity") = false)
      .def(
          "render",
          [](OffscreenRenderer& renderer,
             const std::vector<RenderableDescriptor>& descriptors,
             const OrbitCamera& camera,
             const std::optional<dart::gui::DebugScene>& debug) {
            nb::gil_scoped_release release;
            if (debug.has_value()) {
              return renderer.render(descriptors, camera, *debug);
            }
            return renderer.render(descriptors, camera);
          },
          nb::arg("descriptors"),
          nb::arg("camera"),
          nb::arg("debug") = nb::none())
      .def(
          "render_descriptors",
          [](OffscreenRenderer& renderer,
             const std::vector<RenderableDescriptor>& descriptors,
             const OrbitCamera& camera,
             const std::optional<dart::gui::DebugScene>& debug) {
            nb::gil_scoped_release release;
            if (debug.has_value()) {
              return renderer.render(descriptors, camera, *debug);
            }
            return renderer.render(descriptors, camera);
          },
          nb::arg("descriptors"),
          nb::arg("camera"),
          nb::arg("debug") = nb::none());
}

} // namespace dart::python_nb
