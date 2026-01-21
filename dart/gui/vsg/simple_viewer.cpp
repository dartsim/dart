/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "dart/gui/vsg/simple_viewer.hpp"

#include "dart/gui/vsg/conversions.hpp"
#include "dart/gui/vsg/debug_draw.hpp"

#include <vsg/all.h>

#include <fstream>

namespace dart::gui::vsg {

namespace {

::vsg::ref_ptr<::vsg::RenderPass> createOffscreenRenderPass(
    ::vsg::Device* device, VkFormat imageFormat, VkFormat depthFormat)
{
  auto colorAttachment = ::vsg::defaultColorAttachment(imageFormat);
  auto depthAttachment = ::vsg::defaultDepthAttachment(depthFormat);
  colorAttachment.finalLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;

  ::vsg::RenderPass::Attachments attachments{colorAttachment, depthAttachment};

  ::vsg::AttachmentReference colorAttachmentRef = {};
  colorAttachmentRef.attachment = 0;
  colorAttachmentRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

  ::vsg::AttachmentReference depthAttachmentRef = {};
  depthAttachmentRef.attachment = 1;
  depthAttachmentRef.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

  ::vsg::SubpassDescription subpass = {};
  subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
  subpass.colorAttachments.emplace_back(colorAttachmentRef);
  subpass.depthStencilAttachments.emplace_back(depthAttachmentRef);

  ::vsg::RenderPass::Subpasses subpasses{subpass};

  ::vsg::SubpassDependency colorDep = {};
  colorDep.srcSubpass = VK_SUBPASS_EXTERNAL;
  colorDep.dstSubpass = 0;
  colorDep.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
  colorDep.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
  colorDep.srcAccessMask = 0;
  colorDep.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT
                           | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
  colorDep.dependencyFlags = 0;

  ::vsg::SubpassDependency depthDep = {};
  depthDep.srcSubpass = VK_SUBPASS_EXTERNAL;
  depthDep.dstSubpass = 0;
  depthDep.srcStageMask = VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT
                          | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
  depthDep.dstStageMask = VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT
                          | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
  depthDep.srcAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
  depthDep.dstAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT
                           | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
  depthDep.dependencyFlags = 0;

  ::vsg::RenderPass::Dependencies deps{colorDep, depthDep};

  return ::vsg::RenderPass::create(device, attachments, subpasses, deps);
}

::vsg::ref_ptr<::vsg::ImageView> createColorImageView(
    ::vsg::ref_ptr<::vsg::Device> device,
    const VkExtent2D& extent,
    VkFormat imageFormat)
{
  auto colorImage = ::vsg::Image::create();
  colorImage->imageType = VK_IMAGE_TYPE_2D;
  colorImage->format = imageFormat;
  colorImage->extent = VkExtent3D{extent.width, extent.height, 1};
  colorImage->mipLevels = 1;
  colorImage->arrayLayers = 1;
  colorImage->samples = VK_SAMPLE_COUNT_1_BIT;
  colorImage->tiling = VK_IMAGE_TILING_OPTIMAL;
  colorImage->usage
      = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
  colorImage->initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  colorImage->flags = 0;
  colorImage->sharingMode = VK_SHARING_MODE_EXCLUSIVE;

  return ::vsg::createImageView(device, colorImage, VK_IMAGE_ASPECT_COLOR_BIT);
}

::vsg::ref_ptr<::vsg::ImageView> createDepthImageView(
    ::vsg::ref_ptr<::vsg::Device> device,
    const VkExtent2D& extent,
    VkFormat depthFormat)
{
  auto depthImage = ::vsg::Image::create();
  depthImage->imageType = VK_IMAGE_TYPE_2D;
  depthImage->extent = VkExtent3D{extent.width, extent.height, 1};
  depthImage->mipLevels = 1;
  depthImage->arrayLayers = 1;
  depthImage->samples = VK_SAMPLE_COUNT_1_BIT;
  depthImage->format = depthFormat;
  depthImage->tiling = VK_IMAGE_TILING_OPTIMAL;
  depthImage->usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT
                      | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
  depthImage->initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  depthImage->flags = 0;
  depthImage->sharingMode = VK_SHARING_MODE_EXCLUSIVE;

  return ::vsg::createImageView(
      device, depthImage, ::vsg::computeAspectFlagsForFormat(depthFormat));
}

} // namespace

SimpleViewer::SimpleViewer(int width, int height, const std::string& title)
  : m_width(width), m_height(height), m_title(title), m_headless(false)
{
  m_root = ::vsg::Group::create();
  m_sceneRoot = ::vsg::Group::create();
  m_root->addChild(m_sceneRoot);
  setupWindowedViewer();
}

SimpleViewer::SimpleViewer(HeadlessTag, int width, int height)
  : m_width(width), m_height(height), m_headless(true)
{
  m_root = ::vsg::Group::create();
  m_sceneRoot = ::vsg::Group::create();
  m_root->addChild(m_sceneRoot);
  setupHeadlessViewer();
}

SimpleViewer SimpleViewer::headless(int width, int height)
{
  return SimpleViewer(HeadlessTag{}, width, height);
}

SimpleViewer::~SimpleViewer() = default;

bool SimpleViewer::isHeadless() const
{
  return m_headless;
}

void SimpleViewer::setupWindowedViewer()
{
  auto windowTraits = ::vsg::WindowTraits::create();
  windowTraits->windowTitle = m_title;
  windowTraits->width = m_width;
  windowTraits->height = m_height;
  windowTraits->debugLayer = false;
  windowTraits->apiDumpLayer = false;
  windowTraits->swapchainPreferences.imageUsage
      = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;

  m_viewer = ::vsg::Viewer::create();
  m_window = ::vsg::Window::create(windowTraits);
  if (!m_window) {
    throw std::runtime_error("Failed to create VSG window");
  }
  m_viewer->addWindow(m_window);

  m_viewer->addEventHandler(::vsg::CloseHandler::create(m_viewer));

  setupCamera();
}

void SimpleViewer::setupHeadlessViewer()
{
  ::vsg::Names instanceExtensions;
  ::vsg::Names requestedLayers;
  auto validatedNames = ::vsg::validateInstanceLayerNames(requestedLayers);

  auto instance = ::vsg::Instance::create(instanceExtensions, validatedNames);
  auto [physicalDevice, queueFamily]
      = instance->getPhysicalDeviceAndQueueFamily(VK_QUEUE_GRAPHICS_BIT);

  if (!physicalDevice || queueFamily < 0) {
    throw std::runtime_error(
        "Failed to find suitable physical device for headless rendering");
  }

  m_queueFamily = queueFamily;

  ::vsg::Names deviceExtensions;
  ::vsg::QueueSettings queueSettings{::vsg::QueueSetting{queueFamily, {1.0}}};

  auto deviceFeatures = ::vsg::DeviceFeatures::create();
  deviceFeatures->get().samplerAnisotropy = VK_TRUE;

  m_device = ::vsg::Device::create(
      physicalDevice,
      queueSettings,
      validatedNames,
      deviceExtensions,
      deviceFeatures);

  VkExtent2D extent{
      static_cast<uint32_t>(m_width), static_cast<uint32_t>(m_height)};
  VkFormat imageFormat = VK_FORMAT_R8G8B8A8_UNORM;
  VkFormat depthFormat = VK_FORMAT_D32_SFLOAT;

  m_colorImageView = createColorImageView(m_device, extent, imageFormat);
  m_depthImageView = createDepthImageView(m_device, extent, depthFormat);

  auto renderPass
      = createOffscreenRenderPass(m_device, imageFormat, depthFormat);
  m_framebuffer = ::vsg::Framebuffer::create(
      renderPass,
      ::vsg::ImageViews{m_colorImageView, m_depthImageView},
      extent.width,
      extent.height,
      1);

  m_viewer = ::vsg::Viewer::create();

  setupCamera();
}

void SimpleViewer::setupCamera()
{
  VkExtent2D extent;
  if (m_headless) {
    extent = VkExtent2D{
        static_cast<uint32_t>(m_width), static_cast<uint32_t>(m_height)};
  } else {
    extent = m_window->extent2D();
  }

  double aspectRatio
      = static_cast<double>(extent.width) / static_cast<double>(extent.height);

  m_lookAt = ::vsg::LookAt::create(
      ::vsg::dvec3(5.0, 5.0, 5.0),
      ::vsg::dvec3(0.0, 0.0, 0.0),
      ::vsg::dvec3(0.0, 0.0, 1.0));

  auto perspective
      = ::vsg::Perspective::create(60.0, aspectRatio, 0.01, 1000.0);

  auto viewportState = ::vsg::ViewportState::create(extent);
  m_camera = ::vsg::Camera::create(perspective, m_lookAt, viewportState);

  if (!m_headless) {
    auto trackball = ::vsg::Trackball::create(m_camera);
    m_viewer->addEventHandler(trackball);
  }
}

void SimpleViewer::compile()
{
  if (!m_needsCompile) {
    return;
  }

  if (m_headless) {
    auto renderGraph = ::vsg::RenderGraph::create();
    renderGraph->framebuffer = m_framebuffer;
    renderGraph->renderArea.offset = {0, 0};
    renderGraph->renderArea.extent = VkExtent2D{
        static_cast<uint32_t>(m_width), static_cast<uint32_t>(m_height)};

    VkClearColorValue clearColor{
        {static_cast<float>(m_backgroundColor.x()),
         static_cast<float>(m_backgroundColor.y()),
         static_cast<float>(m_backgroundColor.z()),
         static_cast<float>(m_backgroundColor.w())}};
    renderGraph->setClearValues(clearColor, VkClearDepthStencilValue{0.0f, 0});

    auto view = ::vsg::View::create(m_camera, m_root);
    view->addChild(::vsg::createHeadlight());
    renderGraph->addChild(view);

    m_commandGraph = ::vsg::CommandGraph::create(
        m_device, static_cast<uint32_t>(m_queueFamily));
    m_commandGraph->addChild(renderGraph);

    m_viewer->assignRecordAndSubmitTaskAndPresentation({m_commandGraph});
    m_viewer->compile();
  } else {
    auto commandGraph = ::vsg::createCommandGraphForView(
        m_window, m_camera, m_root, VK_SUBPASS_CONTENTS_INLINE, true);

    m_viewer->assignRecordAndSubmitTaskAndPresentation({commandGraph});
    m_viewer->compile();
  }

  m_needsCompile = false;
}

void SimpleViewer::setScene(::vsg::ref_ptr<::vsg::Node> scene)
{
  m_sceneRoot->children.clear();
  if (scene) {
    m_sceneRoot->addChild(scene);
  }
  m_needsCompile = true;
}

::vsg::ref_ptr<::vsg::Group> SimpleViewer::getRoot() const
{
  return m_sceneRoot;
}

void SimpleViewer::addNode(::vsg::ref_ptr<::vsg::Node> node)
{
  if (node) {
    m_sceneRoot->addChild(node);
    m_needsCompile = true;
  }
}

void SimpleViewer::removeNode(::vsg::ref_ptr<::vsg::Node> node)
{
  if (!node) {
    return;
  }
  auto& children = m_sceneRoot->children;
  auto it = std::find(children.begin(), children.end(), node);
  if (it != children.end()) {
    children.erase(it);
    m_needsCompile = true;
  }
}

void SimpleViewer::clear()
{
  m_sceneRoot->children.clear();
  m_needsCompile = true;
}

void SimpleViewer::lookAt(
    const Eigen::Vector3d& eye,
    const Eigen::Vector3d& center,
    const Eigen::Vector3d& up)
{
  m_lookAt->eye = toVsg(eye);
  m_lookAt->center = toVsg(center);
  m_lookAt->up = toVsg(up);
}

void SimpleViewer::resetCamera()
{
  lookAt(
      Eigen::Vector3d(5.0, 5.0, 5.0),
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d::UnitZ());
}

void SimpleViewer::setBackgroundColor(const Eigen::Vector4d& color)
{
  m_backgroundColor = color;
  m_needsCompile = true;
}

bool SimpleViewer::frame()
{
  compile();

  if (m_headless) {
    if (m_viewer->advanceToNextFrame()) {
      m_viewer->update();
      m_viewer->recordAndSubmit();
      return true;
    }
    return false;
  }

  if (m_viewer->advanceToNextFrame()) {
    m_viewer->handleEvents();
    m_viewer->update();
    m_viewer->recordAndSubmit();
    m_viewer->present();
    return true;
  }
  return false;
}

void SimpleViewer::run()
{
  compile();
  if (m_headless) {
    frame();
    return;
  }
  while (frame()) {
  }
}

bool SimpleViewer::shouldClose() const
{
  if (m_headless) {
    return true;
  }
  return !m_viewer->active();
}

void SimpleViewer::addGrid(double size, double spacing)
{
  addNode(createGrid(size, spacing));
}

void SimpleViewer::addAxes(double length)
{
  addNode(createAxes(length));
}

std::vector<uint8_t> SimpleViewer::captureBuffer()
{
  compile();

  if (!frame()) {
    return {};
  }

  ::vsg::ref_ptr<::vsg::Device> device;
  ::vsg::ref_ptr<::vsg::PhysicalDevice> physicalDevice;
  ::vsg::ref_ptr<::vsg::Image> sourceImage;
  VkFormat sourceImageFormat;
  uint32_t width;
  uint32_t height;
  VkImageLayout srcInitialLayout;
  VkImageLayout srcFinalLayout;
  int queueFamilyIndex;

  if (m_headless) {
    device = m_device;
    physicalDevice = device->getInstance()->getPhysicalDevice(0);
    sourceImage = m_colorImageView->image;
    sourceImageFormat = VK_FORMAT_R8G8B8A8_UNORM;
    width = static_cast<uint32_t>(m_width);
    height = static_cast<uint32_t>(m_height);
    srcInitialLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
    srcFinalLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
    queueFamilyIndex = m_queueFamily;
  } else {
    device = m_window->getDevice();
    physicalDevice = m_window->getPhysicalDevice();
    sourceImage = m_window->imageView(m_window->imageIndex(1))->image;
    sourceImageFormat = m_window->getSwapchain()->getImageFormat();
    width = m_window->extent2D().width;
    height = m_window->extent2D().height;
    srcInitialLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
    srcFinalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
    queueFamilyIndex = physicalDevice->getQueueFamily(VK_QUEUE_GRAPHICS_BIT);
  }

  VkFormat targetImageFormat = VK_FORMAT_R8G8B8A8_UNORM;

  VkFormatProperties srcFormatProperties;
  vkGetPhysicalDeviceFormatProperties(
      *(physicalDevice), sourceImageFormat, &srcFormatProperties);
  VkFormatProperties destFormatProperties;
  vkGetPhysicalDeviceFormatProperties(
      *(physicalDevice), VK_FORMAT_R8G8B8A8_UNORM, &destFormatProperties);

  bool supportsBlit = ((srcFormatProperties.optimalTilingFeatures
                        & VK_FORMAT_FEATURE_BLIT_SRC_BIT)
                       != 0)
                      && ((destFormatProperties.linearTilingFeatures
                           & VK_FORMAT_FEATURE_BLIT_DST_BIT)
                          != 0);

  if (!supportsBlit) {
    targetImageFormat = sourceImageFormat;
  }

  auto destinationImage = ::vsg::Image::create();
  destinationImage->imageType = VK_IMAGE_TYPE_2D;
  destinationImage->format = targetImageFormat;
  destinationImage->extent.width = width;
  destinationImage->extent.height = height;
  destinationImage->extent.depth = 1;
  destinationImage->arrayLayers = 1;
  destinationImage->mipLevels = 1;
  destinationImage->initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  destinationImage->samples = VK_SAMPLE_COUNT_1_BIT;
  destinationImage->tiling = VK_IMAGE_TILING_LINEAR;
  destinationImage->usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT;
  destinationImage->compile(device);

  auto deviceMemory = ::vsg::DeviceMemory::create(
      device,
      destinationImage->getMemoryRequirements(device->deviceID),
      VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT
          | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
  destinationImage->bind(deviceMemory, 0);

  auto commands = ::vsg::Commands::create();

  auto transitionDestToTransferDst = ::vsg::ImageMemoryBarrier::create(
      0,
      VK_ACCESS_TRANSFER_WRITE_BIT,
      VK_IMAGE_LAYOUT_UNDEFINED,
      VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
      VK_QUEUE_FAMILY_IGNORED,
      VK_QUEUE_FAMILY_IGNORED,
      destinationImage,
      VkImageSubresourceRange{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1});

  auto transitionSrcToTransferSrc = ::vsg::ImageMemoryBarrier::create(
      VK_ACCESS_MEMORY_READ_BIT,
      VK_ACCESS_TRANSFER_READ_BIT,
      srcInitialLayout,
      VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
      VK_QUEUE_FAMILY_IGNORED,
      VK_QUEUE_FAMILY_IGNORED,
      sourceImage,
      VkImageSubresourceRange{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1});

  commands->addChild(
      ::vsg::PipelineBarrier::create(
          VK_PIPELINE_STAGE_TRANSFER_BIT,
          VK_PIPELINE_STAGE_TRANSFER_BIT,
          0,
          transitionDestToTransferDst,
          transitionSrcToTransferSrc));

  if (supportsBlit) {
    VkImageBlit region{};
    region.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    region.srcSubresource.layerCount = 1;
    region.srcOffsets[0] = VkOffset3D{0, 0, 0};
    region.srcOffsets[1] = VkOffset3D{
        static_cast<int32_t>(width), static_cast<int32_t>(height), 1};
    region.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    region.dstSubresource.layerCount = 1;
    region.dstOffsets[0] = VkOffset3D{0, 0, 0};
    region.dstOffsets[1] = VkOffset3D{
        static_cast<int32_t>(width), static_cast<int32_t>(height), 1};

    auto blitImage = ::vsg::BlitImage::create();
    blitImage->srcImage = sourceImage;
    blitImage->srcImageLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
    blitImage->dstImage = destinationImage;
    blitImage->dstImageLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    blitImage->regions.push_back(region);
    blitImage->filter = VK_FILTER_NEAREST;
    commands->addChild(blitImage);
  } else {
    VkImageCopy region{};
    region.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    region.srcSubresource.layerCount = 1;
    region.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    region.dstSubresource.layerCount = 1;
    region.extent.width = width;
    region.extent.height = height;
    region.extent.depth = 1;

    auto copyImage = ::vsg::CopyImage::create();
    copyImage->srcImage = sourceImage;
    copyImage->srcImageLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
    copyImage->dstImage = destinationImage;
    copyImage->dstImageLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    copyImage->regions.push_back(region);
    commands->addChild(copyImage);
  }

  auto transitionDestToGeneral = ::vsg::ImageMemoryBarrier::create(
      VK_ACCESS_TRANSFER_WRITE_BIT,
      VK_ACCESS_MEMORY_READ_BIT,
      VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
      VK_IMAGE_LAYOUT_GENERAL,
      VK_QUEUE_FAMILY_IGNORED,
      VK_QUEUE_FAMILY_IGNORED,
      destinationImage,
      VkImageSubresourceRange{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1});

  auto transitionSrcBack = ::vsg::ImageMemoryBarrier::create(
      VK_ACCESS_TRANSFER_READ_BIT,
      VK_ACCESS_MEMORY_READ_BIT,
      VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
      srcFinalLayout,
      VK_QUEUE_FAMILY_IGNORED,
      VK_QUEUE_FAMILY_IGNORED,
      sourceImage,
      VkImageSubresourceRange{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1});

  commands->addChild(
      ::vsg::PipelineBarrier::create(
          VK_PIPELINE_STAGE_TRANSFER_BIT,
          VK_PIPELINE_STAGE_TRANSFER_BIT,
          0,
          transitionDestToGeneral,
          transitionSrcBack));

  auto fence = ::vsg::Fence::create(device);
  auto commandPool = ::vsg::CommandPool::create(
      device, static_cast<uint32_t>(queueFamilyIndex));
  auto queue = device->getQueue(static_cast<uint32_t>(queueFamilyIndex));

  ::vsg::submitCommandsToQueue(
      commandPool,
      fence,
      100000000000,
      queue,
      [&](::vsg::CommandBuffer& commandBuffer) {
        commands->record(commandBuffer);
      });

  VkImageSubresource subResource{VK_IMAGE_ASPECT_COLOR_BIT, 0, 0};
  VkSubresourceLayout subResourceLayout;
  vkGetImageSubresourceLayout(
      *device,
      destinationImage->vk(device->deviceID),
      &subResource,
      &subResourceLayout);

  size_t destRowWidth = width * 4;
  std::vector<uint8_t> result(width * height * 4);

  auto mappedData = ::vsg::MappedData<::vsg::ubyteArray>::create(
      deviceMemory,
      subResourceLayout.offset,
      0,
      ::vsg::Data::Properties{targetImageFormat},
      subResourceLayout.rowPitch * height);

  for (uint32_t row = 0; row < height; ++row) {
    std::memcpy(
        result.data() + row * destRowWidth,
        mappedData->dataPointer(row * subResourceLayout.rowPitch),
        destRowWidth);
  }

  return result;
}

bool SimpleViewer::saveScreenshot(const std::string& filename)
{
  auto buffer = captureBuffer();
  if (buffer.empty()) {
    return false;
  }

  uint32_t width = static_cast<uint32_t>(m_width);
  uint32_t height = static_cast<uint32_t>(m_height);

  if (filename.size() >= 4 && filename.substr(filename.size() - 4) == ".ppm") {
    std::ofstream file(filename, std::ios::binary);
    if (!file) {
      return false;
    }
    file << "P6\n" << width << " " << height << "\n255\n";
    for (uint32_t y = 0; y < height; ++y) {
      for (uint32_t x = 0; x < width; ++x) {
        size_t idx = (y * width + x) * 4;
        file.put(static_cast<char>(buffer[idx]));
        file.put(static_cast<char>(buffer[idx + 1]));
        file.put(static_cast<char>(buffer[idx + 2]));
      }
    }
    return file.good();
  }

  auto imageData = ::vsg::ubvec4Array2D::create(
      width, height, ::vsg::Data::Properties{VK_FORMAT_R8G8B8A8_UNORM});
  std::memcpy(imageData->dataPointer(), buffer.data(), buffer.size());

  return ::vsg::write(imageData, filename);
}

} // namespace dart::gui::vsg
