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

#include "render_context.hpp"

#include "native_window.hpp"

#include <dart/gui/RenderingUnavailableError.hpp>

#include <filament/Engine.h>
#include <filament/Renderer.h>
#include <filament/View.h>
#include <utils/EntityManager.h>

#include <iostream>
#include <stdexcept>
#include <utility>
#include <vector>

#if defined(__linux__)
  #include <bluevk/BlueVK.h>
  #include <dlfcn.h>
#endif

namespace dart::gui::detail {
void checkFilamentBackendAvailability(bool headless)
{
#if defined(__linux__)
  using Library = std::unique_ptr<void, int (*)(void*)>;
  if (!headless) {
    for (const char* name : {"libX11.so.6", "libGL.so.1"}) {
      Library library(dlopen(name, RTLD_NOW | RTLD_LOCAL), dlclose);
      if (!library)
        throw RenderingUnavailableError(
            std::string("Window rendering cannot load ") + name
            + "; install an X11/OpenGL stack (libx11-6, libgl1, "
              "libgl1-mesa-dri) and use an X11 or XWayland display.");
    }
    return;
  }
  constexpr const char* requirement
      = "Headless rendering requires a Vulkan loader and usable ICD/driver "
        "(install libvulkan1 and mesa-vulkan-drivers, or a vendor driver).";
  Library library(dlopen("libvulkan.so.1", RTLD_NOW | RTLD_LOCAL), dlclose);
  if (!library)
    throw RenderingUnavailableError(
        std::string("Cannot load libvulkan.so.1. ") + requirement);
  const auto createInstance = reinterpret_cast<PFN_vkCreateInstance>(
      dlsym(library.get(), "vkCreateInstance"));
  const auto destroyInstance = reinterpret_cast<PFN_vkDestroyInstance>(
      dlsym(library.get(), "vkDestroyInstance"));
  const auto enumerateDevices
      = reinterpret_cast<PFN_vkEnumeratePhysicalDevices>(
          dlsym(library.get(), "vkEnumeratePhysicalDevices"));
  const auto getProperties
      = reinterpret_cast<PFN_vkGetPhysicalDeviceProperties>(
          dlsym(library.get(), "vkGetPhysicalDeviceProperties"));
  const auto getQueues
      = reinterpret_cast<PFN_vkGetPhysicalDeviceQueueFamilyProperties>(
          dlsym(library.get(), "vkGetPhysicalDeviceQueueFamilyProperties"));
  const auto getExtensions
      = reinterpret_cast<PFN_vkEnumerateDeviceExtensionProperties>(
          dlsym(library.get(), "vkEnumerateDeviceExtensionProperties"));
  if (!createInstance || !destroyInstance || !enumerateDevices || !getProperties
      || !getQueues || !getExtensions)
    throw RenderingUnavailableError(requirement);
  // Filament panics on its driver thread if the loader has no usable ICD.
  VkApplicationInfo application{};
  application.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
  application.apiVersion = VK_API_VERSION_1_1;
  VkInstanceCreateInfo information{};
  information.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
  information.pApplicationInfo = &application;
  VkInstance instance = VK_NULL_HANDLE;
  const auto result = createInstance(&information, nullptr, &instance);
  if (result != VK_SUCCESS)
    throw RenderingUnavailableError(
        std::string("Cannot create a Vulkan instance (VkResult ")
        + std::to_string(result) + "). " + requirement);
  std::uint32_t count = 0;
  bool available = false;
  try {
    if (enumerateDevices(instance, &count, nullptr) == VK_SUCCESS
        && count > 0) {
      std::vector<VkPhysicalDevice> devices(count);
      if (enumerateDevices(instance, &count, devices.data()) == VK_SUCCESS) {
        devices.resize(count);
        // Match Filament 1.77.3's physical-device eligibility checks.
        for (auto device : devices) {
          VkPhysicalDeviceProperties properties{};
          getProperties(device, &properties);
          if (properties.apiVersion < VK_API_VERSION_1_1)
            continue;
          std::uint32_t queueCount = 0;
          getQueues(device, &queueCount, nullptr);
          std::vector<VkQueueFamilyProperties> queues(queueCount);
          getQueues(device, &queueCount, queues.data());
          bool graphics = false;
          for (const auto& queue : queues)
            graphics |= (queue.queueFlags & VK_QUEUE_GRAPHICS_BIT) != 0;
          if (!graphics)
            continue;
          std::uint32_t extensionCount = 0;
          if (getExtensions(device, nullptr, &extensionCount, nullptr)
              != VK_SUCCESS)
            continue;
          std::vector<VkExtensionProperties> extensions(extensionCount);
          if (getExtensions(device, nullptr, &extensionCount, extensions.data())
              != VK_SUCCESS)
            continue;
          for (const auto& extension : extensions)
            if (std::string(extension.extensionName)
                == VK_KHR_SWAPCHAIN_EXTENSION_NAME)
              available = true;
          if (available)
            break;
        }
      }
    }
  } catch (...) {
    destroyInstance(instance, nullptr);
    throw;
  }
  destroyInstance(instance, nullptr);
  if (!available)
    throw RenderingUnavailableError(
        std::string("No usable Vulkan physical device. ") + requirement);
#else
  (void)headless;
#endif
}

FilamentRenderContext createFilamentRenderContext(
    int width, int height, bool headless, void* nativeWindow)
{
  if (width <= 0 || height <= 0)
    throw std::invalid_argument(
        "Filament requires positive framebuffer dimensions");
  if (!headless && nativeWindow == nullptr)
    throw std::invalid_argument(
        "Filament requires a native window for interactive rendering");
  checkFilamentBackendAvailability(headless);
  FilamentRenderContext context;
  using Backend = ::filament::Engine::Backend;
#if defined(__APPLE__)
  const Backend backend = Backend::METAL;
  context.backendName = "metal";
#elif defined(__linux__)
  const Backend backend = headless ? Backend::VULKAN : Backend::OPENGL;
  context.backendName = headless ? "vulkan" : "opengl";
  if (headless)
    context.platform.reset(static_cast<::filament::backend::Platform*>(
        dartCreateHeadlessVulkanPlatform()));
#else
  const Backend backend = Backend::OPENGL;
  context.backendName = "opengl";
#endif
  ::filament::Engine::Config configuration;
  // All fixed key/fill/rim lights must contribute when that rig is enabled;
  // the SDK otherwise evaluates only the dominant directional light.
  configuration.enableMultipleDirectionalLights = true;
  auto builder = ::filament::Engine::Builder()
                     .backend(backend)
                     .platform(context.platform.get())
                     .config(&configuration);
#if defined(__linux__)
  if (backend == Backend::OPENGL) {
    // Filament 1.77.3 GLX's infinite telemetry-fence wait overflows
    // libstdc++'s steady-to-system-clock conversion and blocks driver progress.
    // GPU timer queries remain available; only the completion timestamp is off.
    builder.feature("engine.frame_info.disable_gpu_complete_metric", true);
    std::cerr << "[dart::gui] Filament GLX: disabling affected GPU completion "
                 "timestamp wait\n";
  }
#endif
  context.engine = builder.build();
  if (context.engine == nullptr)
    throw std::runtime_error(
        "Failed to initialize Filament " + context.backendName + " backend");
  try {
    auto* platform = context.engine->getPlatform();
    if (platform != nullptr && backend != Backend::METAL) {
      const auto type = backend == Backend::VULKAN
                            ? ::filament::backend::Platform::DeviceInfoType::
                                VULKAN_DEVICE_NAME
                            : ::filament::backend::Platform::DeviceInfoType::
                                OPENGL_RENDERER;
      const auto name = platform->getDeviceInfo(
          type,
          const_cast<::filament::backend::Driver*>(
              context.engine->getDriver()));
      context.deviceName = name.c_str();
    }
#if defined(__APPLE__)
    context.deviceName
        = configureFilamentMetalWindow(*context.engine, nativeWindow);
#endif
    context.renderer = context.engine->createRenderer();
    ::filament::Renderer::DisplayInfo displayInfo;
    displayInfo.refreshRate = 0.0f;
    context.renderer->setDisplayInfo(displayInfo);
    context.swapChain = headless
                            ? context.engine->createSwapChain(
                                static_cast<std::uint32_t>(width),
                                static_cast<std::uint32_t>(height))
                            : context.engine->createSwapChain(nativeWindow);
    context.scene = context.engine->createScene();
    context.view = context.engine->createView();
    context.cameraEntity = utils::EntityManager::get().create();
    context.camera = context.engine->createCamera(context.cameraEntity);
    context.view->setScene(context.scene);
    context.view->setCamera(context.camera);
    if (context.swapChain == nullptr || context.scene == nullptr
        || context.view == nullptr || context.camera == nullptr)
      throw std::runtime_error("Filament render context is incomplete");
  } catch (...) {
    destroyFilamentRenderContext(context);
    throw;
  }
  return context;
}

void destroyFilamentRenderContext(FilamentRenderContext& context)
{
  if (context.engine == nullptr)
    return;
  // Complete all borrowed upload/readback buffers before destroying their
  // owner.
  context.engine->flushAndWait();
  context.engine->pumpMessageQueues();
  if (context.view != nullptr)
    context.engine->destroy(context.view);
  if (context.cameraEntity) {
    context.engine->destroyCameraComponent(context.cameraEntity);
    utils::EntityManager::get().destroy(context.cameraEntity);
  }
  if (context.scene != nullptr)
    context.engine->destroy(context.scene);
  if (context.renderer != nullptr)
    context.engine->destroy(context.renderer);
  if (context.swapChain != nullptr)
    context.engine->destroy(context.swapChain);
  ::filament::Engine::destroy(&context.engine);
  context = {};
}
} // namespace dart::gui::detail
