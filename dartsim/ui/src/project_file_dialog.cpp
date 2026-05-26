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
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER AND
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <dartsim_ui/project_file_dialog.hpp>
#include <nfd.h>

#include <memory>
#include <string>
#include <utility>

namespace dartsim::ui {

namespace {

ProjectFileDialogResult selectedPath(std::string path)
{
  return {ProjectFileDialogStatus::Selected, std::move(path), {}};
}

ProjectFileDialogResult canceledDialog()
{
  return {ProjectFileDialogStatus::Canceled, {}, {}};
}

ProjectFileDialogResult failedDialog(std::string error)
{
  return {ProjectFileDialogStatus::Failed, {}, std::move(error)};
}

std::string nativeDialogError(std::string fallback)
{
  const char* error = NFD_GetError();
  return error == nullptr || *error == '\0' ? std::move(fallback)
                                            : std::string(error);
}

nfdwindowhandle_t nativeParentWindow(void* window)
{
  nfdwindowhandle_t parent{};
  if (window == nullptr) {
    return parent;
  }

#if defined(__linux__)
  parent.type = NFD_WINDOW_HANDLE_TYPE_X11;
  parent.handle = window;
#elif defined(_WIN32)
  parent.type = NFD_WINDOW_HANDLE_TYPE_WINDOWS;
  parent.handle = window;
#elif defined(__APPLE__)
  parent.type = NFD_WINDOW_HANDLE_TYPE_COCOA;
  parent.handle = window;
#else
  (void)window;
#endif
  return parent;
}

} // namespace

ProjectFileDialogResult nativeProjectFileDialog(
    const ProjectFileDialogRequest& request)
{
  const nfdresult_t initResult = NFD_Init();
  if (initResult != NFD_OKAY) {
    return failedDialog(nativeDialogError("native dialog init failed"));
  }

  struct NfdQuit
  {
    ~NfdQuit()
    {
      NFD_Quit();
    }
  } quit;

  const nfdu8filteritem_t filters[] = {
      {"DART project", "dartsim"},
      {"All files", "*"},
  };

  nfdu8char_t* rawPath = nullptr;
  const auto freePath = [](nfdu8char_t* path) {
    NFD_FreePathU8(path);
  };
  std::unique_ptr<nfdu8char_t, decltype(freePath)> pathGuard(nullptr, freePath);

  nfdresult_t result = NFD_ERROR;
  if (request.kind == ProjectFileDialogKind::Open) {
    nfdopendialogu8args_t args{};
    args.filterList = filters;
    args.filterCount = 2;
    args.defaultPath
        = request.defaultPath.empty() ? nullptr : request.defaultPath.c_str();
    args.parentWindow = nativeParentWindow(request.parentNativeWindow);
    result = NFD_OpenDialogU8_With(&rawPath, &args);
  } else {
    nfdsavedialogu8args_t args{};
    args.filterList = filters;
    args.filterCount = 2;
    args.defaultPath
        = request.defaultPath.empty() ? nullptr : request.defaultPath.c_str();
    args.defaultName = request.defaultName.empty()
                           ? kDefaultProjectPath
                           : request.defaultName.c_str();
    args.parentWindow = nativeParentWindow(request.parentNativeWindow);
    result = NFD_SaveDialogU8_With(&rawPath, &args);
  }
  pathGuard.reset(rawPath);

  if (result == NFD_OKAY) {
    return selectedPath(rawPath == nullptr ? std::string() : rawPath);
  }
  if (result == NFD_CANCEL) {
    return canceledDialog();
  }

  return failedDialog(nativeDialogError("native dialog failed"));
}

} // namespace dartsim::ui
