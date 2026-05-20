# Patch: Add null pointer validation to ImGui functions
# Fixes: https://github.com/dartsim/dart/issues/2516
# Fixes: https://github.com/dartsim/dart/issues/2668
#
# ImGui v1.84.2 does not validate input pointers in AddFontFromMemory*
# functions, causing SEGV when nullptr is passed (e.g., from a failed
# resource load). It also assumes ColorPicker4 has a current context and window.
# This patch adds early-return guards.
#
# Usage: cmake -DIMGUI_SOURCE_DIR=<path> -P imgui_null_font_guard.cmake

if(NOT DEFINED IMGUI_SOURCE_DIR)
  message(FATAL_ERROR "IMGUI_SOURCE_DIR must be set")
endif()

set(file "${IMGUI_SOURCE_DIR}/imgui_draw.cpp")

if(NOT EXISTS "${file}")
  message(FATAL_ERROR "imgui_draw.cpp not found at ${file}")
endif()

file(READ "${file}" content)

set(patches_applied FALSE)

# Guard: only patch each issue once (idempotent)
string(FIND "${content}" "issue #2516" issue2516_patched)
if(issue2516_patched EQUAL -1)
  set(before_issue2516 "${content}")

  # Patch AddFontFromMemoryTTF
  string(REPLACE
    "IM_ASSERT(!Locked && \"Cannot modify a locked ImFontAtlas between NewFrame() and EndFrame/Render()!\");
    ImFontConfig font_cfg = font_cfg_template ? *font_cfg_template : ImFontConfig();
    IM_ASSERT(font_cfg.FontData == NULL);
    font_cfg.FontData = ttf_data;"
    "IM_ASSERT(!Locked && \"Cannot modify a locked ImFontAtlas between NewFrame() and EndFrame/Render()!\");
    // [DART patch] Guard against null/empty input (issue #2516)
    if (ttf_data == NULL || ttf_size <= 0)
        return NULL;
    ImFontConfig font_cfg = font_cfg_template ? *font_cfg_template : ImFontConfig();
    IM_ASSERT(font_cfg.FontData == NULL);
    font_cfg.FontData = ttf_data;"
    content "${content}"
  )

  # Patch AddFontFromMemoryCompressedTTF
  string(REPLACE
    "const unsigned int buf_decompressed_size = stb_decompress_length((const unsigned char*)compressed_ttf_data);"
    "// [DART patch] Guard against null/empty input to prevent SEGV in stb_decompress (issue #2516)
    if (compressed_ttf_data == NULL || compressed_ttf_size <= 0)
        return NULL;

    const unsigned int buf_decompressed_size = stb_decompress_length((const unsigned char*)compressed_ttf_data);"
    content "${content}"
  )

  # Patch AddFontFromMemoryCompressedBase85TTF
  string(REPLACE
    "int compressed_ttf_size = (((int)strlen(compressed_ttf_data_base85) + 4) / 5) * 4;"
    "// [DART patch] Guard against null input (issue #2516)
    if (compressed_ttf_data_base85 == NULL)
        return NULL;
    int compressed_ttf_size = (((int)strlen(compressed_ttf_data_base85) + 4) / 5) * 4;"
    content "${content}"
  )

  if(content STREQUAL before_issue2516)
    message(FATAL_ERROR
      "ImGui null-font patch failed: none of the expected patterns were found "
      "in ${file}. The ImGui source may have changed. Update the patch patterns "
      "to match the current ImGui version."
    )
  endif()

  set(patches_applied TRUE)
endif()

set(widgets_file "${IMGUI_SOURCE_DIR}/imgui_widgets.cpp")

if(NOT EXISTS "${widgets_file}")
  message(FATAL_ERROR "imgui_widgets.cpp not found at ${widgets_file}")
endif()

file(READ "${widgets_file}" widgets_content)

string(FIND "${widgets_content}" "issue #2668" issue2668_patched)
if(issue2668_patched EQUAL -1)
  set(before_issue2668 "${widgets_content}")

  string(REPLACE
    "bool ImGui::ColorPicker4(const char* label, float col[4], ImGuiColorEditFlags flags, const float* ref_col)
{
    ImGuiContext& g = *GImGui;
    ImGuiWindow* window = GetCurrentWindow();
    if (window->SkipItems)
        return false;"
    "bool ImGui::ColorPicker4(const char* label, float col[4], ImGuiColorEditFlags flags, const float* ref_col)
{
    // [DART patch] Guard against missing context/window before GetCurrentWindow() (issue #2668)
    if (GImGui == NULL || GImGui->CurrentWindow == NULL)
        return false;

    ImGuiContext& g = *GImGui;
    ImGuiWindow* window = GetCurrentWindow();
    if (window->SkipItems)
        return false;"
    widgets_content "${widgets_content}"
  )

  if(widgets_content STREQUAL before_issue2668)
    message(FATAL_ERROR
      "ImGui ColorPicker patch failed: expected pattern not found in "
      "${widgets_file}. The ImGui source may have changed. Update the patch "
      "pattern to match the current ImGui version."
    )
  endif()

  file(WRITE "${widgets_file}" "${widgets_content}")
  set(patches_applied TRUE)
endif()

if(patches_applied)
  file(WRITE "${file}" "${content}")
  message(STATUS "Patched ImGui with DART null pointer guards")
else()
  message(STATUS "ImGui DART null pointer guards already patched, skipping")
endif()
