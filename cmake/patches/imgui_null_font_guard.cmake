# Patch: Apply DART fixes to fetched ImGui sources
# Fixes: https://github.com/dartsim/dart/issues/2516
# Fixes: https://github.com/dartsim/dart/issues/2668
# Fixes: https://github.com/dartsim/dart/issues/2671
#
# ImGui v1.84.2 does not validate input pointers in AddFontFromMemory*
# functions, causing SEGV when nullptr is passed (e.g., from a failed
# resource load). It also assumes ColorPicker4 has a current context and window.
# This patch adds early-return guards.
#
# ImGui also declares internal formatting helpers with IMGUI_API, which exports
# them from DART's fetched shared library. Keep those helpers hidden from the
# dynamic ABI while preserving ImGui's internal uses.
#
# Usage: cmake -DIMGUI_SOURCE_DIR=<path> -P imgui_null_font_guard.cmake

if(NOT DEFINED IMGUI_SOURCE_DIR)
  message(FATAL_ERROR "IMGUI_SOURCE_DIR must be set")
endif()

set(file "${IMGUI_SOURCE_DIR}/imgui_draw.cpp")
set(internal_file "${IMGUI_SOURCE_DIR}/imgui_internal.h")

if(NOT EXISTS "${file}")
  message(FATAL_ERROR "imgui_draw.cpp not found at ${file}")
endif()

if(NOT EXISTS "${internal_file}")
  message(FATAL_ERROR "imgui_internal.h not found at ${internal_file}")
endif()

file(READ "${internal_file}" internal_content)

string(
  FIND "${internal_content}"
  "IMGUI_INTERNAL_FORMAT_API"
  internal_already_patched
)
if(internal_already_patched EQUAL -1)
  string(
    REPLACE
    "IMGUI_API int           ImFormatString(char* buf, size_t buf_size, const char* fmt, ...) IM_FMTARGS(3);"
    "#ifndef IMGUI_INTERNAL_FORMAT_API
#if defined(__GNUC__) || defined(__clang__)
#define IMGUI_INTERNAL_FORMAT_API __attribute__((visibility(\"hidden\")))
#else
#define IMGUI_INTERNAL_FORMAT_API IMGUI_API
#endif
#endif
IMGUI_INTERNAL_FORMAT_API int ImFormatString(char* buf, size_t buf_size, const char* fmt, ...) IM_FMTARGS(3);"
    internal_content
    "${internal_content}"
  )

  string(
    REPLACE
    "IMGUI_API int           ImFormatStringV(char* buf, size_t buf_size, const char* fmt, va_list args) IM_FMTLIST(3);"
    "IMGUI_INTERNAL_FORMAT_API int ImFormatStringV(char* buf, size_t buf_size, const char* fmt, va_list args) IM_FMTLIST(3);"
    internal_content
    "${internal_content}"
  )

  string(
    FIND "${internal_content}"
    "IMGUI_INTERNAL_FORMAT_API int ImFormatString("
    _format_patch_found
  )
  if(_format_patch_found EQUAL -1)
    message(
      FATAL_ERROR
      "ImGui internal format symbol patch failed: expected declarations were "
      "not found in ${internal_file}. The ImGui source may have changed. "
      "Update the patch patterns to match the current ImGui version."
    )
  endif()

  file(WRITE "${internal_file}" "${internal_content}")
  message(
    STATUS
    "Patched imgui_internal.h to hide internal format helpers (issue #2671)"
  )
else()
  message(STATUS "imgui_internal.h already patched, skipping")
endif()

file(READ "${file}" content)

set(patches_applied FALSE)

# Guard: only patch each issue once (idempotent)
string(FIND "${content}" "issue #2516" issue2516_patched)
if(issue2516_patched EQUAL -1)
  set(before_issue2516 "${content}")

  # Patch AddFontFromMemoryTTF
  string(
    REPLACE
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
    content
    "${content}"
  )

  # Patch AddFontFromMemoryCompressedTTF
  string(
    REPLACE
    "const unsigned int buf_decompressed_size = stb_decompress_length((const unsigned char*)compressed_ttf_data);"
    "// [DART patch] Guard against null/empty input to prevent SEGV in stb_decompress (issue #2516)
    if (compressed_ttf_data == NULL || compressed_ttf_size <= 0)
        return NULL;

    const unsigned int buf_decompressed_size = stb_decompress_length((const unsigned char*)compressed_ttf_data);"
    content
    "${content}"
  )

  # Patch AddFontFromMemoryCompressedBase85TTF
  string(
    REPLACE
    "int compressed_ttf_size = (((int)strlen(compressed_ttf_data_base85) + 4) / 5) * 4;"
    "// [DART patch] Guard against null input (issue #2516)
    if (compressed_ttf_data_base85 == NULL)
        return NULL;
    int compressed_ttf_size = (((int)strlen(compressed_ttf_data_base85) + 4) / 5) * 4;"
    content
    "${content}"
  )

  if(content STREQUAL before_issue2516)
    message(
      FATAL_ERROR
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

  string(
    REPLACE
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
    widgets_content
    "${widgets_content}"
  )

  if(widgets_content STREQUAL before_issue2668)
    message(
      FATAL_ERROR
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
