// Consumers may also write the bare <imgui.h>; it must find the ImGui that
// dart-gui was built with. This translation unit includes only the bare name:
// with a system ImGui, DART's compatibility copy and the original header are
// distinct files, so one translation unit must not include both spellings.
#include <imgui.h>

const char* bareImGuiVersion()
{
  return ImGui::GetVersion();
}
