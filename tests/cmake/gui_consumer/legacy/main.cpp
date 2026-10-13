#include <imgui.h>

int main()
{
  if (!IMGUI_CHECKVERSION())
    return 1;
  ImGui::CreateContext();
  ImGui::DestroyContext();
  return 0;
}
