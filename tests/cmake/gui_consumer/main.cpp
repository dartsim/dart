#include <dart/gui/gui.hpp>

#include <dart/simulation/World.hpp>

#include <dart/external/imgui/imgui.h>

#include <fstream>

#if defined(_MSVC_LANG)
static_assert(_MSVC_LANG == 201703L, "The GUI consumer must remain C++17");
#else
static_assert(__cplusplus == 201703L, "The GUI consumer must remain C++17");
#endif

int main()
{
  auto world = dart::simulation::World::create();
  auto node = std::make_shared<dart::gui::WorldNode>(world);
  dart::gui::PerformanceStatsPanel stats;
  stats.reset();
  dart::gui::Viewer viewer;
  viewer.addWorldNode(node);
  if (node->getWorld() != world || ImGui::GetVersion() == nullptr)
    return 1;
#if !defined(_WIN32)
  // A display-free capture exercises the installed renderer and materials.
  // CI provisions Windows software WGL only for the in-tree test directories.
  const char* capture = "gui_consumer_capture.png";
  viewer.setUpViewForCapture(32, 24);
  viewer.captureScreen(capture);
  viewer.frame();
  std::ifstream file(capture, std::ios::binary | std::ios::ate);
  if (!file || file.tellg() <= 0)
    return 1;
#endif
  return 0;
}
