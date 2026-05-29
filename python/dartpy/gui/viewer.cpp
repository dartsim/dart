// dartpy.gui viewer entrypoints: expose the C++ Filament interactive viewer
// and the multi-scene demos host so the Python `py-demos` runner is a
// first-class peer of the C++ `dart-demos` (PLAN-103).

#include "gui/viewer.hpp"

#include "dart/gui/application.hpp"
#include "dart/simulation/world.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

#include <string>
#include <utility>
#include <vector>

namespace nb = nanobind;

namespace dart::python_nb {

namespace {

class ArgvHolder
{
public:
  explicit ArgvHolder(const std::vector<std::string>& args)
  {
    if (args.empty()) {
      mStorage.emplace_back("py-demos");
    } else {
      mStorage = args;
    }
    mPointers.reserve(mStorage.size() + 1);
    for (auto& s : mStorage) {
      mPointers.push_back(s.data());
    }
    mPointers.push_back(nullptr);
  }

  int argc() const { return static_cast<int>(mStorage.size()); }
  char** argv() { return mPointers.data(); }

private:
  std::vector<std::string> mStorage;
  std::vector<char*> mPointers;
};

} // namespace

void defGuiViewer(nb::module_& m)
{
  // Run the interactive viewer (Filament + ImGui) against a single world.
  // Argv flags propagate to the C++ runtime: --scene, --cycle-scenes,
  // --frames, --headless, --screenshot <path>, --width, --height, --backend.
  m.def(
      "run_application",
      [](const dart::simulation::WorldPtr& world,
         const std::vector<std::string>& argv) {
        dart::gui::ApplicationOptions options;
        options.world = world;
        ArgvHolder holder(argv);
        return dart::gui::runApplication(holder.argc(), holder.argv(), options);
      },
      nb::arg("world"),
      nb::arg("argv") = std::vector<std::string>{},
      "Open the interactive Filament viewer with the given world. Blocks "
      "until the user closes the window (or --headless --frames N exits).");

  // Run the multi-scene demos host. Each scene is a (id, title, category,
  // summary, build_world) tuple where build_world is a Python callable that
  // returns a dart.simulation.World; the world is built lazily when the
  // scene is first selected. Mirrors C++ dart::gui::runDemos.
  m.def(
      "run_demos",
      [](const std::vector<std::tuple<
             std::string,
             std::string,
             std::string,
             std::string,
             nb::callable>>& scenes,
         const std::vector<std::string>& argv) {
        std::vector<dart::gui::DemoSceneEntry> entries;
        entries.reserve(scenes.size());
        for (const auto& scene : scenes) {
          const auto& [id, title, category, summary, factory] = scene;
          dart::gui::DemoSceneEntry entry;
          entry.id = id;
          entry.title = title;
          entry.category = category;
          entry.summary = summary;
          // Capture the Python callable; on invocation, acquire the GIL,
          // call into Python, and unwrap the returned World pointer.
          entry.factory = [factory, scene_id = id]() {
            nb::gil_scoped_acquire gil;
            dart::gui::ApplicationOptions options;
            try {
              nb::object result = factory();
              options.world
                  = nb::cast<dart::simulation::WorldPtr>(std::move(result));
            } catch (const std::exception& e) {
              // Surface the Python-side error to stderr so the demos host
              // can soft-fail on this scene the way the C++ catalog does.
              fprintf(
                  stderr,
                  "py-demos factory error for scene '%s': %s\n",
                  scene_id.c_str(),
                  e.what());
            }
            // Always hand the viewer a valid (possibly empty) world so
            // downstream null-pointer checks don't crash the host. The
            // sidebar still lets the user pick another scene.
            if (options.world == nullptr) {
              options.world
                  = dart::simulation::World::create(scene_id + "_placeholder");
            }
            return options;
          };
          entries.push_back(std::move(entry));
        }
        ArgvHolder holder(argv);
        return dart::gui::runDemos(
            holder.argc(), holder.argv(), std::move(entries));
      },
      nb::arg("scenes"),
      nb::arg("argv") = std::vector<std::string>{},
      "Open the interactive Filament viewer hosting a catalog of scenes; "
      "each scene is (id, title, category, summary, build_world_callable). "
      "Flags --scene/--cycle-scenes/--headless/--frames/--screenshot apply.");
}

} // namespace dart::python_nb
