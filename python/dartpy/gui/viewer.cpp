// dartpy.gui viewer entrypoints: expose the C++ Filament interactive viewer
// and the multi-scene demos host so the Python `py-demos` runner is a
// first-class peer of the C++ `dart-demos` (PLAN-103).

#include "gui/viewer.hpp"

#include "dart/gui/application.hpp"
#include "gui/panel.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/function.h>
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

  int argc() const
  {
    return static_cast<int>(mStorage.size());
  }
  char** argv()
  {
    return mPointers.data();
  }

private:
  std::vector<std::string> mStorage;
  std::vector<char*> mPointers;
};

} // namespace

void defGuiViewer(nb::module_& m)
{
  m.def(
      "is_docking_available",
      &dart::gui::isDockingAvailable,
      "Return whether the GUI build includes Dear ImGui docking support.");

  // Run the interactive viewer (Filament + ImGui) against descriptor-backed
  // application options.
  // Argv flags propagate to the C++ runtime: --scene, --cycle-scenes,
  // --frames, --headless, --screenshot <path>, --width, --height, --backend.
  m.def(
      "run_application",
      [](const std::vector<std::string>& argv) {
        dart::gui::ApplicationOptions options;
        ArgvHolder holder(argv);
        return dart::gui::runApplication(holder.argc(), holder.argv(), options);
      },
      nb::arg("argv") = std::vector<std::string>{},
      "Open the interactive Filament viewer. Blocks "
      "until the user closes the window (or --headless --frames N exits).");

  // Run the multi-scene demos host. Each scene is a (id, title, category,
  // summary, factory) tuple where factory builds descriptor-backed
  // ApplicationOptions lazily when the scene is first selected.
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
          // Capture the Python callable; on invocation, acquire the GIL and
          // translate optional callbacks/panels into ApplicationOptions.
          entry.factory = [factory, scene_id = id]() {
            nb::gil_scoped_acquire gil;
            dart::gui::ApplicationOptions options;
            // Hold the Python pre_step callable (if any) alive past the
            // factory call so the viewer's preStep can invoke it each
            // frame. Captured by a stable shared_ptr so subsequent factory
            // invocations don't tear it down mid-step.
            auto pre_step_holder = std::make_shared<nb::callable>();
            // Hold optional Python callbacks alive past the factory call.
            auto force_drag_holder = std::make_shared<nb::callable>();
            auto renderable_provider_holder = std::make_shared<nb::callable>();
            auto debug_provider_holder = std::make_shared<nb::callable>();
            try {
              nb::object result = factory();
              // Accept either:
              //   factory() -> None
              //   factory() -> pre_step_callable
              //   factory() -> (pre_step_callable, force_drag_callable)
              //   factory() -> (pre_step_callable, force_drag_callable, panels)
              //   factory() -> (pre_step_callable, force_drag_callable, panels,
              //                 renderable_provider_callable)
              //   factory() -> (pre_step_callable, force_drag_callable, panels,
              //                 renderable_provider_callable,
              //                 debug_provider_callable)
              if (nb::isinstance<nb::tuple>(result)) {
                nb::tuple t = nb::cast<nb::tuple>(result);
                if (t.size() >= 1 && !t[0].is_none()) {
                  *pre_step_holder = nb::cast<nb::callable>(t[0]);
                }
                if (t.size() >= 2 && !t[1].is_none()) {
                  *force_drag_holder = nb::cast<nb::callable>(t[1]);
                }
                if (t.size() >= 3 && !t[2].is_none()) {
                  nb::iterable panels = nb::cast<nb::iterable>(t[2]);
                  for (nb::handle panel : panels) {
                    options.panels.push_back(makeGuiPanelFromPython(panel));
                  }
                }
                if (t.size() >= 4 && !t[3].is_none()) {
                  *renderable_provider_holder = nb::cast<nb::callable>(t[3]);
                }
                if (t.size() >= 5 && !t[4].is_none()) {
                  *debug_provider_holder = nb::cast<nb::callable>(t[4]);
                }
              } else if (!result.is_none()) {
                *pre_step_holder = nb::cast<nb::callable>(std::move(result));
              }
            } catch (const std::exception& e) {
              fprintf(
                  stderr,
                  "py-demos factory error for scene '%s': %s\n",
                  scene_id.c_str(),
                  e.what());
              throw;
            }
            if (pre_step_holder && *pre_step_holder) {
              options.preStep = [pre_step_holder]() {
                nb::gil_scoped_acquire gil;
                try {
                  (*pre_step_holder)();
                } catch (const std::exception& e) {
                  fprintf(stderr, "py-demos pre_step error: %s\n", e.what());
                  throw;
                }
              };
            }
            if (force_drag_holder && *force_drag_holder) {
              options.onForceDrag = [force_drag_holder](
                                        const dart::gui::ForceDragEvent& e) {
                nb::gil_scoped_acquire gil;
                try {
                  nb::dict event;
                  event["renderable_id"] = e.renderableId;
                  event["renderable_name"] = e.renderableName;
                  event["application_point"] = nb::cast(e.applicationPoint);
                  event["force"] = nb::cast(e.force);
                  event["active"] = e.active;
                  (*force_drag_holder)(event);
                } catch (const std::exception& err) {
                  fprintf(
                      stderr, "py-demos force_drag error: %s\n", err.what());
                }
              };
            }
            if (renderable_provider_holder && *renderable_provider_holder) {
              options.renderableProvider = [renderable_provider_holder]() {
                nb::gil_scoped_acquire gil;
                try {
                  nb::object result = (*renderable_provider_holder)();
                  return nb::cast<std::vector<dart::gui::RenderableDescriptor>>(
                      result);
                } catch (const std::exception& e) {
                  fprintf(
                      stderr,
                      "py-demos renderable_provider error: %s\n",
                      e.what());
                  throw;
                }
              };
            }
            if (debug_provider_holder && *debug_provider_holder) {
              options.debugProvider = [debug_provider_holder]() {
                nb::gil_scoped_acquire gil;
                try {
                  nb::object result = (*debug_provider_holder)();
                  return nb::cast<dart::gui::DebugScene>(result);
                } catch (const std::exception& e) {
                  fprintf(
                      stderr, "py-demos debug_provider error: %s\n", e.what());
                  throw;
                }
              };
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
      "each scene is (id, title, category, summary, factory_callable). "
      "Flags --scene/--cycle-scenes/--headless/--frames/--screenshot apply.");
}

} // namespace dart::python_nb
