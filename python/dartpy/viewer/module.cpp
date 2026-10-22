// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

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

#include "detail/eigen.hpp"
#include "detail/errors.hpp"
#include "detail/gil.hpp"
#include "gui/ownership.hpp"
#include "viewer/session.hpp"

#include <dart/gui/DragAndDrop.hpp>
#include <dart/gui/GUIEventHandler.hpp>
#include <dart/gui/Viewer.hpp>
#include <dart/gui/ViewerAttachment.hpp>
#include <dart/gui/WorldNode.hpp>

#include <dart/simulation/World.hpp>

#include <dart/dynamics/BodyNode.hpp>

#include <dart/external/imgui/imgui.h>

#include <nanobind/stl/optional.h>

#include <iostream>
#include <stdexcept>
#include <unordered_set>

#include <cstdio>

namespace dartnb::viewer {

namespace app = dart::gui::app;

//==============================================================================
Slot::Slot(nb::object fn, std::string what)
  : fn(std::move(fn)), what(std::move(what))
{
}

//==============================================================================
Slot::~Slot()
{
  // Host objects may drop the last reference with the GIL released.
  if (nb::detail::cleanup_guard guard{})
    fn.reset();
  else
    (void)fn.release(); // The interpreter is gone; leak instead of crashing.
}

//==============================================================================
std::shared_ptr<Slot> makeSlot(nb::handle fn, std::string what)
{
  if (fn.is_none())
    return nullptr;
  if (!PyCallable_Check(fn.ptr()))
    throw nb::type_error((what + " must be callable or None").c_str());
  return std::make_shared<Slot>(nb::borrow(fn), std::move(what));
}

//==============================================================================
nb::object slotFunction(const std::shared_ptr<Slot>& slot)
{
  return slot ? slot->fn : nb::none();
}

//==============================================================================
std::shared_ptr<Session>& currentSession()
{
  static std::shared_ptr<Session> session;
  return session;
}

//==============================================================================
int visitSlot(const std::shared_ptr<Slot>& slot, visitproc visit, void* arg)
{
  if (slot)
    Py_VISIT(slot->fn.ptr());
  return 0;
}

namespace {

//==============================================================================
/// Hands an ordinary exception to the host's guards; stashes anything else,
/// such as KeyboardInterrupt, which stops the run.
void raiseInHost(Session& session, nb::python_error& error)
{
  if (error.matches(PyExc_Exception))
    throw std::runtime_error(error.what());
  if (!session.pending)
    session.pending.emplace(std::move(error));
}

} // namespace

//==============================================================================
void callSlot(const std::weak_ptr<Slot>& weak, bool teardown)
{
  const auto slot = weak.lock();
  if (!slot)
    return;
  dartnb::GilScopedAcquire gil;
  const auto session = currentSession();
  if (!session || slot->fn.is_none() || (session->pending && !teardown))
    return;
  try {
    slot->fn();
  } catch (nb::python_error& error) {
    raiseInHost(*session, error);
  }
}

//==============================================================================
std::function<void()> hostCallback(
    const std::shared_ptr<Slot>& slot, bool teardown)
{
  if (!slot)
    return {};
  return [weak = std::weak_ptr<Slot>(slot), teardown] {
    callSlot(weak, teardown);
  };
}

namespace {

//==============================================================================
struct PyKeyAction
{
  int key = 0;
  std::string label;
  std::shared_ptr<Slot> callback;

  int traverse(visitproc visit, void* arg) const
  {
    return visitSlot(callback, visit, arg);
  }
  void clear()
  {
    callback.reset();
  }
};

//==============================================================================
struct PyScene
{
  std::string id;
  std::string title;
  std::string category;
  std::string summary;
  std::shared_ptr<Slot> factory;

  int traverse(visitproc visit, void* arg) const
  {
    return visitSlot(factory, visit, arg);
  }
  void clear()
  {
    factory.reset();
  }
};

//==============================================================================
struct PySceneSetup
{
  // The Python object, so a World subclass keeps its Python state.
  nb::object world;
  std::shared_ptr<Slot> preStep;
  std::shared_ptr<Slot> postStep;
  std::shared_ptr<Slot> preRefresh;
  std::shared_ptr<Slot> onActivate;
  nb::tuple keyActions;
  nb::object panel = nb::none();
  std::optional<app::CameraHome> cameraHome;
  bool shadows = true;
  bool allowSimulation = true;

  int traverse(visitproc visit, void* arg) const
  {
    for (const auto* slot : {&preStep, &postStep, &preRefresh, &onActivate})
      if (const int result = visitSlot(*slot, visit, arg))
        return result;
    Py_VISIT(world.ptr());
    Py_VISIT(keyActions.ptr());
    Py_VISIT(panel.ptr());
    return 0;
  }
  void clear()
  {
    preStep.reset();
    postStep.reset();
    preRefresh.reset();
    onActivate.reset();
    world = nb::none();
    keyActions = nb::tuple();
    panel = nb::none();
  }
};

//==============================================================================
struct PySceneContext
{
  app::SceneContext native;
  std::vector<std::shared_ptr<Slot>> slots;
  std::vector<nb::object> retained;

  int traverse(visitproc visit, void* arg) const
  {
    for (const auto& slot : slots)
      if (const int result = visitSlot(slot, visit, arg))
        return result;
    for (const auto& object : retained)
      Py_VISIT(object.ptr());
    return 0;
  }
  void clear()
  {
    slots.clear();
    retained.clear();
  }
};

//==============================================================================
/// Forwards viewer events to a Python callable that returns whether it
/// handled the event. The viewer does not guard its handlers, so a failing
/// callable is disabled and reported in the host log.
class CallbackHandler final : public dart::gui::GUIEventHandler
{
public:
  CallbackHandler(std::weak_ptr<Slot> slot, app::SceneContext context)
    : mSlot(std::move(slot)), mContext(std::move(context))
  {
  }

  bool handle(
      const dart::gui::GUIEventAdapter& event,
      dart::gui::GUIActionAdapter&) override
  {
    const auto slot = mSlot.lock();
    if (!slot)
      return false;
    try {
      dartnb::GilScopedAcquire gil;
      const auto session = currentSession();
      if (!session || session->pending || slot->fn.is_none())
        return false;
      try {
        const nb::object result
            = slot->fn(nb::cast(dart::gui::GUIEventAdapter(event)));
        const int handled = PyObject_IsTrue(result.ptr());
        if (handled < 0)
          throw nb::python_error();
        return handled == 1;
      } catch (nb::python_error& error) {
        if (!error.matches(PyExc_Exception)) {
          if (!session->pending)
            session->pending.emplace(std::move(error));
          return false;
        }
        const std::string message = error.what();
        slot->fn = nb::none();
        try {
          mContext.reportFailure(slot->what, message);
        } catch (const std::logic_error&) {
          // The scene is gone; nothing to report to.
        }
        return false;
      }
    } catch (const std::exception&) {
      // The interpreter is finalizing.
      return false;
    }
  }

private:
  std::weak_ptr<Slot> mSlot;
  app::SceneContext mContext;
};

/// AssetsNotPrefetchedError, kept for the life of the process.
PyObject* assetsNotPrefetchedError = nullptr;

//==============================================================================
/// Runs as the scene's last teardown: the scene's Python objects stay alive
/// until every other teardown, including the Python ones, has run.
void releaseScene()
{
  dartnb::GilScopedAcquire gil;
  if (const auto session = currentSession()) {
    session->activeSetup = nb::object();
    // A context saved past teardown must not keep the scene's callables and
    // what they capture alive; every one of them has run by now.
    if (session->activeContext)
      nb::cast<PySceneContext&>(session->activeContext).clear();
    session->activeContext = nb::object();
  }
}

//==============================================================================
void activateScene(const std::shared_ptr<Slot>& held, app::SceneContext& ctx)
{
  dartnb::GilScopedAcquire gil;
  const auto session = currentSession();
  if (!session)
    return;
  // Registered first, so it runs after every other teardown of this scene.
  ctx.addTeardown(&releaseScene);
  session->activeSetup = held->fn;
  session->activeContext = nb::cast(PySceneContext{ctx, {}, {}});

  const auto& setup = nb::cast<const PySceneSetup&>(session->activeSetup);
  if (!setup.panel.is_none())
    nb::cast<PyControl&>(setup.panel).rewire();
  if (!setup.onActivate || setup.onActivate->fn.is_none() || session->pending)
    return;
  try {
    setup.onActivate->fn(session->activeContext);
  } catch (nb::python_error& error) {
    raiseInHost(*session, error);
  }
}

//==============================================================================
app::SceneSetup buildScene(
    const std::weak_ptr<Slot>& weak, const std::string& id)
{
  const auto factory = weak.lock();
  dartnb::GilScopedAcquire gil;
  const auto session = currentSession();
  if (!factory || !session || factory->fn.is_none())
    throw std::runtime_error("the factory of scene '" + id + "' is gone");
  if (session->pending)
    throw std::runtime_error("the run is stopping");

  nb::object result;
  try {
    result = factory->fn();
  } catch (nb::python_error& error) {
    if (error.matches(assetsNotPrefetchedError))
      throw app::AssetsNotPrefetched(
          nb::cast<std::string>(nb::str(error.value())));
    raiseInHost(*session, error);
    throw std::runtime_error("the run is stopping");
  }
  if (!nb::isinstance<PySceneSetup>(result))
    throw std::runtime_error(
        "the factory of scene '" + id + "' returned "
        + nb::cast<std::string>(result.type().attr("__name__"))
        + ", not a dartpy.viewer.SceneSetup");

  const auto& setup = nb::cast<const PySceneSetup&>(result);
  app::SceneSetup native;
  native.world
      = nb::cast<std::shared_ptr<dart::simulation::World>>(setup.world);
  native.preStep = hostCallback(setup.preStep);
  native.postStep = hostCallback(setup.postStep);
  native.preRefresh = hostCallback(setup.preRefresh);
  for (nb::handle item : setup.keyActions) {
    const auto& action = nb::cast<const PyKeyAction&>(item);
    native.keyActions.push_back(
        {action.key, action.label, hostCallback(action.callback)});
  }
  if (!setup.panel.is_none())
    native.panel = nb::cast<const PyControl&>(setup.panel).native;
  native.cameraHome = setup.cameraHome;
  native.enableShadows = setup.shadows;
  native.allowSimulation = setup.allowSimulation;
  // The setup must outlive its activation; activateScene() takes it over.
  auto held = std::make_shared<Slot>(result, "scene setup");
  native.onActivate = [held](app::SceneContext& ctx) {
    activateScene(held, ctx);
  };
  return native;
}

//==============================================================================
bool stopRequested(Session& session)
{
  try {
    dartnb::GilScopedAcquire gil;
    // Runs Python signal handlers, so Ctrl+C raises KeyboardInterrupt here.
    if (!session.pending && PyErr_CheckSignals() != 0)
      session.pending.emplace();
    return session.pending.has_value();
  } catch (const std::exception&) {
    // The interpreter is finalizing.
    return true;
  }
}

//==============================================================================
/// Treats a Python BaseException that reached the host's frame guard, such as
/// one raised in a dartpy.gui override that the viewer called, as a stop.
bool isStopException(const std::exception& error)
{
  const auto* python = dynamic_cast<const nb::python_error*>(&error);
  if (!python)
    return false;
  try {
    dartnb::GilScopedAcquire gil;
    const auto session = currentSession();
    if (!session || python->matches(PyExc_Exception))
      return false;
    if (!session->pending)
      session->pending.emplace(*python);
    return true;
  } catch (const std::exception&) {
    // The interpreter is finalizing.
    return false;
  }
}

//==============================================================================
/// Publishes a session for the duration of run(); clearing it needs the GIL.
class ActiveSession
{
public:
  explicit ActiveSession(std::shared_ptr<Session> session)
  {
    currentSession() = std::move(session);
  }
  ~ActiveSession()
  {
    auto session = std::move(currentSession());
    session->scenes = nb::object();
    session->activeSetup = nb::object();
    session->activeContext = nb::object();
    session->pending.reset();
  }
  ActiveSession(const ActiveSession&) = delete;
  ActiveSession& operator=(const ActiveSession&) = delete;
};

//==============================================================================
int run(nb::handle scenes, nb::handle argv)
{
  // GLFW needs the main thread on macOS.
  const auto threading = nb::module_::import_("threading");
  if (!threading.attr("current_thread")().is(threading.attr("main_thread")()))
    throw std::runtime_error(
        "dartpy.viewer.run() must be called from the main thread");
  if (currentSession())
    throw std::runtime_error("dartpy.viewer.run() is already running");

  std::vector<std::string> args;
  for (nb::handle item : nb::iter(argv))
    args.push_back(nb::cast<std::string>(item));
  if (args.empty())
    args.emplace_back("dartpy.viewer");

  auto session = std::make_shared<Session>();
  session->scenes = nb::module_::import_("builtins").attr("list")(scenes);
  std::vector<app::Scene> native;
  std::unordered_set<std::string> ids;
  for (nb::handle item : session->scenes) {
    if (!nb::isinstance<PyScene>(item))
      throw nb::type_error("scenes must be dartpy.viewer.Scene objects");
    const auto& scene = nb::cast<const PyScene&>(item);
    if (!ids.insert(scene.id).second)
      throw nb::value_error(("duplicate scene id '" + scene.id + "'").c_str());
    native.push_back(
        {scene.id,
         scene.title,
         scene.category,
         scene.summary,
         [weak = std::weak_ptr<Slot>(scene.factory), id = scene.id] {
           return buildScene(weak, id);
         }});
  }

  ActiveSession active(session);
  int code = 0;
  {
    nb::gil_scoped_release release;
    code = app::runCommandLine(
        std::move(native),
        args,
        [] {
          const auto running = currentSession();
          return running && stopRequested(*running);
        },
        &isStopException);
  }
  // Let pipes and capture tools see the host's output before Python's.
  std::cout.flush();
  std::fflush(nullptr);

  if (session->pending) {
    nb::python_error error = std::move(*session->pending);
    session->pending.reset();
    throw error;
  }
  return code;
}

//==============================================================================
int keyCode(nb::handle key)
{
  if (PyUnicode_Check(key.ptr())) {
    const auto text = nb::cast<std::string>(key);
    if (text.size() != 1 || static_cast<unsigned char>(text[0]) >= 128)
      throw nb::value_error("key must be one ASCII character or a key code");
    return static_cast<unsigned char>(text[0]);
  }
  const auto code = nb::cast<long long>(key);
  if (code < 0 || code > std::numeric_limits<int>::max())
    throw nb::value_error("key code must be a non-negative int");
  return static_cast<int>(code);
}

//==============================================================================
bool isSceneId(const std::string& id)
{
  if (id.empty() || id[0] < 'a' || id[0] > 'z')
    return false;
  for (const char c : id)
    if (!((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_'))
      return false;
  return true;
}

} // namespace

} // namespace dartnb::viewer

namespace dart::python {

void dart_viewer(nb::module_& m)
{
  using namespace dartnb::viewer;
  namespace app = dart::gui::app;

  auto sm = m.def_submodule("viewer");
  sm.doc()
      = "The DART demo app host: run a catalog of scenes in one window with "
        "docking panels, or display-free for cycling and captures.";

  sm.def(
      "keyboard_capture_active",
      [] {
        return ImGui::GetCurrentContext() && ImGui::GetIO().WantCaptureKeyboard;
      },
      "Whether scene input should yield to the native panel's keyboard "
      "capture.");

  nb::exception<app::AssetsNotPrefetched>(
      sm, "AssetsNotPrefetchedError", dartnb::operationError());
  assetsNotPrefetchedError = sm.attr("AssetsNotPrefetchedError").ptr();
  Py_INCREF(assetsNotPrefetchedError);

  nb::class_<app::CameraHome>(sm, "CameraHome", nb::is_final())
      .def(
          nb::init<
              const Eigen::Vector3d&,
              const Eigen::Vector3d&,
              const Eigen::Vector3d&>(),
          nb::kw_only(),
          nb::arg("eye"),
          nb::arg("center"),
          nb::arg("up") = Eigen::Vector3d::UnitZ().eval(),
          "Raises ValueError unless the vectors are finite, eye and center "
          "differ, and up is not parallel to the view direction.")
      .def_prop_ro(
          "eye",
          [](const app::CameraHome& home) -> Eigen::Vector3d {
            return home.eye;
          })
      .def_prop_ro(
          "center",
          [](const app::CameraHome& home) -> Eigen::Vector3d {
            return home.center;
          })
      .def_prop_ro("up", [](const app::CameraHome& home) -> Eigen::Vector3d {
        return home.up;
      });

  nb::class_<PyKeyAction>(
      sm,
      "KeyAction",
      nb::type_slots(gcSlots<PyKeyAction>()),
      nb::is_final(),
      nb::is_weak_referenceable(),
      "A keyboard shortcut, also shown as a button in the scene panel.")
      .def(
          "__init__",
          [](PyKeyAction* self,
             nb::handle key,
             std::string label,
             nb::handle callback) {
            const int code = keyCode(key);
            auto slot = makeSlot(callback, "callback");
            if (!slot)
              throw nb::type_error("callback must be callable");
            slot->what = "action '" + label + "'";
            new (self) PyKeyAction{code, std::move(label), std::move(slot)};
          },
          nb::arg("key"),
          nb::arg("label"),
          nb::arg("callback"))
      .def_ro("key", &PyKeyAction::key)
      .def_ro("label", &PyKeyAction::label)
      .def_prop_ro("callback", [](const PyKeyAction& action) {
        return slotFunction(action.callback);
      });

  nb::class_<PyScene>(
      sm,
      "Scene",
      nb::type_slots(gcSlots<PyScene>()),
      nb::is_final(),
      nb::is_weak_referenceable(),
      "One catalog entry; factory() builds a fresh SceneSetup each time the "
      "scene is selected, rebuilt or reset.")
      .def(
          "__init__",
          [](PyScene* self,
             std::string id,
             std::string title,
             std::string category,
             std::string summary,
             nb::handle factory) {
            if (!isSceneId(id))
              throw nb::value_error(
                  "id must be snake_case: a lowercase letter, then lowercase "
                  "letters, digits or underscores");
            auto slot = makeSlot(factory, "factory");
            if (!slot)
              throw nb::type_error("factory must be callable");
            new (self) PyScene{
                std::move(id),
                std::move(title),
                std::move(category),
                std::move(summary),
                std::move(slot)};
          },
          nb::arg("id"),
          nb::kw_only(),
          nb::arg("title"),
          nb::arg("category"),
          nb::arg("summary") = "",
          nb::arg("factory"))
      .def_ro("id", &PyScene::id)
      .def_ro("title", &PyScene::title)
      .def_ro("category", &PyScene::category)
      .def_ro("summary", &PyScene::summary)
      .def_prop_ro("factory", [](const PyScene& scene) {
        return slotFunction(scene.factory);
      });

  nb::class_<PySceneSetup>(
      sm,
      "SceneSetup",
      nb::type_slots(gcSlots<PySceneSetup>()),
      nb::is_final(),
      nb::is_weak_referenceable(),
      "What a scene factory builds: the world, its hooks, key actions and "
      "panel. pre_step and post_step run around every simulation step; "
      "pre_refresh runs once per frame, also while paused.")
      .def(
          "__init__",
          [](PySceneSetup* self,
             nb::handle world,
             nb::handle preStep,
             nb::handle postStep,
             nb::handle preRefresh,
             nb::handle onActivate,
             nb::handle keyActions,
             nb::handle panel,
             std::optional<app::CameraHome> cameraHome,
             bool shadows,
             bool allowSimulation) {
            if (!nb::isinstance<dart::simulation::World>(world))
              throw nb::type_error("world must be a dartpy.simulation.World");
            nb::list actions;
            std::unordered_set<int> keys;
            for (nb::handle item : nb::iter(keyActions)) {
              if (!nb::isinstance<PyKeyAction>(item))
                throw nb::type_error(
                    "key_actions must be dartpy.viewer.KeyAction objects");
              if (!keys.insert(nb::cast<const PyKeyAction&>(item).key).second)
                throw nb::value_error("key_actions bind one key twice");
              actions.append(item);
            }
            if (!panel.is_none()
                && !(
                    nb::isinstance<PyControl>(panel)
                    && nb::cast<const PyControl&>(panel).native->getKind()
                           == app::Control::Kind::Group))
              throw nb::type_error(
                  "panel must be a group Control, such as Panel(), or None");
            new (self) PySceneSetup{
                nb::borrow(world),
                makeSlot(preStep, "pre_step"),
                makeSlot(postStep, "post_step"),
                makeSlot(preRefresh, "pre_refresh"),
                makeSlot(onActivate, "on_activate"),
                nb::tuple(actions),
                nb::borrow(panel),
                std::move(cameraHome),
                shadows,
                allowSimulation};
          },
          nb::arg("world"),
          nb::kw_only(),
          nb::arg("pre_step").none() = nb::none(),
          nb::arg("post_step").none() = nb::none(),
          nb::arg("pre_refresh").none() = nb::none(),
          nb::arg("on_activate").none() = nb::none(),
          nb::arg("key_actions") = nb::tuple(),
          nb::arg("panel").none() = nb::none(),
          nb::arg("camera_home").none() = nb::none(),
          nb::arg("shadows") = true,
          nb::arg("allow_simulation") = true)
      .def_prop_ro(
          "world", [](const PySceneSetup& setup) { return setup.world; })
      .def_prop_ro(
          "pre_step",
          [](const PySceneSetup& setup) { return slotFunction(setup.preStep); })
      .def_prop_ro(
          "post_step",
          [](const PySceneSetup& setup) {
            return slotFunction(setup.postStep);
          })
      .def_prop_ro(
          "pre_refresh",
          [](const PySceneSetup& setup) {
            return slotFunction(setup.preRefresh);
          })
      .def_prop_ro(
          "on_activate",
          [](const PySceneSetup& setup) {
            return slotFunction(setup.onActivate);
          })
      .def_ro("key_actions", &PySceneSetup::keyActions)
      .def_ro("panel", &PySceneSetup::panel)
      // A copy: a view into the setup would pin it from objects the cyclic
      // GC cannot see.
      .def_prop_ro(
          "camera_home",
          [](const PySceneSetup& setup) { return setup.cameraHome; })
      .def_ro("shadows", &PySceneSetup::shadows)
      .def_ro("allow_simulation", &PySceneSetup::allowSimulation);

  nb::class_<PySceneContext>(
      sm,
      "SceneContext",
      nb::type_slots(gcSlots<PySceneContext>()),
      nb::is_final(),
      nb::is_weak_referenceable(),
      "Passed to on_activate. Everything registered through it is removed "
      "when the scene is torn down; afterwards every member raises "
      "RuntimeError.")
      .def_prop_ro(
          "viewer",
          [](const PySceneContext& ctx) {
            return nb::cast(ctx.native.viewerPtr());
          })
      .def_prop_ro(
          "world_node",
          [](const PySceneContext& ctx) {
            return nb::cast(ctx.native.worldNodePtr());
          })
      .def(
          "add_teardown",
          [](PySceneContext& ctx, nb::handle callback) {
            auto slot = makeSlot(callback, "teardown");
            if (!slot)
              throw nb::type_error("callback must be callable");
            ctx.native.addTeardown(hostCallback(slot, true));
            ctx.slots.push_back(std::move(slot));
          },
          nb::arg("callback"),
          "Runs callback when the scene is torn down, newest first.")
      .def(
          "add_attachment",
          [](PySceneContext& ctx,
             std::shared_ptr<dart::gui::ViewerAttachment> attachment) {
            if (!attachment)
              throw nb::type_error(
                  "attachment must be a dartpy.gui.ViewerAttachment");
            // Python never draws: a Python subclass would run its overrides
            // inside the viewer's frames, outside the callback contract.
            if (dartnb::gui::isPythonAttachment(*attachment))
              throw nb::type_error(
                  "attachment must be a native dartpy.gui visual, such as "
                  "GridVisual; put per-frame Python logic in pre_refresh");
            ctx.native.addOwnedAttachment(std::move(attachment));
          },
          nb::arg("attachment"),
          "Adds a native attachment, such as a dartpy.gui.GridVisual, and "
          "removes it at teardown.")
      .def(
          "add_event_handler",
          [](PySceneContext& ctx, nb::handle callback) {
            auto slot = makeSlot(callback, "event handler");
            if (!slot)
              throw nb::type_error("callback must be callable");
            ctx.native.addEventHandler(
                std::make_shared<CallbackHandler>(slot, ctx.native));
            ctx.slots.push_back(std::move(slot));
          },
          nb::arg("callback"),
          "callback(event: dartpy.gui.GUIEventAdapter) returns whether it "
          "handled the event.")
      .def(
          "enable_drag_and_drop",
          [](PySceneContext& ctx,
             nb::handle target,
             bool useExternalIk,
             bool useWholeBody) {
            auto* viewer = ctx.native.viewer();
            const nb::object pyViewer = nb::cast(ctx.native.viewerPtr());
            const nb::object drag
                = nb::isinstance<dart::dynamics::BodyNode>(target)
                      ? pyViewer.attr("enableDragAndDrop")(
                          target, useExternalIk, useWholeBody)
                      : pyViewer.attr("enableDragAndDrop")(target);
            if (drag.is_none())
              throw nb::value_error("the target cannot be dragged");
            const std::weak_ptr<dart::gui::DragAndDrop> weak
                = nb::cast<std::shared_ptr<dart::gui::DragAndDrop>>(drag);
            const auto disable = [viewer, weak] {
              if (const auto interaction = weak.lock())
                viewer->disableDragAndDrop(interaction.get());
            };
            try {
              ctx.native.addTeardown(disable);
            } catch (...) {
              disable();
              throw;
            }
            ctx.retained.push_back(nb::borrow(target));
            ctx.retained.push_back(drag);
            return drag;
          },
          nb::arg("target"),
          nb::kw_only(),
          nb::arg("use_external_ik") = true,
          nb::arg("use_whole_body") = false,
          "Drags a SimpleFrame, an InteractiveFrame or a BodyNode until the "
          "scene is torn down.")
      .def(
          "log",
          [](const PySceneContext& ctx, const std::string& message) {
            ctx.native.log(message);
          },
          nb::arg("message"),
          "Appends a line to the host's log console.");

  bindPanel(sm);

  sm.def(
      "run",
      &run,
      nb::arg("scenes"),
      nb::arg("argv"),
      "Runs the app on `scenes` with command-line arguments `argv` (argv[0] "
      "is the program name; pass sys.argv) and returns the exit code: an "
      "interactive window, or --list-scenes, --cycle-scenes or a --headless "
      "capture. Main thread only. The GIL is released while the host runs; "
      "a KeyboardInterrupt or SystemExit raised in a callback stops the run "
      "and is re-raised after the scene's teardown.");
}

} // namespace dart::python
