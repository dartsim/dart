Visualization and the DART 7 renderer migration
===============================================

The Filament renderer provides the ``dart::gui`` C++ component and the
``dartpy.gui`` Python API. The public C++ interface requires C++17; Filament
types and its implementation dependencies stay behind that interface.
Simulation, collision detection, and model loading work without the GUI
component.

DART 7 replaces the OSG GUI with Filament. GUI applications migrate to
``dart::gui`` / ``dartpy.gui`` and the ``gui`` CMake component. The C++ core,
``dart::utils`` loaders, collision components, and simulation APIs retain
compatibility; Python GUI namespace and renderer-specific API changes require
application updates.

.. note::

   The ``dartpy.gui`` bindings are interim. DART 7's Python rendering API
   replaces them before release, so new Python code should expect changes.

Build and run
-------------

From a checkout, ``pixi run config`` builds the GUI and dartpy. Build and
launch the Filament demo catalog, or build and run the display-free GUI C++ and
Python tests, with:

.. code-block:: console

   pixi run demos
   pixi run test-gui

``pixi run test`` and ``pixi run test-all`` skip these GUI tests. After
``test-gui``, run the native-window tests in a desktop session with
``ctest --test-dir build/default/cpp/Release -L filament-native``. Linux viewer
windows use X11, through XWayland on Wayland desktops.

For a CMake build, ``DART_BUILD_GUI`` (``ON`` by default) builds the Filament
GUI and ImGui; ``OFF`` builds without them. The build checks that the Filament
runtime and material compiler match. ``DART_USE_SYSTEM_FILAMENT`` (``ON`` by
default) uses an installed Filament 1.77.3 SDK; ``OFF`` uses a pinned private
SDK, built from source on Linux and downloaded prebuilt on macOS arm64 and
64-bit Windows. The GUI also needs GLFW 3.4, libpng, libjpeg, and Dear ImGui
1.92.8 or newer; see the :doc:`build requirements </dart/developer_guide/build>`.
``DART_BUILD_DARTPY`` builds the Python bindings, which always include
``dartpy.gui`` and therefore require ``DART_BUILD_GUI``.

An installed C++ application links the ``gui`` component:

.. code-block:: cmake

   find_package(DART CONFIG REQUIRED COMPONENTS gui)
   target_link_libraries(my_viewer PRIVATE dart-gui)
   target_compile_features(my_viewer PRIVATE cxx_std_17)

The ``gui`` component, ``dart-gui`` target, and ``dart/gui/gui.hpp`` aggregate
header reuse names from the GLUT-based GUI that DART 6.19 shipped; they now
provide the Filament viewer, not the GLUT classes.

A minimal viewer wraps an existing world:

.. code-block:: cpp

   #include <dart/gui/Viewer.hpp>
   #include <dart/gui/WorldNode.hpp>
   #include <dart/simulation/World.hpp>

   int main()
   {
     auto world = dart::simulation::World::create();
     dart::gui::Viewer viewer;
     viewer.addWorldNode(std::make_shared<dart::gui::WorldNode>(world));
     viewer.run();
   }

The Python equivalent uses the same class names and camelCase callbacks:

.. code-block:: python

   import dartpy as dart

   world = dart.simulation.World()
   viewer = dart.gui.Viewer()
   viewer.addWorldNode(dart.gui.WorldNode(world))
   viewer.run()

World nodes start paused. Call ``viewer.simulate(True)`` in Python or
``viewer.simulate(true)`` in C++ to enable stepping. Subclasses retain
``customPreRefresh``, ``customPostRefresh``, ``customPreStep``,
``customPostStep``, and ``setupViewer``. ``RealTimeWorldNode`` provides the
real-time target and stepping statistics. Viewers retain their registered
world nodes, attachments, widgets, and event handlers through shared ownership.

Window and UI scaling
---------------------

The default interactive window fits the monitor, and the UI follows its DPI.
``setUpViewInWindow(x, y, width, height)`` takes native window units:
physical pixels on Linux and Windows, and points on macOS.
``setGuiScale(scale)`` overrides automatic scaling with a positive multiplier;
``setAutomaticGuiScale()`` restores automatic scaling. ``DART_GUI_SCALE`` sets
the initial override when the application does not supply one.

Left-drag orbits the camera, middle-drag pans, and right-drag or the mouse
wheel zooms, stopping at the 0.01 m and 1000 m clip distances. Ctrl-drags are
reserved for manipulation tools.
``setCameraHome`` sets the eye, center, and up vectors; ``home`` restores them.
Use ``GUIEventHandler`` for input and ``InteractiveFrame`` with
``enableDragAndDrop`` for DART manipulation tools. Picking uses visible visual
geometry, independently of collision geometry.

Capture and debug output
------------------------

``setUpViewForCapture(width, height)`` takes exact physical pixel dimensions.
On Linux it creates a Vulkan capture context without an X11 or Wayland display.
It supports both a Vulkan GPU driver and an explicitly selected Mesa Lavapipe
software driver. Interactive Linux and Windows use OpenGL; macOS uses Metal.
Windows and macOS capture still require their native graphics facilities.

After setting the camera, request a PNG and render its frame:

.. code-block:: python

   viewer.setUpViewForCapture(640, 480)
   viewer.setCameraHome([2, -3, 2], [0, 0, 0], [0, 0, 1])
   for _ in range(5):
       viewer.frame()
   viewer.captureScreen("frame.png")
   viewer.frame()

PNG rows have their origin at the top left. The capture includes widgets and
debug overlays. Capture viewers do not read or write ``imgui.ini``. ``record`` queues numbered captures; ``pauseRecording`` stops
them. Missing output directories and write failures raise an error.
``CameraMode.DEPTH`` displays nonlinear camera depth as a screenshot, rather
than returning metric depth or segmentation arrays.

``DebugOverlay`` adds world lines, world labels, fixed-pixel world labels, and
screen labels. Screen label positions use physical pixels with a top-left
origin. Custom font and text outline settings belong to the overlay.
``DebugOverlay``, ``GridVisual``, and ``SupportPolygonVisual`` are viewer
attachments. ``PerformanceStatsPanel`` is a C++ ImGui helper, not an
attachment: call its ``render`` from an ``ImGuiWidget::render()`` override. It
has no Python binding.

The repository capture tool uses Filament:

.. code-block:: console

   pixi run agent-capture -- --scene box_on_ground --steps 250 \
       --layers contacts collision_bounds labels --auto-views 1 \
       --out /tmp/dart-visual-evidence

The capture sidecar records the renderer, graphics backend, and device.
Python demos and tutorials use the same ``dartpy.gui`` API.

Porting OSG applications
------------------------

.. list-table:: DART-owned API migration
   :header-rows: 1
   :widths: 42 58

   * - Existing OSG API
     - Neutral API
   * - ``dart::gui::osg::Viewer`` / ``ImGuiViewer``
     - ``dart::gui::Viewer``; widgets are built into the viewer
   * - ``dartpy.gui.osg.Viewer`` / ``ImGuiViewer``
     - ``dartpy.gui.Viewer``
   * - ``ImGuiViewer::getImGuiHandler`` and ``ImGuiHandler::addWidget`` with
       ``visible``, ``hasWidget``, or ``removeAllWidget``
     - ``Viewer::addWidget`` and ``removeWidget``; ``ImGuiWidget::setVisible``
       sets visibility, and C++ ``Viewer::getImGuiContext`` returns the context
   * - ``ImGuiViewer::showAbout`` / ``hideAbout`` and ``AboutWidget``
     - Removed; add an ``ImGuiWidget`` for custom panels
   * - GUI-scale helpers in ``dart/gui/osg/Utils.hpp`` and
       ``ImGuiHandler::setGuiScale``
     - ``setGuiScale``, ``setAutomaticGuiScale``, and ``DART_GUI_SCALE``
   * - OSG ``WorldNode`` / ``RealTimeWorldNode``
     - Neutral classes with the same names and simulation callbacks
   * - ``WorldNode`` shadow-technique constructor argument,
       ``setShadowTechnique``, ``getShadowTechnique``,
       ``createDefaultShadowTechnique``, and the ``RealTimeWorldNode``
       ``shadower`` argument
     - ``WorldNode::setShadowsEnabled`` and ``isShadowed``
   * - ``getLightGroup``, ``getLightSource``, ``setupDefaultLights``, and
       ``setLightingMode``
     - Removed; ``switchHeadlights`` and ``checkHeadlights`` remain
   * - ``osg::ref_ptr`` for DART GUI objects
     - ``std::shared_ptr`` in C++; retained Python objects in Python
   * - ``osgViewer::Viewer`` methods such as ``realize``, ``getCamera``, and
       ``setCameraManipulator``; ``getRootGroup``; ``int run()``; the
       ``setUpViewInWindow`` screen argument
     - ``setUpViewInWindow``, ``setUpViewForCapture``, or the first ``frame()``
       creates the graphics; ``run()`` returns ``void``
   * - ``frame(simulationTime)``
     - ``frame()``, which takes no simulation time
   * - OSG camera look-at and perspective operations
     - ``setCameraHome``, ``setVerticalFieldOfView``, and ``CameraMode``
   * - OSG offscreen helpers such as ``OffscreenSetup``, ``captureOffscreen``,
       and ``defaultAgentCamera``; ``applyDefaultCameraPose``
     - ``setUpViewForCapture``, ``setCameraHome``, ``captureScreen``, and
       ``frame``
   * - OSG event adapters and handlers
     - ``GUIEventAdapter``, ``GUIActionAdapter``, and ``GUIEventHandler``
   * - ``KeySymbol`` constants for printable keys, such as ``KEY_A`` and
       ``KEY_Space``, including in Python
     - Compare ``getKey()`` with the character: ``'a'`` in C++ or
       ``ord('a')`` in Python
   * - DART grids, overlays, support polygons, and drag tools
     - Classes of the same names in ``dart::gui`` / ``dartpy.gui``
   * - ``dart/gui/osg/osg.hpp``
     - ``dart/gui/gui.hpp``
   * - ``dart/gui/osg/IncludeImGui.hpp`` and the installed OpenGL2 backend
       header ``imgui_impl_opengl2.h``
     - ``<dart/external/imgui/imgui.h>``; the viewer renders ImGui itself

Native ``osg::Node`` children, custom OSG shaders, camera manipulators, graphics
contexts, and embedded OSG windows are renderer-specific. The neutral viewer
does not accept an arbitrary OSG scene graph. Represent DART visuals with
``ShapeFrame`` / ``SimpleFrame`` and their ``VisualAspect``, use neutral
attachments and widgets for annotations, and use the camera and event APIs for
navigation. Applications that need their own shader pipeline or embedded
graphics surface should keep that renderer integration in the application.
The OSG component, installed OSG headers, and ``dartpy.gui.osg`` namespace
are removed in DART 7. Requesting the ``gui-osg`` component reports this
migration, ``DART_BUILD_GUI_OSG=ON`` stops configuration, and
``DART_BUILD_GUI_OSG=OFF`` still selects a headless build. Applications that
depend on native OSG APIs must port their viewer integration or continue using
an older DART release.

Behavior changes
----------------

* Handlers added with ``addEventHandler`` run before the
  ``DefaultEventHandler``, and the first handler that returns ``true`` stops
  dispatch.
* Window and event coordinates have a top-left origin; OSG events used a
  bottom-left origin.
* ``GUIEventAdapter`` ``EventType`` values are sequential rather than OSG bit
  flags, so compare them for equality instead of masking. The
  ``DOUBLECLICK``, ``RESIZE``, pen, and ``USER`` event types are removed.
* Printable keys deliver the typed character for the keyboard layout and
  Shift state; for example, Shift+[ delivers ``{``. Held keys repeat as
  ``KEYDOWN``, and each ``KEYUP`` reports the same key as its ``KEYDOWN``.
  Ctrl, Alt, and Super chords deliver the letter with the modifier mask
  instead of a control character such as 8 for Ctrl+H. Non-printable keys use
  ``KeySymbol`` constants such as ``KEY_Left`` and ``KEY_F1``; keys without a
  constant, such as Caps Lock or keypad Enter, report their GLFW key code.
* Left-drag orbits the camera; the OSG viewer orbited with right-drag.
* The default vertical field of view is 30 degrees, compared with about 29.15
  degrees in OSG. In RGBA mode the near and far clip planes are fixed at
  0.01 m and 1000 m; DEPTH mode still fits them to the visible geometry, as
  OSG did.
* ``ViewerAttachment::customAttach(nullptr)`` runs when an attachment is
  removed or its viewer is destroyed. ``addAttachment`` throws for an
  attachment that belongs to another viewer.
* The frame, shape, and body overloads of ``enableDragAndDrop`` throw for a
  null pointer instead of returning ``nullptr``.
* ``switchHeadlights`` and ``switchDefaultEventHandler`` are not virtual.
* ``captureScreen`` and ``record`` write RGBA PNGs instead of RGB.
* A viewer dispatches window input only inside its own ``frame()``.
  ``frame()`` and graphics setup throw ``std::logic_error`` when called from a
  window-event callback.
