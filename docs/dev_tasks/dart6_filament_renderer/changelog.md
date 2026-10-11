# Changelog decision

- Decision: the entry lives under DART 6.21.0 "Breaking Changes" because this
  removes a renderer component, installed headers, the Python OSG namespace,
  and the OpenSceneGraph dependency.
- Content: Filament-backed `dart::gui` / `dartpy.gui` replacement; removed
  `gui-osg` component, `dart/gui/osg` headers and `dartpy.gui.osg`; reuse of
  the GLUT-era `gui` / `dart-gui` / `gui.hpp` names; `DART_BUILD_GUI`
  replacing `DART_BUILD_GUI_OSG` with default `OFF`; GUI dependency floor
  (Filament 1.77.3, GLFW 3.4, libpng, libjpeg, ImGui >= 1.92.8); the
  `external-imgui` component kept without the OpenGL2 backend; a link to the
  Read the Docs visualization migration guide.
- Wording describes resulting behavior only; it makes no equal/better, faster,
  leak-free or release-ready claim.
- The previous 6.21 deprecation / 6.22 removal wording is superseded and removed
  from this unreleased entry. Released historical changelog entries are kept.
- Add the actual PR link only after explicitly approved publication.
