Section: Build

  * Standalone GUI builds now fetch GLFW 3.4 and build it static (X11 only on
    Linux, which needs the X11 development headers), so they work where the
    system GLFW is missing or older than 3.4, such as Ubuntu 24.04's 3.3.10.
    Set `DART_USE_SYSTEM_GLFW=ON` or `DART_USE_SYSTEM_DEPENDENCIES=ON` to keep
    using an installed GLFW 3.4. macOS keeps using the installed GLFW.
    [#3942](https://github.com/dartsim/dart/pull/3942)
