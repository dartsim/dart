Section: Breaking Changes

  * Building IO requires urdfdom 4.0.0 or newer. Ubuntu 22.04's apt urdfdom
    (3.0), vcpkg's (3.1) and FreeBSD's ports (3.0) are below that floor:
    install urdfdom 4 or newer, use Pixi, or set `DART_BUILD_IO=OFF`.
    [#3884](https://github.com/dartsim/dart/pull/3884)
