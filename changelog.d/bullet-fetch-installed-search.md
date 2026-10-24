Section: Build

  * An installed DART built with `DART_USE_SYSTEM_BULLET=OFF` links the
    Bullet that it installed, including a Debug-only installation, even when
    a consumer finds DART through `DART_DIR`, its search paths list another
    Bullet first, or it found another Bullet before DART.
    [#3929](https://github.com/dartsim/dart/pull/3929)
