Section: Build

  * Builds with `DART_BUILD_UTILS_ASSETS=ON` require OpenSSL Crypto 3.0.13 or
    newer, the Ubuntu 24.04 LTS version. OpenSSL 1.1.1 and Ubuntu 22.04's 3.0.2
    are below that floor: install a newer OpenSSL, use Pixi, or leave
    `DART_BUILD_UTILS_ASSETS` off.
