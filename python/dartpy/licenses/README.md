# Wheel dependency notices

These upstream texts accompany libraries that wheel repair can bundle for the
model retriever. Only the curl and OpenSSL notices ship from this directory;
Linux wheels bundle those libraries. The other eight texts remain unshipped for
a future macOS/Windows opt-in.

- `curl.txt`: curl 8.23.0, `COPYING` from the checked release archive.
- `openssl.txt`: OpenSSL 3.5.9, `LICENSE.txt` (Apache 2.0).
- `nghttp2.txt`: libnghttp2 1.68.1, `COPYING`.
- `libssh2.txt`: libssh2 1.11.1, `COPYING`.
- `zlib.txt`: zlib 1.3.2, `LICENSE`.
- `zstd.txt`: Zstandard 1.5.7, `LICENSE` (BSD alternative).
- `krb5.txt`: MIT Kerberos 1.22.2, complete `doc/notice.rst`.
- `libpsl.txt`: libpsl 0.23.1, `LICENSE`.
- `public-suffix-list.txt`: MPL-2.0 notice and source for libpsl's built-in list.
- `icu.txt`: ICU 78.3, complete `LICENSE`.

The additional notices come from `info/licenses/` in the macOS arm64 packages
selected by `py312-wheel` in `pixi.lock`, checked against its archive hashes.
The retriever is currently enabled only in Linux wheels. These notices remain
ready for a future macOS/Windows opt-in after its size and trust gates pass.
The locked macOS curl links nghttp2, libssh2, libpsl, OpenSSL, Kerberos, zstd
and zlib; libpsl also links ICU. The locked Windows curl links libssh2, libpsl
and zlib with Schannel; libpsl depends on ICU there too. Its libssh2 uses
OpenSSL. DART's SHA-256 support
also requires OpenSSL's libcrypto on Windows.
Check the libraries and notices again when updating the lockfile, and inspect
the inventory from the first macOS and Windows retriever opt-in CI runs for
further dependencies.

## Linux wheel notices

Linux wheels also ship a notice for every other library that they bundle or
compile in, and for the third-party code inside assimp and ODE. The repository
does not store these notices. `docker/wheels/Dockerfile.manylinux_2_28` copies
each one, unedited, from the SHA-256-checked source archive or the AlmaLinux 8
package. Then `scripts/wheel_notices.py` adds them to the repaired wheel next to
`curl.txt` and `openssl.txt`. `scripts/verify_wheel.py` maps each library in
`dartpy.libs/` to its notices (`BUNDLED_NOTICES`). It rejects a wheel that lacks
a mapped notice or that bundles a library with no mapping, and names the notice
and the library.

| Notice | Library | Upstream version | License | File in the source |
| --- | --- | --- | --- | --- |
| `assimp.txt` | `libassimp` | assimp 6.0.5 | BSD-3-Clause | `LICENSE` |
| `console_bridge.txt` | `libconsole_bridge` | console_bridge 1.0.2 | BSD-3-Clause | `LICENSE` |
| `fcl.txt` | `libfcl` | FCL 0.7.0 | BSD-3-Clause | `LICENSE` |
| `gz-math.txt` | `libgz-math` | gz-math 9.3.0 | Apache-2.0 | `LICENSE` |
| `gz-utils.txt` | `libgz-utils` | gz-utils 4.0.0 | Apache-2.0 | `LICENSE` |
| `libccd.txt` | `libccd` | libccd 2.1 | BSD-3-Clause | `BSD-LICENSE` |
| `libjpeg-turbo.txt` | `libjpeg` | libjpeg-turbo 1.5.3 (AlmaLinux) | IJG AND BSD-3-Clause AND Zlib | `LICENSE.md` |
| `libpng.txt` | `libpng16` | libpng 1.6.34 (AlmaLinux) | Libpng | `LICENSE` |
| `octomap.txt` | `liboctomap`, `liboctomath` | OctoMap 1.10.1 | BSD-3-Clause | `octomap/LICENSE.txt` |
| `ode.txt` | `libode` | ODE 0.16.6 | BSD-3-Clause option | `LICENSE-BSD.TXT` |
| `sdformat.txt` | `libsdformat` | sdformat 16.1.0 | Apache-2.0 | `LICENSE` |
| `urdfdom.txt` | `liburdfdom_model`, `liburdfdom_sensor`, `liburdfdom_world` | urdfdom 6.0.1 | BSD-3-Clause | `LICENSE` |
| `bullet.txt` | compiled into `dartpy._core` | Bullet 3.25 | Zlib | `LICENSE.txt` |
| `glfw.txt` | compiled into `dartpy._core` | GLFW 3.4 | Zlib | `LICENSE.md` |
| `tinyxml2.txt` | compiled into `dartpy._core`, sdformat and urdfdom | TinyXML-2 11.0.0 | Zlib | `LICENSE.txt` |

The AlmaLinux files come from `/usr/share/licenses/<package>/`. The wheels
load the system's zlib, which the manylinux policy allows, so they ship no zlib
notice. ODE offers LGPL-2.1-or-later or BSD-3-Clause; DART uses the BSD option,
as `NOTICE` records. urdfdom_headers 3.0.1, which urdfdom compiles in, has the
same `LICENSE` text as urdfdom. `scripts/wheel_notices.py` fails rather than
replace a notice that the wheel already has.

### Code inside the libraries

assimp and ODE compile in third-party code that has its own license, and
libjpeg-turbo's `LICENSE.md` refers to a second file. The image copies these
files the same way, and `BUNDLED_NOTICES` requires them for the library:

| Notice | Library | Code | License | File in the source |
| --- | --- | --- | --- | --- |
| `assimp-earcut.txt` | `libassimp` | earcut.hpp | ISC | `contrib/earcut-hpp/LICENSE` |
| `assimp-openddlparser.txt` | `libassimp` | OpenDDL-Parser | MIT | `contrib/openddlparser/LICENSE` |
| `assimp-poly2tri.txt` | `libassimp` | Poly2Tri | BSD-3-Clause | `contrib/poly2tri/LICENSE` |
| `assimp-pugixml.txt` | `libassimp` | pugixml | MIT | `contrib/pugixml/LICENSE.md` |
| `assimp-rapidjson.txt` | `libassimp` | RapidJSON | MIT | `contrib/rapidjson/license.txt` |
| `ode-opcode.txt` | `libode` | OPCODE | ODE's terms (LGPL-2.1+ or BSD) | `OPCODE/COPYING` |
| `ode-ou.txt` | `libode` | OU | BSD-3-Clause option | `ou/LICENSE-BSD.TXT` |
| `libjpeg-turbo-ijg.txt` | `libjpeg` | the IJG license | IJG | `README.ijg` |

assimp's own `LICENSE` includes a Poly2Tri notice, but with older copyright
years than the compiled files carry, so `assimp-poly2tri.txt` ships too.

This list comes from building assimp 6.0.5 and ODE 0.16.6 with the
Dockerfile's flags, not from the source tree. It uses the objects, the
`contrib/` files in the compiler's `.o.d` dependency files, and `nm -C` on the
library. The build also compiles in the code below, which needs no notice of
its own:

| Code | License | Why no notice |
| --- | --- | --- |
| assimp Clipper and utf8cpp | BSL-1.0 | The license exempts machine-executable object code. |
| assimp minizip (`contrib/unzip`) | zlib terms | The license has no notice for binaries. `unzip.c` defines `NOUNCRYPT`, so no Info-ZIP-derived decryption code is compiled. |

The image's assimp has no Open3DGC, the codec of the glTF 1.0 importer, which
is off. Its MIT notice exists only in source headers, and assimp builds it only
when it finds librt, so the image hides librt. ODE links the system libccd,
which ships as `libccd.txt`, and its GIMPACT stays off.

libjpeg-turbo's `LICENSE.md` points to `README.ijg` for the IJG license text.
The AlmaLinux base image sets `tsflags=nodocs`, which skips that file, so the
image installs the `libjpeg-turbo` package once with its docs. The library does
not change, and the file is the one that the package supplies.

When a library joins the wheel, copy its license file in its Dockerfile block
and add it to `BUNDLED_NOTICES`. When assimp or ODE changes version, build it
with the Dockerfile's flags again and check which vendored code the build
compiles. If a new upstream release moves a copied file, the image build fails.
