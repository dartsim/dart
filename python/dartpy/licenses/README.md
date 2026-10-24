# Wheel dependency notices

These upstream texts accompany libraries that wheel repair can bundle for the
model retriever. Only curl and OpenSSL notices currently ship; Linux wheels
bundle those libraries. The other eight texts remain unshipped for a future
macOS/Windows opt-in.

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
