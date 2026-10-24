/*
 * Copyright (c) 2026, The DART development contributors
 * All rights reserved.
 *
 * This file is provided under the BSD license in the root LICENSE file.
 */

#ifndef DART_UTILS_ASSETS_DETAIL_CABUNDLE_HPP_
#define DART_UTILS_ASSETS_DETAIL_CABUNDLE_HPP_

#include <filesystem>
#include <initializer_list>
#include <string>

#ifdef DART_APPLE_WHEEL_TLS
  #include <openssl/crypto.h>
#endif

namespace dart {
namespace utils {
namespace detail {

#ifdef DART_APPLE_WHEEL_TLS
static inline bool initializeWheelCrypto()
{
  // Suppress the build environment's config before our first OpenSSL/curl use.
  static const auto initialized
      = OPENSSL_init_crypto(OPENSSL_INIT_NO_LOAD_CONFIG, nullptr);
  return initialized == 1;
}
#endif

inline const std::initializer_list<std::filesystem::path> caBundleCandidates{
    "/etc/ssl/certs/ca-certificates.crt",
    "/etc/pki/tls/certs/ca-bundle.crt",
    "/etc/ssl/ca-bundle.pem",
    "/etc/ssl/cert.pem"};

static inline const char* defaultCaBundle(const char* bundle)
{
#ifdef DART_APPLE_WHEEL_TLS
  // Wheel relocation leaves the build environment's CA path in libcurl.
  (void)bundle;
  return nullptr;
#else
  return bundle;
#endif
}

// The search list is injectable so tests need no changes to system trust files.
inline std::string findCaBundle(
    const char* curlBundle,
    const char* sslFile,
    const char* defaultBundle,
    std::initializer_list<std::filesystem::path> candidates)
{
  if (curlBundle && *curlBundle)
    return curlBundle;
  if (sslFile && *sslFile)
    return sslFile;

  std::error_code error;
  if (defaultBundle) {
    const bool present = std::filesystem::exists(defaultBundle, error);
    // Preserve curl's failure if its configured path cannot be inspected.
    if (present || error)
      return defaultBundle;
  }
  for (const auto& path : candidates) {
    if (std::filesystem::is_regular_file(path, error))
      return path.string();
  }
  return "";
}

} // namespace detail
} // namespace utils
} // namespace dart

#endif // DART_UTILS_ASSETS_DETAIL_CABUNDLE_HPP_
