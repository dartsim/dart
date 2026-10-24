Section: Python

  * Linux pip wheels now include `dartpy.utils.ModelResourceRetriever` for
    verified HTTPS model downloads with system or explicit CA certificates.
    Wheels carry no sample manifests: point `DART_DATA_PATH` at a DART
    checkout's `data/` directory, or register a local manifest for `model://`
    downloads without it; otherwise the constructor warns about missing
    samples. Linux pip source builds now require libcurl >= 8.5 and OpenSSL
    Crypto >= 3.0.13 development packages. macOS and Windows wheels keep the
    retriever disabled.
    [#3931](https://github.com/dartsim/dart/pull/3931)
