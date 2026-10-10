"""Prefetch or verify the complete bundles used by the humanoid demos."""

import argparse

import dartpy as dart

from .scenes.modern_humanoids import MODEL_URIS


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("models", nargs="*", choices=tuple(MODEL_URIS))
    parser.add_argument("--cache", default="", help="Override the persistent cache directory")
    parser.add_argument("--offline", action="store_true", help="Verify cached bundles only")
    args = parser.parse_args()
    if not hasattr(dart.utils, "ModelResourceRetriever"):
        parser.error("dartpy was built without the utils-assets component")
    retriever = dart.utils.ModelResourceRetriever(args.cache, args.offline)
    for model in args.models or MODEL_URIS:
        try:
            path = retriever.getFilePath(MODEL_URIS[model])
            if not path:
                raise RuntimeError("Verified model bundle is unavailable")
        except RuntimeError as error:
            parser.error(f"{model}: {error}")
        print(f"{model}: {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
