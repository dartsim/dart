# DART Documentation

To install dependencies to build the documentation, navigate to `<dart_root>/docs/readthedocs` and run:

```
pip install -r .\requirements.txt
```

To build the documentation using Sphinx, navigate to `<dart_root>/docs/readthedocs` and run:

```
./make.bat html # on Windows
make html # on other platforms
```

Note that on Windows, you need to use make.bat instead of make. Additionally, you may need to remove the _build directory before building to ensure a clean build.
