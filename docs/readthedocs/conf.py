# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

from pathlib import Path
from datetime import datetime
import importlib
import keyword
import re
import shutil
import subprocess
import sys

from sphinx.util import logging

logger = logging.getLogger(__name__)

# Paths used during the documentation build
DOCS_ROOT = Path(__file__).resolve().parent


def _find_repo_root(docs_root: Path) -> Path:
    """Locate the repository root by walking up until package.xml is found."""

    for candidate in (docs_root, *docs_root.parents):
        if (candidate / "package.xml").exists():
            return candidate

    parents = docs_root.parents
    if len(parents) > 1:
        fallback = parents[1]
    elif parents:
        fallback = parents[0]
    else:
        fallback = docs_root
    logger.warning(
        "Unable to locate package.xml when resolving repository root from %s; "
        "falling back to %s",
        docs_root,
        fallback,
    )
    return fallback


REPO_ROOT = _find_repo_root(DOCS_ROOT)
DOXYFILE_TEMPLATE = REPO_ROOT / "docs" / "doxygen" / "Doxyfile.in"
GENERATED_DOXYFILE = DOCS_ROOT / "_build" / "cpp_api_Doxyfile"
GENERATED_ASSETS_DIR = DOCS_ROOT / "_generated"
CPP_API_OUTPUT_DIR = GENERATED_ASSETS_DIR / "cpp-api"

# ``cpp_api_available`` is consumed by ``sphinx.ext.ifconfig`` to decide
# whether the embedded iframe should be rendered.
cpp_api_available = False


def _rename_placeholder(text: str, name: str) -> str:
    pattern = re.compile(rf"([(\s,]){re.escape(name)}(?=\s*:)")
    counter = 0

    def repl(match: re.Match[str]) -> str:
        nonlocal counter
        counter += 1
        return f"{match.group(1)}{name}_param_{counter}"

    return pattern.sub(repl, text)


def _sanitize_stub_source(text: str) -> str:
    """Rename invalid identifiers originating from the stub generator."""

    def _replace_keyword(match: re.Match[str]) -> str:
        return f"{match.group('prefix')}{match.group('name')}_"

    for kw in keyword.kwlist:
        pattern = re.compile(rf"(?P<prefix>[\(\s,])(?P<name>{kw})(?=\s*:)")
        text = pattern.sub(_replace_keyword, text)

    # Replace C++ namespace separators with valid identifiers.
    text = text.replace("::", "_")

    # Handle placeholder arguments that otherwise duplicate names.
    for placeholder in ("std", "dart"):
        text = _rename_placeholder(text, placeholder)

    return text


def _prepare_stub_modules(package: str) -> bool:
    """Copy *.pyi stubs into a temporary module tree so autodoc can import them."""

    stubs_root = REPO_ROOT / "python" / "stubs"
    src_pkg = stubs_root / package
    if not src_pkg.exists():
        return False

    generated_root = Path(__file__).parent / "_generated_stubs"
    dest_pkg = generated_root / package
    shutil.rmtree(dest_pkg, ignore_errors=True)
    dest_pkg.parent.mkdir(parents=True, exist_ok=True)
    shutil.copytree(src_pkg, dest_pkg, dirs_exist_ok=True)

    # Rename *.pyi -> *.py so CPython's import machinery can load them.
    for stub_file in dest_pkg.rglob("*.pyi"):
        sanitized = _sanitize_stub_source(stub_file.read_text())
        py_path = stub_file.with_suffix(".py")
        py_path.write_text(sanitized)
        stub_file.unlink()

    if str(generated_root) not in sys.path:
        sys.path.insert(0, str(generated_root))
    return True


def _ensure_dartpy_available() -> None:
    """Make sure `import dartpy` succeeds for autodoc, falling back to stubs."""

    try:
        importlib.import_module("dartpy")
    except (ModuleNotFoundError, ImportError) as exc:
        if _prepare_stub_modules("dartpy"):
            sys.stderr.write("Using generated dartpy stubs for autodoc.\n")
        else:
            sys.stderr.write(
                "WARNING: dartpy module and stubs are unavailable; "
                "dartpy API pages will be empty.\n"
            )
        if not isinstance(exc, ModuleNotFoundError):
            sys.stderr.write(
                f"Ignoring installed dartpy due to import failure: {exc}\n"
            )


_ensure_dartpy_available()


def _ensure_cpp_api_extra_path(_app, _config):
    """Create the ``html_extra_path`` target before validation prunes it."""

    CPP_API_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)


def _posix_path(path: Path) -> str:
    """Return a POSIX-style path string for Doxygen."""

    return str(path).replace("\\", "/")


def _render_doxyfile(output_path: Path):
    """Render the configured Doxyfile used to build the C++ API docs."""

    if not DOXYFILE_TEMPLATE.exists():
        raise FileNotFoundError(f"Missing Doxyfile template: {DOXYFILE_TEMPLATE}")

    replacements = {
        "DART_VERSION": get_dart_version(prefix_with_v=False),
        "DOXYGEN_EXCLUDE": _posix_path(REPO_ROOT / "dart" / "external"),
        "DOXYGEN_EXTRA_INPUTS": _posix_path(REPO_ROOT / "docs" / "doxygen" / "mainpage.dox"),
        "DOXYGEN_GENERATE_TAGFILE": _posix_path(CPP_API_OUTPUT_DIR / "dart.tag"),
        "DOXYGEN_INCLUDE_PATH": _posix_path(REPO_ROOT),
        "DOXYGEN_INPUT_ROOT": _posix_path(REPO_ROOT / "dart"),
        "DOXYGEN_OUTPUT_ROOT": _posix_path(CPP_API_OUTPUT_DIR),
        "DOXYGEN_STRIP_FROM_PATH": _posix_path(REPO_ROOT),
    }

    doxyfile_contents = DOXYFILE_TEMPLATE.read_text()
    for key, value in replacements.items():
        doxyfile_contents = doxyfile_contents.replace(f"@{key}@", value)

    output_path.write_text(doxyfile_contents)

def _read_dart_semver():
    """Return the DART semantic version as read from package.xml."""
    import xml.etree.ElementTree as ET

    package_xml = REPO_ROOT / "package.xml"
    try:
        tree = ET.parse(package_xml)
        root = tree.getroot()
        version_elem = root.find("version")
        if version_elem is not None and version_elem.text:
            return version_elem.text.strip()
    except (FileNotFoundError, ET.ParseError):
        pass
    # Fallback to latest stable version
    return "6.16.0"


def get_dart_version(prefix_with_v: bool = True):
    """Return the DART version and optionally prefix it with ``v``."""

    semver = _read_dart_semver()
    return f"v{semver}" if prefix_with_v else semver


def _read_api_versions():
    """Return the list of published documentation versions."""

    versions_file = REPO_ROOT / "scripts" / "docs_versions.txt"
    versions = []
    try:
        for line in versions_file.read_text().splitlines():
            line = line.strip()
            if line and not line.startswith("DART "):
                versions.append(line)
    except FileNotFoundError:
        # Fallback to a safe default when running locally without the helper.
        versions = ["v6.16.0"]
    return versions


# Get current DART version from package.xml
current_version = get_dart_version()
api_versions = _read_api_versions()
if current_version in api_versions:
    api_version_to_link = current_version
else:
    api_version_to_link = api_versions[0] if api_versions else current_version

html_context = {
    "current_version": current_version,
    "api_versions": api_versions,
    "api_version_to_link": api_version_to_link,
    "cpp_api_url": f"https://dartsim.github.io/dart/{api_version_to_link}/",
    "python_api_url": f"https://dartsim.github.io/dart/{api_version_to_link}-py/",
    "gh_pages_url": "https://github.com/dartsim/dart/tree/gh-pages",
}

# Provide easy-to-reuse substitutions in the RST files.
rst_epilog = f"""
.. |python_api_url| replace:: {html_context['python_api_url']}
"""

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = "DART"
copyright = f"2011-{datetime.now().year}, The DART development contributors"
author = "The DART development contributors"

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    "myst_parser",
    "sphinx.ext.autodoc",
    "sphinx.ext.autosummary",
    "sphinx.ext.napoleon",
    "sphinx.ext.viewcode",
    "sphinx.ext.intersphinx",
    "sphinx.ext.todo",
    "sphinx.ext.ifconfig",
    "sphinx_tabs.tabs",
    "sphinx_copybutton",
]

# Napoleon settings for Python API
napoleon_google_docstring = True
napoleon_numpy_docstring = True

# Autodoc settings
autodoc_default_options = {
    "members": True,
    "undoc-members": True,
    "show-inheritance": True,
}

# Intersphinx mapping
intersphinx_mapping = {
    "python": ("https://docs.python.org/3", None),
    "numpy": ("https://numpy.org/doc/stable/", None),
}

# Add any paths that contain templates here, relative to this directory.
templates_path = ["_templates"]

# Map .md files to the m2r2 parser
source_suffix = {
    ".rst": "restructuredtext",
    ".md": "markdown",
}

# The master toctree document.
root_doc = "index"

exclude_patterns = [
    "_build",
    "Thumbs.db",
    ".DS_Store",
    "README.md",
    "dartpy/api/*",
]


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = "sphinx_rtd_theme"
html_static_path = ["_static"]
html_logo = "_static/img/dart_logo_long_200.png"
html_favicon = "_static/img/dart_logo_32.png"
html_theme_options = {
    "logo_only": True,
}
html_extra_path = ["_generated"]

# Setting for multi language
source_encoding = "utf-8"
locale_dirs = ["locales/"]
gettext_uuid = True
gettext_compact = False

# Options for LaTeX output
latex_engine = "xelatex"
latex_elements = {
    "preamble": r"""
        \usepackage{kotex}
    """
}

def build_cpp_api_docs(app):
    """Generate the Doxygen HTML bundle so RTD serves versioned C++ docs."""

    global cpp_api_available

    if app.builder.format != "html":
        return

    cpp_api_available = False

    doxygen_executable = shutil.which("doxygen")
    if not doxygen_executable:
        logger.warning(
            "Doxygen was not found on PATH; skipping embedded C++ API docs."
        )
        app.config.cpp_api_available = False
        return

    if CPP_API_OUTPUT_DIR.exists():
        shutil.rmtree(CPP_API_OUTPUT_DIR)
    CPP_API_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    GENERATED_DOXYFILE.parent.mkdir(parents=True, exist_ok=True)

    try:
        _render_doxyfile(GENERATED_DOXYFILE)
        subprocess.run(
            [doxygen_executable, str(GENERATED_DOXYFILE)],
            check=True,
            cwd=REPO_ROOT,
        )
    except (OSError, subprocess.CalledProcessError) as exc:
        logger.warning("Failed to build C++ API docs via Doxygen: %s", exc)
        app.config.cpp_api_available = False
        return

    cpp_api_available = True
    app.config.cpp_api_available = True
    logger.info("C++ API reference generated at %s", CPP_API_OUTPUT_DIR)


def setup(app):
    app.connect("config-inited", _ensure_cpp_api_extra_path, priority=700)
    app.connect("builder-inited", build_cpp_api_docs)
