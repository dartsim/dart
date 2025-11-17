# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

from pathlib import Path
from datetime import datetime
import shutil
import subprocess

from sphinx.util import logging

logger = logging.getLogger(__name__)

# Paths used during the documentation build
DOCS_ROOT = Path(__file__).resolve().parent


def _find_repo_root(docs_root: Path) -> Path:
    """Locate the repository root by walking up until package.xml is found."""

    for candidate in (docs_root, *docs_root.parents):
        if (candidate / 'package.xml').exists():
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
DOXYFILE_TEMPLATE = REPO_ROOT / 'docs' / 'doxygen' / 'Doxyfile.in'
GENERATED_DOXYFILE = DOCS_ROOT / '_build' / 'cpp_api_Doxyfile'
CPP_API_OUTPUT_DIR = DOCS_ROOT / 'cpp-api'

# ``cpp_api_available`` is consumed by ``sphinx.ext.ifconfig`` to decide
# whether the embedded iframe should be rendered.
cpp_api_available = False


def _read_dart_semver():
    """Return the DART semantic version as read from package.xml."""
    import xml.etree.ElementTree as ET

    package_xml = REPO_ROOT / 'package.xml'
    try:
        tree = ET.parse(package_xml)
        root = tree.getroot()
        version_elem = root.find('version')
        if version_elem is not None and version_elem.text:
            return version_elem.text.strip()
    except (FileNotFoundError, ET.ParseError):
        pass
    # Fallback to latest stable version
    return '6.16.0'


def get_dart_version(prefix_with_v: bool = True):
    """Return the DART version and optionally prefix it with ``v``."""

    semver = _read_dart_semver()
    return f"v{semver}" if prefix_with_v else semver

# Read available API documentation versions from docs_versions.txt
def get_api_versions():
    """Read available API versions from docs_versions.txt."""
    versions_file = REPO_ROOT / 'scripts' / 'docs_versions.txt'
    versions = []
    try:
        with open(versions_file, 'r') as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith('DART '):
                    versions.append(line)
    except FileNotFoundError:
        # Fallback if file not found
        versions = ['v6.16.0', 'v6.15.0']
    return versions


def _posix_path(path: Path) -> str:
    """Return a POSIX-style path string for Doxygen."""

    return str(path).replace('\\', '/')


def _render_doxyfile(output_path: Path):
    """Render the configured Doxyfile used to build the C++ API docs."""

    if not DOXYFILE_TEMPLATE.exists():
        raise FileNotFoundError(f"Missing Doxyfile template: {DOXYFILE_TEMPLATE}")

    replacements = {
        'DART_VERSION': get_dart_version(prefix_with_v=False),
        'DOXYGEN_EXCLUDE': _posix_path(REPO_ROOT / 'dart' / 'external'),
        'DOXYGEN_EXTRA_INPUTS': _posix_path(REPO_ROOT / 'docs' / 'doxygen' / 'mainpage.dox'),
        'DOXYGEN_GENERATE_TAGFILE': _posix_path(CPP_API_OUTPUT_DIR / 'dart.tag'),
        'DOXYGEN_INCLUDE_PATH': _posix_path(REPO_ROOT),
        'DOXYGEN_INPUT_ROOT': _posix_path(REPO_ROOT / 'dart'),
        'DOXYGEN_OUTPUT_ROOT': _posix_path(CPP_API_OUTPUT_DIR),
        'DOXYGEN_STRIP_FROM_PATH': _posix_path(REPO_ROOT),
    }

    doxyfile_contents = DOXYFILE_TEMPLATE.read_text()
    for key, value in replacements.items():
        doxyfile_contents = doxyfile_contents.replace(f"@{key}@", value)

    output_path.write_text(doxyfile_contents)

# Get current DART version from package.xml
current_version = get_dart_version()

# Get list of all available API versions
api_versions = get_api_versions()

# Use current version if available, otherwise use latest from docs_versions.txt
if current_version in api_versions:
    api_version_to_link = current_version
else:
    # For development versions, link to latest stable
    api_version_to_link = api_versions[0] if api_versions else 'v6.16.0'

# Make these available to RST files via html_context
html_context = {
    'current_version': current_version,
    'api_versions': api_versions,
    'api_version_to_link': api_version_to_link,
    'gh_pages_url': 'https://github.com/dartsim/dart/tree/gh-pages',
}

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'DART'
copyright = f'2011-{datetime.now().year}, The DART development contributors'
author = 'The DART development contributors'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'myst_parser',
    'sphinx.ext.autodoc',
    'sphinx.ext.autosummary',
    'sphinx.ext.napoleon',
    'sphinx.ext.viewcode',
    'sphinx.ext.intersphinx',
    'sphinx.ext.todo',
    'sphinx.ext.ifconfig',
    'sphinx_tabs.tabs',
    'sphinx_copybutton',
]

# Napoleon settings for Python API
napoleon_google_docstring = True
napoleon_numpy_docstring = True

# Autodoc settings
autodoc_default_options = {
    'members': True,
    'undoc-members': True,
    'show-inheritance': True,
}

# Intersphinx mapping
intersphinx_mapping = {
    'python': ('https://docs.python.org/3', None),
    'numpy': ('https://numpy.org/doc/stable/', None),
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

exclude_patterns = ["_build", "Thumbs.db", ".DS_Store", "README.md"]


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = "sphinx_rtd_theme"
html_static_path = ["_static"]
html_logo = "_static/img/dart_logo_long_200.png"
html_favicon = "_static/img/dart_logo_32.png"
html_theme_options = {
    "logo_only": True,
}
html_extra_path = ["cpp-api"]

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

    if app.builder.format != 'html':
        return

    cpp_api_available = False

    doxygen_executable = shutil.which('doxygen')
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
    app.add_config_value('cpp_api_available', False, 'env')
    app.connect('builder-inited', build_cpp_api_docs)
