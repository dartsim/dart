# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

from pathlib import Path

# Note: API documentation is hosted on GitHub Pages, not Read the Docs
# This configuration file is for user guides, tutorials, and developer documentation only

# Read DART version from package.xml
def get_dart_version():
    """Read DART version from package.xml."""
    import xml.etree.ElementTree as ET
    package_xml = Path(__file__).parent.parent.parent / 'package.xml'
    try:
        tree = ET.parse(package_xml)
        root = tree.getroot()
        version_elem = root.find('version')
        if version_elem is not None and version_elem.text:
            # Return version with 'v' prefix to match GitHub Pages structure
            return f"v{version_elem.text.strip()}"
    except (FileNotFoundError, ET.ParseError):
        pass
    # Fallback to latest stable version
    return 'v6.13.2'

# Read available API documentation versions from docs_versions.txt
def get_api_versions():
    """Read available API versions from docs_versions.txt."""
    versions_file = Path(__file__).parent.parent.parent / 'scripts' / 'docs_versions.txt'
    versions = []
    try:
        with open(versions_file, 'r') as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith('DART '):
                    versions.append(line)
    except FileNotFoundError:
        # Fallback if file not found
        versions = ['v6.13.2', 'v6.12.2', 'v6.11.1']
    return versions

# Get current DART version from package.xml
current_version = get_dart_version()

# Get list of all available API versions
api_versions = get_api_versions()

# Use current version if available, otherwise use latest from docs_versions.txt
if current_version in api_versions:
    api_version_to_link = current_version
else:
    # For development versions, link to latest stable
    api_version_to_link = api_versions[0] if api_versions else 'v6.13.2'

# Make these available to RST files via html_context
html_context = {
    'current_version': current_version,
    'api_versions': api_versions,
    'api_version_to_link': api_version_to_link,
    'cpp_api_url': f'https://dartsim.github.io/dart/{api_version_to_link}/',
    'python_api_url': f'https://dartsim.github.io/dart/{api_version_to_link}-py/',
    'gh_pages_url': 'https://dartsim.github.io/dart/',
}

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'DART'
copyright = '2011-2024, The DART development contributors'
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
