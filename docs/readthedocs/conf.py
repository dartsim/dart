# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

import os
import sys

# Configure Python path for dartpy autodoc
# Use compiled dartpy module when PYTHONPATH is set (local builds with pixi)
pythonpath_env = os.environ.get('PYTHONPATH', '')
if pythonpath_env:
    # Local build - use compiled dartpy module
    for path in pythonpath_env.split(':'):
        if path and os.path.exists(path):
            sys.path.insert(0, path)
    print("✓ Using compiled dartpy from PYTHONPATH")
    print("  Full Python API documentation will be generated")
else:
    # Read the Docs build - dartpy cannot be compiled
    print("⚠ WARNING: PYTHONPATH not set - dartpy module not available")
    print("  Python API documentation will be empty on Read the Docs")
    print("  This is expected until dartpy wheels are published to PyPI")

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
