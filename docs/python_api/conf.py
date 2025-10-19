# Configuration file for dartpy Python API documentation

import os
import sys

# Add the dartpy module path from the build directory
# The PYTHONPATH environment variable should be set when running sphinx-build
# to point to the build directory containing dartpy
pythonpath_env = os.environ.get('PYTHONPATH', '')
if pythonpath_env:
    for path in pythonpath_env.split(':'):
        if path and os.path.exists(path):
            sys.path.insert(0, path)

# -- Project information -----------------------------------------------------
project = 'dartpy'
copyright = '2011-2025, The DART development contributors'
author = 'The DART development contributors'
version = '7.0.0'
release = '7.0.0-alpha0'

# -- General configuration ---------------------------------------------------
extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon',
    'sphinx.ext.viewcode',
    'sphinx.ext.intersphinx',
]

# Napoleon settings
napoleon_google_docstring = True
napoleon_numpy_docstring = True

# Autodoc settings
autodoc_default_options = {
    'members': True,
    'undoc-members': True,
    'show-inheritance': True,
}

# Templates path
templates_path = ['_templates']

# The master toctree document
root_doc = 'index'

# List of patterns to ignore
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

# -- Options for HTML output -------------------------------------------------
html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']

# -- Options for intersphinx -------------------------------------------------
intersphinx_mapping = {
    'python': ('https://docs.python.org/3', None),
    'numpy': ('https://numpy.org/doc/stable/', None),
}
