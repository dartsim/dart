# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

import xml.etree.ElementTree as ET
from pathlib import Path


def _read_dart_version() -> str:
    package_xml = Path(__file__).resolve().parents[2] / "package.xml"
    try:
        version = ET.parse(package_xml).getroot().findtext("version", default="")
    except (FileNotFoundError, ET.ParseError):
        return "0.0.0"
    return version.strip() or "0.0.0"


# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = "DART: Dynamic Animation and Robotics Toolkit"
copyright = "Copyright (c) 2011, The DART development contributors"
author = "The DART development contributors"
version = _read_dart_version()
release = version

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    "myst_parser",
    "sphinx_copybutton",
    "sphinx_rtd_theme",
    "sphinx_tabs.tabs",
]

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
