# SPDX-FileCopyrightText: 2022 CERN (home.cern)
#
# SPDX-License-Identifier: CC0-1.0

# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
# import os
# import sys
# sys.path.insert(0, os.path.abspath('.'))


# -- Project information -----------------------------------------------------

project = 'FMC-ADC-100M-14B-4CHA'
copyright = u'2022, CERN, documentation released under CC-BY-SA-4.0'
author = 'Matthieu Cattin, Dimitris Lampridis <dimitrios.lampridis@cern.ch>, Federico Vaga <federico.vaga@cern.ch>'

import os
import re

release = os.popen('git describe --tags').read().strip()
version = re.sub('^v', '', release.split('-')[0])

# The suffix(es) of source filenames.
# You can specify multiple suffix as a list of string:
#
# source_suffix = ['.rst', '.md']
source_suffix = '.rst'


# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = ['sphinx.ext.autodoc',
              'sphinx.ext.todo',
              'sphinx.ext.coverage',
              'sphinx.ext.graphviz',
]

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# The root document.
master_doc = 'index'

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']


# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = 'sphinx_rtd_theme'

# If true, links to the reST sources are added to the pages.
#
html_show_sourcelink = False

# If true, "Created using Sphinx" is shown in the HTML footer. Default is True.
#
html_show_sphinx = False

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']

# -- Options for LaTeX output ---------------------------------------------

# Grouping the document tree into LaTeX files. List of tuples
# (source start file, target name, title,
#  author, documentclass [howto, manual, or own class]).
latex_documents = [
    (master_doc,
     'FMCADC100M14bit4Channel.tex',
     'FMC ADC 100M 14 bit 4 Channel Documentation',
     'Matthieu Cattin,\\\Dimitris Lampridis \\textless{}dimitrios.lampridis@cern.ch\\textgreater{},\\\ Federico Vaga \\textless{}federico.vaga@cern.ch\\textgreater{}',
     'manual'),
]
# The name of an image file (relative to this directory) to place at the top of
# the title page.
#
latex_logo = 'fig/ohr_logo_lowres.png'

# Will be appended to every rst source file in order to provide a reference to the latest version
rst_epilog = '.. |latest_release| replace:: %s' % version
