from datetime import date

project = "ENPM818Z-FALL-2025"
author = "Z. Kootbally"
copyright = f"{date.today().year}, {author}"
release = "v1.0"

extensions = [
    "myst_parser",
    "sphinx.ext.autosummary",
    "sphinxcontrib.mermaid",
    "sphinx_autodoc_typehints",
    "sphinx_copybutton",
    "sphinx.ext.autodoc",
    "sphinx.ext.napoleon",
    "sphinx.ext.intersphinx",
    "sphinx_design",
    "sphinxcontrib.pseudocode",
]


intersphinx_mapping = {
    "python": ("https://docs.python.org/3", None),
}


templates_path = ["_templates"]
exclude_patterns = []
# html_theme = 'sphinx_rtd_theme'
html_theme = "furo"
# Furo-specific options (all optional)
html_theme_options = {
    "sidebar_hide_name": True,  # hides large project name in sidebar
    "source_repository": "https://github.com/zeidk/enpm818Z-fall-2025",
    "source_branch": "main",
    # "source_directory": "docs/source/",
    "light_logo": "enpm818Z-light.png",
    "dark_logo": "enpm818Z-dark.png",
}
html_static_path = ["_static"]
master_doc = "index"

html_static_path = ["_static"]
html_css_files = ["custom.css"]


# ----------------------------------------------------------------------
# Custom CSS for Furo Theme
# ----------------------------------------------------------------------


# def setup(app):
#     app.add_css_file(
#         "custom.css",
#         "https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.5.0/css/all.min.css",
#     )
