#!/usr/bin/env python
from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=["azsp_path_follower_converter"],  # python modules to be exported
    package_dir={"": "src"},
)

setup(**setup_args)
