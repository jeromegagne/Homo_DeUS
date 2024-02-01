#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

distutils_setup = generate_distutils_setup(
    packages=['homodeus_hbba_cfg'],
    package_dir={'':'scripts'}
)

setup(**distutils_setup)