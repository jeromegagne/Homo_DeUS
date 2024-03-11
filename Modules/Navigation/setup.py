## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

distutils_setup = generate_distutils_setup(
    packages=['Navigation'],
    package_dir={'':'NavSelector'}
)

setup(**distutils_setup)