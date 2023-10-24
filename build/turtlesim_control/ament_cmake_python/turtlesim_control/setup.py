from setuptools import find_packages
from setuptools import setup

setup(
    name='turtlesim_control',
    version='0.0.0',
    packages=find_packages(
        include=('turtlesim_control', 'turtlesim_control.*')),
)
