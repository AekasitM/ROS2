from setuptools import find_packages
from setuptools import setup

setup(
    name='turtlesim_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('turtlesim_interfaces', 'turtlesim_interfaces.*')),
)
