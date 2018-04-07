#!/usr/bin/env python
# coding: utf-8

import os
from os.path import dirname, join
from setuptools import setup, find_packages

NAME = "rosconversions"
VERSION = "0.1.0"
DESCRIPTION = "This module provides conversions of ROS Message to python data structure and back"

REQUIRES = []
with open(join(dirname(__file__), 'requirements.txt')) as f:
    REQUIRES = f.read().splitlines()

def read(fname):
    return open(join(dirname(__file__), fname)).read()

setup(
    name=NAME,
    version=VERSION,
    description=DESCRIPTION,
    author="Konstantinos Panayiotou",
    author_email="klpanagi@gmail.com",
    maintainer='Konstantinos Panayiotou',
    url="",
    keywords=[],
    install_requires=REQUIRES,
    packages=find_packages(),
    include_package_data=True,
    long_description=read('README.md') if os.path.exists('README.md') else ""
)
