#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jun 29 15:33:04 2020

@author: pi
"""
# https://cython.readthedocs.io/en/latest/src/tutorial/cython_tutorial.html
# python3 setup.py build_ext --inplace

from setuptools import setup
from Cython.Build import cythonize

setup(ext_modules=cythonize("count.pyx"))