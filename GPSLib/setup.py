import os
from setuptools import setup

def read(fname):
    return open(os.path.join(os.path.dirname(__file__), fname)).read()

setup(
    name = "GPSLib",
    version = "0.0.1",
    author = "Rinoy Pazhekattu",
    author_email = "rinoy@rinoyp.com",
    description = ("GPS Helper functions"),
    license = "BSD",
    keywords = "GPS",
    packages = ["GPSLib"],
    platforms = ['linux'],
    long_description = read('README'),
    classifiers = [
        "Development Status :: 3 - Alpha",
        "Topic :: Utilities",
        "License :: OSI Approved :: BSD License",
    ],
)
