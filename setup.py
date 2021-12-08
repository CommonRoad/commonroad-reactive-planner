from setuptools import setup, find_packages
from os import path

this_directory = path.abspath(path.dirname(__file__))
with open(path.join(this_directory, 'requirements.txt'), encoding='utf-8') as f:
    required = f.read().splitlines()
with open("README.md", "r") as fh:
    long_description = fh.read()

setup(
    name="reactive-planner",
    version="2021.12",
    description="Reactive Planner",
    long_description=long_description,
    url="https://gitlab.lrz.de/cps/reactive-planner",
    packages=find_packages(),
    python_requires='>=3.7',
    install_requires=required,
    classifiers=[
        "Programming Language :: Python :: 3.8",
        "License :: OSI Approved :: MIT License",
        "Operating System :: POSIX :: Linux",
        "Operating System :: MacOS"
    ]
)