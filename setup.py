from setuptools import setup, find_packages
from os import path

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
    install_requires=[
        'commonroad_vehicle_models>=2.0.0',
        'matplotlib>=2.2.2',
        'networkx>=2.2',
        'numpy>=1.13',
        'omegaconf>=2.1.1',
        'pytest>=6.2.5',
        'scipy>=1.5.2',
        'commonroad-route-planner>=2022.1',
        'commonroad-io>=2021.4',
        'commonroad-drivability-checker>=2021.4',
        ],
    classifiers=[
        "Programming Language :: Python :: 3.8",
        "License :: OSI Approved :: MIT License",
        "Operating System :: POSIX :: Linux",
        "Operating System :: MacOS"
    ]
)
