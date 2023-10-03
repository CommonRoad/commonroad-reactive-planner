from setuptools import setup, find_packages
import os

setup_dir = os.path.dirname(os.path.realpath(__file__))
with open(f"{setup_dir}/README.md", "r") as fh:
    long_description = fh.read()

setup(
    name="commonroad-reactive-planner",
    version="2023.1",
    description="CommonRoad Reactive Planner: Sampling-based Frenet Planner",
    long_description_content_type='text/markdown',
    long_description=long_description,
    url="https://commonroad.in.tum.de/",
    author='Cyber-Physical Systems Group, Technical University of Munich',
    author_email='commonroad@lists.lrz.de',
    license='BSD',
    packages=find_packages(exclude=['doc', 'unit_tests']),
    zip_safe=False,
    include_package_data=True,
    python_requires='>=3.7',
    install_requires=[
        'commonroad_vehicle_models>=3.0.0',
        'matplotlib>=2.2.2',
        'networkx>=2.2',
        'numpy>=1.13',
        'methodtools',
        'omegaconf>=2.1.1',
        'pytest>=6.2.5',
        'scipy>=1.5.2',
        'commonroad-route-planner>=2022.3',
        'commonroad-io>=2022.3',
        'commonroad-drivability-checker>=2023.1',
        ],
    classifiers=[
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "License :: OSI Approved :: BSD License (BSD)",
        "Operating System :: POSIX :: Linux",
    ]
)
