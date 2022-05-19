from setuptools import setup, find_packages

with open("README.md", "r") as fh:
    long_description = fh.read()

setup(
    name="commonroad-reactive-planner",
    version="2022.1",
    description="Reactive Planner: Sampling-based Frenet Planner",
    long_description_content_type='text/markdown',
    long_description=long_description,
    url="https://commonroad.in.tum.de/",
    author='Cyber-Physical Systems Group, Technical University of Munich',
    author_email='commonroad@lists.lrz.de',
    license='GNU General Public License v3.0',
    packages=find_packages(exclude=['doc', 'unit_tests']),
    zip_safe=False,
    include_package_data=True,
    python_requires='>=3.7',
    install_requires=[
        'commonroad_vehicle_models>=2.0.0',
        'matplotlib>=2.2.2',
        'networkx>=2.2',
        'numpy>=1.13',
        'methodtools',
        'omegaconf>=2.1.1',
        'pytest>=6.2.5',
        'scipy>=1.5.2',
        'commonroad-route-planner>=2022.1',
        'commonroad-io>=2021.4',
        'commonroad-drivability-checker>=2021.4',
        ],
    classifiers=[
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.8",
        "License :: OSI Approved :: GNU General Public License v3 (GPLv3)",
        "Operating System :: POSIX :: Linux",
        "Operating System :: MacOS"
    ]
)
