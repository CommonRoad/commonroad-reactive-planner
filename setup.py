from setuptools import setup, find_packages

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
        'commonroad-io>=2021.4',
        'iso3166>=2.0.2',
        'lxml>=4.6.4',
        'matplotlib>=3.5.0',
        'networkx>=2.6.3',
        'numpy>=1.21.4',
        'omegaconf>=2.1.1',
        'Pillow>=8.4.0',
        'pytest>=6.2.5',
        'scipy>=1.7.3',
        'setuptools>=58.0.4',
        'Shapely>=1.8.0'
    ],
    classifiers=[
        "Programming Language :: Python :: 3.8",
        "License :: OSI Approved :: MIT License",
        "Operating System :: POSIX :: Linux",
        "Operating System :: MacOS",
    ]
)