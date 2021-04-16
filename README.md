# Reactive Planner

Currently, this project generates solutions to trajectory planning problems given in the [commonroad](https://commonroad.gitlab.io) .xml format.
The trajectories are determined according to the sampling-based approach in [1]. 

## Getting Started
These instructions should help you to install the trajectory planner and use it for development and testing purposes. See development to get further information about the functionality of the modules.

### Requirements
The software is  developed and tested on recent versions of Linux and OS X.

The requirements for installing and using the software are: C++11 compiler, CMake and Python 3 with development headers.

For the python installation, we suggest the usage of [Anaconda](http://www.anaconda.com/download/#download).
For the development IDE we suggest [PyCharm](http://www.jetbrains.com/pycharm/)

### Prerequisites
1. add the folder `reactive-planner/src` to your python path

2. this project uses three other framworks:

  * **commonroad-drivability-checker**:

      This is an open-sourced tool to check collision and road compliance between objects. Please clone it from [here](https://gitlab.lrz.de/tum-cps/commonroad-drivability-checker) and follow the installation instructions in the readme.

  * **commonroad-io**:

      The commonroad_io package provides methods to read, write, and visualize CommonRoad scenarios and planning problems. The package can be installed via pip:

      `pip install commonroad-io`

  * **commonroad-curvilinear-coordinate-system**:

      This tool is used to create a Frenet frame. Currently, there is no public access. Please clone the `develop` branch of the [repository](https://gitlab.lrz.de/cps/commonroad-curvilinear-coordinate-system.git) and follow the installation instructions in the readme:

      `git clone -b development https://gitlab.lrz.de/cps/commonroad-curvilinear-coordinate-system.git`

  * (Optional) **commonroad-route-planner**:

    This tool is used to create a polynomial reference path from an initial point to a goal region for a CommonRoad scenario. Please clone the `develop` branch of the [repository](https://gitlab.lrz.de/cps/commonroad-route-planner.git) and follow the installation instructions in the readme:

    `git clone -b develop https://gitlab.lrz.de/cps/commonroad-route-planner.git`

  * (Optional, only for using reactive planner as fail-safe planner) **spot**:

      SPOT is used for the calculation of occupancy sets. Currently, there is no public access to the C++ code. Please clone the [repository](https://gitlab.lrz.de/cps/spot-cpp.git) and follow the installation steps in section `Installation SPOT with Python-Interface (using commonroad-io)`.

      `git clone https://gitlab.lrz.de/cps/spot-cpp.git`



### Example script

Two example scripts are provided:

* `run_combined_planner.py`: plan intended trajectories for motion planning
* `ford_test.py`: use reactive planner as fail-safe planner (in combination with spot)


## Literature
[1] Werling M., et al. *Optimal trajectory generation for dynamic street scenarios in a frenet frame*. In: IEEE International Conference on Robotics and Automation, Anchorage, Alaska, 987–993.
