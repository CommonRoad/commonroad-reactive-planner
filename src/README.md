# Reactive Planner

Currently, this project generates solutions to trajectory planning problems given in the [commonroad](https://commonroad.gitlab.io) .xml format.
The trajectories are determined according to the sampling-based approach in [2].


## Getting Started
These instructions should help you to install the trajectory planner and use it for development and testing purposes. See development to get
further information about the functionality of the modules.

### Requirements
The software is  developed and tested on recent versions of Linux and OS X.

The requirements for installing and using the software are: C++11 compiler, CMake and Python 3 with development headers.

For the python installation, we suggest the usage of [Anaconda](http://www.anaconda.com/download/#download).
For the development IDE we suggest [PyCharm](http://www.jetbrains.com/pycharm/)

### Prerequisites
1. add the folder reactive_planner to your python path

2. his project uses three other framworks:

  * commonroad-collision-checker:

      This is an open-sourced tool to check collision between objects. Please clone it from [here](https://gitlab.lrz.de/tum-cps/commonroad-collision-checker) and follow the installation instructions in the readme.

  * commonroad-io:

      The commonroad_io package provides methods to read, write, and visualize CommonRoad scenarios and planning problems. The package can be installed via pip:

      `pip install commonroad-io`

  * commonroad-curvilinear-coordinate-system:

      This tool is used to create a Frenet frame. Currently, there is no public access. Please extract the commonroad-curvilinear-coordinate-system.zip file we provided and follow the installation instructions in the readme.

  * boundary

      This tool is used to generate obstacles representing road boundaries. It can be found under https://gitlab.lrz.de/cps/commonroad-road-boundary. The cloned folder needs to be added to the python path.

  * spot:

      SPOT is used for the calculation of occupancy sets. Currently, there is no public access to the C++ code. Please extract the spot-cpp-master.zip we provided and follow the installation steps in section `Installation SPOT with Python-Interface (using commonroad-io)`.



### Example script

An example can be executed by

```shell
example_script.py -p /path/to/scenario/folder
```




## Literature
[1] Magdici, S. and Althoff, M. *Fail-Safe Motion Planning of Autonomous Vehicles*. Proc. of the 19th International IEEE Conference on Intelligent Transportation Systems, 2016.

[2] Werling M., et al. *Optimal trajectory generation for dynamic street scenarios in a frenet frame*. In: IEEE International Conference on Robotics and Automation, Anchorage, Alaska, 987–993.
