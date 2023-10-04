# Reactive Planner

This project generates solutions to trajectory planning problems given in the [CommonRoad](https://commonroad.in.tum.de/) scenario format.
The trajectories are generated using the sampling-based approach in [1][2]. This approach plans motions by sampling a 
discrete set of trajectories, represented as quintic polynomials in a Frenet frame and selecting an optimal trajectory according
to a given cost function.

<img src="doc/images/ZAM_Tjunction-1_42_T-1.gif" alt="reactive-planner" width="400"/>


## Getting Started
These instructions should help you to install the trajectory planner and use it for development and testing purposes.

To install the trajectory planner from PyPi, please run:
```shell
pip install commonroad-reactive-planner
```

### Requirements
The software is  developed and tested on recent versions of Linux. The required python dependencies are listed in `requirements.txt`.

For the python installation, we suggest the usage of [Anaconda](http://www.anaconda.com/download/#download).

For the development IDE we suggest [PyCharm](http://www.jetbrains.com/pycharm/)


### Installation from Source
1. Clone this repository & create a new conda environment, e.g.,  `conda create -n commonroad-py37 python=3.7`


2. Install the package:
    * Install the package via pip: `pip install .`
    * **Or** install the dependencies with `pip install -r requirements.txt` and add the root folder to the python path of your interpreter



### How to run

Main example script `run_planner.py`: The example script shows how to run the planner on an exemplary 
CommonRoad scenario. Therein, the following steps are included:
* creating a planner configuration
* instantiating the reactive planner
* running the planner in planning cycles with a fixed re-planning frequency


## Literature
[1] Werling M., et al. *Optimal trajectory generation for dynamic street scenarios in a frenet frame*. In: IEEE International Conference on Robotics and Automation, Anchorage, Alaska, 987–993.

[2] Werling M., et al. *Optimal trajectories for time-critical street scenarios using discretized terminal manifolds* In:
The International Journal of Robotics Research, 2012
