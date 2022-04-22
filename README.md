# Reactive Planner

Currently, this project generates solutions to trajectory planning problems given in the [CommonRoad](https://commonroad.in.tum.de/) scenario format.
The trajectories are generated according to the sampling-based approach in [1]. 

## Getting Started
These instructions should help you to install the trajectory planner and use it for development and testing purposes.

### Requirements
The software is  developed and tested on recent versions of Linux and OS X.

For the python installation, we suggest the usage of [Anaconda](http://www.anaconda.com/download/#download).
For the development IDE we suggest [PyCharm](http://www.jetbrains.com/pycharm/)

### Installation
1. Clone this repository & create a new conda environment `conda create -n commonroad-py37 python=3.7`


2. Install the package:
    * Install the package via pip: `pip install .`
    * **Or** install the dependencies with `pip install -r requirements.txt` and add the root folder to the python path of your interpreter



### Example script

Two example scripts are provided:

* `run_combined_planner.py`: plan intended trajectories for motion planning
* `ford_test.py`: use reactive planner as fail-safe planner (in combination with spot) **-> DEPRECATED**


## Literature
[1] Werling M., et al. *Optimal trajectory generation for dynamic street scenarios in a frenet frame*. In: IEEE International Conference on Robotics and Automation, Anchorage, Alaska, 987–993.
