.. reactive-planner documentation master file, created by
   sphinx-quickstart on Tue Jan 25 11:55:49 2022.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Reactive Planner
================
Currently, this project generates solutions to trajectory planning problems given in the `CommonRoad <https://commonroad.in.tum.de/>`__ scenario format.
The trajectories are generated according to the sampling-based approach in [1][2].



Getting Started
===============
These instructions should help you to install the trajectory planner and use it for development and testing purposes.

Requirements
------------
The software is  developed and tested on recent versions of Linux and OS X.

For the python installation, we suggest the usage of `Anaconda <http://www.anaconda.com/download/#download>`__.

For the development IDE we suggest `PyCharm <http://www.jetbrains.com/pycharm/>`__.

Installation
------------
1. Clone this repository & create a new conda environment:

.. code-block:: python

   git clone git@gitlab.lrz.de:cps/reactive-planner.git
   conda create -n commonroad-py37 python=3.7

2. Install the package:

* Install the package via pip:

.. code-block:: python

   pip install .

* **Or** install the dependencies via the requirements file and add the root folder to the python path of your interpreter.

.. code-block:: python

   pip install -r requirements.txt


Example script
--------------

An example script `run_planner.py` is provided, which plans intended trajectories for motion planning

.. toctree::
   :maxdepth: 2
   :caption: Contents:

   user/index.rst
   api/index.rst


Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

Literature
==========
[1] Werling M., et al. *Optimal trajectory generation for dynamic street scenarios in a frenet frame*. In: IEEE International Conference on Robotics and Automation, Anchorage, Alaska, 987–993.

[2] Werling M., et al. Optimal trajectories for time-critical street scenarios using discretized terminal manifolds In: The International Journal of Robotics Research, 2012
