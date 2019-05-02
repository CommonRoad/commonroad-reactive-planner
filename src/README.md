# Ford Fail-Safe Trajectory Planning
The goal of this project is to develop a safety layer fro trajectory planning according to [1].

Currently, this project generates solutions to trajectory planning problems given in the [commonroad](https://commonroad.gitlab.io) .xml format.
The trajectories are determined according to the sampling-based approach in [2].

**Note:**
The fail-safe planner that is currently implemented does hardly find solutions for the given scenarios. This is due to the too conservative occupancy sets
computed with [SPOT](https://spot.in.tum.de).

## Getting Started
These instructions should help you to install the trajectory planner and use it for development and testing purposes. See development to get 
further information about the functionality of the modules.

### Requirements
The software is  developed and tested on recent versions of Linux and OS X.

The requirements for installing and using the software are: C++11 compiler, CMake and Python 3 with development headers.

For the python installation, we suggest the usage of [Anaconda](http://www.anaconda.com/download/#download).
For the development IDE we suggest [PyCharm](http://www.jetbrains.com/pycharm/)

### Prerequisites
This project uses three other framworks:

* cps/automated-driving:

    From this framework we use the collision checker as well as the trapezoid coordinate system.
    Get access to the repository and checkout *fcl_collision_library* branch.
    
    
    
    Part of the code is written in c++ but python libraries from that code are created with pybind11. To build it, do the following:

    1. Install Eigen library:
    
        ```
        sudo apt-get install libeigen3-dev
        ```
        
    2. install LIBCCD https://github.com/danfis/libccd
    
        Important to download from git master and not a zipped release, because it contains an important bugfix.
        Then run:
        
        ```
        sudo apt-get install cmake
        cd libccd
        mkdir build
        cd build
        cmake -G "Unix Makefiles" -DENABLE_DOUBLE_PRECISION=ON -DBUILD_SHARED_LIBS=ON ..
        make
        sudo make install
        ```
        
    3. install boost:
    
        ```
        sudo apt-get install libboost-dev libboost-thread-dev libboost-test-dev libboost-filesystem-dev
        ```
    4. install FCL https://github.com/flexible-collision-library/fcl
        Download from git master and not zipped Release 
        run:
        
        ```
        cd fcl
        cmake .
        make
        sudo make install
        ```
    5. build the automated-driving framework: find make commands in docs/index.rst.
        set the path to the built library and python_binding.
        
        **Important:**
        while building the project with cmake, make sure the parameters for *reach* and *geometry* are enabled:
        ```
        cmake -DPYTHON_INCLUDE_DIR="/path/to/python/include/python3.5m" -DCMAKE_BUILD_TYPE=Release -DADD_MODULE_REACH=ON -DADD_MODULE_GEOMETRY=ON ..
        ```

    6. probably you have to add the following code to automated-driving/python_binding/src/module_geometry.cc to the TrapezoidCoordinateSystem class.
        
        ```
        .def("find_segment_index", [](geometry::TrapezoidCoordinateSystem &t, double s){
        return t.findSegmentIndex(s);)
        ```
    7. finally, add the following code to automated-driving/python_binding/src/module_collision.cc CollisionChecker class.
        ```
        .def("collision_time", [](const std::shared_ptr<collision::CollisionChecker> &cc,
                       std::shared_ptr<collision::CollisionObject> &co) {
          return cc->collision_time(co);
        })
        ```

* cps/commonroad framework:

    This framework contains commonroad xml-file reader and writer and classes to operate with the scenarios and goal regions.
    It is completely written in python, so you do not have to install it.
    So clone the repository, checkout the branch *feature_commonroad_2018a_scenario* and set the path to commonroad/tools/Python.


* spot:

    SPOT is used for the calculation of occupancy sets. Currently, there is no public access to the c++ code, to we use the offline version in additional_software/spot.zip.
    It includes an installation_guide that has to be followed to install.
    Afterwards copy the file spot/automated-driving_python/spot_predicion.py to commonroad/tools/Python/commonroad/prediction/.
    To test if the installation was successful, run spot_test.py.

After installing everything, you should have the following structure:

```
project/
    ├-Ford/
    ├-automated-driving/
    ├-commonroad/
    └-spot/
```

### Installation of Ford

The code is fully written in python. Only set the path to Ford repository to the interpreter path in PyCharm.

Output videos are created with *ffmpeg*. If you want to use the video option, download the package with the following command:
```
sudo apt-get install ffmpeg
```

### Run
The main script of this project is *trajectory_planner.py*.
Add the following parameter to the Run/Debug configuration of *trajectory_planner.py* in PyCharm:
```
Mandatory:
    file_path: path to .xml scenario in commonroad format
    output_dir: directory to save solution .xml files and videos optionally

Optional:
    -v: Enables the video generation
    -fs: Enables fail-safe planning. Otherwise only a trajectory is planned
```

|   NGSIM/US101     | NGSIM/Lankershim  | Hand-Crafted  |Cooperative          |
|-------------------|-------------------| ------------- |---------------------|
| USA_US101-1_1_T-1 |USA_Lanker-1_1_T-1 |DEU_A9-1_1_T-1 |C-USA_Lanker-1_1_T-1 |
| USA_US101-2_1_T-1 |USA_Lanker-1_2_T-1 |DEU_A9-2_1_T-1 |C-USA_Lanker-1_2_T-1 |
| USA_US101-4_1_T-1 |USA_Lanker-1_3_T-1 |DEU_A99-1_1_T-1|C-USA_Lanker-2_2_T-1 |            
| USA_US101-5_1_T-1 |USA_Lanker-1_6_T-1 |DEU_Ffb-1_2_S-1|C-USA_Lanker-2_3_T-1 |         
| USA_US101-7_1_T-1 |USA_Lanker-1_7_T-1 |DEU_Ffb-2_1_T-1|C-USA_Lanker-2_4_T-1 |           
| USA_US101-8_1_T-1 |USA_Lanker-2_2_T-1 |DEU_Ffb-2_2_Sgit-1|C-USA_US101-30_1_T-1 |            
| USA_US101-9_1_T-1 |USA_Lanker-2_3_T-1 |DEU_Gar-1_1_T-1|C-USA_US101-31_1_T-1 |           
| USA_US101-10_1_T-1|USA_Lanker-2_7_T-1 |               |C-USA_US101-32_1_T-1 | 
| USA_US101-11_1_T-1|USA_Lanker-2_8_T-1 |               |C-USA_US101-33_1_T-1 |
| USA_US101-12_1_T-1|USA_Lanker-2_10_T-1|               |                     |
| USA_US101-13_1_T-1|USA_Lanker-2_13_T-1|               |                     |
| USA_US101-14_1_T-1|USA_Lanker-2_14_T-1|               |                     |
| USA_US101-15_1_T-1|                   |               |                     |
| USA_US101-16_1_T-1|                   |               |                     |
| USA_US101-17_1_T-1|                   |               |                     |
| USA_US101-18_1_T-1|                   |               |                     |
| USA_US101-19_1_T-1|                   |               |                     |
| USA_US101-20_1_T-1|                   |               |                     |
| USA_US101-21_1_T-1|                   |               |                     |
| USA_US101-23_1_T-1|                   |               |                     |
| USA_US101-26_1_T-1|                   |               |                     |
| USA_US101-27_1_T-1|                   |               |                     |
| USA_US101-28_1_T-1|                   |               |                     |


## Developement
The current project structure is
```
Ford/
    ├-trajectory_planner.py
    ├-trajectory_tree.py
    ├-fail_safe_planner.py
    ├-create_trajectory_bundle.py
    ├-trajectory_bundle.py
    ├-combined_trajectory.py
    ├-polynomial_trajectory.py
    ├-sampling_area.py
    ├-utils.py
    ├-parameter.py
    ├-plot_trajectories.py
    ├-modify_scenarios.ipynb
    └-README.md
```

### trajectory_planner.py
Opens the input scenario file. For every planning problem in this scenario, a solution trajectory is planned.
The solution is written to a .xml file and saved in the output directory. If enabled, a video file is created and also saved in the output directory.

### trajectory_tree.py
Since there is no high-level planner yet, trajectories are planned according to a graph search.
To every node there are at most three possible maneuver: *go straight*, *change the lane to the left*, and *change the lane to the right*.
For each of those possiblities, the best trajectory is planned according to [2] and added to the tree.
We perform a uniform-cost search to find the optimal solution to the goal region.

### fail_safe_planner.py
The fail-safe planner takes a scenario and a desired trajectory and tries to find a fail-safe trajectory from the latest possible time index.
For that it first calls SPOT to calculate the occupancy sets of all dynamic obstacles in the scenario. Afterwards it finds the last safe time index by performing
collision checks with the ego occupancy along the desired trajectory and the occupancy sets of the dynamic obstacles.
Then it calculates the sampling area as the safe region for the current lane and the adjacent left and right lane.

**Important**: currently we do not calculate the safe regions. This is hard coded and has to be adapted. The code has to be included from the Bachelor thesis of
Leo Tappe (supervised by Christian Pek).

Finally, it creates polynomial trajectory samples as in [2] to find a feasible and collision free (considering occupancy sets) way to the goal.
Note, that here we do not use a tree search, but assume that we can reach the safe region with one planning cycle.

### create_trajectory_bundle.py
The approach in [2] allows two planning modes: Reach a certain position or reach a certain velocity.
Both modes are implemented here:
reach velocity:
We sample in time domain and generate lateral trajectories as quintic polynomials.
We sample in time and in velocity domain to generate longitudinal trajectories as quartic polynomials.
reach position:
We sample in both directions in the time and position domain and generate quintic polynomials.

### trajectory_bundle.py
For one planning cycle, all sampled trajectories are saved in this class as combined trajectories. The optimal trajectory from this samples can be determined here.

### combined_trajectory.py
This class inherits from the trajectory class in automated-driving/python/scenario/trajectory.py.
Therefore, a collision object can be generated from those combined trajectories.
When initializing an object with a given longitudinal and lateral trajectory and optionally a curvilinear coordinate system, the states are transformed to Cartesian coordinates.
Furthermore, those combined trajectories can be checked for feasibility and collisions.

### polynomial_trajectory.py
According to [2], the longitudinal and lateral trajectories are determined as quintic or quartic polynomials.
For a given start and goal state, the coefficients of both are determined here.

### sampling_area.py
This should be the interface to the computation of safe regions. However, there is no code yet.

### utils.py
This file contains some additional functions, e.g. the creation of curvilinear coordinate systems

### parameter.py
This file contains all parameters:
Vehicle parameter, like dimensions and physical limits.
Planning parameter, e.g. planning stepsize and weighting factors.
Drawing parameters for plotting.

### plot_trajectories.py
This file provides two main functions: one for plotting one image for each dircrete state of the desired trajectory and the fail-safe trajectory and one for creating videos.
Note, that the video creation only covers desired trajectories so far.

### modify_scenarios.ipynb
This notebook helps to modify scenarios from xml files. Since most of the scenarios are not solvable yet, because SPOT occupancy sets are too conservative,
one can remove obstacles or change goal regions or initial states.


## Literature
[1] Magdici, S. and Althoff, M. *Fail-Safe Motion Planning of Autonomous Vehicles*. Proc. of the 19th International IEEE Conference on Intelligent Transportation Systems, 2016.

[2] Werling M., et al. *Optimal trajectory generation for dynamic street scenarios in a frenet frame*. In: IEEE International Conference on Robotics and Automation, Anchorage, Alaska, 987–993.
