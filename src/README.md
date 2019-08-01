## Getting Started
These instructions should help you to install the reactive planner and use it for development and testing purposes. See development to get 
further information about the functionality of the modules.

### Requirements
The software is  developed and tested on recent versions of Linux and OS X.

The requirements for installing and using the software are: C++11 compiler, CMake and Python 3 with development headers.

For the python installation, we suggest the usage of [Anaconda](http://www.anaconda.com/download/#download).
For the development IDE we suggest [PyCharm](http://www.jetbrains.com/pycharm/)

### Prerequisites
This project uses three other libraries from the CommonCoad framwork:

* [CommonRoad](https://commonroad.in.tum.de/)

        pip install commonroad-io
    
* [CommonRoad-Collision-Checker](https://commonroad.in.tum.de/static/docs/collision-checker/user/installation.html#installation)
    
    Please have al look at the [Collision Checker](https://commonroad.in.tum.de/static/docs/collision-checker/user/installation.html#installation)'s documentation.
    Follow the "Installation of Third Party Libraries" and "Full Installation with Anaconda".
    
* cps/commonroad-curvilinear-coordinate-system

    The collision-checker is necessary to install the curvilinear coordinate system. 
    For the Installation follow the instructions in the installation Guide in the README.

    
### Installation of Reactive-Planner

The reactive-planner is completely written in python, so you do not have to install it. 
    
    1. Clone the repository from cps/reactive-planner
    2. Add the root-folder of the Reactive-Planner to your Python-Interpreter (in your environment)

# Reactive-planner
Currently the reactive planner consists of three main componenets: reactive planner, route planner and high level state machine.

# reactive-planner
This project generates solutions to trajectory planning problems given in the [commonroad](https://commonroad.gitlab.io) .xml format.
The trajectories are determined according to the sampling-based approach in [2] using quintic polynomials.

### Main routine
The main script of this project is *main.py*. Currently the high level state machine plans the reference path according to the generated reference paths of the route planner. 
The trajectory planning is executed with the reactive planner.

The following Scenarios are tested and supported by the current version reactive planner:

|   NGSIM/US101     | NGSIM/Lankershim  | Hand-Crafted  |Cooperative          |
|-------------------|-------------------| ------------- |---------------------|
| USA_US101-1_1_T-1 |USA_Lanker-1_1_T-1 |DEU_A9-1_1_T-1 |C-USA_Lanker-1_1_T-1 |
| USA_US101-2_1_T-1 |USA_Lanker-1_2_T-1 |DEU_A9-2_1_T-1 |C-USA_Lanker-1_2_T-1 |
| USA_US101-4_1_T-1 |USA_Lanker-1_3_T-1 |DEU_A99-1_1_T-1|C-USA_Lanker-2_2_T-1 |            
| USA_US101-5_1_T-1 |USA_Lanker-1_6_T-1 |DEU_Ffb-1_2_S-1|C-USA_Lanker-2_3_T-1 |         
| USA_US101-7_1_T-1 |USA_Lanker-1_7_T-1 |DEU_Ffb-2_1_T-1|C-USA_Lanker-2_4_T-1 |           
| USA_US101-8_1_T-1 |USA_Lanker-2_2_T-1 |DEU_Ffb-2_2_Sgit-1|C-USA_US101-30_1_T-1 |            
| USA_US101-9_1_T-1 |USA_Lanker-2_3_T-1 |DEU_Gar-1_1_T-1|C-USA_US101-31_1_T-1 |           
| USA_US101-10_1_T-1|USA_Lanker-2_7_T-1 |ZAM_HW-1_1_S-1 |C-USA_US101-32_1_T-1 | 
| USA_US101-11_1_T-1|USA_Lanker-2_8_T-1 |ZAM_Intersect-1_1_S-1|C-USA_US101-33_1_T-1 |
| USA_US101-12_1_T-1|USA_Lanker-2_10_T-1|ZAM_Intersect-1_2_S-1|                     |
| USA_US101-13_1_T-1|USA_Lanker-2_13_T-1|ZAM_Merge-1_1_T-1|                     |
| USA_US101-14_1_T-1|USA_Lanker-2_14_T-1|ZAM_Over-1_1   |                     |
| USA_US101-15_1_T-1|                   |ZAM_Urban-1_1_S-1|                     |
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

# route planner
The route planner can also be executed independently via the '__main__': function at the end of route_planner.py. 
There the basic functionalities can ve individually tested. 
There are 3 important functions that can be used:
- find_reference_path_and_lanelets_leading_to_goal: Returns all lanelets connecting the source with the goal lanelet and a reference path.
- plan_all_reference_paths: plans a funnel-shaped reference path network connecting all lanelets with the goal. This method is used for the high level planner
- set_reference_lane: Can perform a lane change if plan_all_reference_paths was called before.

In the current main.py an example for using the two latter methods is provided and can currently solve the following scenarios:

|   NGSIM/US101     | Hand-Crafted  |Cooperative          |
|-------------------| ------------- |---------------------|
| USA_US101-1_1_T-1 |DEU_A9-1_1_T-1 |C-USA_US101-31_1_T-1 |
| USA_US101-2_1_T-1 |DEU_A9-2_1_T-1 |C-USA_US101-32_1_T-1 |
| USA_US101-3_1_T-1 |DEU_A99-1_1_T-1|C-USA_US101-33_1_T-1 |
| USA_US101-4_1_T-1 |DEU_B471-1_1_T-1 |      |
| USA_US101-6_1_T-1 |DEU_Muc-4_1_T-1 |  |    
| USA_US101-5_1_T-1 |DEU_Ffb-1_2_S-1| |         
| USA_US101-7_1_T-1 |DEU_Ffb-2_1_T-1| |           
| USA_US101-8_1_T-1 |DEU_Ffb-2_2_Sgit-1| |            
| USA_US101-9_1_T-1 |Deu_Gar-1_1_T-1| |           
| USA_US101-10_1_T-1|ZAM_HW-1_1_S-1 | | 
| USA_US101-11_1_T-1|ZAM_Intersect-1_1_S-1| |
| USA_US101-12_1_T-1|ZAM_Intersect-1_2_S-1| |
| USA_US101-13_1_T-1|ZAM_Merge-1_1_T-1| |
| USA_US101-14_1_T-1|ZAM_Over-1_1   |  |
| USA_US101-15_1_T-1|ZAM_Urban-1_1_S-1| |
| USA_US101-16_1_T-1|DEU_Muc-3_1_T-1| |
| USA_US101-17_1_T-1|DEU_Muc-2_1_T-1| |
| USA_US101-18_1_T-1|DEU_Muc-1_1_T-1| |
| USA_US101-19_1_T-1|                   |               |                     
| USA_US101-20_1_T-1|                   |               |                     
| USA_US101-21_1_T-1|                   |               |                     
| USA_US101-22_1_T-1||               |
| USA_US101-23_1_T-1| |               |
| USA_US101-24_1_T-1||               |
| USA_US101-25_1_T-1|                   |
| USA_US101-26_1_T-1|                   |               |                     
| USA_US101-27_1_T-1|                   |               |                     
| USA_US101-28_1_T-1|                   |               |                     
| USA_US101-29_1_T-1|                   |               | 


## Literature
[1] Magdici, S. and Althoff, M. *Fail-Safe Motion Planning of Autonomous Vehicles*. Proc. of the 19th International IEEE Conference on Intelligent Transportation Systems, 2016.

[2] Werling M., et al. *Optimal trajectory generation for dynamic street scenarios in a frenet frame*. In: IEEE International Conference on Robotics and Automation, Anchorage, Alaska, 987–993.
