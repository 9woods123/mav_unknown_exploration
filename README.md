


![Build Tests (Ubuntu 18.04 + ROS Melodic, Ubuntu 20.04 + ROS Noetic)](https://github.com/ethz-asl/mav_active_3d_planning/actions/workflows/build_test.yml/badge.svg)
# mav\_active\_3d\_planning
**mav\_active\_3d\_planning** is a modular framework for online informative path planner (IPP) design. 

Based on great work of Lukas Schmid who provides a modular framework for creating, evaluating and employing primarily sampling based, receding horizon algorithms that optimize a gain while minimizing a cost. 

We make the motion smoother by using local planner and trajectory optimizer, and test it in GAZEBO. The experiment video is as following:

https://user-images.githubusercontent.com/78521063/220043048-22541d99-4bff-4c22-9a14-28c1d1943787.mp4


**Setup**
* [Packages](#Packages)
* [Dependencies](#Dependencies)
* [Installation](#Installation)
* [Data Repository](#Data-Repository)

**Examples**
* [Configuring a Planner](#Configuring-a-Planner)
* [Run an Experiment](#Run-an-Experiment)

**Documentation**
* [Planner Structure](https://github.com/ethz-asl/mav_active_3d_planning/wiki/Planner-Structure)
* [Planner Design Framework](https://github.com/ethz-asl/mav_active_3d_planning/wiki/Planner-Design-Framework)
* [Running and Evaluating a Simulated Experiment](https://github.com/ethz-asl/mav_active_3d_planning/wiki/Running-and-Evaluating-a-Simulated-Experiment)
* [Code Index](https://github.com/ethz-asl/mav_active_3d_planning/wiki/Code-Index)

For additional information please see the [wiki](https://github.com/ethz-asl/mav_active_3d_planning/wiki).

# Credits
## Paper and Video
If you find this package useful for your research, please consider citing our paper:





* Lukas Schmid, Michael Pantic, Raghav Khanna, Lionel Ott, Roland Siegwart, and Juan Nieto, "**An Efficient Sampling-based Method for Online Informative Path Planning in Unknown Environments**", in *IEEE Robotics and Automation Letters*, vol. 5, no. 2, pp. 1500-1507, April 2020 [[IEEE](https://ieeexplore.ieee.org/abstract/document/8968434) | [ArXiv](https://arxiv.org/abs/1909.09548) | [Video](https://www.youtube.com/watch?v=lEadqJ1_8Do)]
  ```bibtex
  @ARTICLE{Schmid20ActivePlanning,
    author={L. {Schmid} and M. {Pantic} and R. {Khanna} and L. {Ott} and R. {Siegwart} and J. {Nieto}},
    journal={IEEE Robotics and Automation Letters},
    title={An Efficient Sampling-Based Method for Online Informative Path Planning in Unknown Environments},
    year={2020},
    volume={5},
    number={2},
    pages={1500-1507},
    keywords={Motion and path planning;aerial systems;perception and autonomy;reactive and sensor-based planning},
    doi={10.1109/LRA.2020.2969191},
    ISSN={2377-3774},
    month={April},
  }
  ```
   ```
  @Article{s22218429,
  AUTHOR = {Yu, Tianyou and Deng, Baosong and Gui, Jianjun and Zhu, Xiaozhou and Yao, Wen},
  TITLE = {Efficient Informative Path Planning via Normalized Utility in Unknown Environments Exploration},
  JOURNAL = {Sensors},
  VOLUME = {22},
  YEAR = {2022},
  NUMBER = {21},
  ARTICLE-NUMBER = {8429},
  URL = {https://www.mdpi.com/1424-8220/22/21/8429},
  PubMedID = {36366127},
  ISSN = {1424-8220},
  ABSTRACT = {Exploration is an important aspect of autonomous robotics, whether it is for target searching, rescue missions, or reconnaissance in an unknown environment. In this paper, we propose a solution to efficiently explore the unknown environment by unmanned aerial vehicles (UAV). Innovatively, a topological road map is incrementally built based on Rapidly-exploring Random Tree (RRT) and maintained along with the whole exploration process. The topological structure can provide a set of waypoints for searching an optimal informative path. To evaluate the path, we consider the information measurement based on prior map uncertainty and the distance cost of the path, and formulate a normalized utility to describe information-richness along the path. The informative path is determined in every period by a local planner, and the robot executes the planned path to collect measurements of the unknown environment and restructure a map. The proposed framework and its composed modules are verified in two 3-D environments, which exhibit better performance in improving the exploration efficiency than other methods.},
  DOI = {10.3390/s22218429}
  }
  ```
  
# Setup
## Packages
The mav_active_3d_planning package is divided into separate packages, such that only the dependencies necessary for your application package need to be built.

Although packages are organized for the catkin workflow, the *core* package can be built as a stand-alone library for non-ROS use. All packages with a short description are listed below.

## Dependencies
Packages and their dependencies:
* **core:**
 
   Central logic of active\_3d\_planners. Dependencies:
    * `catkin_simple` ([https://github.com/catkin/catkin_simple](https://github.com/catkin/catkin_simple))
    * `glog_catkin` ([https://github.com/ethz-asl/glog_catkin](https://github.com/ethz-asl/glog_catkin))
    * `eigen_catkin` ([https://github.com/ethz-asl/eigen_catkin](https://github.com/ethz-asl/eigen_catkin))
    
* **ros:** 

   Interface to ROS for the general active\_3d\_planner and ROS specific modules.

* **mav:** 

   Modules and interfaces specific to Micro Aerial Vehicles (MAV), using ROS. Dependencies:
    * `mav_trajectory_generation` ([https://github.com/ethz-asl/mav_trajectory_generation](https://github.com/ethz-asl/mav_trajectory_generation))

* **voxblox:**

   Using voxblox as map representation and modules specific to voxblox. Dependencies:
    * `voxblox` ([https://github.com/ethz-asl/voxblox](https://github.com/ethz-asl/voxblox))

* **app_reconstruction:**

   Application package for autonomous 3D reconstruction with MAVs, including automated simulation and evaluation routines. 
   In order to enable simulations uncomment the dependencies in these [lines](https://github.com/ethz-asl/mav_active_3d_planning/blob/46143ae558c3d62fa2673a43cbfdd22fb6de12a9/mav_active_3d_planning/package.xml#L13-L17)
   Dependencies:
    * `unreal_cv_ros` ([https://github.com/ethz-asl/unreal_cv_ros](https://github.com/ethz-asl/unreal_cv_ros))
    * `rotors_simulator` ([https://github.com/ethz-asl/rotors_simulator](https://github.com/ethz-asl/rotors_simulator))
    * `mav_control_rw` ([https://github.com/ethz-asl/mav_control_rw](https://github.com/ethz-asl/mav_control_rw))

## Installation
Installation instructions for Linux.

**Prerequisites**

1. If not already done so, install [ROS](http://wiki.ros.org/ROS/Installation) (Desktop-Full is recommended).

2. If not already done so, create a catkin workspace with [catkin tools](https://catkin-tools.readthedocs.io/en/latest/):

```shell script
sudo apt-get install python-catkin-tools
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
catkin config --extend /opt/ros/melodic  # exchange melodic for your ros distro if necessary
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin config --merge-devel
```

**Installation**

1. Move to your catkin workspace: 
```shell script
cd ~/catkin_ws/src
```

2. Install system dependencies: 
```shell script
sudo apt-get install python-wstool python-catkin-tools
```

3. Download repo using a SSH key or via HTTPS: 
```shell script
git clone git@github.com:9woods123/mav_unknown_exploration.git # SSH
git clone https://github.com/9woods123/mav_unknown_exploration.git # HTTPS
```

4. Download and install the dependencies of the packages you intend to use.

   * **Full Install:** dependencies of **all** packages can be installed using rosinstall:
   ```shell script
   # system dependencies, replace melodic with your ros distro if necessary:
   sudo apt-get install ros-melodic-cmake-modules ros-melodic-control-toolbox ros-melodic-joy ros-melodic-octomap-ros ros-melodic-mavlink ros-melodic-geographic-msgs autoconf libyaml-cpp-dev protobuf-compiler libgoogle-glog-dev liblapacke-dev libgeographic-dev
   pip install future unrealcv

   # If you already intialized ws tool use 'wstool merge -t'
   wstool init . ./mav_active_3d_planning/mav_active_3d_planning_ssh.rosinstall # SSH
   wstool init . ./mav_active_3d_planning/mav_active_3d_planning_https.rosinstall # HTTPS
   wstool update
   ```
   * **Partial Install:** Install dependencies of the packages you intend to use ([listed above](#Dependencies)) and remove unwanted packages from `mav_active_3d_planning/package.xml` as well as their source folders.

5. Source and compile: 
```shell script
source ../devel/setup.bash
catkin build mav_active_3d_planning # Builds this package only
catkin build # Builds entire workspace, recommended for full install.
```

## Run an Experiment
In order to record data of the example planner, run 
```
roslaunch active_3d_planning_app_reconstruction example_gazebo.launch
```
and run 
```
rosservice call /planner/planner_node/toggle_running 1
```
after mav turning to hovering to start the exploration.
