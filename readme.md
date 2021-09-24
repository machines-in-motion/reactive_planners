[![CodeFactor](https://www.codefactor.io/repository/github/machines-in-motion/reactive_planners/badge/master?s=2265cf35a56607421790341030c3b894f59b1c28)](https://www.codefactor.io/repository/github/machines-in-motion/reactive_planners/overview/master)

Readme
------

Contains a list of reactive planners specialized in locomotion of legged robots. The reactive planner adapts the step location and timing of the gait based on feedbck from the CoM states and sends the desired swing foot trajectories to an instantanous controller for tracking.

### Installation

#### Standard dependencies

*Here all the pip and apt install-able stuff*

#### Download the package

Install
[treep](https://gitlab.is.tue.mpg.de/amd-clmc/treep)
and
[colcon](https://github.com/machines-in-motion/machines-in-motion.github.io/wiki/use_colcon)
.


#### Build the package


Then follow the instructions below:
```bash
# install treep and colcon
pip install -U treep colcon-common-extensions
# change directory to your devel folder
mkdir devel
cd devel
# Clone the treep configuration
git clone https://github.com/machines-in-motion/treep_machines_in_motion.git
# Clone the code base
treep --clone REACTIVE_PLANNERS
# go and build the code
cd workspace
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
# source the environment for using the code
source install/setup.bash
```

### Usage

#### Demos/Examples

To run Bolt walking in simulation:
 `python3 src/reactive_planners/demos/demo_reactive_planners_bolt_step_adjustment.py`

To run Solo12 walking in simulation:
 `python3 src/reactive_planners/demos/demo_dgh_sim_solo12_step_adjustment.py`

### Reference

This package contains the implementation of the algorithms depicted in:

- Elham Daneshmand, Majid Khadiv , Felix Grimminger and Ludovic Righetti.
  “Variable Horizon MPC With Swing Foot Dynamicsfor Bipedal Walking Control.”,
  IEEE Robotics and Automation Letters, 6(2).
  https://arxiv.org/abs/2010.08198 (2021)

- Majid Khadiv, Alexander Herzog, S. Ali A. Moosavian and Ludovic Righetti.
  “Walking Control Based on Step Timing Adaptation.”,
  IEEE Transactions on Robotics, 36(3).
  https://arxiv.org/abs/1704.01271 (2020)

### License and Copyrights

License BSD-3-Clause
Copyright (c) 2020, New York University and Max Planck Gesellschaft.
