Readme
------

### What is it

Contains a list of reactive planners specialized in locomotion of legged robots.

### Authors

- Avadesh Meduri
- Julian Viereck
- Majid Kadhiv
- Maximilien Naveau
- Elham Daneshmand

### Copyrights

Copyright (c) 2020, New York University and Max Planck Gesellschaft.

### Getting started

Install
[treep](https://gitlab.is.tue.mpg.de/amd-clmc/treep)
and
[colcon](https://github.com/machines-in-motion/machines-in-motion.github.io/wiki/use_colcon)
.

Then follow the instructions below:
```bash
# install treep and colcon
pip install -U treep colcon-common-extensions
# change directory to your devel folder
mkdir ~/devel
cd ~/devel
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

### Reference

This package contains the implementation of the algorithms depicted in:

- Khadiv, Majid, Alexander Herzog, S. Ali A. Moosavian and Ludovic Righetti.
  “A robust walking controller based on online step location and duration
  optimization for bipedal locomotion.”, https://arxiv.org/abs/1704.01271 (2017)
