reactive_planners {#mainpage}
=================

## What is it

Contains a list of reactive planners specialized in locomotion of legged robots.

## Authors

- Avadesh Meduri
- Julian Viereck
- Majid Kadhiv
- Maximilien Naveau
- Elham Daneshmand

## Copyrights

Copyright (c) 2020, New York University and Max Planck Gesellschaft.

## Dependencies

### Third party

- gurobi: https://www.gurobi.com/, see installation guide
[here](https://www.gurobi.com/documentation/9.0/quickstart_linux/software_installation_guid.html)
- urdf_parser_py: `pip install urdf-parser-py`

### Treep dependencies

In your workspace (e.g. `~/devel`) please execute the following instructions:

```bash
git clone git@github.com:machines-in-motion/treep_machines_in_motion.git
treep --clone yaml_cpp_catkin
treep --clone mpi_cmake_modules
treep --clone pybind11_catkin
```

## Reference

This package contains the implementation of the algorithms depicted in:

- Khadiv, Majid, Alexander Herzog, S. Ali A. Moosavian and Ludovic Righetti.
  “A robust walking controller based on online step location and duration
  optimization for bipedal locomotion.”, https://arxiv.org/abs/1704.01271 (2017)
