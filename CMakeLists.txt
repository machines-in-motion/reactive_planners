#
# Copyright (c) 2019, New York University and Max Planck Gesellschaft.
#
# License BSD-3 clause
#

#
# set up the project
#
cmake_minimum_required(VERSION 3.10.2)

project(reactive_planners)

# specify the C++ 17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

#
# Dependencies.
#

# Build.
find_package(mpi_cmake_modules REQUIRED)

# Internal.
find_package(yaml_utils REQUIRED)

# External.
find_package(eigen-quadprog REQUIRED)
find_package(pinocchio REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Eigen3 REQUIRED)

#
# Main library
#

# Source files.
set(main_lib_src
    src/stepper_head.cpp
    src/dcm_vrp_planner.cpp
    src/polynomial_end_effector_trajectory.cpp
    src/dynamically_consistent_end_effector_trajectory.cpp
    src/dcm_reactive_stepper.cpp)
# Create library target.
add_library(${PROJECT_NAME} SHARED ${main_lib_src})
# Includes.
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)
# Dependencies.
target_link_libraries(${PROJECT_NAME} yaml_utils::yaml_utils)
target_link_libraries(${PROJECT_NAME} pinocchio::pinocchio)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
target_link_libraries(${PROJECT_NAME} eigen-quadprog::eigen-quadprog)
# Export.
list(APPEND all_targets ${PROJECT_NAME})

#
# Stores the path to the config folder in the CONFIG_PATH variable.
#
get_filename_component(CONFIG_PATH config ABSOLUTE)

#
# Demo executables.
#

install_scripts(
  demos/demo_reactive_planners_bolt_step_adjustment.py
  demos/demo_reactive_planners_dcm_reactive_stepper.py
  demos/demo_reactive_planners_end_effector_trajectory3d.py
  demos/demo_reactive_planners_stepper_head.py
  demos/demo_reactive_planners_walking.py
  demos/demo_reactive_planners_re_split_dcm.py
  demos/demo_reactive_planners_split_dcm_uneven_terrain.py
  demos/demo_reactive_planners_step_adaptation_timing_planner.py
  demos/demo_reactive_planners_step_adjustment.py
  DESTINATION
  bin)

#
# Python bindings with pybind11.
#
set(py_lib_src
    srcpy/reactive_planners srcpy/stepper_head.cpp srcpy/dcm_vrp_planner.cpp
    srcpy/polynomial_end_effector_trajectory.cpp srcpy/dcm_reactive_stepper.cpp)
pybind11_add_module(${PROJECT_NAME}_python_binding MODULE ${py_lib_src})
target_include_directories(
  ${PROJECT_NAME}_python_binding
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_python_binding PRIVATE pybind11::module)
target_link_libraries(${PROJECT_NAME}_python_binding PRIVATE ${PROJECT_NAME})
get_python_install_dir(python_install_dir)
install(TARGETS ${PROJECT_NAME}_python_binding
        DESTINATION ${python_install_dir})
set_target_properties(${PROJECT_NAME}_python_binding PROPERTIES OUTPUT_NAME
                                                                ${PROJECT_NAME})

#
# Unit tests
#
include(CTest)
if(BUILD_TESTING)
  find_package(GTest CONFIG REQUIRED)
  include(GoogleTest)

  macro(ADD_UNIT_TEST src_file)
    set(test_name ${PROJECT_NAME}_${src_file})
    add_executable(${test_name} tests/main.cpp tests/${src_file}.cpp)
    target_link_libraries(${test_name} ${PROJECT_NAME})
    target_link_libraries(${test_name} GTest::gtest)
    set_target_properties(
      ${test_name} PROPERTIES COMPILE_DEFINITIONS
                              CONFIG_FOLDER_PATH="${CONFIG_PATH}")
    gtest_add_tests(TARGET ${test_name})
  endmacro()

  # add_unit_test(dcm_vrp_planner_ut)
endif()

#
# Install all and export
#

# Python files.
get_python_install_dir(python_install_dir)
install(
  DIRECTORY python/
  DESTINATION "${python_install_dir}"
  PATTERN "*.pyc" EXCLUDE
  PATTERN "__pycache__" EXCLUDE)

# Command to install the library and binaries.
install(
  TARGETS ${all_targets}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)
# Put the cmake files at the right place.
generate_cmake_package(SKIP_TARGET_EXPORT)

#
# building documentation
#
add_documentation()
