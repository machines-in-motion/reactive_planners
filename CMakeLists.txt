###################
# Standard header #
###################
cmake_minimum_required(VERSION 3.5.1) # ubuntu 16.04 minimum
project(reactive_planners)

# required to use std::shared_ptr in code and to link the python bindings
if(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--no-as-needed")
endif()
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

# Use C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED on)

set(CMAKE_SKIP_BUILD_RPATH  FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)

################
# Dependencies #
################

find_package(catkin REQUIRED COMPONENTS
    yaml_cpp_catkin
    mpi_cmake_modules
    pybind11_catkin
)

search_for_boost()
search_for_eigen()
set(catkin_INCLUDE_DIRS ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

# Depend on eigen-quadprog
catkin_add_required_dependency("eigen-quadprog >= 1.0.0")
set(catkin_LIBRARIES ${catkin_LIBRARIES} eigen-quadprog)
catkin_add_required_dependency("pinocchio >= 2.0.0")

################
# Include dirs #
################
include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS})

##############################
# Declare the catkin package #
##############################

# this package exports its include folder and its libraries for other catkin packages to enjoy
catkin_package(
 LIBRARIES ${PROJECT_NAME} # exporting the ci_example library (libs and executable built in other catkin package will be able to link against them)
 INCLUDE_DIRS include # exporting the include file: source file hosted by other catkin package will be able to include the related header files)
 CATKIN_DEPENDS
)

##############################
# Declare the python package #
##############################

catkin_python_setup()

#######################
# Declare the library #
#######################

add_library( ${PROJECT_NAME} SHARED
  src/stepper_head.cpp
  src/dcm_vrp_planner.cpp
  src/com_planner.cpp
  src/end_effector_trajectory_3d.cpp
  src/new_end_effector_trajectory_3d.cpp
  src/dcm_reactive_stepper.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


#####################################################################
# Stores the path to the config folder in the CONFIG_PATH variable. #
#####################################################################
get_filename_component(CONFIG_PATH config ABSOLUTE)


###############
#  unit tests #
###############
message(STATUS "################# ${catkin_LIBRARIES}")
macro(add_unit_test src_file)
    catkin_add_gtest(${PROJECT_NAME}_${src_file}
        tests/main.cpp tests/${src_file}.cpp)
    target_link_libraries(${PROJECT_NAME}_${src_file} ${PROJECT_NAME})
    target_link_libraries(${PROJECT_NAME}_${src_file} ${catkin_LIBRARIES})
    set_target_properties(${PROJECT_NAME}_${src_file} PROPERTIES
        COMPILE_DEFINITIONS CONFIG_FOLDER_PATH="${CONFIG_PATH}")
endmacro()

add_unit_test(dcm_vrp_planner_ut)


####################
# Demo executables #
####################

add_executable(demo_end_effector_trajectory_3d demos/demo_end_effector_trajectory_3d.cpp)
target_link_libraries(demo_end_effector_trajectory_3d ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(demo_stepper_head demos/demo_stepper_head.cpp)
target_link_libraries(demo_stepper_head ${PROJECT_NAME} ${catkin_LIBRARIES})


##########################
# building documentation #
##########################
build_doxygen_documentation()

################################
# python bindings with pybind11#
################################
pybind11_add_module(py_${PROJECT_NAME}
    srcpy/reactive_planners
    srcpy/stepper_head.cpp
    srcpy/dcm_vrp_planner.cpp
    srcpy/end_effector_trajectory_3d.cpp
    srcpy/dcm_reactive_stepper.cpp
)
target_link_libraries(
    py_${PROJECT_NAME} PRIVATE ${PROJECT_NAME} ${catkin_LIBRARIES})
set_target_properties(py_${PROJECT_NAME} PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}
  OUTPUT_NAME ${PROJECT_NAME}
)
install(TARGETS py_${PROJECT_NAME} DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})
