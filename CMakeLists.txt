###################
# Standard header #
###################
cmake_minimum_required(VERSION 3.5.1) # ubuntu 16.04 minimum
project(ci_example_cpp) 

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

################
# Include dirs #
################
include_directories( include ${catkin_INCLUDE_DIRS}  )

##############################
# Declare the catkin package #
##############################

# this package exports its include folder and its libraries for other catkin packages to enjoy
catkin_package(
 LIBRARIES ci_example # exporting the ci_example library (libs and executable built in other catkin package will be able to link against them)
 INCLUDE_DIRS include # exporting the include file: source file hosted by other catkin package will be able to include the related header files)
 CATKIN_DEPENDS roscpp std_msgs 
)

##############################
# Declare the python package #
##############################

catkin_python_setup()

#######################
# Declare the library #
#######################

# add_library( ${PROJECT_NAME} SHARED
#   src/
# )
# target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} )

#####################################################################
# Stores the path to the config folder in the CONFIG_PATH variable. #
#####################################################################
get_filename_component(CONFIG_PATH config ABSOLUTE)


###############
#  unit tests #
###############

catkin_add_gtest(${PROJECT_NAME}_ut
  tests/main.cpp
  tests/${PROJECT_NAME}.cpp
)
target_link_libraries(${PROJECT_NAME}_ut ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME}_ut PROPERTIES 
  COMPILE_DEFINITIONS CONFIG_FOLDER_PATH="${CONFIG_PATH}")

####################
# Demo executables #
####################

# add_executable(demo_${PROJECT_NAME} demos/demo_${PROJECT_NAME}.cpp)
# target_link_libraries(demo_pid ${PROJECT_NAME})

##########################
# building documentation #
##########################
build_doxygen_documentation()

################################
# python bindings with pybind11#
################################
# pybind11_add_module(basic_pid srcpy/python_bindings.cpp)
# target_link_libraries(basic_pid PRIVATE ci_example ${catkin_LIBRARIES})
# pkg_config_use_dependency(basic_pid yaml-cpp)
# set_target_properties(basic_pid PROPERTIES
#   LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION})
# install(TARGETS ci_example DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})

# this will create a module of name "basic_pid". After calling catkin_make
# and sourcing <workspace>/devel/setup.bash, the following should work in python:

# import basic_pid
# p = basic_pid.PID()
# p.compute(1,1,1,1)
# p.reset_integral()
