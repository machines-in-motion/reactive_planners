###################
# Standard header #
###################
cmake_minimum_required(VERSION 3.10.2) # ubuntu 18.04 minimum
project(reactive_planners)

# Add support for C++11
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(ament_cmake REQUIRED)
find_package(mpi_cmake_modules REQUIRED)
find_package(yaml_utils REQUIRED)
find_package(pybind11 REQUIRED)
find_package(eigen-quadprog REQUIRED)
find_package(pinocchio REQUIRED)
# C++ unit-tests.
find_package(ament_cmake_gtest)

set(INCLUDE_DIRS include ${ament_cmake_INCLUDE_DIRS}
  ${mpi_cmake_modules_INCLUDE_DIRS} ${yaml_utils_INCLUDE_DIRS}
  ${pybind11_INCLUDE_DIRS}  ${eigen-quadprog_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${ament_cmake_LIBRARY_DIRS} ${mpi_cmake_modules_LIBRARY_DIRS}
  ${yaml_utils_LIBRARY_DIRS} ${pybind11_LIBRARY_DIRS} ${eigen-quadprog_INCLUDE_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${ament_cmake_LIBRARIES} ${mpi_cmake_modules_LIBRARIES}
  ${yaml_utils_LIBRARIES} ${pybind11_LIBRARIES} ${eigen-quadprog_LIBRARIES}  ${pinocchio_LIBRARIES})
#set(LIBSB ${eigen-quadprog_LIBRARIES})

set(my_src_SRC_FILES
        SHARED src/stepper_head.cpp
        src/dcm_vrp_planner.cpp
        src/end_effector_trajectory_3d.cpp
        src/new_end_effector_trajectory_3d.cpp
        src/dcm_reactive_stepper.cpp)

add_library(my_src SHARED ${my_src_SRC_FILES})
list(APPEND all_targets my_src)

target_link_libraries(my_src eigen-quadprog::eigen-quadprog)
target_link_libraries(my_src Eigen3::Eigen)
target_include_directories(
        my_src PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
#target_link_libraries(${my_src} ${LIBSB})
# required to use std::shared_ptr in code and to link the python bindings
if(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--no-as-needed")
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

# Use C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED on)

set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)

################
# Dependencies #
################

#search_for_boost()
#search_for_eigen()
set(catkin_INCLUDE_DIRS ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

# Depend on eigen-quadprog
#ament_export_dependencies(eigen-quadprog)
##set(catkin_LIBRARIES ${catkin_LIBRARIES} eigen-quadprog::eigen_quadprog)
#ament_export_dependencies(pinocchio)

#add_library(${PROJECT_NAME}
#  SHARED src/stepper_head.cpp src/dcm_vrp_planner.cpp
#  src/end_effector_trajectory_3d.cpp src/new_end_effector_trajectory_3d.cpp
#  src/dcm_reactive_stepper.cpp)
#target_link_libraries(${PROJECT_NAME} ${LIBS})

#####################################################################
# Stores the path to the config folder in the CONFIG_PATH variable. #
#####################################################################
get_filename_component(CONFIG_PATH config ABSOLUTE)

###############
#  unit tests #
###############
#message(STATUS "################# ${catkin_LIBRARIES}")
macro(add_unit_test src_file)
  ament_add_gtest(${PROJECT_NAME}_${src_file} tests/main.cpp tests/${src_file}.cpp)
  target_link_libraries(${PROJECT_NAME}_${src_file} ${PROJECT_NAME})
  target_link_libraries(${PROJECT_NAME}_${src_file} ${LIBS})
  set_target_properties(${PROJECT_NAME}_${src_file}
    PROPERTIES
    COMPILE_DEFINITIONS CONFIG_FOLDER_PATH="${CONFIG_PATH}")
endmacro()

#add_unit_test(dcm_vrp_planner_ut)

####################
# Demo executables #
####################

#add_executable(demo_end_effector_trajectory_3d
#  demos/demo_end_effector_trajectory_3d.cpp)
#target_link_libraries(demo_end_effector_trajectory_3d ${PROJECT_NAME} ${LIBS})
#
#add_executable(demo_stepper_head demos/demo_stepper_head.cpp)
#target_link_libraries(demo_stepper_head ${PROJECT_NAME} ${LIBS})

##########################
# building documentation #
##########################
add_documentation()

################################
# python bindings with pybind11#
################################
pybind11_add_module(py_${PROJECT_NAME} srcpy/reactive_planners
  srcpy/stepper_head.cpp srcpy/dcm_vrp_planner.cpp
  srcpy/end_effector_trajectory_3d.cpp srcpy/dcm_reactive_stepper.cpp)
target_link_libraries(py_${PROJECT_NAME} PRIVATE ${PROJECT_NAME} ${LIBS})

# then we build the different entities in a dynamic-graph plugin.
#set(cpp_source_files
#        src/dcm_reactive_stepper.cpp
#        src/dcm_vrp_planner.cpp
#        src/end_effector_trajectory.cpp
#        src/new_end_effector_trajectory_3d.cpp
#        src/stepper_head.cpp
#        )

#add_library(${lib_name} SHARED ${cpp_source_files})
# Add the include dependencies.

#target_link_libraries(${PROJECT_NAME} eigen-quadprog::eigen-quadprog)
#target_link_libraries(${PROJECT_NAME} pinocchio::pinocchio)
set_target_properties(py_${PROJECT_NAME}
  PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY
   ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}
  OUTPUT_NAME ${PROJECT_NAME})
install(TARGETS py_${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})

ament_export_dependencies(ament_cmake)
ament_export_dependencies(mpi_cmake_modules)
ament_export_dependencies(yaml_utils)
ament_export_dependencies(pybind11)
ament_export_include_directories(${INCLUDE_DIRS})

#add_subdirectory(src)

ament_package()
