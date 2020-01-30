/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Demo of the EndEffectorTrajectory3D class
 */

#include "reactive_planners/end_effector_trajectory_3d.hpp"
#include <Eigen/Eigen>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

bool run(std::ofstream &myfile, double duration) {
  // Random start and stop.
  Eigen::Vector3d start_pose = Eigen::Vector3d::Random();
  start_pose(2) = 0.0;
  //   start_pose << 1.0, 1.0, 0.5;
  Eigen::Vector3d target_pose = Eigen::Vector3d::Random();
  target_pose(2) = 0.0;
  //   target_pose << 5.0, -2.0, 0.5;
  double mid_air_height = 0.05;

  // Timing
  double start_time = 0.1;
  double end_time = start_time + duration;
  double fluct_end_time = 0.5 * duration;

  double traj_time = start_time + (end_time - start_time) + 1.0;
  double dt = 0.001;
  int N = std::round(traj_time / dt);

  // variable data
  Eigen::Vector3d current_pose = start_pose;
  Eigen::Vector3d current_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d current_acceleration = Eigen::Vector3d::Zero();
  double current_time = 0.0;
  double end_time_fluct = 0.0;

  // Create the trajectory computor.
  reactive_planners::EndEffectorTrajectory3D traj;
  traj.set_mid_air_height(mid_air_height);

  // Compute the trajectory and log.
  myfile << current_time << "\t" << current_pose.transpose() << "\t"
         << current_velocity.transpose() << "\t"
         << current_acceleration.transpose() << std::endl;
  for (int i = 0; i < N + 1; ++i) {
    end_time_fluct = end_time + 2 * (double)(rand() - RAND_MAX / 2) /
                                    (double)RAND_MAX * fluct_end_time;
    if (!traj.compute(start_pose, current_pose, current_velocity,
                      current_acceleration, target_pose, start_time,
                      current_time, end_time_fluct)) {
      std::cout << "failure at time : " << current_time << "/" << end_time_fluct
                << " ; " << std::round((end_time_fluct - current_time) / dt)
                << std::endl;
      return false;
    }
    traj.get_next_state(current_time + dt, current_pose, current_velocity,
                        current_acceleration);
    current_time += dt;

    myfile << current_time << "\t" << current_pose.transpose() << "\t"
           << current_velocity.transpose() << "\t"
           << current_acceleration.transpose() << std::endl;
  }
  return true;
}

int main(int, char **) {
  std::ofstream myfile;
  myfile.open("/tmp/demo_end_effector_trajectory_3d.dat");
  unsigned N = 15;
  for (unsigned i = 0; i < N; ++i) {
    double duration = 0.1 * (i + 1);
    std::cout << "Running test: " << i << " ; duration: " << duration
              << std::endl;

    for (unsigned j = 0; j < 30; ++j) {
      if (!run(myfile, duration)) {
        std::cout << "Running test: " << j << " ; duration: " << duration
                  << std::endl;
        std::cout << "Fail to compute the QP!" << std::endl;
      }
      for (unsigned j = 0; j < 1000; ++j) {
        Eigen::VectorXd nan_vec = Eigen::VectorXd::Zero(10);
        nan_vec.fill(std::nan(""));
        myfile << nan_vec.transpose() << std::endl;
      }
    }
  }
  myfile.close();
  return 0;
}