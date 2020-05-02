/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Implement the reactive_planners::NewEndEffectorTrajectory3D class
 */

#include "reactive_planners/new_end_effector_trajectory_3d.hpp"
#include <iostream>

namespace reactive_planners {

NewEndEffectorTrajectory3D::NewEndEffectorTrajectory3D() {
  // Constant problem parameter.
  mid_air_height_ = 0.05;
//  nb_var_x_ = 6;
//  nb_var_y_ = 6;
//  nb_var_z_ = 9;
//  cost_x_ = 1e1;
//  cost_y_ = 1e1;
//  cost_z_ = 1e0;
//  double hess_regul = 1e-9;
    cost_ = 1;
    cost_epsilon_ = 10000000;

  // Variable parameters.
  // clang-format off
//  time_vec_x_.resize(nb_var_x_); time_vec_x_.setZero();
//  time_vec_y_.resize(nb_var_y_); time_vec_y_.setZero();
//  time_vec_z_.resize(nb_var_z_); time_vec_z_.setZero();
  // clang-format on
  start_pose_.setZero();
  current_pose_.setZero();
  current_velocity_.setZero();
  current_acceleration_.setZero();
  target_pose_.setZero();
  start_time_ = 0.0;
  current_time_ = 0.0;
  end_time_ = 0.0;
  previous_solution_pose_.setZero();
  sampling_time = 0.001;

  // QP parameter.
  nb_sampling_time = 10;
  nb_var_ = 3 * nb_sampling_time + 1;
  nb_var_axis_ = nb_sampling_time;
  // number of sampling time nodes for the z_min < z(t) < z_max.
  nb_ineq_ = nb_sampling_time * 2;
  // current and final conditions
  nb_eq_ = 3 * 2 + 2;//xn=0,vn=0,an[2]=0,xn/2[2]=xd
  resize_matrices();

  x_opt_.resize(nb_var_);
  x_opt_.setZero();

  //TODO
  M_inv_.resize(3, 3);
  M_inv_ << 0.06, 0.0, 0.03,
           0.0, 0.05, 0.01,
           0.03, 0.01, 0.05;
  M_inv_ = M_inv_.inverse();

  A_.resize(6, 6);
  A_ << 1., sampling_time, 0., 0., 0., 0.,
        0., 1., 0., 0., 0., 0.,
        0., 0., 1., sampling_time, 0., 0.,
        0., 0., 0., 1., 0., 0.,
        0., 0., 0., 0., 1., sampling_time,
        0., 0., 0., 0., 0., 1.;

  B_.resize(6, 3);
  B_ << sampling_time * sampling_time / 2, 0., 0.,
        sampling_time, 0., 0.,
        0., sampling_time * sampling_time / 2, 0.,
        0., sampling_time, 0.,
        0., 0., sampling_time * sampling_time / 2,
        0., 0., sampling_time;

//  Q_regul_ = Eigen::MatrixXd::Identity(nb_var_, nb_var_) * hess_regul;
}

NewEndEffectorTrajectory3D::~NewEndEffectorTrajectory3D() {}

bool NewEndEffectorTrajectory3D::compute(
    Eigen::Ref<const Eigen::Vector3d> start_pose,
    Eigen::Ref<const Eigen::Vector3d> current_pose,
    Eigen::Ref<const Eigen::Vector3d> current_velocity,
    Eigen::Ref<const Eigen::Vector3d> current_acceleration,
    Eigen::Ref<const Eigen::Vector3d> target_pose, const double &start_time,
    const double &current_time, const double &end_time) {
  // scaling the problem
  std::cout << "Lhum cur end" << current_pose << " # " << target_pose << std::endl;
  std::cout << "Lhum vel" << current_velocity << std::endl;
  double step_duration = end_time - start_time;
  double duration = end_time - current_time;
  nb_sampling_time = ceil(step_duration / sampling_time);
  double nb_local_sampling_time = ceil(duration / sampling_time);
  int nb_mid_sampling_time = nb_local_sampling_time - (nb_sampling_time / 2);
  nb_var_ = 3 * nb_local_sampling_time + 1;
  nb_var_axis_ = nb_local_sampling_time;
  nb_ineq_ = nb_local_sampling_time * 2;
  std::cout << "Lhum step_duration " << step_duration <<
               "\nLhum duration " << duration <<
               "\nLhum local_sampling_list " <<  sampling_time <<
               "\nLhum nb_sampling_time " << nb_sampling_time <<
               "\nLhum nb_local_sampling_time " << nb_local_sampling_time <<
               "\nLhum nb_mid_sampling_time " <<  nb_mid_sampling_time << std::endl;
  // save args:
  start_pose_ = start_pose;
  current_pose_ = current_pose;
  current_velocity_ = current_velocity;
//  current_acceleration_ = current_acceleration;
  target_pose_ = target_pose;
  start_time_ = start_time;
  current_time_ = current_time;
  end_time_ = end_time;

  // Do not compute the QP if the solution is trivial or too close to the end of
  // the trajectory.
//  if (current_time_ < start_time_ || (current_time - start_time) / step_duration >= 0.7) {
//    return true;
//  } else {
    last_end_time_seen_ = end_time;
//  }
  resize_matrices();

  /*
   * Quadratic cost
   */
  // Q_x
  Eigen::MatrixXd sub_Q;
  sub_Q.resize(nb_var_axis_, nb_var_axis_);
  sub_Q.setZero();
//  for(int i = 0 ; i < nb_sampling_time - 1 ; i++){
//    sub_Q(i + 1, i) = -1  * cost_;
//    sub_Q(i, i + 1) = -1  * cost_;
//  }
  for(int i = 0 ; i < nb_var_axis_ ; i++)
    sub_Q(i, i) = 1 * cost_;
  Q_.block(0, 0, nb_var_axis_, nb_var_axis_) = sub_Q;
  // Q_y
  Q_.block(nb_var_axis_, nb_var_axis_, nb_var_axis_, nb_var_axis_) = sub_Q;
  // Q_z
  Q_.block(nb_var_axis_ * 2, nb_var_axis_ * 2, nb_var_axis_, nb_var_axis_) = sub_Q;
  // Q_epsilon
  Q_(nb_var_axis_ * 3, nb_var_axis_ * 3) = 1 * cost_epsilon_;
  // Q_regul
//  Q_ += Q_regul_;

  /*
   * Equality constraints.
   */
  // clang-format off
  A_eq_.setZero();
  Eigen::VectorXd acc;
  Eigen::VectorXd acc_ep;
  acc.resize(nb_var_ - 1);
  acc.setZero();
  acc_ep.resize(nb_var_);
  acc_ep.setZero();
  // X end constraints
  std::cout << "Lhum end current start " << end_time_ << " " << current_time_ << " " << start_time_ << std::endl;
  std::cout << "Lhumlocal_ST " << nb_local_sampling_time << std::endl;
  std::cout << "mid " << nb_mid_sampling_time << std::endl;
  for(int i = 0; i < nb_var_ - 1; i++)
    acc[i] = 0;
  Eigen::MatrixXd x;
  x.resize(6, 3);
  x = B_;
  for(int i = nb_local_sampling_time - 1 ; i >= 0; i--){
    acc[i] = x(0, 0) * M_inv_(0, 0) + x(0, 1) * M_inv_(1, 0) + x(0, 2) * M_inv_(2, 0);
    acc[i + nb_var_axis_] = x(0, 0) * M_inv_(0, 1) + x(0, 1) * M_inv_(1, 1) + x(0, 2) * M_inv_(2, 1);
    acc[i + 2 * nb_var_axis_] = x(0, 0) * M_inv_(0, 2) + x(0, 1) * M_inv_(1, 2) + x(0, 2) * M_inv_(2, 2);
    x = A_ * x;
  }
  A_eq_.row(0).head(nb_var_  - 1) = acc;//xn[0]
  x = B_;
  for(int i = nb_local_sampling_time - 1 ; i >= 0; i--){
    acc[i] = x(2, 0) * M_inv_(0, 0) + x(2, 1) * M_inv_(1, 0) + x(2, 2) * M_inv_(2, 0);
    acc[i + nb_var_axis_] = x(2, 0) * M_inv_(0, 1) + x(2, 1) * M_inv_(1, 1) + x(2, 2) * M_inv_(2, 1);
    acc[i + 2 * nb_var_axis_] = x(2, 0) * M_inv_(0, 2) + x(2, 1) * M_inv_(1, 2) + x(2, 2) * M_inv_(2, 2);
    x = A_ * x;
  }
  A_eq_.row(1).head(nb_var_ - 1) = acc;//xn[1]
  x = B_;
  for(int i = nb_local_sampling_time - 1 ; i >= 0; i--){
    acc[i] = x(4, 0) * M_inv_(0, 0) + x(4, 1) * M_inv_(1, 0) + x(4, 2) * M_inv_(2, 0);
    acc[i + nb_var_axis_] = x(4, 0) * M_inv_(0, 1) + x(4, 1) * M_inv_(1, 1) + x(4, 2) * M_inv_(2, 1);
    acc[i + 2 * nb_var_axis_] = x(4, 0) * M_inv_(0, 2) + x(4, 1) * M_inv_(1, 2) + x(4, 2) * M_inv_(2, 2);
    x = A_ * x;
  }
  A_eq_.row(2).head(nb_var_ - 1) = acc;//xn[2]

  // V end constraints
  if(nb_local_sampling_time != 1){
      for(int i = 0; i < nb_var_ - 1; i++)
        acc[i] = 0;
      x = B_;
      for(int i = nb_local_sampling_time - 2 ; i >= 0; i--){
        acc[i] = x(0, 0) * M_inv_(0, 0) + x(0, 1) * M_inv_(1, 0) + x(0, 2) * M_inv_(2, 0);
        acc[i + nb_var_axis_] = x(0, 0) * M_inv_(0, 1) + x(0, 1) * M_inv_(1, 1) + x(0, 2) * M_inv_(2, 1);
        acc[i + 2 * nb_var_axis_] = x(0, 0) * M_inv_(0, 2) + x(0, 1) * M_inv_(1, 2) + x(0, 2) * M_inv_(2, 2);
        x = A_ * x;
      }
      A_eq_.row(3).head(nb_var_ - 1) = acc;//vn[0]
      x = B_;
      for(int i = nb_local_sampling_time - 2 ; i >= 0; i--){
        acc[i] = x(2, 0) * M_inv_(0, 0) + x(2, 1) * M_inv_(1, 0) + x(2, 2) * M_inv_(2, 0);
        acc[i + nb_var_axis_] = x(2, 0) * M_inv_(0, 1) + x(2, 1) * M_inv_(1, 1) + x(2, 2) * M_inv_(2, 1);
        acc[i + 2 * nb_var_axis_] = x(2, 0) * M_inv_(0, 2) + x(2, 1) * M_inv_(1, 2) + x(2, 2) * M_inv_(2, 2);
        x = A_ * x;
      }
      A_eq_.row(4).head(nb_var_ - 1) = acc;//vn[1]
      x = B_;
      for(int i = nb_local_sampling_time - 2 ; i >= 0; i--){
        acc[i] = x(4, 0) * M_inv_(0, 0) + x(4, 1) * M_inv_(1, 0) + x(4, 2) * M_inv_(2, 0);
        acc[i + nb_var_axis_] = x(4, 0) * M_inv_(0, 1) + x(4, 1) * M_inv_(1, 1) + x(4, 2) * M_inv_(2, 1);
        acc[i + 2 * nb_var_axis_] = x(4, 0) * M_inv_(0, 2) + x(4, 1) * M_inv_(1, 2) + x(4, 2) * M_inv_(2, 2);
        x = A_ * x;
      }
      A_eq_.row(5).head(nb_var_ - 1) = acc;//vn[2]
  }

  for(int i = 0; i < nb_var_ - 1; i++)
    acc[i] = 0;
  acc[nb_local_sampling_time - 1] = M_inv_(0, 2);
  acc[nb_local_sampling_time - 1 + nb_var_axis_] = M_inv_(1, 2);
  acc[nb_local_sampling_time - 1 + 2 * nb_var_axis_] = M_inv_(2, 2);
//  A_eq_.row(6).head(nb_var_ - 1) = acc;//an[2]

  for(int i = 0; i < nb_var_ - 1; i++)
    acc_ep[i] = 0;
  x = B_;
  for(int i = nb_mid_sampling_time - 1 ; i >= 0; i--){
    acc_ep[i] = x(4, 0) * M_inv_(0, 0) + x(4, 1) * M_inv_(1, 0) + x(4, 2) * M_inv_(2, 0);
    acc_ep[i + nb_var_axis_] = x(4, 0) * M_inv_(0, 1) + x(4, 1) * M_inv_(1, 1) + x(4, 2) * M_inv_(2, 1);
    acc_ep[i + 2 * nb_var_axis_] = x(4, 0) * M_inv_(0, 2) + x(4, 1) * M_inv_(1, 2) + x(4, 2) * M_inv_(2, 2);
    x = A_ * x;
  }
  acc_ep[nb_var_ - 1] = -1;//epsilon
  A_eq_.row(7).head(nb_var_) = acc_ep;//xn/2[2]

  int flag = nb_local_sampling_time != 1 ? 1: 0;
  double mid_z = nb_mid_sampling_time > 0 ? - current_pose_(2) + mid_air_height_ - current_velocity_(2) * sampling_time * nb_mid_sampling_time: 0;
  B_eq_.setZero();
  B_eq_ << - current_pose_(0) + target_pose_(0) - current_velocity_(0) * sampling_time * nb_local_sampling_time,
           - current_pose_(1) + target_pose_(1) - current_velocity_(1) * sampling_time * nb_local_sampling_time,
           - current_pose_(2) + target_pose_(2) - current_velocity_(2) * sampling_time * nb_local_sampling_time,
           flag * (- current_pose_(0) + target_pose_(0) - current_velocity_(0) * sampling_time * (nb_local_sampling_time - 1)),
           flag * (- current_pose_(1) + target_pose_(1) - current_velocity_(1) * sampling_time * (nb_local_sampling_time - 1)),
           flag * (- current_pose_(2) + target_pose_(2) - current_velocity_(2) * sampling_time * (nb_local_sampling_time - 1)),
           0.0,
           mid_z;
  // clang-format on

  /*
   * Inequality constraints
   */

  for(int i = 0; i < nb_var_ - 1; i++)
    acc[i] = 0;
  for (int i = 1; i <= nb_local_sampling_time; ++i) {
    // z >= zmin   =>   -z <= -z_min
      x = B_;
    for(int j = i - 1 ; j >= 0; j--){
      acc[j] = x(4, 0) * M_inv_(0, 0) + x(4, 1) * M_inv_(1, 0) + x(4, 2) * M_inv_(2, 0);
      acc[j + nb_var_axis_] = x(4, 0) * M_inv_(0, 1) + x(4, 1) * M_inv_(1, 1) + x(4, 2) * M_inv_(2, 1);
      acc[j + 2 * nb_var_axis_] = x(4, 0) * M_inv_(0, 2) + x(4, 1) * M_inv_(1, 2) + x(4, 2) * M_inv_(2, 2);
      x = A_ * x;
    }
//    A_ineq_.row(i - 1).head(nb_var_ - 1) = -acc;
//    B_ineq_(i - 1) = -std::min(start_pose(2), target_pose(2)) + 0.0001 + current_velocity_(2) * sampling_time * i;
//
//    // z <= z_max
//    A_ineq_.row(i - 1 + nb_local_sampling_time).head(nb_var_ - 1) = acc;
//    B_ineq_(i - 1 + nb_local_sampling_time) =
//        std::max(start_pose(2), target_pose(2)) + mid_air_height_ - current_velocity_(2) * sampling_time * i;
  }
//  B_ineq_(nb_local_sampling_time - 1) = -std::min(start_pose(2), target_pose(2));

  bool failure = false;
  if (!failure) {
    if (!qp_solver_.solve(Q_, q_, A_eq_, B_eq_, A_ineq_, B_ineq_)) {
      std::string error = "NewEndEffectorTrajectory3D::compute(): "
                          "failed to solve the QP.";
      std::cout << "Error: " << error << std::endl;
      //   print_solver();
      failure = true;
    }
  }

  if (failure) {
    // https://github.com/jrl-umi3218/eigen-quadprog/blob/master/src/QuadProg/c/solve.QP.compact.c#L94
    if (qp_solver_.fail() == 1) {
      std::cout << "NewEndEffectorTrajectory3D::compute -> the minimization "
                   "problem has no "
                   "solution!"
                << std::endl;
    } else {
      std::cout << "lhum fail"<< qp_solver_.fail() << std::endl;
      std::cout
          << "NewEndEffectorTrajectory3D::compute -> problems with decomposing D!"
          << std::endl;
    }
  }
//  print_solver();
  return !failure;
}

int NewEndEffectorTrajectory3D::get_forces(
    Eigen::Ref<Eigen::VectorXd> forces, Eigen::Ref<Eigen::Vector3d> next_pose,
    Eigen::Ref<Eigen::Vector3d> next_velocity, Eigen::Ref<Eigen::Vector3d> next_acceleration) {
  if (current_time_ < start_time_) {
    forces.setZero();
  } else if (current_time_ >= last_end_time_seen_ - 1e-4) {
    forces.setZero();
  } else {
    // Extract the information from the solution.
    x_opt_.resize(nb_var_);
    x_opt_ = qp_solver_.result();
    for(int i = 0 ; i < nb_var_ / 3; i++){
        forces(i * 3) = x_opt_[i];
        forces(i * 3 + 1) = x_opt_[nb_var_axis_ + i];
        forces(i * 3 + 2) = x_opt_[2 * nb_var_axis_ + i];
    }
  }
  next_acceleration << forces[0] * M_inv_(0, 0) + forces[1] * M_inv_(1, 0) + forces[2] * M_inv_(2, 0),
                       forces[0] * M_inv_(0, 1) + forces[1] * M_inv_(1, 1) + forces[2] * M_inv_(2, 1),
                       forces[0] * M_inv_(0, 2) + forces[1] * M_inv_(1, 2) + forces[2] * M_inv_(2, 2);

  next_velocity << next_acceleration(0) * sampling_time + current_velocity_(0),
                   next_acceleration(1) * sampling_time + current_velocity_(1),
                   next_acceleration(2) * sampling_time + current_velocity_(2);

  next_pose << 0.5 * next_acceleration(0) * sampling_time * sampling_time +
               current_velocity_(0) * sampling_time + current_pose_(0),
               0.5 * next_acceleration(1) * sampling_time * sampling_time +
               current_velocity_(1) * sampling_time + current_pose_(1),
               0.5 * next_acceleration(2) * sampling_time * sampling_time +
               current_velocity_(2) * sampling_time + current_pose_(2);
  std::cout << "Lhum pos" << next_pose << std::endl;

  return nb_var_ - 1;
}

std::string NewEndEffectorTrajectory3D::to_string() const {
  std::ostringstream oss;
  oss << "Solver info:" << std::endl;
  oss << "Q:" << std::endl << Q_ << std::endl;
  oss << "q:" << q_.transpose() << std::endl;
  oss << "A_eq:" << std::endl << A_eq_ << std::endl;
  oss << "B_eq:" << B_eq_.transpose() << std::endl;
  oss << "A_ineq:" << std::endl << A_ineq_ << std::endl;
  oss << "B_ineq:" << B_ineq_.transpose();
  if (qp_solver_.fail() == 0) {
    oss << "QP solution: " << qp_solver_.result() << std::endl;
  } else {
    oss << "QP failed with code error: " << qp_solver_.fail() << std::endl;
  }

  return oss.str();
}

void NewEndEffectorTrajectory3D::print_solver() const {
  std::cout << to_string() << std::endl;
}

} // namespace reactive_planners
