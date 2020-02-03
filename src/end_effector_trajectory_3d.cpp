/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Implement the reactive_planners::EndEffectorTrajectory3D class
 */

#include "reactive_planners/end_effector_trajectory_3d.hpp"
#include <iostream>

namespace reactive_planners {

EndEffectorTrajectory3D::EndEffectorTrajectory3D() {
  // Constant problem parameter.
  mid_air_height_ = 0.05;
  nb_var_x_ = 6; // Polynome order 5
  nb_var_y_ = 6; // Polynome order 5
  nb_var_z_ = 9; // Polynome order 7
  cost_x_ = 1e0; /** @todo EndEffectorTrajectory3D::cost_x_ to be tuned. */
  cost_y_ = 1e0; /** @todo EndEffectorTrajectory3D::cost_y_ to be tuned. */
  cost_z_ = 1e0; /** @todo EndEffectorTrajectory3D::cost_z_ to be tuned. */
  double hess_regul = 1e-6;

  // Variable parameters.
  // clang-format off
  time_vec_x_.resize(nb_var_x_); time_vec_x_.setZero();
  time_vec_y_.resize(nb_var_y_); time_vec_y_.setZero();
  time_vec_z_.resize(nb_var_z_); time_vec_z_.setZero();
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

  // QP parameter.
  nb_var_ = nb_var_x_ + nb_var_y_ + nb_var_z_;
  // Around 10 nodes for the z_min < z(t) < z_max.
  nb_ineq_ = 2 * 10;
  // current and final conditions
  nb_eq_ = 5 + 5 + 6; // + 1;

  x_opt_.resize(nb_var_);
  x_opt_.setZero();
  x_opt_lb_.resize(nb_var_);
  x_opt_lb_.setZero();
  x_opt_ub_.resize(nb_var_);
  x_opt_ub_.setZero();

  Q_.resize(nb_var_, nb_var_);
  Q_.setZero();
  Q_regul_ = Eigen::MatrixXd::Identity(nb_var_, nb_var_) * hess_regul;
  q_.resize(nb_var_);
  q_.setZero();

  A_eq_.resize(nb_eq_, nb_var_);
  A_eq_.setZero();
  B_eq_.resize(nb_eq_);
  B_eq_.setZero();

  A_ineq_.resize(nb_ineq_, nb_var_);
  A_ineq_.setZero();
  B_ineq_.resize(nb_ineq_);
  B_ineq_.setZero();
  qp_solver_.problem(nb_var_, nb_eq_, nb_ineq_);
}

EndEffectorTrajectory3D::~EndEffectorTrajectory3D() {}

bool EndEffectorTrajectory3D::compute(
    const Eigen::Vector3d &start_pose,
    const Eigen::Vector3d &current_pose,
    const Eigen::Vector3d &current_velocity,
    const Eigen::Vector3d &current_acceleration,
    const Eigen::Vector3d &target_pose, const double &start_time,
    const double &current_time, const double &end_time) {
  // save args:
  start_pose_ = start_pose;
  current_pose_ = current_pose;
  current_velocity_ = current_velocity;
  current_acceleration_ = current_acceleration;
  target_pose_ = target_pose;
  start_time_ = start_time;
  current_time_ = current_time;
  end_time_ = end_time;

  // Do not compute the QP if the solution is trivial or too close to the end of
  // the trajectory
  double safe_end_time = end_time_ * 0.70;
  if (current_time_ < start_time_ || current_time_ >= safe_end_time) {
    return true;
  } else {
    last_end_time_seen_ = end_time;
  }

  // scaling the problem
  double duration = (last_end_time_seen_ - start_time);
  double local_current_time = (current_time - start_time) / duration;
  double local_end_time = 1.0;
  double mid_time = 0.5;

  /*
   * Quadratic cost
   */
  Q_.setZero();
  // Q_x
  t_vec(local_end_time, time_vec_x_);
  Q_.block(0, 0, nb_var_x_, nb_var_x_) =
      time_vec_x_ * time_vec_x_.transpose() * cost_x_;
  // Q_y
  t_vec(local_end_time, time_vec_y_);
  Q_.block(nb_var_x_, nb_var_x_, nb_var_y_, nb_var_y_) =
      time_vec_y_ * time_vec_y_.transpose() * cost_y_;
  // Q_z
  t_vec(mid_time, time_vec_z_);
  Q_.block(nb_var_x_ + nb_var_y_, nb_var_x_ + nb_var_y_, nb_var_z_, nb_var_z_) =
      time_vec_z_ * time_vec_z_.transpose() * cost_z_;
  // Q_regul
  Q_ += Q_regul_;

  // q_x
  t_vec(local_end_time, time_vec_x_);
  q_.head(nb_var_x_) = -time_vec_x_ * target_pose(0) * cost_x_;
  // q_y
  t_vec(local_end_time, time_vec_y_);
  q_.segment(nb_var_x_, nb_var_y_) = -time_vec_y_ * target_pose(1) * cost_y_;
  // q_z
  t_vec(mid_time, time_vec_z_);
  q_.tail(nb_var_z_) =
      -time_vec_z_ *
      (std::max(start_pose(2), target_pose(2)) + mid_air_height_) * cost_z_;

  /*
   * Equality constraints.
   */
  // clang-format off
  A_eq_.setZero();
  // X current constraints
  t_vec(local_current_time, time_vec_x_);
  A_eq_.row(0).head(nb_var_x_) = time_vec_x_;
  dt_vec(local_current_time, time_vec_x_);
  A_eq_.row(1).head(nb_var_x_) = time_vec_x_;
  ddt_vec(local_current_time, time_vec_x_);
  A_eq_.row(2).head(nb_var_x_) = time_vec_x_;
  // X end constraints
  dt_vec(local_end_time, time_vec_x_);
  A_eq_.row(3).head(nb_var_x_) = time_vec_x_;
  ddt_vec(local_end_time, time_vec_x_);
  A_eq_.row(4).head(nb_var_x_) = time_vec_x_;
  // Y current constraints
  t_vec(local_current_time, time_vec_y_);
  A_eq_.row(5).segment(nb_var_x_, nb_var_y_) = time_vec_y_;
  dt_vec(local_current_time, time_vec_y_);
  A_eq_.row(6).segment(nb_var_x_, nb_var_y_) = time_vec_y_;
  ddt_vec(local_current_time, time_vec_y_);
  A_eq_.row(7).segment(nb_var_x_, nb_var_y_) = time_vec_y_;
  // Y end constraints
  dt_vec(local_end_time, time_vec_y_);
  A_eq_.row(8).segment(nb_var_x_, nb_var_y_) = time_vec_y_;
  ddt_vec(local_end_time, time_vec_y_);
  A_eq_.row(9).segment(nb_var_x_, nb_var_y_) = time_vec_y_;
  // Z current constraints
  t_vec(local_current_time, time_vec_z_);
  A_eq_.row(10).tail(nb_var_z_) = time_vec_z_;
  dt_vec(local_current_time, time_vec_z_);
  A_eq_.row(11).tail(nb_var_z_) = time_vec_z_;
  ddt_vec(local_current_time, time_vec_z_);
  A_eq_.row(12).tail(nb_var_z_) = time_vec_z_;
  // Z end constraints
  t_vec(local_end_time, time_vec_z_);
  A_eq_.row(13).tail(nb_var_z_) = time_vec_z_;
  dt_vec(local_end_time, time_vec_z_);
  A_eq_.row(14).tail(nb_var_z_) = time_vec_z_;
  ddt_vec(local_end_time, time_vec_z_);
  A_eq_.row(15).tail(nb_var_z_) = time_vec_z_;
  // Z mid constraints
//   dt_vec(mid_time, time_vec_z_);
//   A_eq_.row(16).tail(nb_var_z_) = time_vec_z_;

  B_eq_.setZero();
  B_eq_ << current_pose(0), current_velocity(0), current_acceleration(0), 0.0, 0.0,
           current_pose(1), current_velocity(1), current_acceleration(1), 0.0, 0.0,
           current_pose(2), current_velocity(2), current_acceleration(2),
           target_pose(2), 0.0, 0.0;
        //    0.0;
  // clang-format on

  /*
   * Inequality constraints
   */
  A_ineq_.setZero();
  B_ineq_.setZero();

  int n = A_ineq_.rows() / 2;
  double t = 0.0;
  double dt = 1.0 / (double)n;

  for (int i = 0; i < n; ++i) {
    // time vector
    t_vec(t, time_vec_z_);

    // z >= zmin   =>   -z <= -z_min
    A_ineq_.row(i).tail(nb_var_z_) = -time_vec_z_;
    B_ineq_(i) = -std::min(start_pose(2), target_pose(2)) + 0.0001;
    // B_ineq_(i) = std::numeric_limits<double>::max();

    // z <= z_max
    A_ineq_.row(i + n).tail(nb_var_z_) = time_vec_z_;
    // B_ineq_(i + n) = std::numeric_limits<double>::max();
    B_ineq_(i + n) =
        std::max(start_pose(2), target_pose(2)) + 1.0 * mid_air_height_;

    // Update the time.
    t += dt;
  }

  bool failure = false;
  if (!failure) {
    if (!qp_solver_.solve(Q_, q_, A_eq_, B_eq_, A_ineq_, B_ineq_)) {
      std::string error = "EndEffectorTrajectory3D::compute(): "
                          "failed to solve the QP.";
      std::cout << "Error: " << error << std::endl;
      //   print_solver();
      failure = true;
    }
  }

  if (failure) {
    // https://github.com/jrl-umi3218/eigen-quadprog/blob/master/src/QuadProg/c/solve.QP.compact.c#L94
    if (qp_solver_.fail() == 1) {
      std::cout << "EndEffectorTrajectory3D::compute -> the minimization "
                   "problem has no "
                   "solution!"
                << std::endl;
    } else {
      std::cout
          << "EndEffectorTrajectory3D::compute -> problems with decomposing D!"
          << std::endl;
    }
  }
  return !failure;
}

void EndEffectorTrajectory3D::get_next_state(
    const double &next_time, Eigen::Ref<Eigen::Vector3d> next_pose,
    Eigen::Ref<Eigen::Vector3d> next_velocity, Eigen::Ref<Eigen::Vector3d> next_acceleration) {

  double duration = (last_end_time_seen_ - start_time_);
  double local_current_time = (next_time - start_time_) / duration;

  if (current_time_ < start_time_) {
    next_pose = start_pose_;
    next_velocity.setZero();
    next_acceleration.setZero();
  } else if (current_time_ >= last_end_time_seen_ - 1e-4) {
    next_pose = previous_solution_pose_;
    next_velocity.setZero();
    next_acceleration.setZero();
  } else {
    // Extract the information from the solution.
    x_opt_ = qp_solver_.result();
    t_vec(local_current_time, time_vec_x_);
    t_vec(local_current_time, time_vec_y_);
    t_vec(local_current_time, time_vec_z_);
    next_pose << x_opt_.head(nb_var_x_).transpose() * time_vec_x_,
        x_opt_.segment(nb_var_x_, nb_var_y_).transpose() * time_vec_y_,
        x_opt_.tail(nb_var_z_).transpose() * time_vec_z_;

    dt_vec(local_current_time, time_vec_x_);
    dt_vec(local_current_time, time_vec_y_);
    dt_vec(local_current_time, time_vec_z_);
    next_velocity << x_opt_.head(nb_var_x_).transpose() * time_vec_x_,
        x_opt_.segment(nb_var_x_, nb_var_y_).transpose() * time_vec_y_,
        x_opt_.tail(nb_var_z_).transpose() * time_vec_z_;

    ddt_vec(local_current_time, time_vec_x_);
    ddt_vec(local_current_time, time_vec_y_);
    ddt_vec(local_current_time, time_vec_z_);
    next_acceleration << x_opt_.head(nb_var_x_).transpose() * time_vec_x_,
        x_opt_.segment(nb_var_x_, nb_var_y_).transpose() * time_vec_y_,
        x_opt_.tail(nb_var_z_).transpose() * time_vec_z_;
  }
  previous_solution_pose_ = next_pose;
}

std::string EndEffectorTrajectory3D::to_string() const {
  std::ostringstream oss;
  oss << "Solver info:" << std::endl;
  oss << "Q:" << std::endl << Q_ << std::endl;
  oss << "q:" << q_.transpose() << std::endl;
  oss << "A_eq:" << std::endl << A_eq_ << std::endl;
  oss << "B_eq:" << B_eq_.transpose() << std::endl;
  oss << "A_ineq:" << std::endl << A_ineq_ << std::endl;
  oss << "B_ineq:" << B_ineq_.transpose();
  return oss.str();
}

void EndEffectorTrajectory3D::print_solver() const {
  std::cout << to_string() << std::endl;
}

} // namespace reactive_planners