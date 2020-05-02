/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Implements the "Walking Control Based on Step Timing Adaptation" foot
 * trajectory QP. The pdf can be found in https://arxiv.org/abs/1704.01271, and
 * in the `doc/` folder in this repository.
 */

#pragma once

#include <Eigen/Eigen>
#include <eigen-quadprog/QuadProg.h>
#include <iostream>
#include <limits>

namespace reactive_planners {

/**
 * @brief
 */
class NewEndEffectorTrajectory3D {
  /*
   * Private methods
   */
public:
  /** @brief Constructor. */
  NewEndEffectorTrajectory3D();

  /** @brief Destructor. */
  ~NewEndEffectorTrajectory3D();

  bool compute(Eigen::Ref<const Eigen::Vector3d> start_pose,
               Eigen::Ref<const Eigen::Vector3d> current_pose,
               Eigen::Ref<const Eigen::Vector3d> current_velocity,
               Eigen::Ref<const Eigen::Vector3d> current_acceleration,
               Eigen::Ref<const Eigen::Vector3d> target_pose, const double &start_time,
               const double &current_time, const double &end_time);

  /** @brief Get all the forces until landing foot. Returns the number of forces.*/
  int get_forces(Eigen::Ref<Eigen::VectorXd> forces, Eigen::Ref<Eigen::Vector3d> next_pose,
    Eigen::Ref<Eigen::Vector3d> next_velocity, Eigen::Ref<Eigen::Vector3d> next_acceleration);

  /** @brief Display the matrices of the Problem. */
  void print_solver() const;

  /** @brief Convert the inner data to a string format. */
  std::string to_string() const;

  /*
   * Getters
   */

  /** @brief Get the height of the flying foot. */
  double get_mid_air_height() { return mid_air_height_; }

  /** @brief Get the last end time taken into account during the foot trajectory
   * computation. */
  double get_last_end_time_taken_into_account() { return last_end_time_seen_; }

  /*
   * Setters
   */

  /** @brief Set the height of the flying foot.
   *
   * @param mid_air_height
   */
  void set_mid_air_height(double mid_air_height) {
    mid_air_height_ = mid_air_height;
  }
  /*
   * Private methods
   */
private:
  /** @brief resize QP variable at each step.
   */
  void resize_matrices(){
      Q_.resize(nb_var_, nb_var_);
      Q_.setZero();
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

  /*
   * Constant problem parameters.
   */

  /** @brief Flying foot apex to be reach mid-air. */
  double mid_air_height_;

  /** @brief Number of force variables in the optimization problem on the
   * one of the axes. */
  int nb_var_axis_;

  /*
   * Variable problem parameters.
   */

  /** @brief Initial position before the motion. */
  Eigen::Vector3d start_pose_;

  /** @brief Current position. */
  Eigen::Vector3d current_pose_;

  /** @brief Previous computed position from the QP. */
  Eigen::Vector3d previous_solution_pose_;

  /** @brief Current velocity. */
  Eigen::Vector3d current_velocity_;

  /** @brief Current acceleration. */
  Eigen::Vector3d current_acceleration_;

  /** @brief Target pose after the motion. */
  Eigen::Vector3d target_pose_;

  /** @brief Initial time. */
  double start_time_;

  /** @brief Current time. */
  double current_time_;

  /** @brief Final time and the end of the motion. */
  double end_time_;

  /** @brief Last end time register when we computed the QP. */
  double last_end_time_seen_;

  /** @brief Last end time register when we computed the QP. */
  double sampling_time;

  /*
   * QP variables
   */

  /** @brief Number of variables in the optimization problem. */
  int nb_var_;

  /** @brief Number of equality constraints in the optimization problem. */
  int nb_eq_;

  /** @brief Number of inequality in the optimization problem. */
  int nb_ineq_;

  /** @brief Number of sampling time in the optimization problem. */
  double nb_sampling_time;

  /** @brief Quadratic program solver.
   *
   * This is an eigen wrapper around the quad_prog fortran solver.
   */
  Eigen::QuadProgDense qp_solver_;

  /** @brief Solution of the optimization problem.
   * @see NewEndEffectorTrajectory3D */
  Eigen::VectorXd x_opt_;

  /** @brief Quadratic term of the quadratic cost.
   * @see NewEndEffectorTrajectory3D */
  Eigen::MatrixXd Q_;

  /** @brief inverse estimation of mass matrix.
   * @see NewEndEffectorTrajectory3D */
  Eigen::MatrixXd M_inv_;

//  /** @brief Quadratic term added to the quadratic cost in order regularize the
//   * system.
//   * @see NewEndEffectorTrajectory3D */
//  Eigen::MatrixXd Q_regul_;
//
  /** @brief Cost weights for forces. */
  double cost_;

  /** @brief Cost weights for the epsilon. */
  double cost_epsilon_;

  /** @brief Linear term of the quadratic cost.
   * @see NewEndEffectorTrajectory3D */
  Eigen::VectorXd q_;

  /** @brief Linear equality matrix.
   * @see NewEndEffectorTrajectory3D */
  Eigen::MatrixXd A_eq_;

  /** @brief Linear equality vector.
   * @see NewEndEffectorTrajectory3D */
  Eigen::VectorXd B_eq_;

  /** @brief Linear inequality matrix.
   * @see NewEndEffectorTrajectory3D */
  Eigen::MatrixXd A_ineq_;

  /** @brief Linear inequality vector.
   * @see NewEndEffectorTrajectory3D */
  Eigen::VectorXd B_ineq_;

  /** @brief Quadratic term of the quadratic cost.
   * @see NewEndEffectorTrajectory3D */
  Eigen::MatrixXd A_;

  /** @brief Quadratic term of the quadratic cost.
   * @see NewEndEffectorTrajectory3D */
  Eigen::MatrixXd B_;
};

} // namespace reactive_planners
