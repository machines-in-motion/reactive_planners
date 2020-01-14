/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Definition of the DcmVrpPlanner class
 */

#include "reactive_planners/dcm_vrp_planner.hpp"
#include <cmath>

namespace reactive_planners {
DcmVrpPlanner::DcmVrpPlanner(const double& l_min, const double& l_max,
                             const double& w_min, const double& w_max,
                             const double& t_min, const double& t_max,
                             const Eigen::Vector2d& v_des, const double& l_p,
                             const double& ht) {
  initialize(l_min, l_max, w_min, w_max, t_min, t_max, v_des, l_p, ht);
}

void DcmVrpPlanner::initialize(const double& l_min, const double& l_max,
                               const double& w_min, const double& w_max,
                               const double& t_min, const double& t_max,
                               const Eigen::Vector2d& v_des, const double& l_p,
                               const double& ht) {
  l_min_ = l_min;
  l_max_ = l_max;
  w_min_ = w_min;
  w_max_ = w_max;
  t_min_ = t_min;
  t_max_ = t_max;
  v_des_ = v_des;
  l_p_ = l_p;
  ht_ = ht;

  omega_ = sqrt(9.81 / ht_);
  // Equation 7 and 9 in the paper.
  bx_max_ = l_max_ / (exp(omega_ * t_min_) - 1);
  bx_min_ = l_min_ / (exp(omega_ * t_max_) - 1);
  by_max_out_ =
      l_p_ / (1 + exp(omega_ * t_min)) +
      (w_max_ - w_min_ * exp(omega_ * t_min_)) / (1 - exp(2 * omega_ * t_min_));
  by_max_in_ =
      l_p_ / (1 + exp(omega_ * t_min)) +
      (w_min_ - w_max_ * exp(omega_ * t_min_)) / (1 - exp(2 * omega_ * t_min_));

  nb_var_ = 9;  // u_x, u_y, tau, b_x, b_y, psi_0, psi_1, psi_2, psi_3
  x_opt_.resize(nb_var_);
  x_opt_.setZero();
  x_opt_lb_.resize(nb_var_);
  x_opt_lb_.setZero();
  x_opt_ub_.resize(nb_var_);
  x_opt_ub_.setZero();
  Q_.resize(nb_var_, nb_var_);
  Q_.setZero();
  q_.resize(nb_var_);
  q_.setZero();

  nb_eq_ = 2;
  A_eq_.resize(nb_eq_, nb_var_);
  A_eq_.setZero();
  B_eq_.resize(nb_eq_);
  B_eq_.setZero();

  nb_ineq_ = 10;
  A_ineq_.resize(nb_ineq_, nb_var_);
  A_ineq_.setZero();
  B_ineq_.resize(nb_ineq_);
  B_ineq_.setZero();

  qp_solver_.problem(nb_var_, nb_eq_, nb_ineq_);
}

void DcmVrpPlanner::compute_nominal_step_values(
    const double& is_left_leg_in_contact) {
  assert((is_left_leg_in_contact == 1.0 || is_left_leg_in_contact == 2.0) &&
         "wrong input");
  double b_l(0.0), b_u(0.0);
  Eigen::Vector3d max_or_min_time;
  if ((v_des_.array().abs() < 1e-5).any()) {
    b_l = t_min_;
    b_u = t_max_;
  } else {
    max_or_min_time << l_min_ / v_des_(0), w_min_ / v_des_(1), t_min_;
    b_l = max_or_min_time.maxCoeff();
    max_or_min_time << l_max_ / v_des_(0), w_max_ / v_des_(1), t_max_;
    b_u = max_or_min_time.minCoeff();
  }

  t_nom_ = (b_l + b_u) / 2.0;
  tau_nom_ = exp(omega_ * t_nom_);
  l_nom_ = v_des_(0) * t_nom_;
  w_nom_ = v_des_(1) * t_nom_;

  bx_nom_ = l_nom_ / (exp(omega_ * t_nom_) - 1);
  by_nom_ =
      (pow(-1, is_left_leg_in_contact) * (l_p_ / (1 + exp(omega_ * t_nom_)))) -
      w_nom_ / (1 - exp(omega_ * t_nom_));
}

void DcmVrpPlanner::compute_current_dcm(const Eigen::Vector3d& com_meas,
                                        const Eigen::Vector3d& com_vel_meas) {
  com_meas_ = com_meas;
  com_vel_meas_ = com_vel_meas;
  dcm_meas_ = com_vel_meas_.head<2>() / omega_ + com_meas_.head<2>();
}

void DcmVrpPlanner::compute_adapted_step_locations(
    const Eigen::Vector2d& current_step_location,
    const double& time_from_last_step_touchdown,
    const bool& is_left_leg_in_contact, const Eigen::Vector3d& com_meas,
    const Eigen::Vector3d& com_vel_meas, const Eigen::Vector9d& cost_weights) {
  compute_current_dcm(com_meas, com_vel_meas);
  compute_nominal_step_values(is_left_leg_in_contact);

  Q_ = cost_weights.diagonal();
  q_(0) = -cost_weights(0) * l_nom_;
  q_(1) = -cost_weights(1) * w_nom_;
  q_(2) = -cost_weights(2) * tau_nom_;
  q_(3) = -cost_weights(3) * bx_nom_;
  q_(4) = -cost_weights(4) * by_nom_;
  q_.tail<4>().setZero();

  // clang-format off
  //          ux    uy    tau   bx    by    psi0  psi1  psi2  psi3
  A_ineq_ <<  1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   // 0
              0.0,  1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   // 1
             -1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   // 2
              0.0, -1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   // 3
              0.0,  0.0,  1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   // 4
              0.0,  0.0, -1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   // 5
              0.0,  0.0,  0.0,  1.0,  0.0,  0.0, -1.0,  0.0,  0.0,   // 6
              0.0,  0.0,  0.0, -1.0,  0.0,  1.0,  0.0,  0.0,  0.0,   // 7
              0.0,  0.0,  0.0,  0.0,  1.0,  0.0,  0.0, -1.0,  0.0,   // 8
              0.0,  0.0,  0.0,  0.0, -1.0,  0.0,  0.0,  0.0,  1.0;   // 9

  B_ineq_ <<  l_max_,                // 0
              w_max_,                // 1
             -l_min_,                // 2
             -w_min_,                // 3
              exp(omega_ * t_max_),  // 4
             -exp(omega_ * t_min_),  // 5
              bx_max_,               // 6
             -bx_min_,               // 7
              by_max_in_,            // 8
             -by_max_out_;           // 9

  double tmp0 = (dcm_meas_(0) - current_step_location[0]) *
                exp(-1.0 * omega_ * time_from_last_step_touchdown);
  double tmp1 = (dcm_meas_(1) - current_step_location[1]) *
                exp(-1.0 * omega_ * time_from_last_step_touchdown);

  A_eq_ << 1.0, 0.0, -1.0 * tmp0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 1.0, -1.0 * tmp1, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;

  B_eq_ << current_step_location[0],
           current_step_location[1];
  // clang-format on

  /* Warning, eigen-quadprog define the cost as \f$ (1/2) x^T Q' x - q'^T x \f$.
   * Notice the "-" before the \f$ q'^T \f$. So we have to change the sign
   * internally upon usage.
   */
  qp_solver_.solve(Q_, -q_, A_eq_, B_eq_, A_ineq_, B_ineq_);
}

}  // namespace reactive_planners