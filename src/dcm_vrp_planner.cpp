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
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace reactive_planners {
DcmVrpPlanner::DcmVrpPlanner(
    const double& l_min, const double& l_max, const double& w_min,
    const double& w_max, const double& t_min, const double& t_max,
    const double& l_p, const double& ht,
    Eigen::Ref<const Eigen::Vector9d> cost_weights_local) {
  initialize(l_min, l_max, w_min, w_max, t_min, t_max, l_p, ht,
             cost_weights_local);
}

void DcmVrpPlanner::initialize(
    const double& l_min, const double& l_max, const double& w_min,
    const double& w_max, const double& t_min, const double& t_max,
    const double& l_p, const double& ht,
    Eigen::Ref<const Eigen::Vector9d> cost_weights_local) {
  l_min_ = l_min;
  l_max_ = l_max;
  w_min_ = w_min;
  w_max_ = w_max;
  t_min_ = t_min;
  t_max_ = t_max;
  l_p_ = l_p;
  ht_ = ht;
  cost_weights_local_ = cost_weights_local;

  // std::cout << "void DcmVrpPlanner::initialize" << std::endl;
  // std::cout << l_min << "," << l_max << "," << w_min << "," << w_max << ","
  //           << t_min << "," << t_max << ","
  //           << l_p << "," << ht << std::endl;
  // std::cout << cost_weights_local << std::endl;

  omega_ = sqrt(9.81 / ht_);
  tau_min_ = exp(omega_ * t_min_);
  tau_max_ = exp(omega_ * t_max_);

  // Equation 7 and 9 in the paper.
  bx_max_ = l_max_ / (tau_min_ - 1);
  bx_min_ = l_min_ / (tau_max_ - 1);
  by_max_out_ = l_p_ / (1 + tau_min_) +
                (w_max_ - w_min_ * tau_min_) / (1 - exp(2 * omega_ * t_min_));
  by_max_in_ = l_p_ / (1 + tau_min_) +
               (w_min_ - w_max_ * tau_min_) / (1 - exp(2 * omega_ * t_min_));

  // psi are slack variables here.
  // u_x, u_y, tau, b_x, b_y, psi_0, psi_1, psi_2, psi_3
  nb_var_ = 9;
  x_opt_.resize(nb_var_);
  x_opt_.setZero();
  x_opt_lb_.resize(nb_var_);
  x_opt_lb_.setZero();
  x_opt_ub_.resize(nb_var_);
  x_opt_ub_.setZero();
  slack_variables_.resize(4);
  Q_.resize(nb_var_, nb_var_);
  Q_.setZero();
  Q_.diagonal() = cost_weights_local_;
  q_.resize(nb_var_);
  q_.setZero();

  nb_eq_ = 2;
  A_eq_.resize(nb_eq_, nb_var_);
  A_eq_.setZero();
  B_eq_.resize(nb_eq_);
  B_eq_.setZero();

  nb_ineq_ = 10;
  A_ineq_.resize(nb_ineq_, nb_var_);
  // clang-format off
  //          ux    uy    tau   bx    by    psi0  psi1  psi2  psi3
  A_ineq_ <<  1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   // 0
              0.0,  1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   // 1
             -1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   // 2
              0.0, -1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   // 3
              0.0,  0.0,  1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   // 4
              0.0,  0.0, -1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   // 5
              0.0,  0.0,  0.0,  1.0,  0.0,  0.0,  1.0,  0.0,  0.0,   // 6
              0.0,  0.0,  0.0, -1.0,  0.0,  1.0,  0.0,  0.0,  0.0,   // 7
              0.0,  0.0,  0.0,  0.0,  1.0,  0.0,  0.0,  1.0,  0.0,   // 8
              0.0,  0.0,  0.0,  0.0, -1.0,  0.0,  0.0,  0.0,  1.0;   // 9
  // clang-format on
  B_ineq_.resize(nb_ineq_);
  B_ineq_.setZero();
  qp_solver_.problem(nb_var_, nb_eq_, nb_ineq_);

  dcm_nominal_.setZero();
}

void DcmVrpPlanner::compute_nominal_step_values(
    const bool& is_left_leg_in_contact, const Eigen::Vector3d& v_des_local) {
  double contact_switcher = is_left_leg_in_contact ? 1.0 : 2.0;
  double t_lower_bound(0.0), t_upper_bound(0.0);
  Eigen::Vector3d max_or_min_time;
  /** @todo Better manage the lower and upper bound on `t`. */
  // if ((v_des_local.head<2>().array().abs() < 1e-5).any()) {
  if (abs(v_des_local(0)) < 1e-5 || abs(v_des_local(1)) < 1e-5) {
    t_lower_bound = t_min_;
    t_upper_bound = t_max_;
  } else {
    max_or_min_time << l_min_ / v_des_local(0), w_min_ / v_des_local(1), t_min_;
    t_lower_bound = max_or_min_time.maxCoeff();
    max_or_min_time << l_max_ / v_des_local(0), w_max_ / v_des_local(1), t_max_;
    t_upper_bound = max_or_min_time.minCoeff();
  }

  t_nom_ = (t_lower_bound + t_upper_bound) * 0.5;
  tau_nom_ = exp(omega_ * t_nom_);
  l_nom_ = v_des_local(0) * t_nom_;
  w_nom_ = v_des_local(1) * t_nom_;

  bx_nom_ = l_nom_ / (tau_nom_ - 1);
  by_nom_ = (pow(-1, contact_switcher) * (l_p_ / (1 + tau_nom_))) -
            w_nom_ / (1 - tau_nom_);
}

#define dbg_dump(var) std::cout << "  " << #var << ": " << var << std::endl


void DcmVrpPlanner::py_update(const Eigen::Vector3d& current_step_location,
                              const double& time_from_last_step_touchdown,
                              const bool& is_left_leg_in_contact,
                              const Eigen::Vector3d& v_des,
                              const Eigen::Vector3d& com,
                              const Eigen::Vector3d& com_vel,
                              const double yaw) {
  pinocchio::SE3 world_M_base(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
  update(current_step_location, time_from_last_step_touchdown, is_left_leg_in_contact, v_des, com, com_vel, world_M_base);
}


void DcmVrpPlanner::update(const Eigen::Vector3d& current_step_location,
                           const double& time_from_last_step_touchdown,
                           const bool& is_left_leg_in_contact,
                           const Eigen::Vector3d& v_des,
                           const Eigen::Vector3d& com,
                           const Eigen::Vector3d& com_vel,
                           const pinocchio::SE3& world_M_base) {

  // std::cout << "DcmVrpPlanner::update" << std::endl;
  // dbg_dump(current_step_location);
  // dbg_dump(time_from_last_step_touchdown);
  // dbg_dump(is_left_leg_in_contact);
  // dbg_dump(v_des);
  // dbg_dump(com);
  // dbg_dump(com_vel);
  // dbg_dump(world_M_base);

  // Ground height.
  double ground_height = 0.0;

  // Local frame parallel to the world frame and aligned with the base yaw.
  world_M_local_.translation() << world_M_base.translation()(0),
      world_M_base.translation()(1), ground_height;
  Eigen::Vector3d rpy = world_M_base.rotation().eulerAngles(0, 1, 2);
  world_M_local_.rotation() =
      Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ());

  // Compute the DCM in the local frame.
  dcm_local_.head<2>() = com_vel.head<2>() / omega_ + com.head<2>();
  dcm_local_(2) = ground_height;
  dcm_local_ = world_M_local_.actInv(dcm_local_);

  // Express the desired velocity in the local frame.
  v_des_local_ = v_des;
  v_des_local_ = world_M_local_.rotation().transpose() * v_des_local_;

  // Nominal value tracked by the QP.
  compute_nominal_step_values(is_left_leg_in_contact, v_des_local_);

  // Current step location in the local frame.
  current_step_location_local_ = world_M_local_.actInv(current_step_location);

  // dcm nominal
  dcm_nominal_ = (com_vel / omega_ + com - current_step_location) * tau_nom_;
  dcm_nominal_(2) = 0.0;

  // Quadratic cost matrix is constant
  // Q_ = cost_weights_local_.diagonal();

  // Quadratic cost Vector
  q_(0) = -cost_weights_local_(0) * l_nom_;
  q_(1) = -cost_weights_local_(1) * w_nom_;
  q_(2) = -cost_weights_local_(2) * tau_nom_;
  q_(3) = -cost_weights_local_(3) * bx_nom_;
  q_(4) = -cost_weights_local_(4) * by_nom_;
  q_.tail<4>().setZero();

  // Inequality constraints
  // A_ineq_ is constant see initialize.

  // clang-format off
  B_ineq_ <<  l_max_,                // 0
              w_max_,                // 1
             -l_min_,                // 2
             -w_min_,                // 3
              tau_max_,              // 4
             -tau_min_,              // 5
              bx_max_,               // 6
             -bx_min_,               // 7
              by_max_in_,            // 8
             -by_max_out_;           // 9
  // clang-format on

  // Equality constraints
  double tmp0 = (dcm_local_(0) - current_step_location_local_[0]) *
                exp(-1.0 * omega_ * time_from_last_step_touchdown);
  double tmp1 = (dcm_local_(1) - current_step_location_local_[1]) *
                exp(-1.0 * omega_ * time_from_last_step_touchdown);

  // clang-format off
  A_eq_ << 1.0, 0.0, -1.0 * tmp0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 1.0, -1.0 * tmp1, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;

  B_eq_ << 0.0,
           0.0;
  // clang-format on
}

bool DcmVrpPlanner::solve() {
  bool failure = false;
  if (!internal_checks()) {
    std::cout << "DcmVrpPlanner::solve(): Error, internal checks failed."
              << std::endl;
    failure = true;
  }

  if (!failure) {
    if (!qp_solver_.solve(Q_, q_, A_eq_, B_eq_, A_ineq_, B_ineq_)) {
      std::string error =
          "DcmVrpPlanner::compute_adapted_step_locations(): "
          "failed to solve the QP.";
      std::cout << "Error: " << error << std::endl;
      print_solver();
      failure = true;
    }
  }

  if (!failure) {
    // Extract the information from the solution.
    x_opt_ = qp_solver_.result();
    next_step_location_ =
        current_step_location_local_ +
        (Eigen::Vector3d() << x_opt_(0), x_opt_(1), 0.0).finished();
    next_step_location_ = world_M_local_.act(next_step_location_);
    duration_before_step_landing_ = log(x_opt_(2)) / omega_;
    slack_variables_ = x_opt_.tail<4>();
  } else {
    // https://github.com/jrl-umi3218/eigen-quadprog/blob/master/src/QuadProg/c/solve.QP.compact.c#L94
    if (qp_solver_.fail() == 1) {
      std::cout << "DcmVrpPlanner::solve() -> the minimization problem has no solution!" << std::endl;
    } else {
      std::cout << "DcmVrpPlanner::solve() -> problems with decomposing D!" << std::endl;
    }
    slack_variables_.setZero();

    duration_before_step_landing_ = t_nom_;
    next_step_location_ << l_nom_, w_nom_, 0.0;
    next_step_location_ += current_step_location_local_;
    next_step_location_ = world_M_local_.act(next_step_location_);
  }

  return !failure;
}

// clang-format off
#define assert_DcmVrpPlanner(test)                  \
  if(!(test))                                       \
  {                                                 \
    std::ostringstream oss;                         \
    oss << "Warning: " << #test << " not true!";    \
    std::cout << oss.str() << std::endl;            \
    return false;                                   \
  }
// clang-format on

bool DcmVrpPlanner::internal_checks() {
  assert_DcmVrpPlanner(l_min_ <= l_max_);
  assert_DcmVrpPlanner(w_min_ <= w_max_);
  assert_DcmVrpPlanner(t_min_ <= t_max_);
  assert_DcmVrpPlanner(l_p_ == l_p_);
  assert_DcmVrpPlanner(ht_ > 0.0);
  assert_DcmVrpPlanner((cost_weights_local_.array() >= 0.0).all());
  assert_DcmVrpPlanner(bx_min_ < bx_max_);
  assert_DcmVrpPlanner(by_max_out_ <= by_max_in_);
  assert_DcmVrpPlanner(Q_.rows() == x_opt_.size());
  assert_DcmVrpPlanner(Q_.cols() == x_opt_.size());
  assert_DcmVrpPlanner(q_.size() == x_opt_.size());
  assert_DcmVrpPlanner(A_eq_.rows() == 2);
  assert_DcmVrpPlanner(A_eq_.cols() == x_opt_.size());
  assert_DcmVrpPlanner(B_eq_.size() == 2);
  assert_DcmVrpPlanner(A_ineq_.rows() == 10);
  assert_DcmVrpPlanner(A_ineq_.cols() == x_opt_.size());
  assert_DcmVrpPlanner(B_ineq_.size() == 10);
  assert_DcmVrpPlanner((t_min_ - log(tau_min_) / omega_) *
                           (t_min_ - log(tau_min_) / omega_) <
                       1e-8);
  assert_DcmVrpPlanner((t_max_ - log(tau_max_) / omega_) *
                           (t_max_ - log(tau_max_) / omega_) <
                       1e-8);
  return true;
}

std::string DcmVrpPlanner::to_string() const {
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

void DcmVrpPlanner::print_solver() const {
  std::cout << to_string() << std::endl;
}

}  // namespace reactive_planners