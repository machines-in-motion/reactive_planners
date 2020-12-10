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
  cost_ = 1;
  cost_epsilon_z_mid_ = 1e6;
  cost_epsilon_x_ = 1e4;
  cost_epsilon_y_ = 1e4;
  cost_epsilon_z_ = 1e4;
  cost_epsilon_vel_ = 1e3;
  cost_dcm_epsilon_x_i_ = 1;
  cost_dcm_epsilon_y_i_ = 1;
  cost_epsilon_x_i_ = 1e5;
  cost_epsilon_y_i_ = 1e3;

  start_pose_.setZero();
  current_pose_.setZero();
  current_velocity_.setZero();
  current_acceleration_.setZero();
  target_pose_.setZero();
  previous_solution_pose_.setZero();
  v_des_.setZero();
  start_time_ = 0.0;
  current_time_ = 0.0;
  sampling_time = 0.010;
  control_loop = 0.001;
  is_left_leg_in_contact_ = false;

  // QP parameter.
  nb_sampling_time = 10;
  nb_var_ = 3 * nb_sampling_time + 7;
  nb_eq_ = 3 * 2 + 1;//xn=0,vn=0,x_(n/2)[2]=x_des
  nb_local_sampling_time_ = nb_sampling_time;

  // number of sampling time nodes for the z_min < z(t) < z_max.
  nb_ineq_ = nb_sampling_time * (2/*z_min & z_max*/ + 3 * 2/*force_min & force_max*/);
  // current and final conditions
  resize_matrices();
  resize_matrices_t_min(1);

  x_opt_.resize(nb_var_);
  x_opt_.setZero();
  /*
   * variables: 3 * nb_local_sampling_time_ + 7
   * F_x[0]  ... F_x[nb_local_sampling_time_ - 1] F_y[0] ... F_y[nb_local_sampling_time_ - 1] F_z[0] ... F_z[nb_local_sampling_time_ - 1]
   *
   * slack variables: 7
   * mid_z x[nb_local_sampling_time_ - 1] y[nb_local_sampling_time_ - 1] z[nb_local_sampling_time_ - 1]\.
   * v_x[nb_local_sampling_time_ - 1] v_y[nb_local_sampling_time_ - 1] v_z[nb_local_sampling_time_ - 1]
   *
   * equation: 7
   * x y z v_x v_y v_z z_mid
   *
   * inequality: nb_sampling_time * (2 + 3 * 2)
   * z_min[1] ... z_min[nb_local_sampling_time_ - 1] z_max[1] ... z_max[nb_local_sampling_time_ - 1] \.
   * F_x_min[0] F_y_min[0] F_z_min[0] ... \.
   * F_x_min[nb_local_sampling_time_ - 1] F_y_min[nb_local_sampling_time_ - 1] F_z_min[nb_local_sampling_time_ - 1] \.
   * F_x_max[0] F_y_max[0] F_z_max[0] ... \.
   * F_x_max[nb_local_sampling_time_ - 1] F_y_max[nb_local_sampling_time_ - 1] F_z_max[nb_local_sampling_time_ - 1]
   */

  M_inv_  = new Eigen::MatrixXd[2];
  M_inv_[0].resize(3, 3);
  M_inv_[1].resize(3, 3);
  M_inv_[0] << 0.045, 0.0, 0.0,
               0.0, 0.045, 0.0,
               0.0, 0.0, 0.09;//Left_swing
  M_inv_[1] << 0.045, -0.0, 0.0,
               -0.0, 0.045, -0.0,
               0.0, -0.0, 0.09;//Right_swing
  M_inv_[0] = M_inv_[0].inverse();
  M_inv_[1] = M_inv_[1].inverse();

  calculate_acceleration();
}

NewEndEffectorTrajectory3D::~NewEndEffectorTrajectory3D() = default;

void NewEndEffectorTrajectory3D::calculate_acceleration(){
  acceleration_terms_x_  = new Eigen::MatrixXd[2];
  acceleration_terms_y_  = new Eigen::MatrixXd[2];
  acceleration_terms_z_  = new Eigen::MatrixXd[2];
  velocity_terms_x_  = new Eigen::MatrixXd[2];
  velocity_terms_y_  = new Eigen::MatrixXd[2];
  velocity_terms_z_  = new Eigen::MatrixXd[2];
  g = new Eigen::MatrixXd[2];
  acceleration_terms_x_[0].resize(MAX_VAR, 3 * MAX_VAR);
  acceleration_terms_x_[1].resize(MAX_VAR, 3 * MAX_VAR);
  acceleration_terms_x_[0].setZero();
  acceleration_terms_x_[1].setZero();
  acceleration_terms_y_[0].resize(MAX_VAR, 3 * MAX_VAR);
  acceleration_terms_y_[1].resize(MAX_VAR, 3 * MAX_VAR);
  acceleration_terms_y_[0].setZero();
  acceleration_terms_y_[1].setZero();
  acceleration_terms_z_[0].resize(MAX_VAR, 3 * MAX_VAR);
  acceleration_terms_z_[1].resize(MAX_VAR, 3 * MAX_VAR);
  acceleration_terms_z_[0].setZero();
  acceleration_terms_z_[1].setZero();
  velocity_terms_x_[0].resize(MAX_VAR, 3 * MAX_VAR);
  velocity_terms_x_[1].resize(MAX_VAR, 3 * MAX_VAR);
  velocity_terms_x_[0].setZero();
  velocity_terms_x_[1].setZero();
  velocity_terms_y_[0].resize(MAX_VAR, 3 * MAX_VAR);
  velocity_terms_y_[1].resize(MAX_VAR, 3 * MAX_VAR);
  velocity_terms_y_[0].setZero();
  velocity_terms_y_[1].setZero();
  velocity_terms_z_[0].resize(MAX_VAR, 3 * MAX_VAR);
  velocity_terms_z_[1].resize(MAX_VAR, 3 * MAX_VAR);
  velocity_terms_z_[0].setZero();
  velocity_terms_z_[1].setZero();
  g[0].resize(MAX_VAR, 3);
  g[1].resize(MAX_VAR, 3);
  g[0].setZero();
  g[1].setZero();
  h << -0.4, 0., .8;

  Eigen::MatrixXd A_;
  Eigen::MatrixXd B_;
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

  Eigen::MatrixXd x;
  x.resize(6, 3);
  for(int k = 0 ; k < 2 ; k++) {
      for (int i = 0; i < MAX_VAR; ++i) {
          x = B_;
          for (int j = i - 1; j >= 0; j--) {
              acceleration_terms_x_[k](i, j) =
                      x(0, 0) * M_inv_[k](0, 0) + x(0, 1) * M_inv_[k](1, 0) + x(0, 2) * M_inv_[k](2, 0);
              acceleration_terms_x_[k](i, j + MAX_VAR) =
                      x(0, 0) * M_inv_[k](0, 1) + x(0, 1) * M_inv_[k](1, 1) + x(0, 2) * M_inv_[k](2, 1);
              acceleration_terms_x_[k](i, j + 2 * MAX_VAR) =
                      x(0, 0) * M_inv_[k](0, 2) + x(0, 1) * M_inv_[k](1, 2) + x(0, 2) * M_inv_[k](2, 2);
              g[k](i, 0) += acceleration_terms_x_[k](i, j) * h[0] +
                           acceleration_terms_x_[k](i, j + MAX_VAR) * h[1] +
                           acceleration_terms_x_[k](i, j + 2 * MAX_VAR) * h[2];

              acceleration_terms_y_[k](i, j) =
                      x(2, 0) * M_inv_[k](0, 0) + x(2, 1) * M_inv_[k](1, 0) + x(2, 2) * M_inv_[k](2, 0);
              acceleration_terms_y_[k](i, j + MAX_VAR) =
                      x(2, 0) * M_inv_[k](0, 1) + x(2, 1) * M_inv_[k](1, 1) + x(2, 2) * M_inv_[k](2, 1);
              acceleration_terms_y_[k](i, j + 2 * MAX_VAR) =
                      x(2, 0) * M_inv_[k](0, 2) + x(2, 1) * M_inv_[k](1, 2) + x(2, 2) * M_inv_[k](2, 2);
              g[k](i, 1) += acceleration_terms_y_[k](i, j) * h[0] +
                           acceleration_terms_y_[k](i, j + MAX_VAR) * h[1] +
                           acceleration_terms_y_[k](i, j + 2 * MAX_VAR) * h[2];

              acceleration_terms_z_[k](i, j) =
                      x(4, 0) * M_inv_[k](0, 0) + x(4, 1) * M_inv_[k](1, 0) + x(4, 2) * M_inv_[k](2, 0);
              acceleration_terms_z_[k](i, j + MAX_VAR) =
                      x(4, 0) * M_inv_[k](0, 1) + x(4, 1) * M_inv_[k](1, 1) + x(4, 2) * M_inv_[k](2, 1);
              acceleration_terms_z_[k](i, j + 2 * MAX_VAR) =
                      x(4, 0) * M_inv_[k](0, 2) + x(4, 1) * M_inv_[k](1, 2) + x(4, 2) * M_inv_[k](2, 2);
              g[k](i, 2) += acceleration_terms_z_[k](i, j) * h[0] +
                           acceleration_terms_z_[k](i, j + MAX_VAR) * h[1] +
                           acceleration_terms_z_[k](i, j + 2 * MAX_VAR) * h[2];

              velocity_terms_x_[k](i, j) =
                      x(1, 0) * M_inv_[k](0, 0) + x(1, 1) * M_inv_[k](1, 0) + x(1, 2) * M_inv_[k](2, 0);
              velocity_terms_x_[k](i, j + MAX_VAR) =
                      x(1, 0) * M_inv_[k](0, 1) + x(1, 1) * M_inv_[k](1, 1) + x(1, 2) * M_inv_[k](2, 1);
              velocity_terms_x_[k](i, j + 2 * MAX_VAR) =
                      x(1, 0) * M_inv_[k](0, 2) + x(1, 1) * M_inv_[k](1, 2) + x(1, 2) * M_inv_[k](2, 2);

              velocity_terms_y_[k](i, j) =
                      x(3, 0) * M_inv_[k](0, 0) + x(3, 1) * M_inv_[k](1, 0) + x(3, 2) * M_inv_[k](2, 0);
              velocity_terms_y_[k](i, j + MAX_VAR) =
                      x(3, 0) * M_inv_[k](0, 1) + x(3, 1) * M_inv_[k](1, 1) + x(3, 2) * M_inv_[k](2, 1);
              velocity_terms_y_[k](i, j + 2 * MAX_VAR) =
                      x(3, 0) * M_inv_[k](0, 2) + x(3, 1) * M_inv_[k](1, 2) + x(3, 2) * M_inv_[k](2, 2);

              velocity_terms_z_[k](i, j) =
                      x(5, 0) * M_inv_[k](0, 0) + x(5, 1) * M_inv_[k](1, 0) + x(5, 2) * M_inv_[k](2, 0);
              velocity_terms_z_[k](i, j + MAX_VAR) =
                      x(5, 0) * M_inv_[k](0, 1) + x(5, 1) * M_inv_[k](1, 1) + x(5, 2) * M_inv_[k](2, 1);
              velocity_terms_z_[k](i, j + 2 * MAX_VAR) =
                      x(5, 0) * M_inv_[k](0, 2) + x(5, 1) * M_inv_[k](1, 2) + x(5, 2) * M_inv_[k](2, 2);
              x = A_ * x;
          }
      }
  }
}

double NewEndEffectorTrajectory3D::calculate_t_min(const Eigen::Ref<const Eigen::Vector3d> &current_pose,
                                                   const Eigen::Ref<const Eigen::Vector3d> &current_velocity,
                                                   const bool &is_left_leg_in_contact) {
    int index = 1;
    do {
        resize_matrices_t_min(index);
        A_eq_t_min_.row(0).head(index) = acceleration_terms_z_[is_left_leg_in_contact].row(index).head(index);//xn[2]
        A_eq_t_min_.row(0).segment(index, index) = acceleration_terms_z_[is_left_leg_in_contact].row(index).segment(MAX_VAR, index);//xn[2]
        A_eq_t_min_.row(0).segment(2 * index, index) = acceleration_terms_z_[is_left_leg_in_contact].row(index).segment(2 * MAX_VAR, index);//xn[2]
        if(index == 1){
            B_eq_t_min_ << -current_pose(2) + 0.0 - current_velocity(2) * sampling_time * index + g[is_left_leg_in_contact](index, 2),
                    0.;//-current_pose(2) + 0.0;
        }
        else {
//            A_eq_t_min_.row(1).head(index) = acceleration_terms_z_[is_left_leg_in_contact].row(index - 1).head(index);//vn[2]
//            A_eq_t_min_.row(1).segment(index, index) = acceleration_terms_z_[is_left_leg_in_contact].row(index - 1).segment(MAX_VAR, index);//vn[2]
//            A_eq_t_min_.row(1).segment(2 * index, index) = acceleration_terms_z_[is_left_leg_in_contact].row(index - 1).segment(2 * MAX_VAR, index);//vn[2]
            B_eq_t_min_ << -current_pose(2) + 0.0 - current_velocity(2) * sampling_time * index + g[is_left_leg_in_contact](index, 2);//,
//                    -current_pose(2) + 0.0 - current_velocity(2) * sampling_time * (index - 1) + g[is_left_leg_in_contact](index - 1, 2);
        }
        for(int i = (index - 1) * 3; i < index * 3; i++) {
            A_ineq_t_min_(i, i) = 1;
            B_ineq_t_min_(i) = 18;
            A_ineq_t_min_(3 * index + i, i) = -1;
            B_ineq_t_min_(3 * index + i) = 18;
        }
        index++;
    }while(!qp_solver_t_min_.solve(Q_t_min_, q_t_min_, A_eq_t_min_, B_eq_t_min_, A_ineq_t_min_, B_ineq_t_min_));
    index--;
    return index * sampling_time;
}

void NewEndEffectorTrajectory3D::init_calculate_dcm(
    const Eigen::Ref<const Eigen::Vector3d>& v_des,
    const double& ht,
    const double& t_lower_bound,
    const double& t_upper_bound){
  ht_ = ht;
  v_des_ = v_des;
  t_nom_ = (t_lower_bound + t_upper_bound) * 0.5;
  omega_ = sqrt(9.81 / ht_);
  tau_nom_ = exp(omega_ * t_nom_);
  l_nom_ = v_des_(0) * t_nom_;
  bx_nom_ = l_nom_ / (tau_nom_ - 1);
}

bool NewEndEffectorTrajectory3D::compute(
    const Eigen::Ref<const Eigen::Vector3d>& start_pose,
    const Eigen::Ref<const Eigen::Vector3d>& current_pose,
    const Eigen::Ref<const Eigen::Vector3d>& current_velocity,
    const Eigen::Ref<const Eigen::Vector3d>& target_pose, const double &start_time,
    const double &current_time, const double &end_time,
    const bool& is_left_leg_in_contact) {
  // scaling the problem
  double step_duration = end_time - start_time - EPSILON;//To solve numeric problem
  double duration = end_time - current_time - EPSILON;//To solve numeric problem
  nb_sampling_time = ceil(step_duration / sampling_time);
  nb_local_sampling_time_ = ceil(duration / sampling_time);
  const int nb_mid_sampling_time = nb_local_sampling_time_ - (nb_sampling_time / 2);
  nb_var_ = 3 * nb_local_sampling_time_ + 7;
  nb_eq_ = 3 * 2 + 1 ;//xn=0,vn=0,x_(n/2)[2]=x_des
  nb_ineq_ = nb_local_sampling_time_ * (2/*z_min & z_max*/ + 3 * 2/*force_min & force_max*/);
  is_left_leg_in_contact_ = is_left_leg_in_contact;
  // save args:
  start_pose_ = start_pose;
  target_pose_ = target_pose;
  current_pose_ = current_pose;
  current_velocity_ = current_velocity;
  start_time_ = start_time;
  current_time_ = current_time;
  last_end_time_seen_ = end_time;

  resize_matrices();

  /*
   * Quadratic cost
   */
  // Q_x
  Eigen::MatrixXd sub_Q;
  sub_Q.resize(nb_local_sampling_time_, nb_local_sampling_time_);
  sub_Q.setZero();
  // Q_x
  for(int i = 0 ; i < nb_local_sampling_time_ ; i++){
      Q_(i, i) = cost_;
      // Q_y
      Q_(nb_local_sampling_time_ + i,  nb_local_sampling_time_ + i) = cost_;
      // Q_z
      Q_(2 * nb_local_sampling_time_ + i, 2 * nb_local_sampling_time_ + i) = cost_;
  }

  Q_(nb_var_ - 7, nb_var_ - 7) = 1 * cost_epsilon_z_mid_;
  Q_(nb_var_ - 6, nb_var_ - 6) = 1 * cost_epsilon_x_;
  Q_(nb_var_ - 5, nb_var_ - 5) = 1 * cost_epsilon_y_;
  Q_(nb_var_ - 4, nb_var_ - 4) = 1 * cost_epsilon_z_;
  Q_(nb_var_ - 3, nb_var_ - 3) = 1 * cost_epsilon_vel_;
  Q_(nb_var_ - 2, nb_var_ - 2) = 1 * cost_epsilon_vel_;
  Q_(nb_var_ - 1, nb_var_ - 1) = 1 * cost_epsilon_vel_ * 1000;
  // Q_regul
  double hess_regul = 1e-9;
  Q_regul_ = Eigen::MatrixXd::Identity(nb_var_, nb_var_) * hess_regul;
  Q_ += Q_regul_;

  /*
   * Equality constraints.
   */
  A_eq_.row(0).head(nb_local_sampling_time_) = acceleration_terms_x_[is_left_leg_in_contact_].row(nb_local_sampling_time_).head(nb_local_sampling_time_);//xn[0]
  A_eq_.row(0).segment(nb_local_sampling_time_, nb_local_sampling_time_) = acceleration_terms_x_[is_left_leg_in_contact_].row(nb_local_sampling_time_).segment(MAX_VAR, nb_local_sampling_time_);//xn[0]
  A_eq_.row(0).segment(2 * nb_local_sampling_time_, nb_local_sampling_time_) = acceleration_terms_x_[is_left_leg_in_contact_].row(nb_local_sampling_time_).segment(2 * MAX_VAR, nb_local_sampling_time_);//xn[0]
  A_eq_(0, nb_var_ - 6) = -1;

  A_eq_.row(1).head(nb_local_sampling_time_) = acceleration_terms_y_[is_left_leg_in_contact_].row(nb_local_sampling_time_).head(nb_local_sampling_time_);//xn[1]
  A_eq_.row(1).segment(nb_local_sampling_time_, nb_local_sampling_time_) = acceleration_terms_y_[is_left_leg_in_contact_].row(nb_local_sampling_time_).segment(MAX_VAR, nb_local_sampling_time_);//xn[1]
  A_eq_.row(1).segment(2 * nb_local_sampling_time_, nb_local_sampling_time_) = acceleration_terms_y_[is_left_leg_in_contact_].row(nb_local_sampling_time_).segment(2 * MAX_VAR, nb_local_sampling_time_);//xn[1]
  A_eq_(1, nb_var_ - 5) = -1;

  A_eq_.row(2).head(nb_local_sampling_time_) = acceleration_terms_z_[is_left_leg_in_contact_].row(nb_local_sampling_time_).head(nb_local_sampling_time_);//xn[2]
  A_eq_.row(2).segment(nb_local_sampling_time_, nb_local_sampling_time_) = acceleration_terms_z_[is_left_leg_in_contact_].row(nb_local_sampling_time_).segment(MAX_VAR, nb_local_sampling_time_);//xn[2]
  A_eq_.row(2).segment(2 * nb_local_sampling_time_, nb_local_sampling_time_) = acceleration_terms_z_[is_left_leg_in_contact_].row(nb_local_sampling_time_).segment(2 * MAX_VAR, nb_local_sampling_time_);//xn[2]
//  A_eq_(2, nb_var_ - 4) = -1;

  // V end constraints
  if(nb_local_sampling_time_ > 1){
      A_eq_.row(3).head(nb_local_sampling_time_) = acceleration_terms_x_[is_left_leg_in_contact_].row(nb_local_sampling_time_ - 1).head(nb_local_sampling_time_);//vn[0]
      A_eq_.row(3).segment(nb_local_sampling_time_, nb_local_sampling_time_) = acceleration_terms_x_[is_left_leg_in_contact_].row(nb_local_sampling_time_ - 1).segment(MAX_VAR, nb_local_sampling_time_);//vn[0]
      A_eq_.row(3).segment(2 * nb_local_sampling_time_, nb_local_sampling_time_) = acceleration_terms_x_[is_left_leg_in_contact_].row(nb_local_sampling_time_ - 1).segment(2 * MAX_VAR, nb_local_sampling_time_);//vn[0]
      A_eq_(3, nb_var_ - 6) = -1;
      A_eq_(3, nb_var_ - 3) = -1;

      A_eq_.row(4).head(nb_local_sampling_time_) = acceleration_terms_y_[is_left_leg_in_contact_].row(nb_local_sampling_time_ - 1).head(nb_local_sampling_time_);//vn[1]
      A_eq_.row(4).segment(nb_local_sampling_time_, nb_local_sampling_time_) = acceleration_terms_y_[is_left_leg_in_contact_].row(nb_local_sampling_time_ - 1).segment(MAX_VAR, nb_local_sampling_time_);//vn[1]
      A_eq_.row(4).segment(2 * nb_local_sampling_time_, nb_local_sampling_time_) = acceleration_terms_y_[is_left_leg_in_contact_].row(nb_local_sampling_time_ - 1).segment(2 * MAX_VAR, nb_local_sampling_time_);//vn[1]
      A_eq_(4, nb_var_ - 5) = -1;
      A_eq_(4, nb_var_ - 2) = -1;

      A_eq_.row(5).head(nb_local_sampling_time_) = acceleration_terms_z_[is_left_leg_in_contact_].row(nb_local_sampling_time_ - 1).head(nb_local_sampling_time_);//vn[2]
      A_eq_.row(5).segment(nb_local_sampling_time_, nb_local_sampling_time_) = acceleration_terms_z_[is_left_leg_in_contact_].row(nb_local_sampling_time_ - 1).segment(MAX_VAR, nb_local_sampling_time_);//vn[2]
      A_eq_.row(5).segment(2 * nb_local_sampling_time_, nb_local_sampling_time_) = acceleration_terms_z_[is_left_leg_in_contact_].row(nb_local_sampling_time_ - 1).segment(2 * MAX_VAR, nb_local_sampling_time_);//vn[2]
//      if(h[0] != 0.)//uncomment them if you want final velocity would not be equal to zero
//          A_eq_(5, nb_var_ - 1) = -1;
  }

  if(nb_mid_sampling_time >= 0) {
    A_eq_.row(6).head(nb_local_sampling_time_) = acceleration_terms_z_[is_left_leg_in_contact_].row(nb_mid_sampling_time).head(nb_local_sampling_time_);//xn/2[2]
    A_eq_.row(6).segment(nb_local_sampling_time_, nb_local_sampling_time_) = acceleration_terms_z_[is_left_leg_in_contact_].row(nb_mid_sampling_time).segment(MAX_VAR, nb_local_sampling_time_);//xn/2[2]
    A_eq_.row(6).segment(2 * nb_local_sampling_time_, nb_local_sampling_time_) = acceleration_terms_z_[is_left_leg_in_contact_].row(nb_mid_sampling_time).segment(2 * MAX_VAR, nb_local_sampling_time_);//xn/2[2]
  }
  A_eq_(6, nb_var_ - 7) = -1;

  bool flag = nb_local_sampling_time_ > 1;
  double mid_z = nb_mid_sampling_time > 0 ? - current_pose_(2) + mid_air_height_ - current_velocity_(2) * sampling_time * nb_mid_sampling_time + g[is_left_leg_in_contact_](nb_mid_sampling_time, 2): 0;
  B_eq_.head(7) << - current_pose_(0) + target_pose_(0) - current_velocity_(0) * sampling_time * nb_local_sampling_time_ + g[is_left_leg_in_contact_](nb_local_sampling_time_, 0),
                   - current_pose_(1) + target_pose_(1) - current_velocity_(1) * sampling_time * nb_local_sampling_time_ + g[is_left_leg_in_contact_](nb_local_sampling_time_, 1),
                   - current_pose_(2) + target_pose_(2) - current_velocity_(2) * sampling_time * nb_local_sampling_time_ + g[is_left_leg_in_contact_](nb_local_sampling_time_, 2),
                   flag * (- current_pose_(0) + target_pose_(0) - current_velocity_(0) * sampling_time * (nb_local_sampling_time_ - 1) + g[is_left_leg_in_contact_](nb_local_sampling_time_ - 1, 0)),
                   flag * (- current_pose_(1) + target_pose_(1) - current_velocity_(1) * sampling_time * (nb_local_sampling_time_ - 1) + g[is_left_leg_in_contact_](nb_local_sampling_time_ - 1, 1)),
                   flag * (- current_pose_(2) + target_pose_(2) - current_velocity_(2) * sampling_time * (nb_local_sampling_time_ - 1) + g[is_left_leg_in_contact_](nb_local_sampling_time_ - 1, 2)),
                   mid_z;

  for (int i = 1; i < nb_local_sampling_time_; ++i) {
      // z >= zmin   =>   -z <= -z_min
      A_ineq_.row(i - 1).head(nb_local_sampling_time_) = -acceleration_terms_z_[is_left_leg_in_contact_].row(i).head(nb_local_sampling_time_);
      A_ineq_.row(i - 1).segment(nb_local_sampling_time_, nb_local_sampling_time_) = -acceleration_terms_z_[is_left_leg_in_contact_].row(i).segment(MAX_VAR, nb_local_sampling_time_);
      A_ineq_.row(i - 1).segment(2 * nb_local_sampling_time_, nb_local_sampling_time_) = -acceleration_terms_z_[is_left_leg_in_contact_].row(i).segment(2 * MAX_VAR, nb_local_sampling_time_);
      B_ineq_(i - 1) = current_pose_(2) - std::min(start_pose(2), target_pose(2)) + 0.0001 + current_velocity_(2) * sampling_time * i + g[is_left_leg_in_contact_](i, 2);
      // z <= z_max, i <= n/2 || z_i <= z_i - 1, i > n/2
      if (i <= nb_mid_sampling_time || i >= nb_local_sampling_time_ - 1 || h[0] != 0.) {//Lhum TODO delete h[0] != 0.
          A_ineq_.row(i - 1 + nb_local_sampling_time_).head(nb_local_sampling_time_) = acceleration_terms_z_[is_left_leg_in_contact_].row(i).head(nb_local_sampling_time_);
          A_ineq_.row(i - 1 + nb_local_sampling_time_).segment(nb_local_sampling_time_, nb_local_sampling_time_) = acceleration_terms_z_[is_left_leg_in_contact_].row(i).segment(MAX_VAR, nb_local_sampling_time_);
          A_ineq_.row(i - 1 + nb_local_sampling_time_).segment(2 * nb_local_sampling_time_, nb_local_sampling_time_) = acceleration_terms_z_[is_left_leg_in_contact_].row(i).segment(2 * MAX_VAR, nb_local_sampling_time_);
          B_ineq_(i - 1 + nb_local_sampling_time_) = -current_pose_(2) + std::max(start_pose(2), target_pose(2)) + mid_air_height_ - current_velocity_(2) * sampling_time * i + g[is_left_leg_in_contact_](i, 2);//Lhum add 0.02 just for test!!
      } else {
          A_ineq_.row(i - 1 + nb_local_sampling_time_).head(nb_local_sampling_time_) = velocity_terms_z_[is_left_leg_in_contact_].row(i).head(nb_local_sampling_time_);
          A_ineq_.row(i - 1 + nb_local_sampling_time_).segment(nb_local_sampling_time_, nb_local_sampling_time_) = velocity_terms_z_[is_left_leg_in_contact_].row(i).segment(MAX_VAR, nb_local_sampling_time_);
          A_ineq_.row(i - 1 + nb_local_sampling_time_).segment(2 * nb_local_sampling_time_, nb_local_sampling_time_) = velocity_terms_z_[is_left_leg_in_contact_].row(i).segment(2 * MAX_VAR, nb_local_sampling_time_);
          B_ineq_(i - 1 + nb_local_sampling_time_) = -current_velocity_(2);// + g[is_left_leg_in_contact_](1, 2);// Lhum TODO G doesn't work here!
      }
  }

  for(int i = 0; i < nb_local_sampling_time_ * 3; i++){
    A_ineq_(nb_local_sampling_time_ * 2 + i, i) = 1;
    B_ineq_(nb_local_sampling_time_ * 2 + i) = 18;
    A_ineq_(nb_local_sampling_time_ * (2 + 3) + i, i) = -1;
    B_ineq_(nb_local_sampling_time_ * (2 + 3) + i) = 18;
  }
  if (!qp_solver_.solve(Q_, q_, A_eq_, B_eq_, A_ineq_, B_ineq_)) {
    std::string error = "NewEndEffectorTrajectory3D::compute(): "
                        "failed to solve the QP.";
    std::cout << RED << "Error: " << error << RESET << std::endl;
    // https://github.com/jrl-umi3218/eigen-quadprog/blob/master/src/QuadProg/c/solve.QP.compact.c#L94
    if (qp_solver_.fail() == 1) {
      std::cout << RED << "NewEndEffectorTrajectory3D::compute -> the minimization "
                   "problem has no "
                   "solution!"
                << RESET << std::endl;
    } else {
      std::cout << RED << "NewEndEffectorTrajectory3D::compute -> problems with decomposing D!"
                << RESET << std::endl;
    }
    return false;
  }
  return true;
}

int NewEndEffectorTrajectory3D::get_forces(
    Eigen::Ref<Eigen::VectorXd> forces, Eigen::Ref<Eigen::Vector3d> next_pose,
    Eigen::Ref<Eigen::Vector3d> next_velocity, Eigen::Ref<Eigen::Vector3d> next_acceleration) {
  if (current_time_ < start_time_ || current_time_ >= last_end_time_seen_ - 1e-4) {
    forces.setZero();
  }
  else {
    if(!qp_solver_.solve(Q_, q_, A_eq_, B_eq_, A_ineq_, B_ineq_)) {
        forces = forces.tail(forces.size() - 3);
    }
    else {
        // Extract the information from the solution.
        x_opt_.resize(nb_var_);
        x_opt_ = qp_solver_.result();
        for (int i = 0; i < nb_local_sampling_time_; i++) {
            forces(i * 3) = x_opt_[i];
            forces(i * 3 + 1) = x_opt_[nb_local_sampling_time_ + i];
            forces(i * 3 + 2) = x_opt_[2 * nb_local_sampling_time_ + i];
        }
    }
    slack_variables_ << x_opt_(nb_var_ - 6), x_opt_(nb_var_ - 5), x_opt_(nb_var_ - 4);
  }
  next_acceleration << (forces[0] - h[0]) * M_inv_[is_left_leg_in_contact_](0, 0) + (forces[1] - h[1]) * M_inv_[is_left_leg_in_contact_](1, 0) + (forces[2] - h[2]) * M_inv_[is_left_leg_in_contact_](2, 0),
                       (forces[0] - h[0]) * M_inv_[is_left_leg_in_contact_](0, 1) + (forces[1] - h[1]) * M_inv_[is_left_leg_in_contact_](1, 1) + (forces[2] - h[2]) * M_inv_[is_left_leg_in_contact_](2, 1),
                       (forces[0] - h[0]) * M_inv_[is_left_leg_in_contact_](0, 2) + (forces[1] - h[1]) * M_inv_[is_left_leg_in_contact_](1, 2) + (forces[2] - h[2]) * M_inv_[is_left_leg_in_contact_](2, 2);

  next_velocity << next_acceleration(0) * control_loop + current_velocity_(0),
                   next_acceleration(1) * control_loop + current_velocity_(1),
                   next_acceleration(2) * control_loop + current_velocity_(2);

  next_pose << 0.5 * next_acceleration(0) * control_loop * control_loop +
               current_velocity_(0) * control_loop + current_pose_(0),
               0.5 * next_acceleration(1) * control_loop * control_loop +
               current_velocity_(1) * control_loop + current_pose_(1),
               0.5 * next_acceleration(2) * control_loop * control_loop +
               current_velocity_(2) * control_loop + current_pose_(2);

  return nb_local_sampling_time_ * 3;
}

void NewEndEffectorTrajectory3D::update_robot_status(Eigen::Ref<Eigen::Vector3d> next_pose,
        Eigen::Ref<Eigen::Vector3d> next_velocity, Eigen::Ref<Eigen::Vector3d> next_acceleration) {
    if (current_time_ < start_time_) {
        return;
    }
    current_pose_ = next_pose;
    current_velocity_ = next_velocity;
    Eigen::Vector3d forces;
    forces << x_opt_[0], x_opt_[nb_local_sampling_time_], x_opt_[2 * nb_local_sampling_time_];

    next_acceleration << (forces[0] - h[0]) * M_inv_[is_left_leg_in_contact_](0, 0) + (forces[1] - h[1]) * M_inv_[is_left_leg_in_contact_](1, 0) + (forces[2] - h[2]) * M_inv_[is_left_leg_in_contact_](2, 0),
                         (forces[0] - h[0]) * M_inv_[is_left_leg_in_contact_](0, 1) + (forces[1] - h[1]) * M_inv_[is_left_leg_in_contact_](1, 1) + (forces[2] - h[2]) * M_inv_[is_left_leg_in_contact_](2, 1),
                         (forces[0] - h[0]) * M_inv_[is_left_leg_in_contact_](0, 2) + (forces[1] - h[1]) * M_inv_[is_left_leg_in_contact_](1, 2) + (forces[2] - h[2]) * M_inv_[is_left_leg_in_contact_](2, 2);

    next_velocity << next_acceleration(0) * control_loop + current_velocity_(0),
            next_acceleration(1) * control_loop + current_velocity_(1),
            next_acceleration(2) * control_loop + current_velocity_(2);

    next_pose << 0.5 * next_acceleration(0) * control_loop * control_loop +
                 current_velocity_(0) * control_loop + current_pose_(0),
            0.5 * next_acceleration(1) * control_loop * control_loop +
            current_velocity_(1) * control_loop + current_pose_(1),
            0.5 * next_acceleration(2) * control_loop * control_loop +
            current_velocity_(2) * control_loop + current_pose_(2);
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
