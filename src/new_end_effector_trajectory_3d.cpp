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
//    cost_ = 1;
//    cost_epsilon_z_mid_ = 1e5;
//    cost_epsilon_x_ = 1e9;
//    cost_epsilon_y_ = 1e9;
//    cost_epsilon_z_ = 1e9;
//    cost_epsilon_vel_ = 1e9;
//    cost_dcm_epsilon_x_i_ = 1;
//    cost_dcm_epsilon_y_i_ = 1;
//    cost_epsilon_x_i_ = 1e3;

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
  sampling_time = 0.010;
  control_loop =0.001;

  // QP parameter.
  nb_sampling_time = 10;
  nb_var_ = 3 * nb_sampling_time + 7;
  nb_var_axis_ = nb_sampling_time;
  // number of sampling time nodes for the z_min < z(t) < z_max.
  nb_ineq_ = nb_sampling_time * 2 + (nb_var_ - 7) * 2;
  // current and final conditions
  nb_eq_ = 3 * 2 + 2;//xn=0,vn=0,an[2]=0,xn/2[2]=xd
  resize_matrices();

  /*
   * variables: 4 * nb_local_sampling_time - 1
   * F_x[0]  ... F_x[nb_local_sampling_time - 1] F_y[0] ... F_y[nb_local_sampling_time - 1] F_z[0] ... F_z[nb_local_sampling_time - 1]
   * slack variables: 7 + 2 * (nb_local_sampling_time - start_mid + 1)
   * x[0] ... x[nb_local_sampling_time - 2]\.
   * x[nb_local_sampling_time - 1] y[nb_local_sampling_time - 1] ... x[start_mid] y[start_mid] //DCM \.
   * mid_z x[nb_local_sampling_time - 1] y[nb_local_sampling_time - 1] z[nb_local_sampling_time - 1]\.
   * v_x[nb_local_sampling_time - 1] v_y[nb_local_sampling_time - 1] v_z[nb_local_sampling_time - 1]
   * // Lhum: Delete two first slack variables
   */
  x_opt_.resize(nb_var_);
  x_opt_.setZero();
  /*
   * equation:
   * x y z v_x v_y v_z a_z z_mid \.
   * x_dcm[start_mid] y_dcm[start_mid] ... x_dcm[nb_local_sampling_time - 1] y_dcm[nb_local_sampling_time - 1]\.
   * x[0] ... x[nb_local_sampling_time - 2]  //x_des
   *
   * inequation:
   * z_min[1] ... z_min[nb_local_sampling_time - 1] z_max[1] ... z_max[nb_local_sampling_time - 1] \.
   * F_x_min[0] F_y_min[0] F_z_min[0] ... \.
   * F_x_min[nb_local_sampling_time - 1] F_y_min[nb_local_sampling_time - 1] F_z_min[nb_local_sampling_time - 1] \.
   * F_x_max[0] F_y_max[0] F_z_max[0] ... \.
   * F_x_max[nb_local_sampling_time - 1] F_y_max[nb_local_sampling_time - 1] F_z_max[nb_local_sampling_time - 1]
   *
   */


  //Lhum TODO update M_inv_ with inertia
  M_inv_  = new Eigen::MatrixXd[2];
  M_inv_[0].resize(3, 3);
  M_inv_[1].resize(3, 3);
//    M_inv_ << 0.06, 0.0, 0.03,
//            0.0, 0.05, 0.01,
//            0.03, 0.01, 0.05;
//  M_inv_[0] << 0.045, 0.005, 0.043,
//           0.005, 0.045, 0.01,
//           0.043, 0.01, 0.09;//Left_swing
//  M_inv_[1] << 0.045, -0.008, 0.045,
//           -0.008, 0.042, -0.007,
//           0.045, -0.007, 0.09;//Right_swing
  M_inv_[0] << 0.045, 0.0, 0.0,
           0.0, 0.045, 0.0,
           0.0, 0.0, 0.09;//Left_swing
  M_inv_[1] << 0.045, -0.0, 0.0,
           -0.0, 0.042, -0.0,
           0.0, -0.0, 0.09;//Right_swing
//  M_inv_ << 0.045, -0.0, 0.0,
//           -0.0, 0.042, 0.0,
//           0.0, 0.0, 0.065;
  M_inv_[0] = M_inv_[0].inverse();
  M_inv_[1] = M_inv_[1].inverse();

  v_des_.setZero();

//  Q_regul_ = Eigen::MatrixXd::Identity(nb_var_, nb_var_) * hess_regul;
    calculate_acceleration();
}

NewEndEffectorTrajectory3D::~NewEndEffectorTrajectory3D() {}

void NewEndEffectorTrajectory3D::calculate_acceleration(){
  acceleration_terms_x_  = new Eigen::MatrixXd[2];
  acceleration_terms_y_  = new Eigen::MatrixXd[2];
  acceleration_terms_z_  = new Eigen::MatrixXd[2];
  velocity_terms_x_  = new Eigen::MatrixXd[2];
  velocity_terms_y_  = new Eigen::MatrixXd[2];
  velocity_terms_z_  = new Eigen::MatrixXd[2];
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

              acceleration_terms_y_[k](i, j) =
                      x(2, 0) * M_inv_[k](0, 0) + x(2, 1) * M_inv_[k](1, 0) + x(2, 2) * M_inv_[k](2, 0);
              acceleration_terms_y_[k](i, j + MAX_VAR) =
                      x(2, 0) * M_inv_[k](0, 1) + x(2, 1) * M_inv_[k](1, 1) + x(2, 2) * M_inv_[k](2, 1);
              acceleration_terms_y_[k](i, j + 2 * MAX_VAR) =
                      x(2, 0) * M_inv_[k](0, 2) + x(2, 1) * M_inv_[k](1, 2) + x(2, 2) * M_inv_[k](2, 2);

              acceleration_terms_z_[k](i, j) =
                      x(4, 0) * M_inv_[k](0, 0) + x(4, 1) * M_inv_[k](1, 0) + x(4, 2) * M_inv_[k](2, 0);
              acceleration_terms_z_[k](i, j + MAX_VAR) =
                      x(4, 0) * M_inv_[k](0, 1) + x(4, 1) * M_inv_[k](1, 1) + x(4, 2) * M_inv_[k](2, 1);
              acceleration_terms_z_[k](i, j + 2 * MAX_VAR) =
                      x(4, 0) * M_inv_[k](0, 2) + x(4, 1) * M_inv_[k](1, 2) + x(4, 2) * M_inv_[k](2, 2);

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

void NewEndEffectorTrajectory3D::init_calculate_dcm(
    Eigen::Ref<const Eigen::Vector3d> v_des,
    const double& ht,
    const double& l_p,
    const double& t_lower_bound,
    const double& t_upper_bound){
  ht_ = ht;
  v_des_ = v_des;
  l_p_ = l_p;
  t_nom_ = (t_lower_bound + t_upper_bound) * 0.5;
  omega_ = sqrt(9.81 / ht_);
  tau_nom_ = exp(omega_ * t_nom_);
  l_nom_ = v_des_(0) * t_nom_;
  bx_nom_ = l_nom_ / (tau_nom_ - 1);
}

void NewEndEffectorTrajectory3D::calculate_dcm(
        Eigen::Ref<const Eigen::Vector3d> com,
        Eigen::Ref<const Eigen::Vector3d> com_vel,
        Eigen::Ref<const Eigen::Vector3d> current_support_foot_location,
        const double& time,
        const double& current_time,
        const bool& is_left_leg_in_contact){
  t_nom_ = time;
  tau_nom_ = exp(omega_ * t_nom_);
  l_nom_ = v_des_(0) * t_nom_;
  bx_nom_ = l_nom_ / (tau_nom_ - 1);

  double contact_switcher = is_left_leg_in_contact ? 2.0 : 1.0;
  by_nom_ = (pow(-1, contact_switcher) * (l_p_ / (1 + tau_nom_))) - v_des_(1) * t_nom_ / (1 - tau_nom_);

  Eigen::Vector3d dcm_local;
  dcm_local.head<2>() = com_vel.head<2>() / omega_ + com.head<2>();
  dcm_local(2) = 0.0;//non local

  double ux = ((dcm_local(0) - current_support_foot_location[0]) *
               exp(-1.0 * omega_ * current_time)) * tau_nom_ - bx_nom_ + current_support_foot_location[0];
  double uy = ((dcm_local(1) - current_support_foot_location[1]) *
               exp(-1.0 * omega_ * current_time)) * tau_nom_ - by_nom_ + current_support_foot_location[1];

  u_ << ux, uy, 0;
//  std::cout << "bx " << bx_nom_ << " " << by_nom_ << std::endl;
//  std::cout << "ux" << u_ << std::endl;
}

bool NewEndEffectorTrajectory3D::compute(
    Eigen::Ref<const Eigen::Vector3d> start_pose,
    Eigen::Ref<const Eigen::Vector3d> current_pose,
    Eigen::Ref<const Eigen::Vector3d> current_velocity,
    Eigen::Ref<const Eigen::Vector3d> current_acceleration,
    Eigen::Ref<const Eigen::Vector3d> target_pose, const double &start_time,
    const double &current_time, const double &end_time,
    Eigen::Ref<const Eigen::Vector3d> com_pos,
    Eigen::Ref<const Eigen::Vector3d> com_vel,
    Eigen::Ref<const Eigen::Vector3d> current_support_foot_location,
    const bool& is_left_leg_in_contact) {
  // scaling the problem
  double step_duration = end_time - start_time;
  double duration = end_time - current_time;
  nb_sampling_time = ceil(step_duration / sampling_time);
  double nb_local_sampling_time = ceil(duration / sampling_time);
//  nb_sampling_time = round((step_duration + EPSION) / sampling_time); //Lhum it is equal to ceil and it avoids floating point problem.
//  double nb_local_sampling_time = round((duration + EPSION) / sampling_time);
  int nb_mid_sampling_time = nb_local_sampling_time - (nb_sampling_time / 2);
  int start_mid = std::max(0, nb_mid_sampling_time + 1);
  nb_var_ = 3 * nb_local_sampling_time + 7 + 2 * (nb_local_sampling_time - start_mid + 1) + (nb_local_sampling_time - 1) * 2;
  nb_var_axis_ = nb_local_sampling_time;
  nb_eq_ = 3 * 2 + 2 + (nb_local_sampling_time - start_mid + 1) * 2 + (nb_local_sampling_time - 1) * 2;//+ 3;//xn=0,vn=0,an[2]=0,xn/2[2]=xd,F_x[n]=0,f_y[n]=0,f_z[n]=0
  nb_ineq_ = nb_local_sampling_time * 2 + nb_local_sampling_time * 3 * 2;
  is_left_leg_in_contact_ = is_left_leg_in_contact;
//  std::cout << "Lhum cost " << step_duration << " " << duration << " " << sampling_time << " " << nb_sampling_time << " " <<
//            nb_sampling_time << " " << nb_local_sampling_time << " " << nb_mid_sampling_time << " " << nb_var_ << " " << nb_ineq_ << std::endl;
//  std::cout << "Lhum step_duration " << step_duration <<
//               "\nLhum duration " << duration <<
//               "\nLhum sampling_time " << sampling_time <<
//               "\nLhum nb_sampling_time " << nb_sampling_time <<
//               "\nLhum nb_local_sampling_time " << nb_local_sampling_time <<
//               "\nLhum nb_mid_sampling_time " <<  nb_mid_sampling_time <<
//               "\nLhum nb_var_ " <<  nb_var_ <<
//               "\nLhum nb_ineq_ " <<  nb_ineq_ << std::endl;
  // save args:
  start_pose_ = start_pose;
  current_pose_ = current_pose;
  current_velocity_ = current_velocity;
//  current_acceleration_ = current_acceleration;
  target_pose_ = target_pose;
//  std::cout << "Lhum traget " << target_pose << std::endl;
//  target_pose_(2) = -0.03;// Lhum Uncomment this line when you run with dcm equations
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
  // Q_x
  for(int i = 0 ; i < nb_var_axis_ ; i++){
      Q_(i, i) = cost_;
//      std::cout << 2. * cost_ * i / nb_var_axis_ << std::endl;
      // Q_y
      Q_(nb_var_axis_ + i,  nb_var_axis_ + i) = cost_;
      // Q_z
      Q_(2 * nb_var_axis_ + i, 2 * nb_var_axis_ + i) = cost_;
  }
//  Q_.block(0, 0, nb_var_axis_, nb_var_axis_) = sub_Q;
//  Q_.block(nb_var_axis_, nb_var_axis_, nb_var_axis_, nb_var_axis_) = sub_Q;
//  Q_.block(nb_var_axis_ * 2, nb_var_axis_ * 2, nb_var_axis_, nb_var_axis_) = sub_Q;
  // Q_epsilon
  for(int i = 0; i < nb_local_sampling_time; i++) {
      Q_(3 * nb_var_axis_ + i, 3 * nb_var_axis_ + i) = cost_epsilon_x_i_ * 1;
      Q_(3 * nb_var_axis_ + i + nb_local_sampling_time - 1, 3 * nb_var_axis_ + i + nb_local_sampling_time - 1) = cost_epsilon_y_i_ * 1;
  }
  for(int i = start_mid; i <= nb_local_sampling_time; i++){
//    std::cout << "Q" << nb_var_ - 2 * (i - start_mid) - 8 << " " << nb_var_ - 2 * (i - start_mid) - 8 - 1 << std::endl;
    Q_(nb_var_ - 2 * (i - start_mid) - 8, nb_var_ - 2 * (i - start_mid) - 8) = cost_dcm_epsilon_x_i_;
    Q_(nb_var_ - 2 * (i - start_mid) - 1 - 8, nb_var_ - 2 * (i - start_mid) - 1 - 8) = cost_dcm_epsilon_y_i_;
  }
  Q_(nb_var_ - 7, nb_var_ - 7) = 1 * cost_epsilon_z_mid_;
  Q_(nb_var_ - 6, nb_var_ - 6) = 1 * cost_epsilon_x_;
  Q_(nb_var_ - 5, nb_var_ - 5) = 1 * cost_epsilon_y_;
  Q_(nb_var_ - 4, nb_var_ - 4) = 1 * cost_epsilon_z_;
  Q_(nb_var_ - 3, nb_var_ - 3) = 1 * cost_epsilon_vel_;
  Q_(nb_var_ - 2, nb_var_ - 2) = 1 * cost_epsilon_vel_;
  Q_(nb_var_ - 1, nb_var_ - 1) = 1 * cost_epsilon_vel_;
  // Q_regul
  double hess_regul = 1e-9;
  Q_regul_ = Eigen::MatrixXd::Identity(nb_var_, nb_var_) * hess_regul;
  Q_ += Q_regul_;

  /*
   * Equality constraints.
   */
  // clang-format off
  A_eq_.setZero();
//  Eigen::VectorXd acc;
//  Eigen::VectorXd acc_ep;
//  acc.resize(nb_var_ - 7);
//  acc.setZero();
//  acc_ep.resize(nb_var_);
//  acc_ep.setZero();
  // X end constraints
//  Eigen::MatrixXd x;
//  x.resize(6, 3);
//  x = B_;
//  for(int i = nb_local_sampling_time - 1 ; i >= 0; i--){
//    acc_ep[i] = x(0, 0) * M_inv_(0, 0) + x(0, 1) * M_inv_(1, 0) + x(0, 2) * M_inv_(2, 0);
//    acc_ep[i + nb_var_axis_] = x(0, 0) * M_inv_(0, 1) + x(0, 1) * M_inv_(1, 1) + x(0, 2) * M_inv_(2, 1);
//    acc_ep[i + 2 * nb_var_axis_] = x(0, 0) * M_inv_(0, 2) + x(0, 1) * M_inv_(1, 2) + x(0, 2) * M_inv_(2, 2);
//    x = A_ * x;
//  }
//  acc_ep[nb_var_ - 6] = -1;//epsilon_x
//  A_eq_.row(0).head(nb_var_) = acc_ep;//xn[0]
  A_eq_.row(0).head(nb_var_axis_) = acceleration_terms_x_[is_left_leg_in_contact_].row(nb_local_sampling_time).head(nb_var_axis_);//xn[0]
  A_eq_.row(0).segment(nb_var_axis_, nb_var_axis_) = acceleration_terms_x_[is_left_leg_in_contact_].row(nb_local_sampling_time).segment(MAX_VAR, nb_var_axis_);//xn[0]
  A_eq_.row(0).segment(2 * nb_var_axis_, nb_var_axis_) = acceleration_terms_x_[is_left_leg_in_contact_].row(nb_local_sampling_time).segment(2 * MAX_VAR, nb_var_axis_);//xn[0]
  A_eq_(0, nb_var_ - 6) = -1;
//  acc_ep[nb_var_ - 6] = 0;//delete epsilon_x
//  x = B_;
//  for(int i = nb_local_sampling_time - 1 ; i >= 0; i--){
//    acc_ep[i] = x(2, 0) * M_inv_(0, 0) + x(2, 1) * M_inv_(1, 0) + x(2, 2) * M_inv_(2, 0);
//    acc_ep[i + nb_var_axis_] = x(2, 0) * M_inv_(0, 1) + x(2, 1) * M_inv_(1, 1) + x(2, 2) * M_inv_(2, 1);
//    acc_ep[i + 2 * nb_var_axis_] = x(2, 0) * M_inv_(0, 2) + x(2, 1) * M_inv_(1, 2) + x(2, 2) * M_inv_(2, 2);
//    x = A_ * x;
//  }
//  acc_ep[nb_var_ - 5] = -1;//epsilon_y
//  A_eq_.row(1).head(nb_var_) = acc_ep;//xn[1]
  A_eq_.row(1).head(nb_var_axis_) = acceleration_terms_y_[is_left_leg_in_contact_].row(nb_local_sampling_time).head(nb_var_axis_);//xn[1]
  A_eq_.row(1).segment(nb_var_axis_, nb_var_axis_) = acceleration_terms_y_[is_left_leg_in_contact_].row(nb_local_sampling_time).segment(MAX_VAR, nb_var_axis_);//xn[1]
  A_eq_.row(1).segment(2 * nb_var_axis_, nb_var_axis_) = acceleration_terms_y_[is_left_leg_in_contact_].row(nb_local_sampling_time).segment(2 * MAX_VAR, nb_var_axis_);//xn[1]
  A_eq_(1, nb_var_ - 5) = -1;
//  acc_ep[nb_var_ - 5] = 0;//delete epsilon_y
//  x = B_;
//  for(int i = nb_local_sampling_time - 1 ; i >= 0; i--){
//    acc_ep[i] = x(4, 0) * M_inv_(0, 0) + x(4, 1) * M_inv_(1, 0) + x(4, 2) * M_inv_(2, 0);
//    acc_ep[i + nb_var_axis_] = x(4, 0) * M_inv_(0, 1) + x(4, 1) * M_inv_(1, 1) + x(4, 2) * M_inv_(2, 1);
//    acc_ep[i + 2 * nb_var_axis_] = x(4, 0) * M_inv_(0, 2) + x(4, 1) * M_inv_(1, 2) + x(4, 2) * M_inv_(2, 2);
//    x = A_ * x;
//  }
//  acc_ep[nb_var_ - 4] = -1;//epsilon_z
  A_eq_.row(2).head(nb_var_axis_) = acceleration_terms_z_[is_left_leg_in_contact_].row(nb_local_sampling_time).head(nb_var_axis_);//xn[2]
  A_eq_.row(2).segment(nb_var_axis_, nb_var_axis_) = acceleration_terms_z_[is_left_leg_in_contact_].row(nb_local_sampling_time).segment(MAX_VAR, nb_var_axis_);//xn[2]
  A_eq_.row(2).segment(2 * nb_var_axis_, nb_var_axis_) = acceleration_terms_z_[is_left_leg_in_contact_].row(nb_local_sampling_time).segment(2 * MAX_VAR, nb_var_axis_);//xn[2]
//  A_eq_(2, nb_var_ - 4) = -1;
//  acc_ep[nb_var_ - 4] = 0;//delete epsilon_z

  // V end constraints
  if(nb_local_sampling_time > 1){
//      std::cout << "here " << std::endl;
//      for(int i = 0; i < nb_var_; i++)
//          acc_ep[i] = 0;
//      x = B_;
//      for(int i = nb_local_sampling_time - 2 ; i >= 0; i--){
//        acc_ep[i] = x(0, 0) * M_inv_(0, 0) + x(0, 1) * M_inv_(1, 0) + x(0, 2) * M_inv_(2, 0);
//        acc_ep[i + nb_var_axis_] = x(0, 0) * M_inv_(0, 1) + x(0, 1) * M_inv_(1, 1) + x(0, 2) * M_inv_(2, 1);
//        acc_ep[i + 2 * nb_var_axis_] = x(0, 0) * M_inv_(0, 2) + x(0, 1) * M_inv_(1, 2) + x(0, 2) * M_inv_(2, 2);
//        x = A_ * x;
//      }
//      acc_ep[nb_var_ - 3] = -1;//epsilon_vel
//      A_eq_.row(3).head(nb_var_) = acc_ep;//vn[0]
      A_eq_.row(3).head(nb_var_axis_) = acceleration_terms_x_[is_left_leg_in_contact_].row(nb_local_sampling_time - 1).head(nb_var_axis_);//vn[0]
      A_eq_.row(3).segment(nb_var_axis_, nb_var_axis_) = acceleration_terms_x_[is_left_leg_in_contact_].row(nb_local_sampling_time - 1).segment(MAX_VAR, nb_var_axis_);//vn[0]
      A_eq_.row(3).segment(2 * nb_var_axis_, nb_var_axis_) = acceleration_terms_x_[is_left_leg_in_contact_].row(nb_local_sampling_time - 1).segment(2 * MAX_VAR, nb_var_axis_);//vn[0]
      A_eq_(3, nb_var_ - 6) = -1;
      A_eq_(3, nb_var_ - 3) = -1;
//      acc_ep[nb_var_ - 3] = 0;//delete epsilon_vel
//      x = B_;
//      for(int i = nb_local_sampling_time - 2 ; i >= 0; i--){
//        acc_ep[i] = x(2, 0) * M_inv_(0, 0) + x(2, 1) * M_inv_(1, 0) + x(2, 2) * M_inv_(2, 0);
//        acc_ep[i + nb_var_axis_] = x(2, 0) * M_inv_(0, 1) + x(2, 1) * M_inv_(1, 1) + x(2, 2) * M_inv_(2, 1);
//        acc_ep[i + 2 * nb_var_axis_] = x(2, 0) * M_inv_(0, 2) + x(2, 1) * M_inv_(1, 2) + x(2, 2) * M_inv_(2, 2);
//        x = A_ * x;
//      }
//      acc_ep[nb_var_ - 2] = -1;//epsilon_vel
//      A_eq_.row(4).head(nb_var_) = acc_ep;//vn[1]
      A_eq_.row(4).head(nb_var_axis_) = acceleration_terms_y_[is_left_leg_in_contact_].row(nb_local_sampling_time - 1).head(nb_var_axis_);//vn[1]
      A_eq_.row(4).segment(nb_var_axis_, nb_var_axis_) = acceleration_terms_y_[is_left_leg_in_contact_].row(nb_local_sampling_time - 1).segment(MAX_VAR, nb_var_axis_);//vn[1]
      A_eq_.row(4).segment(2 * nb_var_axis_, nb_var_axis_) = acceleration_terms_y_[is_left_leg_in_contact_].row(nb_local_sampling_time - 1).segment(2 * MAX_VAR, nb_var_axis_);//vn[1]
      A_eq_(4, nb_var_ - 5) = -1;
      A_eq_(4, nb_var_ - 2) = -1;
//      acc_ep[nb_var_ - 2] = 0;//delete epsilon_vel
//      x = B_;
//      for(int i = nb_local_sampling_time - 2 ; i >= 0; i--){
//        acc_ep[i] = x(4, 0) * M_inv_(0, 0) + x(4, 1) * M_inv_(1, 0) + x(4, 2) * M_inv_(2, 0);
//        acc_ep[i + nb_var_axis_] = x(4, 0) * M_inv_(0, 1) + x(4, 1) * M_inv_(1, 1) + x(4, 2) * M_inv_(2, 1);
//        acc_ep[i + 2 * nb_var_axis_] = x(4, 0) * M_inv_(0, 2) + x(4, 1) * M_inv_(1, 2) + x(4, 2) * M_inv_(2, 2);
//        x = A_ * x;
//      }
//      acc_ep[nb_var_ - 1] = -1;//epsilon_vel
//      A_eq_.row(5).head(nb_var_) = acc_ep;//vn[2]
      A_eq_.row(5).head(nb_var_axis_) = acceleration_terms_z_[is_left_leg_in_contact_].row(nb_local_sampling_time - 1).head(nb_var_axis_);//vn[2]
      A_eq_.row(5).segment(nb_var_axis_, nb_var_axis_) = acceleration_terms_z_[is_left_leg_in_contact_].row(nb_local_sampling_time - 1).segment(MAX_VAR, nb_var_axis_);//vn[2]
      A_eq_.row(5).segment(2 * nb_var_axis_, nb_var_axis_) = acceleration_terms_z_[is_left_leg_in_contact_].row(nb_local_sampling_time - 1).segment(2 * MAX_VAR, nb_var_axis_);//vn[2]
//      A_eq_(5, nb_var_ - 4) = -1;
//      acc_ep[nb_var_ - 1] = 0;//delete epsilon_vel
  }

//  for(int i = 0; i < nb_var_ - 7; i++)
//    acc[i] = 0;
//  acc[nb_local_sampling_time - 1] = M_inv_(0, 2);
//  acc[nb_local_sampling_time - 1 + nb_var_axis_] = M_inv_(1, 2);
//  acc[nb_local_sampling_time - 1 + 2 * nb_var_axis_] = M_inv_(2, 2);
//  A_eq_.row(6).head(nb_var_ - 7) = acc;//an[2]

//  for(int i = 0; i < nb_var_; i++)
//    acc_ep[i] = 0;
//  x = B_;
//  for(int i = nb_mid_sampling_time - 1 ; i >= 0; i--){
//    acc_ep[i] = x(4, 0) * M_inv_(0, 0) + x(4, 1) * M_inv_(1, 0) + x(4, 2) * M_inv_(2, 0);
//    acc_ep[i + nb_var_axis_] = x(4, 0) * M_inv_(0, 1) + x(4, 1) * M_inv_(1, 1) + x(4, 2) * M_inv_(2, 1);
//    acc_ep[i + 2 * nb_var_axis_] = x(4, 0) * M_inv_(0, 2) + x(4, 1) * M_inv_(1, 2) + x(4, 2) * M_inv_(2, 2);
//    x = A_ * x;
//  }
//  acc_ep[nb_var_ - 7] = -1;//epsilon_z_mid
//  A_eq_.row(7).head(nb_var_) = acc_ep;//xn/2[2]
  if(nb_mid_sampling_time >= 0) {
    A_eq_.row(7).head(nb_var_axis_) = acceleration_terms_z_[is_left_leg_in_contact_].row(nb_mid_sampling_time).head(nb_var_axis_);//xn/2[2]
    A_eq_.row(7).segment(nb_var_axis_, nb_var_axis_) = acceleration_terms_z_[is_left_leg_in_contact_].row(nb_mid_sampling_time).segment(MAX_VAR, nb_var_axis_);//xn/2[2]
    A_eq_.row(7).segment(2 * nb_var_axis_, nb_var_axis_) = acceleration_terms_z_[is_left_leg_in_contact_].row(nb_mid_sampling_time).segment(2 * MAX_VAR, nb_var_axis_);//xn/2[2]
  }
  A_eq_(7, nb_var_ - 7) = -1;
//  acc_ep[nb_var_ - 7] = 0;//epsilon_z_mid

  int flag = nb_local_sampling_time > 1 ? 1: 0;
  double mid_z = nb_mid_sampling_time > 0 ? - current_pose_(2) + mid_air_height_ - current_velocity_(2) * sampling_time * nb_mid_sampling_time: 0;
  B_eq_.setZero();
//  std::cout << "\nCurrent_pose " << current_pose_ << "\ntarget_pose " <<  target_pose_ << "\ncurrent_velocity " << current_velocity_ <<
//               "\nsampling_time " <<  sampling_time << "\nnb_local_sampling_time " << nb_local_sampling_time << std::endl;
  B_eq_.head(8) << - current_pose_(0) + target_pose_(0) - current_velocity_(0) * sampling_time * nb_local_sampling_time,
                - current_pose_(1) + target_pose_(1) - current_velocity_(1) * sampling_time * nb_local_sampling_time,
                - current_pose_(2) + target_pose_(2) - current_velocity_(2) * sampling_time * nb_local_sampling_time,
                flag * (- current_pose_(0) + target_pose_(0) - current_velocity_(0) * sampling_time * (nb_local_sampling_time - 1)),
                flag * (- current_pose_(1) + target_pose_(1) - current_velocity_(1) * sampling_time * (nb_local_sampling_time - 1)),
                flag * (- current_pose_(2) + target_pose_(2) - current_velocity_(2) * sampling_time * (nb_local_sampling_time - 1)),
                0.0,
                mid_z;

  // Use DCM from half of the step
//  for(int i = 0; i < nb_var_; i++)
//    acc_ep[i] = 0;
//    std::cout << "____________\n";

//  for (int i = start_mid; i < nb_local_sampling_time - 1 ; ++i) {
////      std::cout << "i " << i << std::endl;
//    calculate_dcm(com_pos, com_vel, current_support_foot_location, i / 1000., current_time_, is_left_leg_in_contact);
////    std::cout <<"all u" << i << " " << u_[0] << " " << u_[1] << " " << target_pose[0] << " " << target_pose[1] << std::endl;
////    x = B_;
////    for (int j = i - 1; j >= 0; j--) {
////      acc_ep[j] = x(0, 0) * M_inv_(0, 0) + x(0, 1) * M_inv_(1, 0) + x(0, 2) * M_inv_(2, 0);
////      acc_ep[j + nb_var_axis_] = x(0, 0) * M_inv_(0, 1) + x(0, 1) * M_inv_(1, 1) + x(0, 2) * M_inv_(2, 1);
////      acc_ep[j + 2 * nb_var_axis_] = x(0, 0) * M_inv_(0, 2) + x(0, 1) * M_inv_(1, 2) + x(0, 2) * M_inv_(2, 2);
////      x = A_ * x;
////    }
////    acc_ep[nb_var_ - 2 * (i - start_mid) - 8] = -1;//epsilon_x_i
////    A_eq_.row(8 + (i - start_mid) * 2).head(nb_var_) = acc_ep;
//    A_eq_.row(8 + (i - start_mid) * 2).head(nb_var_axis_) = acceleration_terms_x_.row(i).head(nb_var_axis_);//xi[0]
//    A_eq_.row(8 + (i - start_mid) * 2).segment(nb_var_axis_, nb_var_axis_) = acceleration_terms_x_.row(i).segment(MAX_VAR, nb_var_axis_);//xi[0]
//    A_eq_.row(8 + (i - start_mid) * 2).segment(2 * nb_var_axis_, nb_var_axis_) = acceleration_terms_x_.row(i).segment(2 * MAX_VAR, nb_var_axis_);//xi[0]
//    A_eq_(8 + (i - start_mid) * 2, nb_var_ - 2 * (i - start_mid) - 8) = -1;
////    acc_ep[nb_var_ - 2 * (i - start_mid) - 8] = 0;//epsilon_x_i
//    B_eq_(8 + (i - start_mid)  * 2) = - current_pose_(0) + u_(0) - current_velocity_(0) * sampling_time * i;
//
////    x = B_;
////    for (int j = i - 1; j >= 0; j--) {
////      acc_ep[j] = x(2, 0) * M_inv_(0, 0) + x(2, 1) * M_inv_(1, 0) + x(2, 2) * M_inv_(2, 0);
////      acc_ep[j + nb_var_axis_] = x(2, 0) * M_inv_(0, 1) + x(2, 1) * M_inv_(1, 1) + x(2, 2) * M_inv_(2, 1);
////      acc_ep[j + 2 * nb_var_axis_] = x(2, 0) * M_inv_(0, 2) + x(2, 1) * M_inv_(1, 2) + x(2, 2) * M_inv_(2, 2);
////      x = A_ * x;
////    }
////    acc_ep[nb_var_ - 2 * (i - start_mid) - 1 - 8] = -1;//epsilon_y_i
////    A_eq_.row(8 + (i - start_mid) * 2 + 1).head(nb_var_) = acc_ep;
//    A_eq_.row(8 + (i - start_mid) * 2 + 1).head(nb_var_axis_) = acceleration_terms_y_.row(i).head(nb_var_axis_);//xi[1]
//    A_eq_.row(8 + (i - start_mid) * 2 + 1).segment(nb_var_axis_, nb_var_axis_) = acceleration_terms_y_.row(i).segment(MAX_VAR, nb_var_axis_);//xi[1]
//    A_eq_.row(8 + (i - start_mid) * 2 + 1).segment(2 * nb_var_axis_, nb_var_axis_) = acceleration_terms_y_.row(i).segment(2 * MAX_VAR, nb_var_axis_);//xi[1]
////    acc_ep[nb_var_ - 2 * (i - start_mid) - 1 - 8] = 0;//epsilon_y_i
//    A_eq_(8 + (i - start_mid) * 2 + 1, nb_var_ - 2 * (i - start_mid) - 1 - 8) = -1;
//    B_eq_(8 + (i - start_mid) * 2 + 1) = - current_pose_(1) + u_(1) - current_velocity_(1) * sampling_time * i;
////    std::cout << nb_var_ - 2 * (i - start_mid) - 1 - 8 << "       " << nb_var_ - 2 * (i - start_mid) - 8 << std::endl;
//  }
//    std::cout << "____________\n";
//  calculate_dcm(com_pos, com_vel, current_support_foot_location, end_time, current_time_, is_left_leg_in_contact);
//  std::cout << nb_var_ - 7 << " " << nb_var_axis_ * 3 << std::endl;
//  std::cout <<"final u" << " " << u_[0] << " " << u_[1] << " " << target_pose[0] << " " << target_pose[1] << std::endl;

//des_x&y cost
//  for (int i = 0; i < nb_local_sampling_time - 1 ; ++i) {//x[i]cost
//      A_eq_.row(8 + (nb_local_sampling_time - start_mid + 1) * 2 + i).head(nb_var_axis_) = acceleration_terms_x_[is_left_leg_in_contact_].row(i).head(nb_var_axis_);//xi[0]
//      A_eq_.row(8 + (nb_local_sampling_time - start_mid + 1) * 2 + i).segment(nb_var_axis_, nb_var_axis_) = acceleration_terms_x_[is_left_leg_in_contact_].row(i).segment(MAX_VAR, nb_var_axis_);//xi[0]
//      A_eq_.row(8 + (nb_local_sampling_time - start_mid + 1) * 2 + i).segment(2 * nb_var_axis_, nb_var_axis_) = acceleration_terms_x_[is_left_leg_in_contact_].row(i).segment(2 * MAX_VAR, nb_var_axis_);//xi[0]
//      A_eq_(8 + (nb_local_sampling_time - start_mid + 1) * 2 + i, 3 * nb_var_axis_ + i) = -1;
//      B_eq_(8 + (nb_local_sampling_time - start_mid + 1) * 2 + i) = -current_pose_(0) + target_pose_(0) - current_velocity_(0) * sampling_time * i;
//
//      A_eq_.row(8 + (nb_local_sampling_time - start_mid + 1) * 2 + i + nb_local_sampling_time - 1).head(nb_var_axis_) = acceleration_terms_y_[is_left_leg_in_contact_].row(i).head(nb_var_axis_);//xi[0]
//      A_eq_.row(8 + (nb_local_sampling_time - start_mid + 1) * 2 + i + nb_local_sampling_time - 1).segment(nb_var_axis_, nb_var_axis_) = acceleration_terms_y_[is_left_leg_in_contact_].row(i).segment(MAX_VAR, nb_var_axis_);//xi[0]
//      A_eq_.row(8 + (nb_local_sampling_time - start_mid + 1) * 2 + i + nb_local_sampling_time - 1).segment(2 * nb_var_axis_, nb_var_axis_) = acceleration_terms_y_[is_left_leg_in_contact_].row(i).segment(2 * MAX_VAR, nb_var_axis_);//xi[0]
//      A_eq_(8 + (nb_local_sampling_time - start_mid + 1) * 2 + i + nb_local_sampling_time - 1, 3 * nb_var_axis_ + i + nb_local_sampling_time - 1) = -1;
//      B_eq_(8 + (nb_local_sampling_time - start_mid + 1) * 2 + i + nb_local_sampling_time - 1) = -current_pose_(1) + target_pose_(1) - current_velocity_(1) * sampling_time * i;
//  }
  // clang-format on

  /*
   * Inequality constraints
   */

//  for(int i = 0; i < nb_var_ - 7; i++)
//    acc[i] = 0;
//  std::cout << "ST" << nb_local_sampling_time  << std::endl;
  for (int i = 1; i < nb_local_sampling_time; ++i) {
      // z >= zmin   =>   -z <= -z_min
//    x = B_;
//    for(int j = i - 1 ; j >= 0; j--){
//      acc[j] = x(4, 0) * M_inv_(0, 0) + x(4, 1) * M_inv_(1, 0) + x(4, 2) * M_inv_(2, 0);
//      acc[j + nb_var_axis_] = x(4, 0) * M_inv_(0, 1) + x(4, 1) * M_inv_(1, 1) + x(4, 2) * M_inv_(2, 1);
//      acc[j + 2 * nb_var_axis_] = x(4, 0) * M_inv_(0, 2) + x(4, 1) * M_inv_(1, 2) + x(4, 2) * M_inv_(2, 2);
//      x = A_ * x;
//    }
//    A_ineq_.row(i - 1).head(nb_var_ - 7) = -acc;
      A_ineq_.row(i - 1).head(nb_var_axis_) = -acceleration_terms_z_[is_left_leg_in_contact_].row(i).head(nb_var_axis_);
      A_ineq_.row(i - 1).segment(nb_var_axis_, nb_var_axis_) = -acceleration_terms_z_[is_left_leg_in_contact_].row(i).segment(MAX_VAR, nb_var_axis_);
      A_ineq_.row(i - 1).segment(2 * nb_var_axis_, nb_var_axis_) = -acceleration_terms_z_[is_left_leg_in_contact_].row(i).segment(2 * MAX_VAR, nb_var_axis_);
      B_ineq_(i - 1) = current_pose_(2) - std::min(start_pose(2), target_pose(2)) + 0.0001 + current_velocity_(2) * sampling_time * i;
      // z <= z_max, i <= n/2 || z_i <= z_i - 1, i > n/2
//    A_ineq_.row(i - 1 + nb_local_sampling_time).head(nb_var_ - 7) = acc;
      if (i <= nb_mid_sampling_time || i >= nb_local_sampling_time - 1) {
          A_ineq_.row(i - 1 + nb_local_sampling_time).head(nb_var_axis_) = acceleration_terms_z_[is_left_leg_in_contact_].row(i).head(nb_var_axis_);
          A_ineq_.row(i - 1 + nb_local_sampling_time).segment(nb_var_axis_, nb_var_axis_) = acceleration_terms_z_[is_left_leg_in_contact_].row(i).segment(MAX_VAR, nb_var_axis_);
          A_ineq_.row(i - 1 + nb_local_sampling_time).segment(2 * nb_var_axis_, nb_var_axis_) = acceleration_terms_z_[is_left_leg_in_contact_].row(i).segment(2 * MAX_VAR, nb_var_axis_);
          B_ineq_(i - 1 + nb_local_sampling_time) = -current_pose_(2) + std::max(start_pose(2), target_pose(2)) + mid_air_height_ - current_velocity_(2) * sampling_time * i;
      } else {
          A_ineq_.row(i - 1 + nb_local_sampling_time).head(nb_var_axis_) = velocity_terms_z_[is_left_leg_in_contact_].row(i).head(nb_var_axis_);
          A_ineq_.row(i - 1 + nb_local_sampling_time).segment(nb_var_axis_, nb_var_axis_) = velocity_terms_z_[is_left_leg_in_contact_].row(i).segment(MAX_VAR, nb_var_axis_);
          A_ineq_.row(i - 1 + nb_local_sampling_time).segment(2 * nb_var_axis_, nb_var_axis_) = velocity_terms_z_[is_left_leg_in_contact_].row(i).segment(2 * MAX_VAR, nb_var_axis_);
          B_ineq_(i - 1 + nb_local_sampling_time) = -current_velocity_(2);
      }
  }
//  B_ineq_(nb_local_sampling_time - 1) = -std::min(start_pose(2), target_pose(2));
  //Force
//  std::cout << nb_eq_ << std::endl;
//  A_eq_(nb_eq_ - 1, nb_local_sampling_time - 1) = 1;
//  B_eq_(nb_eq_ - 1) = 0;
//  A_eq_(nb_eq_ - 2, 2 * nb_local_sampling_time - 1) = 1;
//  B_eq_(nb_eq_ - 2) = 0;
//  A_eq_(nb_eq_ - 3, 3 * nb_local_sampling_time - 1) = 1;
//  B_eq_(nb_eq_ - 3) = 0;
//  std::cout << "Done\n";

    for(int i = 0; i < nb_local_sampling_time * 3; i++){
//    if(i % 3 != 2) {
        A_ineq_(nb_local_sampling_time * 2 + i, i) = 1;
        B_ineq_(nb_local_sampling_time * 2 + i) = 18;
        A_ineq_(nb_local_sampling_time * 2 + nb_local_sampling_time * 3 + i, i) = -1;
        B_ineq_(nb_local_sampling_time * 2 + nb_local_sampling_time * 3 + i) = 18;
//    }
  }
//  auto stop = std::chrono::high_resolution_clock::now();
//  auto duration2 = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
//  std::cout << "Lhum duration:      " << duration2.count() << std::endl;
//  start = std::chrono::high_resolution_clock::now();
  bool failure = false;
  if (!failure) {
    if (!qp_solver_.solve(Q_, q_, A_eq_, B_eq_, A_ineq_, B_ineq_)) {
      std::string error = "NewEndEffectorTrajectory3D::compute(): "
                          "failed to solve the QP.";
      std::cout << RED << "Error: " << error << RESET << std::endl;
      //   print_solver();
      failure = true;
    }
  }

  if (failure) {
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
  }
//  stop = std::chrono::high_resolution_clock::now();
//  duration2 = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
//  if(nb_local_sampling_time == 6)
//    print_solver();
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
    if(!qp_solver_.solve(Q_, q_, A_eq_, B_eq_, A_ineq_, B_ineq_)) {
        forces = forces.tail(forces.size() - 3);
    }
    else {
        // Extract the information from the solution.
        x_opt_.resize(nb_var_);
        x_opt_ = qp_solver_.result();
        for (int i = 0; i < nb_var_axis_; i++) {
            forces(i * 3) = x_opt_[i];
            forces(i * 3 + 1) = x_opt_[nb_var_axis_ + i];
            forces(i * 3 + 2) = x_opt_[2 * nb_var_axis_ + i];
        }
    }
    slack_variables_ << x_opt_(nb_var_ - 6), x_opt_(nb_var_ - 5), x_opt_(nb_var_ - 4);
  }
  next_acceleration << forces[0] * M_inv_[is_left_leg_in_contact_](0, 0) + forces[1] * M_inv_[is_left_leg_in_contact_](1, 0) + forces[2] * M_inv_[is_left_leg_in_contact_](2, 0),
                       forces[0] * M_inv_[is_left_leg_in_contact_](0, 1) + forces[1] * M_inv_[is_left_leg_in_contact_](1, 1) + forces[2] * M_inv_[is_left_leg_in_contact_](2, 1),
                       forces[0] * M_inv_[is_left_leg_in_contact_](0, 2) + forces[1] * M_inv_[is_left_leg_in_contact_](1, 2) + forces[2] * M_inv_[is_left_leg_in_contact_](2, 2);

  next_velocity << next_acceleration(0) * control_loop + current_velocity_(0),
                   next_acceleration(1) * control_loop + current_velocity_(1),
                   next_acceleration(2) * control_loop + current_velocity_(2);

  next_pose << 0.5 * next_acceleration(0) * control_loop * control_loop +
               current_velocity_(0) * control_loop + current_pose_(0),
               0.5 * next_acceleration(1) * control_loop * control_loop +
               current_velocity_(1) * control_loop + current_pose_(1),
               0.5 * next_acceleration(2) * control_loop * control_loop +
               current_velocity_(2) * control_loop + current_pose_(2);

  return nb_var_axis_ * 3;
}

void NewEndEffectorTrajectory3D::update_robot_status(Eigen::Ref<Eigen::Vector3d> next_pose,
        Eigen::Ref<Eigen::Vector3d> next_velocity, Eigen::Ref<Eigen::Vector3d> next_acceleration) {
    if (current_time_ < start_time_) {
        return;
    }
    current_pose_ = next_pose;
    current_velocity_ = next_velocity;
    Eigen::Vector3d forces;
    forces << x_opt_[0], x_opt_[nb_var_axis_], x_opt_[2 * nb_var_axis_];

    next_acceleration << forces[0] * M_inv_[is_left_leg_in_contact_](0, 0) + forces[1] * M_inv_[is_left_leg_in_contact_](1, 0) + forces[2] * M_inv_[is_left_leg_in_contact_](2, 0),
            forces[0] * M_inv_[is_left_leg_in_contact_](0, 1) + forces[1] * M_inv_[is_left_leg_in_contact_](1, 1) + forces[2] * M_inv_[is_left_leg_in_contact_](2, 1),
            forces[0] * M_inv_[is_left_leg_in_contact_](0, 2) + forces[1] * M_inv_[is_left_leg_in_contact_](1, 2) + forces[2] * M_inv_[is_left_leg_in_contact_](2, 2);

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
