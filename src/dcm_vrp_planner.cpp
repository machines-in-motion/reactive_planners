/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Definition of the DcmVrpPlanner class
 */

#include "reactive_planners/dcm_vrp_planner.hpp"

namespace reactive_planners
{
DcmVrpPlanner::DcmVrpPlanner(
    const double& l_min,
    const double& l_max,
    const double& w_min,
    const double& w_max,
    const double& t_min,
    const double& t_max,
    const double& l_p,
    const double& ht,
    const double& omega,
    const double& t_s_nom,
    const Eigen::Ref<const Eigen::Vector10d>& cost_weights_local)
{
    initialize(
        l_min, l_max, w_min, w_max, t_min, t_max, l_p, ht, omega, t_s_nom, cost_weights_local);
}

void DcmVrpPlanner::initialize(
    const double& l_min,
    const double& l_max,
    const double& w_min,
    const double& w_max,
    const double& t_min,
    const double& t_max,
    const double& l_p,
    const double& z_0,
    const double& omega,
    const double& t_s_nom,
    const Eigen::Ref<const Eigen::Vector10d>& cost_weights_local)
{
    l_min_ = l_min;
    l_max_ = l_max;
    w_min_ = w_min;
    w_max_ = w_max;
    t_min_ = t_min;
    t_max_ = t_max;
    l_p_ = l_p;
    z_0_ = z_0;
    omega_ = omega;
    t_s_nom_ = t_s_nom;
    cost_weights_local_ = cost_weights_local;

    tau_min_ = exp(omega_ * t_min_);
    tau_max_ = exp(omega_ * t_max_);
    bx_min_ = l_min_ / (tau_min_ - 1);
    bx_max_ = l_max_ / (tau_min_ - 1);
    by_max_in_ = l_p_ / (1 + tau_min_) +
                 (w_min_ - w_max_ * tau_min_) / (1 - exp(2 * omega_ * t_min_));
    by_max_out_ = l_p_ / (1 + tau_min_) +
                  (w_max_ - w_min_ * tau_min_) / (1 - exp(2 * omega_ * t_min_));
    nb_var_ = 10;

    // psi are slack variables here.
    // u_x, u_y, tau, b_x, b_y, psi_0, psi_1, psi_2, psi_3
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

    nb_ineq_ = 12;
    A_ineq_.resize(nb_ineq_, nb_var_);
    // clang-format off
    //          ux    uy    tau   bx    by    t_f    psi0  psi1  psi2  psi3
    A_ineq_ <<  1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   // 0
                0.0,  1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   // 1
               -1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   // 2
                0.0, -1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   // 3
                0.0,  0.0,  1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   // 4
                0.0,  0.0, -1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   // 5
                0.0,  0.0,  0.0,  1.0,  0.0,  0.0,  1.0,  0.0,  0.0,  0.0,   // 6
                0.0,  0.0,  0.0, -1.0,  0.0,  0.0,  0.0,  1.0,  0.0,  0.0,   // 7
                0.0,  0.0,  0.0,  0.0,  1.0,  0.0,  0.0,  0.0,  1.0,  0.0,   // 8
                0.0,  0.0,  0.0,  0.0, -1.0,  0.0,  0.0,  0.0,  0.0,  1.0,   // 9
                0.0,  0.0,  0.0,  0.0,  0.0,  1.0,  0.0,  0.0,  0.0,  0.0,   // 10
                0.0,  0.0,  0.0,  0.0,  0.0, -1.0,  0.0,  0.0,  0.0,  0.0;   // 11
    // clang-format on
    B_ineq_.resize(nb_ineq_);
    B_ineq_.setZero();

    qp_solver_.problem(nb_var_, nb_eq_, nb_ineq_);
    set_new_motion(z_0, omega, t_s_nom);
    x_dot_t_s_.setZero();
}

void DcmVrpPlanner::set_new_motion(const double& z_0,
                                   const double& omega,
                                   const double& t_s_nom){
    omega_ = omega;
    z_0_ = z_0;
    t_s_nom_ = t_s_nom;

    tau_min_ = exp(omega_ * t_min_);
    tau_max_ = exp(omega_ * t_max_);
    tau_nom_ = exp(omega_ * t_s_nom_);
    t_f_nom_ = 2 * omega_ * (tau_nom_ - 1) / (9.81 * (tau_nom_ + 1)) * (z_0_ - 9.81 / (omega_ * omega_));
    T_nom_ = t_s_nom_ + t_f_nom_;

}

void DcmVrpPlanner::compute_nominal_step_values(
    const contact& is_left_leg_in_contact,
    const Eigen::Ref<const Eigen::Vector3d>& v_des_local,
    const Eigen::Ref<const Eigen::Vector3d>& com,
    const Eigen::Ref<const Eigen::Vector3d>& com_vel)
{
    contact local_is_left_leg_in_contact = is_left_leg_in_contact;
    if(is_left_leg_in_contact > 2){
        local_is_left_leg_in_contact = contact(is_left_leg_in_contact - 3);
    }
    contact_switcher_ = local_is_left_leg_in_contact ? 2.0 : 1.0;

    double v_0 = omega_ * (tau_nom_ - 1) * (9.81 / (omega_ * omega_) - z_0_) / (tau_nom_ + 1);
    double z_min = 0.1;
    double z_max = 0.4;
    double z_nom = z_0_;

    if(is_left_leg_in_contact > 2){
        t_f_min_ = std::max(current_time_ - time_from_last_step_touchdown_,
                            (sqrt(2 * 9.81 * (com[2] - z_max) + pow(com_vel[2], 2)) + com_vel[2]) / 9.81);
        t_f_max_ = (sqrt(2 * 9.81 * (com[2] - z_min) + pow(com_vel[2], 2)) + com_vel[2]) / 9.81;
        if(t_f_min_ != t_f_min_) //NULL
            t_f_min_ = current_time_ - time_from_last_step_touchdown_ - EPSILON;
        if(t_f_max_ != t_f_max_)
            t_f_max_ = current_time_ - time_from_last_step_touchdown_;
    }
    else{
        t_f_min_ = (sqrt(2 * 9.81 * (z_0_ - z_max) + pow(-v_0, 2)) + -v_0) / 9.81;
        t_f_max_ = (sqrt(2 * 9.81 * (z_0_ - z_min) + pow(-v_0, 2)) + -v_0) / 9.81;

        if(t_f_min_ != t_f_min_)
            t_f_min_ = 0;
        if(t_f_max_ != t_f_max_)
            t_f_max_ = 0;
    }
    t_f_min_ = std::max(t_f_min_, current_time_ - time_from_last_step_touchdown_);
    l_nom_ = v_des_local(0) * T_nom_;
    w_nom_ = local_is_left_leg_in_contact ? v_des_local(1) * T_nom_ - l_p_
                                    : v_des_local(1) * T_nom_ + l_p_;

    double v_x = l_nom_ / (t_f_nom_ + 2 * (tau_nom_ - 1) / (omega_ * (tau_nom_ + 1))) ,
           v_r = (v_des_local(1) * T_nom_ + l_p_) / (t_f_nom_ + 2 * (tau_nom_ + 1) / (omega_ * (tau_nom_ - 1))),
           v_l = (v_des_local(1) * T_nom_ - l_p_) / (t_f_nom_ + 2 * (tau_nom_ + 1) / (omega_ * (tau_nom_ - 1)));
    bx_nom_ = (l_nom_ - v_x * t_f_nom_) / (tau_nom_ - 1);
    by_nom_ = pow(-1, contact_switcher_) * l_p_ / (tau_nom_ + 1) +
              v_des_local(1) * T_nom_ / (tau_nom_ - 1) +
              t_f_nom_ * pow(-1, contact_switcher_) * (-v_r * tau_nom_ - v_l) / (pow(tau_nom_, 2) - 1);

//    bx_min_ = (l_min_ - v_x * t_f_nom_) / ((tau_min_ - 1) * (1 + omega_ * omega_ / 9.81 * (z_0_ - 9.81 / (omega_ * omega_))));
//    bx_max_ = (l_max_ - v_x * t_f_nom_) / ((tau_min_ - 1) * (1 + omega_ * omega_ / 9.81 * (z_0_ - 9.81 / (omega_ * omega_))));
//    by_max_in_ = (pow(-1, contact_switcher_) * (l_p_ / (1 + tau_min_))) +
//              v_des_local(1) * T_nom_ / (tau_min_ - 1) +
//              t_f_nom_ * (pow(-1, contact_switcher_) * -v_r * tau_min_
//                          - pow(-1, contact_switcher_) * v_l) / (pow(tau_min_, 2) - 1);
//    by_max_in_ = l_p_ / (1 + tau_min_) +
//                 (w_min_ - w_max_ * tau_min_) / (1 - exp(2 * omega_ * t_min_));
//    by_max_in_ = (pow(-1, contact_switcher_) * (l_p_ / (1 + tau_min_))) +
//                 v_des_local(1) * T_nom_ / (tau_min_ - 1) +
//                 t_f_nom_ * (pow(-1, contact_switcher_) * -v_r * tau_min_
//                             - pow(-1, contact_switcher_) * v_l) / (pow(tau_min_, 2) - 1);
//    by_max_out_ = l_p_ / (1 + tau_min_) +
//                  (w_max_ - w_min_ * tau_min_) / (1 - exp(2 * omega_ * t_min_));

}

#define dbg_dump(var) std::cout << "  " << #var << ": " << var << std::endl

void DcmVrpPlanner::update(
    const Eigen::Ref<const Eigen::Vector3d>& current_step_location,
    const double& time_from_last_step_touchdown,
    const contact& is_left_leg_in_contact,
    const Eigen::Ref<const Eigen::Vector3d>& v_des,
    const Eigen::Ref<const Eigen::Vector3d>& com,
    const Eigen::Ref<const Eigen::Vector3d>& com_vel,
    const double& yaw,
    const double& omega)
{
    pinocchio::SE3 world_M_base(
        Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()).toRotationMatrix(),
        current_step_location);

    update(current_step_location,
           time_from_last_step_touchdown,
           is_left_leg_in_contact,
           v_des,
           com,
           com_vel,
           world_M_base,
           omega);
}

void DcmVrpPlanner::update(
    const Eigen::Ref<const Eigen::Vector3d>& current_step_location,
    const double& time_from_last_step_touchdown,
    const contact& is_left_leg_in_contact,
    const Eigen::Ref<const Eigen::Vector3d>& v_des,
    const Eigen::Ref<const Eigen::Vector3d>& com,
    const Eigen::Ref<const Eigen::Vector3d>& com_vel,
    const pinocchio::SE3& world_M_base,
    const double& omega)
{
    contact local_is_left_leg_in_contact = is_left_leg_in_contact;
    if(is_left_leg_in_contact > 2){
        local_is_left_leg_in_contact = contact(is_left_leg_in_contact - 3);
    }
    omega_ = omega;
    if(time_from_last_step_touchdown < duration_before_step_landing_) {
        time_from_last_step_touchdown_ = time_from_last_step_touchdown;//stance duration has passed
    }
    else{
        time_from_last_step_touchdown_ = duration_before_step_landing_;
    }
    current_time_ = time_from_last_step_touchdown;
    double ground_height = 0.0;

    // Local frame parallel to the world frame and aligned with the base yaw.
    world_M_local_.translation() << world_M_base.translation()(0),
        world_M_base.translation()(1), ground_height;
    Eigen::Vector3d rpy = world_M_base.rotation().eulerAngles(0, 1, 2);
    world_M_local_.rotation() = world_M_base.rotation();

    // Compute the DCM in the local frame.
    dcm_local_.head<2>() = com_vel.head<2>() / omega_ + com.head<2>();
    dcm_local_(2) = ground_height;
    dcm_local_ = world_M_local_.actInv(dcm_local_);

    com_local_ = com;
    com_local_ = world_M_local_.actInv(com_local_);

    // Express the desired velocity in the local frame.
    v_des_local_ = world_M_local_.rotation().transpose() * v_des;
    com_vel_local_ = world_M_local_.rotation().transpose() * com_vel;

    compute_nominal_step_values(is_left_leg_in_contact, v_des_local_, com_local_, com_vel_local_);

    // Current step location in the local frame.
    const Eigen::Vector3d& tmp = current_step_location;
    current_step_location_local_ = world_M_local_.actInv(tmp);

    // Quadratic cost matrix is constant
    Q_.diagonal() = cost_weights_local_;

    // Quadratic cost Vector
    q_(0) = -cost_weights_local_(0) * l_nom_;
    q_(1) = -cost_weights_local_(1) * w_nom_;
    q_(2) = -cost_weights_local_(2) * tau_nom_;
    q_(3) = -cost_weights_local_(3) * bx_nom_;
    q_(4) = -cost_weights_local_(4) * by_nom_;
    q_(5) = -cost_weights_local_(5) * t_f_nom_;
    q_.tail<4>().setZero();

    x_dot_t_s_[0] = omega_ / 2 * ((com_local_[0] + com_vel_local_[0] / omega_) * exp(omega_ * std::max(0.0, t_s_nom_ - time_from_last_step_touchdown_)) -
                                     (com_local_[0] - com_vel_local_[0] / omega_) * exp(-omega_ * std::max(0.0, t_s_nom_ - time_from_last_step_touchdown_)));
    x_dot_t_s_[1] = omega_ / 2 * ((com_local_[1] + com_vel_local_[1] / omega_) * exp(omega_ * std::max(0.0, t_s_nom_ - time_from_last_step_touchdown_)) -
                                     (com_local_[1] - com_vel_local_[1] / omega_) * exp(-omega_ * std::max(0.0, t_s_nom_ - time_from_last_step_touchdown_)));
    x_dot_t_s_[2] = 0;
    x_dot_t_s_ = world_M_local_.actInv(x_dot_t_s_);


    if(local_is_left_leg_in_contact == left || local_is_left_leg_in_contact == right){
        A_ineq_(0, 5) = -x_dot_t_s_[0];
        A_ineq_(1, 5) = -x_dot_t_s_[1];
        A_ineq_(2, 5) = x_dot_t_s_[0];
        A_ineq_(3, 5) = x_dot_t_s_[1];

    }
    else{
        A_ineq_(0, 5) = -com_vel_local_[0];
        A_ineq_(1, 5) = -com_vel_local_[1];
        A_ineq_(2, 5) = com_vel_local_[0];
        A_ineq_(3, 5) = com_vel_local_[1];
    }

    switch(local_is_left_leg_in_contact){
        case left:
        case flight_l:
            w_max_local_ = -w_min_ - l_p_;
            w_min_local_ = -w_max_ - l_p_;
            by_max_ = -by_max_out_;
            by_min_ = -by_max_in_;
            break;
        case right:
        case flight_r:
            w_max_local_ = w_max_ + l_p_;
            w_min_local_ = w_min_ + l_p_;
            by_max_ = by_max_in_;
            by_min_ = by_max_out_;
            break;
        default:
            break;
    }
    // Inequality constraints
    // A_ineq_ is constant see initialize.
    // clang-format off

    if(is_left_leg_in_contact > 2) {
        B_ineq_ << 10000,                // 0
                10000,           // 1
                10000,                // 2
                10000,           // 3
                x_opt_(2),              // 4
                -x_opt_(2),              // 5
                10000,               // 6
                10000,               // 7
                10000,                // 8
                10000,               // 9
                10000,               //10
                -t_f_min_;                // 11
                std::cout << "F\n";
    }
    else{
        B_ineq_ << 10000,                // 0
                10000,           // 1
                10000,                // 2
                10000,           // 3
                10000,              // 4
                -tau_min_,              // 5
                10000,               // 6
                10000,               // 7
                10000,                // 8
                10000,               // 9
                10000,               //10
                -t_f_min_;                // 11
                std::cout << "S\n" << tau_min_ << std::endl;
    }

    double tmp0 = (dcm_local_(0) - current_step_location_local_[0]) *
                  exp(-1.0 * omega_ * time_from_last_step_touchdown_);
    double tmp1 = (dcm_local_(1) - current_step_location_local_[1]) *
                  exp(-1.0 * omega_ * time_from_last_step_touchdown_);

    if(is_left_leg_in_contact > 2){
        A_eq_ << 1.0, 0.0, -1.0 * tmp0, 1.0, 0.0, -com_vel_local_[0], 0.0, 0.0, 0.0, 0.0,
                 0.0, 1.0, -1.0 * tmp1, 0.0, 1.0, -com_vel_local_[1], 0.0, 0.0, 0.0, 0.0;
        B_eq_ << current_step_location_local_[0],
                 current_step_location_local_[1];
    }
    else {
        A_eq_ << 1.0, 0.0, -1.0 * tmp0, 1.0, 0.0, -x_dot_t_s_[0], 0.0, 0.0, 0.0, 0.0,
                 0.0, 1.0, -1.0 * tmp1, 0.0, 1.0, -x_dot_t_s_[1], 0.0, 0.0, 0.0, 0.0;
        B_eq_ << current_step_location_local_[0],
                 current_step_location_local_[1];
    }
    // clang-format on
}

bool DcmVrpPlanner::solve()
{
    bool failure = false;
    if (!internal_checks())
    {
        std::cout << "DcmVrpPlanner::solve(): Error, internal checks failed."
                  << std::endl;
        failure = true;
    }
    if (!failure)
    {
        if (!qp_solver_.solve(Q_, q_, A_eq_, B_eq_, A_ineq_, B_ineq_))
        {
            std::string error =
                "DcmVrpPlanner::compute_adapted_step_locations(): "
                "failed to solve the QP.";
            std::cout << "Error: " << error << std::endl;
            failure = true;
        }
    }

    if (!failure)
    {
        // Extract the information from the solution.
        x_opt_ = qp_solver_.result();
        next_step_location_ =
            current_step_location_local_ +
            (Eigen::Vector3d() << x_opt_(0), x_opt_(1), 0.0).finished();
        next_step_location_ = world_M_local_.act(next_step_location_);
        dcm_offset_ = (Eigen::Vector3d() << x_opt_(3), x_opt_(4), 0.0).finished();
        dcm_offset_ = world_M_local_.act(dcm_offset_);
        duration_before_step_landing_ = log(x_opt_(2)) / omega_;
        duration_of_flight_phase_ = x_opt_(5);
//        std::cout << "finalt_f" << t_f_min_ << " <  " << t_f_nom_ << " = " << t_f_nom_ << " <  " << t_f_max_ << "  = " << duration_before_step_landing_ << " " << x_opt_(5) << std::endl;
        std::cout << "finalRESULT" <<
                      -B_ineq_(1) << " < " << x_opt_(0) << " = " << l_nom_ << " < " << B_ineq_(0) << " \n" <<
                      -B_ineq_(3) << " < " << x_opt_(1) << " = " << w_nom_ << " < " << B_ineq_(2) << " \n" <<
                      -B_ineq_(5) << " < " << x_opt_(2) << " = " << tau_nom_ << " < " << B_ineq_(4) << " \n" <<
                      -B_ineq_(7) << " < " << x_opt_(3) << " = " << bx_nom_ << " < " << B_ineq_(6) << " \n" <<
                      -B_ineq_(9) << " < " << x_opt_(4) << " = " << by_nom_ << " < " << B_ineq_(8) << " \n" <<
                      -B_ineq_(11) << " < " << x_opt_(5) << " = " << t_f_nom_ << " < " << B_ineq_(10) << " \n" <<
                      "";

        double tmp0 = (dcm_local_(0) - current_step_location_local_[0]) *
                      exp(-1.0 * omega_ * time_from_last_step_touchdown_);
        std::cout << "u_x " << x_opt_(2) * tmp0 - x_opt_(3) + x_dot_t_s_[0] * (x_opt_(5) + 0.2) + current_step_location_local_[0] << std::endl;
        std::cout << "v_t_s   v_mea" << x_dot_t_s_[0] << "     "  << com_vel_local_ << std::endl;
        slack_variables_ = x_opt_.tail<4>();
//        std::cout << "bx_" << x_opt_(3) << std::endl;

    }
    else
    {
        // https://github.com/jrl-umi3218/eigen-quadprog/blob/master/src/QuadProg/c/solve.QP.compact.c#L94
        if (qp_solver_.fail() == 1)
        {
            std::cout << BLUE
                      << "DcmVrpPlanner::solve() -> the minimization problem "
                         "has no solution!"
                      << RESET << std::endl;
        }
        else
        {
            std::cout
                << BLUE
                << "DcmVrpPlanner::solve() -> problems with decomposing D!"
                << RESET << std::endl;
        }
        slack_variables_.setZero();

        duration_before_step_landing_ = t_s_nom_;
        duration_of_flight_phase_ = t_f_nom_;
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
  }                                                 \
  static_assert(true, "")
// clang-format on

bool DcmVrpPlanner::internal_checks()
{
    assert_DcmVrpPlanner(l_min_ <= l_max_);
    assert_DcmVrpPlanner(w_min_ <= w_max_);
    assert_DcmVrpPlanner(t_min_ <= t_max_);
    assert_DcmVrpPlanner(l_p_ == l_p_);
    assert_DcmVrpPlanner(z_0_ > 0.0);
    assert_DcmVrpPlanner((cost_weights_local_.array() >= 0.0).all());
    assert_DcmVrpPlanner(bx_min_ < bx_max_);
    assert_DcmVrpPlanner(by_max_out_ <= by_max_in_);
    assert_DcmVrpPlanner(Q_.rows() == x_opt_.size());
    assert_DcmVrpPlanner(Q_.cols() == x_opt_.size());
    assert_DcmVrpPlanner(q_.size() == x_opt_.size());
    assert_DcmVrpPlanner(A_eq_.rows() == 2);
    assert_DcmVrpPlanner(A_eq_.cols() == x_opt_.size());
    assert_DcmVrpPlanner(B_eq_.size() == 2);
    assert_DcmVrpPlanner(A_ineq_.rows() == 12);
    assert_DcmVrpPlanner(A_ineq_.cols() == x_opt_.size());
    assert_DcmVrpPlanner(B_ineq_.size() == 12);
    //    assert_DcmVrpPlanner((t_min_ - log(tau_min_) / omega_) *
    //                             (t_min_ - log(tau_min_) / omega_) <
    //                         1e-8)// Lhum TODO
    assert_DcmVrpPlanner((t_max_ - log(tau_max_) / omega_) *
                             (t_max_ - log(tau_max_) / omega_) <
                         1e-8);
    return true;
}

std::string DcmVrpPlanner::to_string() const
{
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

void DcmVrpPlanner::print_solver() const
{
    std::cout << to_string() << std::endl;
}

}  // namespace reactive_planners