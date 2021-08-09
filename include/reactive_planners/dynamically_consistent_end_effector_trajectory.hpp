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

#include <eigen-quadprog/QuadProg.h>
#include <Eigen/Eigen>
#include <chrono>
#include <iostream>
#include <limits>
#define RESET "\033[0m"
#define RED "\033[31m"  /* Red */
#define BLUE "\033[34m" /* Blue */
#define MAX_VAR 500
#define EPSILON 1e-9

namespace reactive_planners
{
/**
 * @brief
 */
class DynamicallyConsistentEndEffectorTrajectory
{
    /*
     * Private methods
     */
public:
    /** @brief Constructor. */
    DynamicallyConsistentEndEffectorTrajectory();

    /** @brief Destructor. */
    ~DynamicallyConsistentEndEffectorTrajectory();

    bool compute(const Eigen::Ref<const Eigen::Vector3d>& start_pose,
                 const Eigen::Ref<const Eigen::Vector3d>& current_pose,
                 const Eigen::Ref<const Eigen::Vector3d>& current_velocity,
                 const Eigen::Ref<const Eigen::Vector3d>& target_pose,
                 const double& start_time,
                 const double& current_time,
                 const double& end_time,
                 const bool& is_left_leg_in_contact);

    void update_robot_status(Eigen::Ref<Eigen::Vector3d> next_pose,
                             Eigen::Ref<Eigen::Vector3d> next_velocity,
                             Eigen::Ref<Eigen::Vector3d> next_acceleration);

    /** @brief Get all the forces until landing foot. Returns the number of
     * forces.*/
    int get_forces(Eigen::Ref<Eigen::VectorXd> forces,
                   Eigen::Ref<Eigen::Vector3d> next_pose,
                   Eigen::Ref<Eigen::Vector3d> next_velocity,
                   Eigen::Ref<Eigen::Vector3d> next_acceleration);

    /** @brief Display the matrices of the Problem. */
    void print_solver() const;

    /** @brief Convert the inner data to a string format. */
    std::string to_string() const;

    /*
     * Getters
     */

    /** @brief Get the height of the flying foot. */
    double get_mid_air_height() const
    {
        return mid_air_height_;
    }

    /*
     * Setters
     */
    /** @brief Set the height of the flying foot.
     *
     * @param mid_air_height
     */
    void set_planner_loop(double planner_loop)
    {
        planner_loop_ = planner_loop;
        init_acceleration_velocity_terms();
    }

    /** @brief Set the height of the flying foot.
     *
     * @param mid_air_height
     */
    void set_mid_air_height(double mid_air_height)
    {
        mid_air_height_ = mid_air_height;
    }

    /**
     * @brief return cost.
     *
     * @return double
     */
    double cost()
    {
        x_opt_.resize(nb_var_);
        x_opt_ = qp_solver_.result();
        return (0.5 * x_opt_.transpose() * Q_ * x_opt_ +
                q_.transpose() * x_opt_)(0, 0);
    }

    /**
     * @brief Return the slack values from the last solution.
     **/
    const Eigen::Vector3d& get_slack_variables() const
    {
        return slack_variables_;
    }

    double calculate_t_min(
        const Eigen::Ref<const Eigen::Vector3d>& current_pose,
        const Eigen::Ref<const Eigen::Vector3d>& current_velocity,
        const bool& is_left_leg_in_contact);
    /*
     * Private methods
     */
private:
    /** @brief resize QP variable at each step.
     */
    void resize_matrices()
    {
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
    /** @brief resize QP variable at each step.
     */
    void resize_matrices_t_min(int index)
    {
        Q_t_min_.resize(3 * index, 3 * index);
        Q_t_min_ = Eigen::MatrixXd::Identity(3 * index, 3 * index);
        q_t_min_.resize(3 * index);
        q_t_min_.setZero();

        A_eq_t_min_.resize(2, 3 * index);
        A_eq_t_min_.setZero();
        B_eq_t_min_.resize(2);
        B_eq_t_min_.setZero();

        A_ineq_t_min_.resize(2 * 3 * index, 3 * index);
        A_ineq_t_min_.setZero();
        B_ineq_t_min_.resize(2 * 3 * index);
        B_ineq_t_min_.setZero();
        qp_solver_t_min_.problem(3 * index, 2, 2 * 3 * index);
    }

    /** @brief initialie acceleration_terms and velocity_terms.
     */
    void init_acceleration_velocity_terms();

    /*
     * Constant problem parameters.
     */

    /** @brief Flying foot apex to be reach mid-air. */
    double mid_air_height_;

    /** @brief Number of force variables in the optimization problem on the
     * one of the axes. */
    int nb_local_sampling_time_;

    /** @brief non_linear_terms.
     */
    Eigen::Vector3d h_c;

    /*
     * Variable problem parameters.
     */

    /** @brief affect of forces_x to the positions.
     * There are two separate matrices for each leg.
     */
    Eigen::MatrixXd* position_terms_F_x_;

    /** @brief affect of forces_y to the positions.
     * There are two separate matrices for each leg.
     */
    Eigen::MatrixXd* position_terms_F_y_;

    /** @brief affect of forces_z to the positions.
     * There are two separate matrices for each leg.
     */
    Eigen::MatrixXd* position_terms_F_z_;

    /** @brief Desired velocity. */
    Eigen::Vector3d v_des_;

    /** @brief Slack variable values corresponding to last solution. */
    Eigen::Vector3d slack_variables_;

    /** @brief affect of non_linear_terms in position. */
    Eigen::MatrixXd* non_linear_terms;

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

    /** @brief Last end time register when we computed the QP. */
    double last_end_time_seen_;

    /** @brief Control loop. */
    double control_loop_;

    /** @brief planner loop. */
    double planner_loop_;

    /*
     * QP variables
     */

    /** @brief Number of variables in the optimization problem. */
    int nb_var_;

    /** @brief Number of equality constraints in the optimization problem. */
    int nb_eq_;

    /** @brief Number of inequality in the optimization problem. */
    int nb_ineq_;

    /** @brief Quadratic program solver.
     *
     * This is an eigen wrapper around the quad_prog fortran solver.
     */
    Eigen::QuadProgDense qp_solver_;

    /** @brief Quadratic program solver.
     *
     * This is an eigen wrapper around the quad_prog fortran solver.
     */
    Eigen::QuadProgDense qp_solver_t_min_;

    /** @brief Solution of the optimization problem.
     * @see DynamicallyConsistentEndEffectorTrajectory */
    Eigen::VectorXd x_opt_;

    /** @brief Quadratic term of the quadratic cost.
     * @see DynamicallyConsistentEndEffectorTrajectory */
    Eigen::MatrixXd Q_;
    /** @brief Quadratic term of the quadratic cost.
     * @see DynamicallyConsistentEndEffectorTrajectory */
    Eigen::MatrixXd Q_t_min_;

    /** @brief inverse estimation of mass matrix.
     * @see DynamicallyConsistentEndEffectorTrajectory */
    Eigen::MatrixXd* M_inv_;

    /** @brief Is the left foot in contact? otherwise the right foot is. */
    bool is_left_leg_in_contact_;

    /** @brief Quadratic term added to the quadratic cost in order regularize
     * the system.
     * @see DynamicallyConsistentEndEffectorTrajectory */
    Eigen::MatrixXd Q_regul_;

    /** @brief Cost weights for forces. */
    double cost_;

    /** @brief Cost weights for the epsilon_z_mid. */
    double cost_epsilon_z_mid_;

    /** @brief Cost weights for the epsilon_x. */
    double cost_epsilon_x_;

    /** @brief Cost weights for the epsilon_y. */
    double cost_epsilon_y_;

    /** @brief Cost weights for the epsilon_z. */
    double cost_epsilon_z_;

    /** @brief Cost weights for the epsilon_z. */
    double cost_epsilon_vel_;

    /** @brief Linear term of the quadratic cost.
     * @see DynamicallyConsistentEndEffectorTrajectory */
    Eigen::VectorXd q_;

    /** @brief Linear equality matrix.
     * @see DynamicallyConsistentEndEffectorTrajectory */
    Eigen::MatrixXd A_eq_;

    /** @brief Linear equality vector.
     * @see DynamicallyConsistentEndEffectorTrajectory */
    Eigen::VectorXd B_eq_;

    /** @brief Linear inequality matrix.
     * @see DynamicallyConsistentEndEffectorTrajectory */
    Eigen::MatrixXd A_ineq_;

    /** @brief Linear inequality vector.
     * @see DynamicallyConsistentEndEffectorTrajectory */
    Eigen::VectorXd B_ineq_;

    /** @brief Linear term of the quadratic cost.
     * @see DynamicallyConsistentEndEffectorTrajectory */
    Eigen::VectorXd q_t_min_;

    /** @brief Linear equality matrix.
     * @see DynamicallyConsistentEndEffectorTrajectory */
    Eigen::MatrixXd A_eq_t_min_;

    /** @brief Linear equality vector.
     * @see DynamicallyConsistentEndEffectorTrajectory */
    Eigen::VectorXd B_eq_t_min_;

    /** @brief Linear inequality matrix.
     * @see DynamicallyConsistentEndEffectorTrajectory */
    Eigen::MatrixXd A_ineq_t_min_;

    /** @brief Linear inequality vector.
     * @see DynamicallyConsistentEndEffectorTrajectory */
    Eigen::VectorXd B_ineq_t_min_;
};

}  // namespace reactive_planners
