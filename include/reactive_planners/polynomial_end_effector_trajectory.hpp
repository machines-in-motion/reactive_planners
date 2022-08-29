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

namespace reactive_planners
{
/**
 * @brief
 */
class PolynomialEndEffectorTrajectory
{
    /*
     * Private methods
     */
public:
    /** @brief Constructor. */
    PolynomialEndEffectorTrajectory();

    /** @brief Destructor. */
    ~PolynomialEndEffectorTrajectory();

    bool compute(const Eigen::Ref<const Eigen::Vector3d> &start_pose,
                 const Eigen::Ref<const Eigen::Vector3d> &current_pose,
                 const Eigen::Ref<const Eigen::Vector3d> &current_velocity,
                 const Eigen::Ref<const Eigen::Vector3d> &current_acceleration,
                 const Eigen::Ref<const Eigen::Vector3d> &target_pose,
                 const double &start_time,
                 const double &current_time,
                 const double &end_time);

    void get_next_state(const double &next_time,
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
    double get_mid_air_height()
    {
        return mid_air_height_;
    }

    /** @brief Get the last end time taken into account during the foot
     * trajectory computation. */
    double get_last_end_time_taken_into_account()
    {
        return last_end_time_seen_;
    }

    /*
     * Setters
     */

    /** @brief Set the height of the flying foot.
     *
     * @param mid_air_height
     */
    void set_mid_air_height(double mid_air_height)
    {
        mid_air_height_ = mid_air_height;
    }
    /** @brief Set the costs of x, y, z axes, and hessian regularization.
     *
     * @param cost_x
     * @param cost_y
     * @param cost_z
     * @param hess_regularization
     */
    void set_costs(double cost_x,
                   double cost_y,
                   double cost_z,
                   double hess_regularization)
    {
        cost_x_ = cost_x;
        cost_y_ = cost_y;
        cost_z_ = cost_z;
        Q_regul_ =
            Eigen::MatrixXd::Identity(nb_var_, nb_var_) * hess_regularization;
    }
    /*
     * Private methods
     */
private:
    /**
     * @brief Compute the time vector: \f$ [1, t, ..., t^{ORDER}] \f$.
     *
     * @param time
     * @param time_vec
     */
    void t_vec(const double &time, Eigen::VectorXd &time_vec)
    {
        time_vec(0) = 1.0;
        for (int i = 1; i < time_vec.size(); ++i)
        {
            time_vec(i) = std::pow(time, i);
        }
    }

    /**
     * @brief Compute the time vector first derivative.
     *
     * \f[ t_vec =  [0, 1, 2*t, ..., ORDER * t^{ORDER-1}] \f]
     *
     * @param time
     * @param time_vec
     */
    void dt_vec(const double &time, Eigen::VectorXd &time_vec)
    {
        time_vec(0) = 0.0;
        time_vec(1) = 1.0;
        for (int i = 2; i < time_vec.size(); ++i)
        {
            double id = i;
            time_vec(i) = id * std::pow(time, i - 1);
        }
    }

    /**
     * @brief Compute the time vector second derivative:
     *
     * \f[ t_vec =  [0, 0, 2, 3*2*t..., ORDER * (ORDER-1) * t^{ORDER-2}] \f]
     *
     * @param time
     * @param time_vec
     */
    void ddt_vec(const double &time, Eigen::VectorXd &time_vec)
    {
        time_vec(0) = 0.0;
        time_vec(1) = 0.0;
        time_vec(2) = 2.0;
        for (int i = 3; i < time_vec.size(); ++i)
        {
            double id = i;
            time_vec(i) = id * (id - 1.0) * std::pow(time, i - 2);
        }
    }

    /*
     * Attributes
     */
private:
    /*
     * Constant problem parameters.
     */

    /** @brief Flying foot apex to be reach mid-air. */
    double mid_air_height_;

    /** @brief Number of the polynome coefficient for the trajectory on the
     * X-axis. */
    int nb_var_x_;

    /** @brief Number of the polynome coefficient for the trajectory on the
     * Y-axis. */
    int nb_var_y_;

    /** @brief Number of the polynome coefficient for the trajectory on the
     * Z-axis. */
    int nb_var_z_;

    /*
     * Variable problem parameters.
     */

    /** @brief Time vector used to compute X(t). */
    Eigen::VectorXd time_vec_x_;

    /** @brief Time vector used to compute Y(t). */
    Eigen::VectorXd time_vec_y_;

    /** @brief Time vector used to compute Z(t). */
    Eigen::VectorXd time_vec_z_;

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

    /*
     * QP variables
     */

    /** @brief Number of variabes in the optimization problem. */
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

    /** @brief Solution of the optimization problem.
     * @see PolynomialEndEffectorTrajectory */
    Eigen::VectorXd x_opt_;

    /** @brief Lower Bound on the solution of the optimization problem.
     * @see PolynomialEndEffectorTrajectory */
    Eigen::VectorXd x_opt_lb_;

    /** @brief Upper Bound on the solution of the optimization problem.
     * @see PolynomialEndEffectorTrajectory */
    Eigen::VectorXd x_opt_ub_;

    /** @brief Quadratic term of the quadratic cost.
     * @see PolynomialEndEffectorTrajectory */
    Eigen::MatrixXd Q_;

    /** @brief Quadratic term added to the quadratic cost in order regularize
     * the system.
     * @see PolynomialEndEffectorTrajectory */
    Eigen::MatrixXd Q_regul_;

    /** @brief Cost weights for the X-axis. */
    double cost_x_;

    /** @brief Cost weights for the Y-axis. */
    double cost_y_;

    /** @brief Cost weights for the Z-axis. */
    double cost_z_;

    /** @brief Linear term of the quadratic cost.
     * @see PolynomialEndEffectorTrajectory */
    Eigen::VectorXd q_;

    /** @brief Linear equality matrix.
     * @see PolynomialEndEffectorTrajectory */
    Eigen::MatrixXd A_eq_;

    /** @brief Linear equality vector.
     * @see PolynomialEndEffectorTrajectory */
    Eigen::VectorXd B_eq_;

    /** @brief Linear inequality matrix.
     * @see PolynomialEndEffectorTrajectory */
    Eigen::MatrixXd A_ineq_;

    /** @brief Linear inequality vector.
     * @see PolynomialEndEffectorTrajectory */
    Eigen::VectorXd B_ineq_;
};

}  // namespace reactive_planners
