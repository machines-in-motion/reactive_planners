/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Implements the "Walking Control Based on Step Timing Adaptation" step
 * planner. The pdf can be found in https://arxiv.org/abs/1704.01271, and in the
 * `doc/` folder in this repository.
 */

#pragma once

#include <eigen-quadprog/QuadProg.h>
#include <Eigen/Eigen>
#include <cmath>
#include <iostream>
#include <pinocchio/spatial/se3.hpp>
#include <sstream>
#include <stdexcept>
#define RESET "\033[0m"
#define RED "\033[31m"  /* Red */
#define BLUE "\033[34m" /* Blue */

namespace Eigen
{
/** @brief Column vector of size 9 which correspond to the number of
 * optimization variables. */
typedef Matrix<double, 9, 1> Vector9d;
}  // namespace Eigen

namespace reactive_planners
{
/**
 * @brief Implements the "Walking Control Based on Step Timing Adaptation" step
 * planner. The pdf can be found in https://arxiv.org/abs/1704.01271, and in the
 * `doc/` folder in this repository.
 *
 * All quantities are here expressed in the local frame. Which means for a robot
 * that the quantities are expressed in the "base" frame.
 *
 * @todo write here the formulation of the QP.
 */
class DcmVrpPlanner
{
public:
    /**
     * @brief Construct a new DcmVrpPlanner object and initialize it.
     *
     * @param l_min [in] Minimum step length in the x direction (in the
     * direction of forward motion).
     * @param l_max [in] Maximum step length in the x direction (in the
     * direction of forward motion).
     * @param w_min [in] Minimum step length in the y direction (in the lateral
     * direction).
     * @param w_max [in] Maximum step length in the y direction (in the lateral
     * direction).
     * @param t_min [in] Minimum step time.
     * @param t_max [in] Maximum step time.
     * @param v_des [in] Desired average velocity in the x and y ([v_x, v_y]) 2d
     * vector.
     * @param l_p [in] Default lateral step length. Typically useful for
     * humanoid robot where this value refer to the distance between the 2 feet
     * while in the half-sitting/neutral position.
     * @param ht [in] Average desired height of the com above the ground.
     * @param cost_weights_local [in] Weights of the QP cost expressed in the
     * local frame.
     */
    DcmVrpPlanner(const double& l_min,
                  const double& l_max,
                  const double& w_min,
                  const double& w_max,
                  const double& t_min,
                  const double& t_max,
                  const double& l_p,
                  const double& ht,
                  const Eigen::Ref<const Eigen::Vector9d>& cost_weights_local);

    /**
     * @brief Construct a new DcmVrpPlanner object with some default parameters.
     */
    DcmVrpPlanner()
    {
        initialize(
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Eigen::Vector9d::Zero());
    }

    /**
     * @brief Initialize The inner variable that are not time varying.
     *
     * @copydoc DcmVrpPlanner::DcmVrpPlanner()
     */
    void initialize(
        const double& l_min,
        const double& l_max,
        const double& w_min,
        const double& w_max,
        const double& t_min,
        const double& t_max,
        const double& l_p,
        const double& ht,
        const Eigen::Ref<const Eigen::Vector9d>& cost_weights_local);

    /**
     * @brief Set the nominal steptime.
     */
    void set_steptime_nominal(double t_nom);

    /**
     * @brief Computes adapted step location.
     *
     * We use a QP formulation with the following notation:
     * \f{eqnarray*}{
     *    minimize   & \\
     *       x       & (1/2) x^T Q x + q^T x \\
     *    subject to & \\
     *               & A_{ineq} x \leq b_{ineq} \\
     *               & A_{eq} x = b_{eq} \\
     *               & x_{opt_{lb}} \leq x \leq x_{opt_{ub}}
     * \f}
     * We use the off-the-shelf QP solver eigen-quadprog in order to solve it.
     * @see DcmVrpPlanner for the full formulation.
     *
     * @param current_step_location is the location of the previous foot step
     * location (2d vector) [ux, uy].
     * @param time_from_last_step_touchdown is the time elapsed since the last
     * foot step landed.
     * @param is_left_leg_in_contact is true is the current foot is on the left
     * side, false if it is on the right side.
     * @param v_des
     * @param com is the CoM position.
     * @param com_vel is the CoM velocity.
     * @param world_M_base SE3 position of the robot base expressed in the world
     * frame.
     */
    void update(const Eigen::Ref<const Eigen::Vector3d>& current_step_location,
                const double& time_from_last_step_touchdown,
                const bool& is_left_leg_in_contact,
                const Eigen::Ref<const Eigen::Vector3d>& v_des,
                const Eigen::Ref<const Eigen::Vector3d>& com,
                const Eigen::Ref<const Eigen::Vector3d>& com_vel,
                const pinocchio::SE3& world_M_base,
                const double& new_t_min);
    /**
     * @brief Computes adapted step location for python3.
     *
     * We use a QP formulation with the following notation:
     * \f{eqnarray*}{
     *    minimize   & \\
     *       x       & (1/2) x^T Q x + q^T x \\
     *    subject to & \\
     *               & A_{ineq} x \leq b_{ineq} \\
     *               & A_{eq} x = b_{eq} \\
     *               & x_{opt_{lb}} \leq x \leq x_{opt_{ub}}
     * \f}
     * We use the off-the-shelf QP solver eigen-quadprog in order to solve it.
     * @see DcmVrpPlanner for the full formulation.
     *
     * @param current_step_location is the location of the previous foot step
     * location (2d vector) [ux, uy].
     * @param time_from_last_step_touchdown is the time elapsed since the last
     * foot step landed.
     * @param is_left_leg_in_contact is true is the current foot is on the left
     * side, false if it is on the right side.
     * @param v_des
     * @param com is the CoM position.
     * @param com_vel is the CoM velocity.
     * @param yaw
     */
    void update(const Eigen::Ref<const Eigen::Vector3d>& current_step_location,
                const double& time_from_last_step_touchdown,
                const bool& is_left_leg_in_contact,
                const Eigen::Ref<const Eigen::Vector3d>& v_des,
                const Eigen::Ref<const Eigen::Vector3d>& com,
                const Eigen::Ref<const Eigen::Vector3d>& com_vel,
                const double& yaw,
                const double& new_t_min);

    /**
     * @brief Solve the Quadratic program and extract the solution. Use
     * DcmVrpPlanner::get_next_step_location() and
     * DcmVrpPlanner::get_duration_before_step_landing() to access the results.
     */
    bool solve();

    /**
     * @brief Perform an internal checks on the solver matrices. A warning
     * message is displayed in case of problems.
     *
     * @return true is everything seems fine.
     * @return false is something wrong.
     */
    bool internal_checks();

    /**
     * @brief Display the matrices of the Problem.
     */
    void print_solver() const;

    /**
     * @brief Convert the inner data to a string format.
     */
    std::string to_string() const;

    /*
     * Getters
     */

    /**
     * @brief @copydoc DcmVrpPlanner::t_nom_
     *
     * @return const double&
     */
    const double& get_t_nom() const
    {
        return t_nom_;
    }

    /**
     * @brief @copydoc DcmVrpPlanner::tau_nom_
     *
     * @return const double&
     */
    const double& get_tau_nom() const
    {
        return tau_nom_;
    }

    /**
     * @brief @copydoc DcmVrpPlanner::l_nom_
     *
     * @return const double&
     */
    const double& get_l_nom() const
    {
        return l_nom_;
    }

    /**
     * @brief @copydoc DcmVrpPlanner::w_nom_
     *
     * @return const double&
     */
    const double& get_w_nom() const
    {
        return w_nom_;
    }

    /**
     * @brief @copydoc DcmVrpPlanner::bx_nom_
     *
     * @return const double&
     */
    const double& get_bx_nom() const
    {
        return bx_nom_;
    }

    /**
     * @brief @copydoc DcmVrpPlanner::by_nom_
     *
     * @return const double&
     */
    const double& get_by_nom() const
    {
        return by_nom_;
    }

    /**
     * @brief @copydoc DcmVrpPlanner::world_M_local_
     *
     * @return const pinocchio::SE3&
     */
    const pinocchio::SE3& get_world_M_local() const
    {
        return world_M_local_;
    }

    /**
     * @brief @copydoc DcmVrpPlanner::dcm_local_
     *
     * @return Eigen::Ref<const Eigen::Vector3d>
     */

    Eigen::Vector3d dcm_local_REF;
    Eigen::Ref<const Eigen::Vector3d> get_dcm_local()
    {
        dcm_local_REF = world_M_local_.act(dcm_local_);
        dcm_local_REF = dcm_local_;
        return dcm_local_REF;
    }

    /**
     * @brief @copydoc DcmVrpPlanner::current_step_location_local_
     *
     * @return Eigen::Ref<const Eigen::Vector3d>
     */
    Eigen::Ref<const Eigen::Vector3d> get_current_step_location_local() const
    {
        return current_step_location_local_;
    }

    /**
     * @brief @copydoc DcmVrpPlanner::v_des_local_
     *
     * @return Eigen::Ref<const Eigen::Vector3d>
     */
    Eigen::Ref<const Eigen::Vector3d> get_v_des_local() const
    {
        return v_des_local_;
    }

    /**
     * @brief @copydoc DcmVrpPlanner::dcm_nominal_
     *
     * @return Eigen::Ref<const Eigen::Vector3d>
     */
    Eigen::Ref<const Eigen::Vector3d> get_dcm_nominal() const
    {
        return dcm_nominal_;
    }

    /*
     * Output
     */

    /**
     * @brief @copydoc DcmVrpPlanner::next_step_location_
     *
     * @return Eigen::Ref<const Eigen::Vector3d>
     */
    Eigen::Ref<const Eigen::Vector3d> get_next_step_location() const
    {
        return next_step_location_;
    }

    /**
     * @brief Return the slack values from the last solution.
     **/
    const Eigen::Vector4d& get_slack_variables() const
    {
        return slack_variables_;
    }

    /**
     * @brief @copydoc DcmVrpPlanner::duration_before_step_landing_
     *
     * @return const double&
     */
    const double& get_duration_before_step_landing() const
    {
        return duration_before_step_landing_;
    }

    /**
     * @brief Get the desired com height.
     *
     * @return const double&
     */
    const double& get_com_height() const
    {
        return ht_;
    }

    /**
     * @brief return cost.
     *
     * @return double
     */
    double cost()
    {
        return (0.5 * x_opt_.transpose() * Q_ * x_opt_ +
                q_.transpose() * x_opt_)(0, 0);
    }

    /**
     * @brief add time equation for fixing tau.
     */
    void add_t_eq(double time)
    {
        A_eq_(2, 2) = 1;
        B_eq_(2) = exp(omega_ * time);
    }

    /*
     * Private methods
     */
private:
    /**
     * @brief Compute the nominal step location from the user input.
     *
     * @param is_left_leg_in_contact [in] is used notably to define which
     * surface to use for the next contact.
     * - 1 if left leg is in contact
     * - 2 if right leg is in contact
     * @param v_des_local in the local frame.
     */
    void compute_nominal_step_values(
        const bool& is_left_leg_in_contact,
        const Eigen::Ref<const Eigen::Vector3d>& v_des_local);

    /*
     * Attributes
     */
private:
    /** @brief Minimum step length in the x direction (in the direction of
     * forward motion). */
    double l_min_;

    /** @brief Maximum step length in the x direction (in the direction of
     * forward motion). */
    double l_max_;

    /** @brief Nominal step length in the x direction (in the direction of
     * forward motion). */
    double l_nom_;

    /** @brief Minimum step length in the y direction (in the lateral
     * direction).
     */
    double w_min_;

    /** @brief Maximum step length in the y direction (in the lateral
     * direction).
     */
    double w_max_;

    /** @brief Nominal step length in the y direction (in the lateral
     * direction).
     */
    double w_nom_;

    /** @brief Minimum step time. */
    double t_min_;

    /** @brief Maximum step time. */
    double t_max_;

    /** @brief Nominal step time. */
    double t_nom_ = 0.1;

    /** @brief Minimum step time in logarithmic scale:
     * \f$ e^{\omega t_{nom}} \f$*/
    double tau_min_;

    /** @brief Maximum step time in logarithmic scale:
     * \f$ e^{\omega t_{nom}} \f$*/
    double tau_max_;

    /** @brief Nominal step time in logarithmic scale:
     * \f$ e^{\omega t_{nom}} \f$*/
    double tau_nom_;

    /** @brief Default step width. */
    double l_p_;

    /** @brief Average desired height of the com above the ground. */
    double ht_;

    /** @brief Natural frequency of the pendulum: \f$ \omega =
     * \sqrt{\frac{g}{z_0}} \f$. */
    double omega_;

    /** @brief Maximum DCM offset along the X-axis. */
    double bx_max_;

    /** @brief Minimum DCM offset along the X-axis. */
    double bx_min_;

    /** @brief Nominal DCM offset along the Y-axis. */
    double bx_nom_;

    /** @brief Maximum DCM offset along the Y-axis. */
    double by_max_out_;

    /** @brief Minimum DCM offset along the Y-axis. */
    double by_max_in_;

    /** @brief Nominal DCM offset along the Y-axis. */
    double by_nom_;

    /** @brief SE3 position of the robot base in the world frame. */
    pinocchio::SE3 world_M_local_;

    /** @brief Current DCM computed from the CoM estimation. */
    Eigen::Vector3d dcm_local_;

    /** @brief Nominal DCM computed from the CoM estimation and nominal time. */
    Eigen::Vector3d dcm_nominal_;

    /** @brief Current DCM computed from the CoM estimation. */
    Eigen::Vector3d current_step_location_local_;

    /** @brief Current DCM computed from the CoM estimation. */
    Eigen::Vector3d v_des_local_;

    /** @brief Store the time from last step touchdown in order to stop
     * optimizing after t_min_ is passed. */
    double time_from_last_step_touchdown_;

    /*
     * Problem results
     */

    /** @brief Next step location expressed in the world frame. */
    Eigen::Vector3d next_step_location_;

    /** @brief Slack variable values corresponding to last solution. */
    Eigen::Vector4d slack_variables_;

    /** @brief Remaining time before the step landing. */
    double duration_before_step_landing_;

    /**
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
     * @see DcmVrpPlanner::compute_adapted_step_locations. */
    Eigen::VectorXd x_opt_;

    /** @brief Lower Bound on the solution of the optimization problem.
     * @see DcmVrpPlanner::compute_adapted_step_locations. */
    Eigen::VectorXd x_opt_lb_;

    /** @brief Upper Bound on the solution of the optimization problem.
     * @see DcmVrpPlanner::compute_adapted_step_locations. */
    Eigen::VectorXd x_opt_ub_;

    /** @brief Quadratic term of the quadratic cost.
     * @see DcmVrpPlanner::compute_adapted_step_locations. */
    Eigen::MatrixXd Q_;

    /** @brief Cost weights expressed in the local frame. */
    Eigen::Vector9d cost_weights_local_;

    /** @brief Linear term of the quadratic cost.
     * @see DcmVrpPlanner::compute_adapted_step_locations. */
    Eigen::VectorXd q_;

    /** @brief Linear equality matrix.
     * @see DcmVrpPlanner::compute_adapted_step_locations. */
    Eigen::MatrixXd A_eq_;

    /** @brief Linear equality vector.
     * @see DcmVrpPlanner::compute_adapted_step_locations. */
    Eigen::VectorXd B_eq_;

    /** @brief Linear inequality matrix.
     * @see DcmVrpPlanner::compute_adapted_step_locations. */
    Eigen::MatrixXd A_ineq_;

    /** @brief Linear inequality vector.
     * @see DcmVrpPlanner::compute_adapted_step_locations. */
    Eigen::VectorXd B_ineq_;
};

}  // namespace reactive_planners
