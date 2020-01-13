/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Implements the "Walking Control Based on Step Timing Adaptation" step
 * planner.
 */

#pragma once

#include <eigen-quadprog/QuadProg.h>
#include <Eigen/Eigen>

namespace Eigen {
typedef Matrix<double, 5, 1> Vector5d;
typedef Matrix<double, 5, 5> Matrix5d;
}  // namespace Eigen

namespace reactive_planners {

class DcmVrpPlanner {
 public:
  /**
   * @brief Construct a new DcmVrpPlanner object and initialize it.
   *
   * @param l_min [in] Minimum step length in the x direction (in the direction
   * of forward motion).
   * @param l_max [in] Maximum step length in the x direction (in the direction
   * of forward motion).
   * @param w_min [in] Minimum step length in the y direction (in the lateral
   * direction).
   * @param w_max [in] Maximum step lenght in the y direction (in the lateratl
   * direction).
   * @param t_min [in] Minimum step time.
   * @param t_max [in] Maximum step time.
   * @param v_des [in] Desired average velocity in the x and y ([v_x, v_y]) 2d
   * vector.
   * @param l_p [in] Default step width.
   * @param ht [in] Average desired height of the com above the ground.
   */
  DcmVrpPlanner(const double& l_min, const double& l_max, const double& w_min,
                const double& w_max, const double& t_min, const double& t_max,
                const Eigen::Vector2d& v_des, const double& l_p,
                const double& ht);

  /**
   * @brief Construct a new DcmVrpPlanner object with some default parameters.
   */
  DcmVrpPlanner() {
    initialize(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Eigen::Vector2d::Zero(), 0.0, 0.0);
  }

  /**
   * @brief Construct a new DcmVrpPlanner object and initialize it.
   *
   * @param l_min [in] Minimum step length in the x direction (in the direction
   * of forward motion).
   * @param l_max [in] Maximum step length in the x direction (in the direction
   * of forward motion).
   * @param w_min [in] Minimum step length in the y direction (in the lateral
   * direction).
   * @param w_max [in] Maximum step lenght in the y direction (in the lateratl
   * direction).
   * @param t_min [in] Minimum step time.
   * @param t_max [in] Maximum step time.
   * @param v_des [in] Desired average velocity in the x and y ([v_x, v_y]) 2d
   * vector.
   * @param l_p [in] Default step width.
   * @param ht [in] Average desired height of the com above the ground.
   */
  void initialize(const double& l_min, const double& l_max, const double& w_min,
                  const double& w_max, const double& t_min, const double& t_max,
                  const Eigen::Vector2d& v_des, const double& l_p,
                  const double& ht);

  /**
   * @brief
   * Input:
   *     u :
   *     t : time elapsed after the previous step has occured
   *     n : 1 if left leg and 2 if right le is in contact
   *     psi_current : current dcm location [psi_x, psi_y]
   *     W : wieght array 5d
   */

  /**
   * @brief Computes adapted step location solving a QP. We use the following
   * notation:
   * \f{eqnarray*}{
   *    minimize   & \\
   *       x       & (1/2) x^T Q x + q^T x \\
   *    subject to & \\
   *               & A_{ineq} x \leq b_{ineq} \\
   *               & A_{eq} x = b_{eq} \\
   *               & x_{opt_{lb}} \leq x \leq x_{opt_{ub}}
   * \f}
   *
   * We use the off-the-shelf QP solver eigen-quadprog in order to solve this.
   *
   * @param current_support_location is the location of the previous foot step
   * location (2d vector) [ux, uy].
   * @param time_from_last_step_touchdown is the time elapsed since the last
   * foot step landed.
   * @param is_left_leg_in_contact is true is the current foot is on the left
   * side, false if it is on the right side.
   * @param com_meas is the CoM position.
   * @param com_vel_meas is the CoM velocity.
   * @param cost_weights is the weights.
   */
  void compute_adapted_step_locations(
      const Eigen::Vector2d& current_step_location,
      const double& time_from_last_step_touchdown,
      const bool& is_left_leg_in_contact, const Eigen::Vector3d& com_meas,
      const Eigen::Vector3d& com_vel_meas, const Eigen::Vector5d& cost_weights);

  /**
   * Private methods
   */
 private:
  /**
   * @brief Compute the nominal step location from the user input.
   *
   * @param is_left_leg_in_contact [in] is used notably to define which surface
   * to use for the next contact.
   * - 1 if left leg is in contact
   * - 2 if right leg is in contact
   */
  void compute_nominal_step_values(const double& is_left_leg_in_contact);

  /**
   * @brief Computes the current location of the dcm.
   *
   * @param com [in] center of mass location at current time step
   * @param com_vel [in] center of mass velocity of current time step
   */
  void compute_current_dcm(const Eigen::Vector3d& com_meas,
                           const Eigen::Vector3d& com_vel_meas);

  /**
   * Attributes
   */
 private:
  /** @brief Minimum step length in the x direction (in the direction of
   * forward motion). */
  double l_min_;

  /** @brief Maximum step length in the x direction (in the direction of forward
   * motion). */
  double l_max_;

  /** @brief Nominal step length in the x direction (in the direction of forward
   * motion). */
  double l_nom_;

  /** @brief Minimum step length in the y direction (in the lateral direction).
   */
  double w_min_;

  /** @brief Maximum step length in the y direction (in the lateratl direction).
   */
  double w_max_;

  /** @brief Nominal step length in the y direction (in the lateratl direction).
   */
  double w_nom_;

  /** @brief Minimum step time. */
  double t_min_;

  /** @brief Maximum step time. */
  double t_max_;

  /** @brief Nominal step time. */
  double t_nom_;

  /** @brief Nominal step time in logarithmic scale:
   * \f$ e^{\omega t_{nom}} \f$*/
  double tau_nom_;

  /** @brief Desired average velocity in the x and y ([v_x, v_y]) 2d vector. */
  Eigen::Vector2d v_des_;

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

  /** @brief Current DCM computed from the CoM estimation. */
  Eigen::Vector2d dcm_meas_;

  /** @brief Current CoM position. */
  Eigen::Vector3d com_meas_;

  /** @brief Current CoM velocity. */
  Eigen::Vector3d com_vel_meas_;

  /**
   * QP variables
   */

  /** @brief Solution of the optimization problem,
   * @see DcmVrpPlanner::compute_adapted_step_locations */
  Eigen::VectorXd x_opt_;

  /** @brief Lower Bound on the solution of the optimization problem,
   * @see DcmVrpPlanner::compute_adapted_step_locations */
  Eigen::VectorXd x_opt_lb_;

  /** @brief Upper Bound on the solution of the optimization problem,
   * @see DcmVrpPlanner::compute_adapted_step_locations */
  Eigen::VectorXd x_opt_ub_;

  /** @brief Quadratic term of the quadratic cost,
   * @see DcmVrpPlanner::compute_adapted_step_locations */
  Eigen::MatrixXd Q_;

  /** @brief Linear term of the quadratic cost,
   * @see DcmVrpPlanner::compute_adapted_step_locations */
  Eigen::VectorXd q_;

  /** @brief Linear equality matrix,
   * @see DcmVrpPlanner::compute_adapted_step_locations */
  Eigen::MatrixXd A_eq_;

  /** @brief Linear equality vector,
   * @see DcmVrpPlanner::compute_adapted_step_locations */
  Eigen::VectorXd B_eq_;

  /** @brief Linear inequality matrix,
   * @see DcmVrpPlanner::compute_adapted_step_locations */
  Eigen::MatrixXd A_ineq_;
  
  /** @brief Linear inequality vector,
   * @see DcmVrpPlanner::compute_adapted_step_locations */
  Eigen::VectorXd B_ineq_;
};

}  // namespace reactive_planners
