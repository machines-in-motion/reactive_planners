/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2023, New York University and Max Planck
 * Gesellschaft
 *
 * @brief A C++ translation of the demos/demo_reactive_planners_solo12_step_adjustment_walk.ipynb script (the first part)
 *
 * The 4 fields to control the direction of SOLO are yaw_velocity_des, yaw_des, com_des, and v_des
 */
#pragma once

#include <Eigen/Dense>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/data.hpp>
#include "mim_control/impedance_controller.hpp"
#include "mim_control/centroidal_pd_controller.hpp"
#include "mim_control/centroidal_force_qp_controller.hpp"
#include "reactive_planners/quadruped_dcm_reactive_stepper.hpp"

namespace reactive_planners
{
class ReactivePlannersControllerSolo {
public:
    ReactivePlannersControllerSolo();

    ~ReactivePlannersControllerSolo();

    /**
     * @brief Initializes the class
     *
     * @param path_to_urdf path to SOLO's URDF to build the pinocchio model
     */
    ReactivePlannersControllerSolo(std::string path_to_urdf);

    /**
     * @brief Initializes the quadruped_dcm_reactive_stepper
     * Sets the parameters for some controllers used by the script (e.g., params related to SOLO's direction).
     *
     * @param q base + joint configuration
     * @param v_des {velocity desired in x, velocity desired in y, desired yaw velocity}
     */
    void initialize(Eigen::Matrix<double, 19, 1> &q, Eigen::Vector3d &v_des);

    /**
     * @brief Main function: computes the torques from the reactive planner
     *
     * @param q vector of 19 elements composed on [base position (3 elements), base orientation (4 elements), joint configurations (12 elements)]
     * @param dq vector of 18 elements composed of [base velocity (3), base ang vel (3), joint velocities (12)]
     * @param control_time time parameter for the quadruped_dcm_reactive_stepper
     * @param v_des {velocity desired in x, velocity desired in y, desired yaw}
     * @return torques to send to SOLO
     */
    Eigen::VectorXd compute_torques(Eigen::Matrix<double, 19, 1> &q, Eigen::Matrix<double, 18, 1> &dq, double control_time, Eigen::Vector3d &v_des);

    /**
     * @brief Start the stepping
     */
    void quadruped_dcm_reactive_stepper_start();

    /**
     * @brief Stop the stepping
     */
    void quadruped_dcm_reactive_stepper_stop();

private:

    /* Pinocchio */

    /** @brief Pinocchio model */
    pinocchio::Model model;

    /** @brief Pinoochio data */
    pinocchio::Data data;

    /* Impedance controller variables */

    /** @brief Impedance control proportional gain */
    Eigen::VectorXd kp;

    /** @brief Impedance control derivative gain */
    Eigen::VectorXd kd;

    /** @brief End effector names */
    std::vector<std::string> endeff_names;

    /** @brief Vector of impedance controllers for each foot */
    std::vector<mim_control::ImpedanceController> imp_ctrls;

    /* Centroidal Controller Variables*/

    /** @brief Friction coefficient */
    double mu;

    /** @brief Proportional gain for COM position */
    Eigen::Vector3d kc;

    /** @brief Derivative gain for COM position */
    Eigen::Vector3d dc;

    /** @brief Proportional gain for COM orientation */
    Eigen::Vector3d kb;

    /** @brief Derivative gain for COM orientation */
    Eigen::Vector3d db;

    /** @brief QP linear penalty */
    Eigen::Vector3d qp_penalty_lin;

    /** @brief QP angular penalty */
    Eigen::Vector3d qp_penalty_ang;

    /** @brief linear velocity desired */
    Eigen::Vector3d linear_vel_des;

    /** @brief Centroidal PD controller */
    mim_control::CentroidalPDController centrl_pd_ctrl;

    /** @brief Quadratic program for desired wrench */
    mim_control::CentroidalForceQPController force_qp;

    /* Quadruped DCM reactive stepper */

    /** @brief Left foot is in contact with the ground, if
     * not then the right foot is. */
    bool is_left_leg_in_contact;

    /** @brief Lower bound in the forward direction where to step. */
    double l_min;

    /** @brief Upper bound in the forward direction where to step. */
    double l_max;

    /** @brief Lower bound in the lateral direction where to step. */
    double w_min;

    /** @brief Upper bound in the lateral direction where to step. */
    double w_max;

    /** @brief The minimum time required to step. */
    double t_min;

    /** @brief The maximum time required to step. */
    double t_max;

    /** @brief The nominal stride length. */
    double l_p;

    /** @brief Center of mass height from the ground. */
    double com_height;

    /** @brief Total weight of the robot. */
    Eigen::VectorXd weight;

    /** @brief Maximum height of the foot in mid-air. */
    double mid_air_foot_height;

    /** @brief Robot control period */
    double control_period;

    /** @brief Period of the end-effector trajectory update */
    double planner_loop;

    /** @brief Quadruped DCM reactive stepper*/
    QuadrupedDcmReactiveStepper quadruped_dcm_reactive_stepper;

    /* fields related to the direction of SOLO */

    /** @brief desired speed of yaw angle */
    double yaw_velocity_des;

    /** @brief desired yaw */
    double yaw_des;

    /** @brief desired base position */
    Eigen::Vector3d com_des;

    bool open_loop;

    /* Other variables */

    /** @brief 3D front left foot position */
    Eigen::Vector3d front_left_foot_position = Eigen::Vector3d::Zero();

    /** @brief 3D front right foot position */
    Eigen::Vector3d front_right_foot_position = Eigen::Vector3d::Zero();

    /** @brief 3D hind left foot position */
    Eigen::Vector3d hind_left_foot_position = Eigen::Vector3d::Zero();

    /** @brief 3D hind right foot position */
    Eigen::Vector3d hind_right_foot_position = Eigen::Vector3d::Zero();

    /** @brief Desired feet positions in local frame */
    Eigen::Matrix<double, 12, 1> x_des_local = Eigen::Matrix<double, 12, 1>::Zero(12);

    /** @brief Temporary variable holding the position of a foot*/
    Eigen::Vector3d foot_des_local = Eigen::Vector3d::Zero();

    /** @brief The desired pos of a foot */
    Eigen::Vector3d desired_pos;

    /** @brief The desired velocity of a foot */
    pinocchio::Motion xd_des;

    /** @brief Impedance proportional gains */
    Eigen::Matrix<double, 6, 1> kp_array;

    /** @brief Impedance derivative gains */
    Eigen::Matrix<double, 6, 1> kd_array;

    /**
     * @brief Extracts the yaw from the current position of the robot
     *
     * @param q The base position, orientation, and joint positions of the robot
     * @return double The yaw of the robot
     */
    static double yaw(Eigen::Matrix<double, 19, 1> &q);

    bool print_once = true;
};

}  // namespace reactive_planners
