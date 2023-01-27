#include <Eigen/Dense>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/data.hpp>
#include "mim_control/impedance_controller.hpp"
#include "mim_control/centroidal_pd_controller.hpp"
#include "mim_control/centroidal_force_qp_controller.hpp"
#include "reactive_planners/quadruped_dcm_reactive_stepper.hpp"

/**
 * A C++ translation of https://github.com/machines-in-motion/reactive_planners/blob/master/demos/demo_reactive_planners_solo12_step_adjustment_walk.ipynb (the first part)
 * The 4 fields to control the direction of SOLO are y_des, yaw_des, com_des, and v_des
 */
class DemoReactivePlanner {
public:
    DemoReactivePlanner();

    /**
     * Initializes the class
     * @param path_to_urdf path to SOLO's URDF to build the pinocchio model
     */
    DemoReactivePlanner(std::string path_to_urdf);

    /**
     * Initializes the quadruped_dcm_reactive_stepper
     * Sets the parameters for some controllers used by the script (e.g., params related to SOLO's direction).
     * @param q base + joint configuration
     * @param direction either "forward", "right", "left", "turn_right", "turn_left", "stay" (anything else will be equivalent to stay)
     */
    void initialize(Eigen::Matrix<double, 19, 1> &q, std::string direction);

    Eigen::VectorXd
    /**
     * Reactive planner script
     * @param q vector of 19 elements composed on [base position (3 elements), base orientation (4 elements), joint configurations (12 elements)]
     * @param dq vector of 18 elements composed of [base velocity (3), base ang vel (3), joint velocities (12)]
     * @param control_time parameter for self.quadruped_dcm_reactive_stepper
     * @param direction either "forward", "right", "left", "turn_right", "turn_left", "stay" (anything else will be equivalent to stay)
     * @return torques to send to SOLO
     */
    compute_torques(Eigen::Matrix<double, 19, 1> &q, Eigen::Matrix<double, 18, 1> &dq, double control_time, const std::string& direction);

    void quadruped_dcm_reactive_stepper_start();
    void quadruped_dcm_reactive_stepper_stop();

private:

    // pinocchio
    pinocchio::Model model;
    pinocchio::Data data;

    // impedance controller
    Eigen::VectorXd kp;
    Eigen::VectorXd kd;
    std::vector<std::string> endeff_names;
    std::vector<mim_control::ImpedanceController> imp_ctrls;

    // centroidal controller
    double mu;
    Eigen::Vector3d kc;
    Eigen::Vector3d dc;
    Eigen::Vector3d kb;
    Eigen::Vector3d db;
    Eigen::Vector3d qp_penalty_lin;
    Eigen::Vector3d qp_penalty_ang;
    mim_control::CentroidalPDController centrl_pd_ctrl;
    mim_control::CentroidalForceQPController force_qp;

    // Quadruped DCM reactive stepper
    bool is_left_leg_in_contact;
    double l_min;
    double l_max;
    double w_min;
    double w_max;
    double t_min;
    double t_max;
    double l_p;
    double com_height;
    Eigen::VectorXd weight;
    double mid_air_foot_height;
    double control_period;
    double planner_loop;
    reactive_planners::QuadrupedDcmReactiveStepper quadruped_dcm_reactive_stepper;

    // fields related to the direction of SOLO
    double y_des; // desired speed of yaw angle
    double yaw_des; // desired yaw
    Eigen::Vector3d v_des; // desired base speed
    Eigen::Vector3d com_des; // ddesired base position
    // more fields
    Eigen::Vector2d cnt_array;
    bool open_loop;
    Eigen::Vector3d dcm_force;

    // vectors used in the compute_torques() method
    Eigen::Vector3d front_left_foot_position = Eigen::Vector3d::Zero();
    Eigen::Vector3d front_right_foot_position = Eigen::Vector3d::Zero();
    Eigen::Vector3d hind_left_foot_position = Eigen::Vector3d::Zero();
    Eigen::Vector3d hind_right_foot_position = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 12, 1> x_des_local = Eigen::Matrix<double, 12, 1>::Zero(12);
    mim_control::ImpedanceController imp;
    Eigen::Vector3d foot_des_local = Eigen::Vector3d::Zero();

    // intermediate variables
    Eigen::Vector3d desired_pos;
    pinocchio::Motion xd_des;
    Eigen::Matrix<double, 6, 1> kp_array;
    Eigen::Matrix<double, 6, 1> kd_array;

    // helper methods
    static double yaw(Eigen::Matrix<double, 19, 1> &q);

    // prints some values for debugging
    bool print_once = true;
};