#include "reactive_planners/demo_reactive_planners_solo12_step_adjustment.hpp"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/math/rpy.hpp>
#include <cmath>

DemoReactivePlanner::DemoReactivePlanner() {}

DemoReactivePlanner::DemoReactivePlanner(std::string path_to_urdf) {
    // build the pinocchio model
    pinocchio::JointModelFreeFlyer root_joint;
    pinocchio::urdf::buildModel(path_to_urdf, root_joint, model);
    data = pinocchio::Data(model);

    // parameters for the centroidal controller
    mu = 0.6; // friction coef
    kc = {0.0, 0.0, 200.0}; // base position coef
    dc = {10.0, 10.0, 10.0}; // base velocity coef
    kb = {25.0, 25.0, 25.0}; // orientation coef
    db = {22.5, 22.5, 22.5}; // angular velocity coef
    // parameters for the centroidal force QP controller
    qp_penalty_lin = {1e0, 1e0, 1e6};
    qp_penalty_ang = {1e6, 1e6, 1e6};
    Eigen::VectorXd qp_penalty_weights(qp_penalty_lin.size() + qp_penalty_ang.size());
    qp_penalty_weights << qp_penalty_lin, qp_penalty_ang;

    // initialize the centroidal controllers
    centrl_pd_ctrl = mim_control::CentroidalPDController();
    force_qp = mim_control::CentroidalForceQPController();
    force_qp.initialize(4, mu, qp_penalty_weights);

    // parameters for the impedance controllers
    kp = Eigen::VectorXd::Zero(12);
    kd = Eigen::VectorXd::Zero(12);
    kp << 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0; // joint pos coef
    kd << 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0; // joint vel coef

    // initialize the impedance controllers and the reactive planner
    std::vector<std::string> frame_root_names = {"FL_HFE", "FR_HFE", "HL_HFE", "HR_HFE"};
    endeff_names = {"FL_FOOT", "FR_FOOT", "HL_FOOT", "HR_FOOT"};
    imp_ctrls = {mim_control::ImpedanceController(),
                 mim_control::ImpedanceController(),
                 mim_control::ImpedanceController(),
                 mim_control::ImpedanceController()};
    for (int i = 0; i < 4; i++) {
        imp_ctrls[i].initialize(model, frame_root_names[i], endeff_names[i]);
    }
    quadruped_dcm_reactive_stepper = reactive_planners::QuadrupedDcmReactiveStepper();
}

void DemoReactivePlanner::initialize(Eigen::Matrix<double, 19, 1> &q, std::string direction) {
    // initialize fields for the quadruped Dcm reactive stepper
    open_loop = true;
    is_left_leg_in_contact = true;
    l_min = -0.1;
    l_max = 0.1;
    w_min = -0.08;
    w_max = 0.2;
    t_min = 0.1;
    t_max = 1.0;
    l_p = 0.00;
    com_height = q(2);
    weight = Eigen::VectorXd::Zero(9);
    weight << 1, 1, 5, 1000, 1000, 100000, 100000, 100000, 100000;
    mid_air_foot_height = 0.05;
    control_period = 0.001;
    planner_loop = 0.010;
    pinocchio::framesForwardKinematics(model, data, q);
    Eigen::VectorXd base_pose = q.head(7);
    front_left_foot_position = data.oMf[imp_ctrls[0].get_endframe_index()].translation();
    front_right_foot_position = data.oMf[imp_ctrls[1].get_endframe_index()].translation();
    hind_left_foot_position = data.oMf[imp_ctrls[2].get_endframe_index()].translation();
    hind_right_foot_position = data.oMf[imp_ctrls[3].get_endframe_index()].translation();

    // centroidal initialization
    data.M.fill(0);
    pinocchio::crba(model, data, q);
    Eigen::Vector3d inertia = data.M.block<3, 3>(3, 3).diagonal();
    centrl_pd_ctrl.initialize(2.5, inertia);

    // reactive planner initialization
    quadruped_dcm_reactive_stepper.initialize(
            is_left_leg_in_contact,
            l_min,
            l_max,
            w_min,
            w_max,
            t_min,
            t_max,
            l_p,
            com_height,
            weight,
            mid_air_foot_height,
            control_period,
            planner_loop,
            base_pose,
            front_left_foot_position,
            front_right_foot_position,
            hind_left_foot_position,
            hind_right_foot_position
    );

    // default parameters related to the direction of SOLO
    y_des = 0.0;
    yaw_des = yaw(q);
    v_des = {0.0, 0.0, 0.0};
    com_des = {q(0), q(1), com_height};
    if (direction == "forward") {
        v_des(0) = 0.2;
    } else if (direction == "left") {
        v_des(1) = 0.2;
    } else if (direction == "right") {
        v_des(1) = -0.2;
    }

    quadruped_dcm_reactive_stepper.set_desired_com_velocity(v_des);
    quadruped_dcm_reactive_stepper.set_polynomial_end_effector_trajectory();
    quadruped_dcm_reactive_stepper.set_steptime_nominal(0.13);

    print_once = true;
}

Eigen::VectorXd DemoReactivePlanner::compute_torques(Eigen::Matrix<double, 19, 1> &q, Eigen::Matrix<double, 18, 1> &dq,
                                                     double control_time, const std::string& direction) {
    // transform the base velocity to the local frame
    Eigen::Quaterniond curr_quat(q(6), q(3), q(4), q(5));
    curr_quat.normalize();
    Eigen::Vector3d local_base_vel = curr_quat.toRotationMatrix().transpose() * dq.head(3);
    Eigen::Vector3d local_base_ang_vel = curr_quat.toRotationMatrix().transpose() * dq.segment(3, 3);
    dq[0] = local_base_vel(0);
    dq[1] = local_base_vel(1);
    dq[2] = local_base_vel(2);
    dq[3] = local_base_ang_vel(0);
    dq[4] = local_base_ang_vel(1);
    dq[5] = local_base_ang_vel(2);

    // update pinocchio
    pinocchio::framesForwardKinematics(model, data, q);
    pinocchio::computeCentroidalMomentum(model, data, q, dq);

    // get x_com and xd_com
    Eigen::Matrix<double, 3, 1, 0> x_com = data.com[0];
    Eigen::MatrixXd xd_com = data.vcom[0];

    // update the relevant parameters related to SOLO's direction
    if (direction == "forward") {
        com_des(0) = q(0) + v_des(0) * 0.001;
    } else if (direction == "left" || direction == "right") {
        com_des(1) = q(1) + v_des(1) * 0.001;
    } else if (direction == "turn_left") {
        y_des = 0.2;
        yaw_des += 0.001 * y_des;
    } else if (direction == "turn_right") {
        y_des = 0.2;
        yaw_des -= 0.001 * y_des;
    }

    // get feet position
    front_left_foot_position = data.oMf[imp_ctrls[0].get_endframe_index()].translation();
    front_right_foot_position = data.oMf[imp_ctrls[1].get_endframe_index()].translation();
    hind_left_foot_position = data.oMf[imp_ctrls[2].get_endframe_index()].translation();
    hind_right_foot_position = data.oMf[imp_ctrls[3].get_endframe_index()].translation();

    // get feet velocity
    Eigen::Vector3d front_left_foot_velocity = pinocchio::getFrameVelocity(model, data,
                                                                           imp_ctrls[0].get_endframe_index(),
                                                                           pinocchio::LOCAL_WORLD_ALIGNED).linear();
    Eigen::Vector3d front_right_foot_velocity = pinocchio::getFrameVelocity(model, data,
                                                                            imp_ctrls[1].get_endframe_index(),
                                                                            pinocchio::LOCAL_WORLD_ALIGNED).linear();
    Eigen::Vector3d hind_left_foot_velocity = pinocchio::getFrameVelocity(model, data,
                                                                          imp_ctrls[2].get_endframe_index(),
                                                                          pinocchio::LOCAL_WORLD_ALIGNED).linear();
    Eigen::Vector3d hind_right_foot_velocity = pinocchio::getFrameVelocity(model, data,
                                                                           imp_ctrls[3].get_endframe_index(),
                                                                           pinocchio::LOCAL_WORLD_ALIGNED).linear();

    // compute x_des local
    quadruped_dcm_reactive_stepper.run(
            control_time,
            front_left_foot_position,
            front_right_foot_position,
            hind_left_foot_position,
            hind_right_foot_position,
            front_left_foot_velocity,
            front_right_foot_velocity,
            hind_left_foot_velocity,
            hind_right_foot_velocity,
            x_com,
            xd_com,
            yaw(q),
            !open_loop
    );

    x_des_local <<
            quadruped_dcm_reactive_stepper.get_front_left_foot_position(),
            quadruped_dcm_reactive_stepper.get_front_right_foot_position(),
            quadruped_dcm_reactive_stepper.get_hind_left_foot_position(),
            quadruped_dcm_reactive_stepper.get_hind_right_foot_position();

    Eigen::Vector4d contact_array = quadruped_dcm_reactive_stepper.get_contact_array(); // cnt_array

    for (int j = 0; j < 4; j++) {
        foot_des_local = data.oMf[imp_ctrls[j].get_rootframe_index()].translation();
        x_des_local.segment(3 * j, 3) = x_des_local.segment(3 * j, 3) - foot_des_local;
    }

    // compute w_com
    double roll = 0.0;
    double pitch = 0.0;
    Eigen::Quaterniond x_ori;
    x_ori = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(yaw_des, Eigen::Vector3d::UnitZ());
    Eigen::Vector3d x_angvel = {0.0, 0.0, y_des};
    Eigen::Quaterniond curr_orientation = Eigen::Quaterniond(q(6), q(3), q(4), q(5));
    curr_orientation.normalize();
    Eigen::MatrixXd curr_rot = curr_orientation.toRotationMatrix();

    centrl_pd_ctrl.run(
            kc,
            dc,
            kb,
            db,
            q.head(3),
            com_des,
            curr_rot * dq.head(3), // rotate to world frame
            v_des,
            q.segment(3, 4),
            x_ori.coeffs(),
            dq.segment(3, 3),
            x_angvel
    );

    Eigen::VectorXd w_com = Eigen::VectorXd::Zero(6);
    w_com(2) = w_com(2) + 9.81 * 2.5;
    w_com = w_com + centrl_pd_ctrl.get_wrench();

    // compute ee_forces
    Eigen::Vector3d com = pinocchio::centerOfMass(model, data, q);
    Eigen::VectorXd rel_eff = Eigen::VectorXd::Zero(12);

    int i = 0;
    for (const std::string &eff_name: endeff_names) {
        int id = model.getFrameId(eff_name);
        Eigen::Vector3d diff = data.oMf[id].translation() - q.head(3);
        rel_eff(i * 3) = diff(0);
        rel_eff(i * 3 + 1) = diff(1);
        rel_eff(i * 3 + 2) = diff(2);
        i++;
    }

    force_qp.run(w_com, rel_eff, contact_array);
    Eigen::VectorXd ee_forces = force_qp.get_forces();

    // get des_vel
    Eigen::VectorXd des_vel = Eigen::VectorXd::Zero(12);
    des_vel << quadruped_dcm_reactive_stepper.get_front_left_foot_velocity(),
            quadruped_dcm_reactive_stepper.get_front_right_foot_velocity(),
            quadruped_dcm_reactive_stepper.get_hind_left_foot_velocity(),
            quadruped_dcm_reactive_stepper.get_hind_right_foot_velocity();

    // passing forces to the impedance controller
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(12);

    for (int i = 0; i < 4; i++) {
        desired_pos = x_des_local.segment(3 * i, 3);
        xd_des = pinocchio::Motion(des_vel.segment(3 * i, 3), Eigen::Vector3d({0, 0, 0}));
        if (contact_array(i) == 1) {
            kp_array = Eigen::VectorXd::Zero(6);
            kd_array = Eigen::VectorXd::Zero(6);
        } else {
            kp_array << kp.segment(3 * i, 3), 0, 0, 0;
            kd_array << kd.segment(3 * i, 3), 0, 0, 0;
        }
        imp_ctrls[i].run_local(
                q,
                dq,
                kp_array.array(),
                kd_array.array(),
                1.0,
                pinocchio::SE3(Eigen::Matrix3d::Identity(), desired_pos),
                xd_des,
                pinocchio::Force(ee_forces.segment(3 * i, 3), Eigen::Vector3d::Zero(3))
        );
        tau = tau + imp_ctrls[i].get_joint_torques();
    }

    // if we want to debug
//    if (print_once) {
//        std::cout << "######################################" << std::endl;
//        std::cout << "control time = " << control_time << std::endl;
//        std::cout << "q = " << q << std::endl;
//        std::cout << "dq = " << dq << std::endl;
//        std::cout << "base position = " << q.head(3) << std::endl;
//        std::cout << "base velocity = " << dq.head(3) << std::endl;
//        std::cout << "dq.head(3) = " << dq.head(3) << std::endl;
//        std::cout << "current orientation = " << q.segment(3, 4) << std::endl;
//        std::cout << "current yaw = " << yaw(q) << std::endl;
//        std::cout << "angular velocity = " << dq.segment(3, 3) << std::endl;
//        std::cout << "x_des_local = " << x_des_local << std::endl;
//        std::cout << "des_vel = " << des_vel << std::endl;
//        std::cout << "yaw_des = " << yaw_des << std::endl;
//        std::cout << "com_des = " << com_des << std::endl;
//        std::cout << "v_des = " << v_des << std::endl;
//        std::cout << "desired orientation = " << x_ori.coeffs() << std::endl;
//        std::cout << "desired angular velocity = " << x_angvel << std::endl;
//        std::cout << "w_com = " << w_com << std::endl;
//        std::cout << "F = " << ee_forces << std::endl;
//        std::cout << "tau = " << tau << std::endl;
//        std::cout << "######################################" << std::endl;
//        print_once = false;
//    }

    return tau;
}

double DemoReactivePlanner::yaw(Eigen::Matrix<double, 19, 1> &q) {
    Eigen::Vector4d quat = q.segment(3, 4);
    double x = quat(0);
    double y = quat(1);
    double z = quat(2);
    double w = quat(3);
    return atan2(2.0f * (w * z + x * y), w * w + x * x - y * y - z * z);
}

void DemoReactivePlanner::quadruped_dcm_reactive_stepper_start() {
    quadruped_dcm_reactive_stepper.start();
}

void DemoReactivePlanner::quadruped_dcm_reactive_stepper_stop() {
    quadruped_dcm_reactive_stepper.stop();
}