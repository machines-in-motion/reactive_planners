//
// Created by edaneshmand on 18.11.20.
//

#include "reactive_planners/com_planner.hpp"

namespace reactive_planners
{

    ComPlanner::ComPlanner()
    {
        com_.setZero();
        com_d_.setZero();
        com_dd_.setZero();
        com_mea_.setZero();
        com_d_mea_.setZero();
        com_dd_mea_.setZero();
        last_x_com_.setZero();
        last_xd_com_.setZero();
        r_vrp_current_step_.setZero();
        r_vrp_.setZero();
    }

    ComPlanner::~ComPlanner()
    {
    }

    void ComPlanner::update_com_single_support(double omega, double time,
                                               Eigen::Ref<const Eigen::Vector3d> u_current_step,
                                               Eigen::Ref<const Eigen::Vector3d> previous_x_com,
                                               Eigen::Ref<const Eigen::Vector3d> previous_xd_com){
        if(time < 0.01){
//            std::cout << "I am updating variales *****************\n";
            last_x_com_ = previous_x_com;
            last_xd_com_ = previous_xd_com;
            r_vrp_current_step_ = u_current_step;
            r_vrp_current_step_[2] += 9.81 / (omega * omega);
        }
        com_ = 0.5 * (last_x_com_ - r_vrp_current_step_ + last_xd_com_ / omega) * exp(omega * time) +
               0.5 * (last_x_com_ - r_vrp_current_step_ - last_xd_com_ / omega) * exp(-omega * time) +
               r_vrp_current_step_;
        com_d_ = omega * 0.5 * (last_x_com_ - r_vrp_current_step_ + (last_xd_com_ / omega)) * exp(omega * time) -
                 omega * 0.5 * (last_x_com_ - r_vrp_current_step_ - (last_xd_com_ / omega)) * exp(-omega * time);
        com_dd_ = pow(omega, 2) * (com_ - r_vrp_current_step_);
        com_dd_[2] += 9.81;
    }

    void ComPlanner::update_com_in_t_s_(double omega, double time,
                                                   Eigen::Ref<const Eigen::Vector3d> u_current_step,
                                                   Eigen::Ref<const Eigen::Vector3d> previous_x_com,
                                                   Eigen::Ref<const Eigen::Vector3d> previous_xd_com){
        double t_s = 0.1;
        if(time < 0.000001){
            last_x_com_ = previous_x_com;
            last_xd_com_ = previous_xd_com;
            r_vrp_current_step_ = u_current_step;
            r_vrp_current_step_[2] += 9.81 / (omega * omega);
        }
        com_mea_ = 0.5 * (last_x_com_ - r_vrp_current_step_ + last_xd_com_ / omega) * exp(omega * t_s) +
                   0.5 * (last_x_com_ - r_vrp_current_step_ - last_xd_com_ / omega) * exp(-omega * t_s) +
                   r_vrp_current_step_;
        com_d_mea_ = omega * 0.5 * (last_x_com_ - r_vrp_current_step_ + last_xd_com_ / omega) * exp(omega * t_s) -
                     omega * 0.5 * (last_x_com_ - r_vrp_current_step_ - (last_xd_com_ / omega)) * exp(-omega * t_s);
        com_dd_mea_ = pow(omega, 2) * (last_x_com_ - r_vrp_current_step_);

    }

    void ComPlanner::update_com_double_support(Eigen::Ref<const Eigen::Vector3d> start_pos,
                                               Eigen::Ref<const Eigen::Vector3d> start_vel,
                                               Eigen::Ref<const Eigen::Vector3d> end_pos,
                                               Eigen::Ref<const Eigen::Vector3d> end_vel,
                                               double time, double end_time){
        Eigen::Vector3d x[4];
        x[2] = start_vel;
        x[3] = start_pos;
        x[0] = (-start_vel * end_time - start_pos + end_pos - end_time * end_time * (end_vel - start_vel) / 2 * end_time) / (2.5 * end_time * end_time * end_time);
        x[1] = (end_vel - start_vel - 3 * x[0] * end_time * end_time) / 2 * end_time;
        com_ = x[0] * time * time * time + x[1] * time * time + x[2] * time + x[3];
        com_d_ = 3 * x[0] * time * time + 2 * x[1] * time + x[2];
        com_dd_ = 6 * x[0] * time + 2 * x[1];
    }

}