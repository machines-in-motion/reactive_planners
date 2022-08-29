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
        last_x_com_.setZero();
        last_xd_com_.setZero();
        r_vrp_current_step_.setZero();
        g_ = 9.81;
    }

    void ComPlanner::update_com_single_support(double omega, double time,
                                               Eigen::Ref<const Eigen::Vector3d> u_current_step,
                                               Eigen::Ref<const Eigen::Vector3d> previous_x_com,
                                               Eigen::Ref<const Eigen::Vector3d> previous_xd_com){
        if(time < 0.01){
            last_x_com_ = previous_x_com;
            last_xd_com_ = previous_xd_com;
            r_vrp_current_step_ = u_current_step;
            r_vrp_current_step_[2] = g_ / (omega * omega);
        }
        com_ = 0.5 * (last_x_com_ - r_vrp_current_step_ + last_xd_com_ / omega) * exp(omega * time) +
               0.5 * (last_x_com_ - r_vrp_current_step_ - last_xd_com_ / omega) * exp(-omega * time) +
               r_vrp_current_step_;
//        com_[2] += u_current_step[2];

        com_d_ = omega * 0.5 * (last_x_com_ - r_vrp_current_step_ + (last_xd_com_ / omega)) * exp(omega * time) -
                 omega * 0.5 * (last_x_com_ - r_vrp_current_step_ - (last_xd_com_ / omega)) * exp(-omega * time);
        com_dd_ = pow(omega, 2) * (com_ - r_vrp_current_step_);
        com_dd_[2] += g_;
    }

}