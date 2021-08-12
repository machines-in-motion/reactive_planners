
/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Declare a class that calculates com trajectory.
 */



#pragma once

#include <eigen-quadprog/QuadProg.h>
#include <Eigen/Eigen>
#include <pinocchio/spatial/se3.hpp>
#include <iostream>
#include <cmath>
#include <sstream>
#include <stdexcept>
#define EPSILON 1e-6

namespace reactive_planners {
/**
* @brief
*/

class ComPlanner {
    public:
        ComPlanner();
        ~ComPlanner(){};

        void update_com_single_support(double omega, double time,
                                       Eigen::Ref<const Eigen::Vector3d> u_current_step,
                                       Eigen::Ref<const Eigen::Vector3d> previous_x_com,
                                       Eigen::Ref<const Eigen::Vector3d> previous_xd_com);

        /**
         * @brief Get the right foot 3d velocity.
         *
         * @return const Eigen::Vector3d&
         */
        Eigen::Ref<const Eigen::Vector3d>  get_com() const
        {
            return com_;
        }

        /**
         * @brief Get the right foot 3d velocity.
         *
         * @return const Eigen::Vector3d&
         */
        Eigen::Ref<const Eigen::Vector3d> get_com_d() const
        {
            return com_d_;
        }

        /**
         * @brief Get the right foot 3d velocity.
         *
         * @return const Eigen::Vector3d&
         */
        Eigen::Ref<const Eigen::Vector3d> get_com_dd() const
        {
            return com_dd_;
        }


    private:
        /** @brief The center of mass position. */
        Eigen::Vector3d com_;

        /** @brief The center of mass velocity. */
        Eigen::Vector3d com_d_;

        /** @brief The center of mass acceleration. */
        Eigen::Vector3d com_dd_;

        /** @brief The previous center of mass position. */
        Eigen::Vector3d last_x_com_;

        /** @brief The previous center of mass velocity. */
        Eigen::Vector3d last_xd_com_;

        /** @brief The r_vrp_current_step. */
        Eigen::Vector3d r_vrp_current_step_;

};

}  // namespace reactive_planners
