
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
        ~ComPlanner();

        void update_com_single_support(double omega, double time,
                                       Eigen::Ref<const Eigen::Vector3d> u_current_step,
                                       Eigen::Ref<const Eigen::Vector3d> previous_x_com,
                                       Eigen::Ref<const Eigen::Vector3d> previous_xd_com);

        void update_com_in_t_s_(double omega, double time,
                                           Eigen::Ref<const Eigen::Vector3d> u_current_step,
                                           Eigen::Ref<const Eigen::Vector3d> previous_x_com,
                                           Eigen::Ref<const Eigen::Vector3d> previous_xd_com);

        void update_com_double_support(Eigen::Ref<const Eigen::Vector3d> start_pos,
                                       Eigen::Ref<const Eigen::Vector3d> start_vel,
                                       Eigen::Ref<const Eigen::Vector3d> end_pos,
                                       Eigen::Ref<const Eigen::Vector3d> end_vel,
                                       double time, double end_time);

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
        /**
         * @brief Get the right foot 3d velocity.
         *
         * @return const Eigen::Vector3d&
         */
        Eigen::Ref<const Eigen::Vector3d>  get_com_mea() const
        {
            return com_mea_;
        }

        /**
         * @brief Get the right foot 3d velocity.
         *
         * @return const Eigen::Vector3d&
         */
        Eigen::Ref<const Eigen::Vector3d> get_com_d_mea() const
        {
            return com_d_mea_;
        }

        /**
         * @brief Get the right foot 3d velocity.
         *
         * @return const Eigen::Vector3d&
         */
        Eigen::Ref<const Eigen::Vector3d> get_com_dd_mea() const
        {
            return com_dd_mea_;
        }


    private:
        /** @brief The center of mass position. */
        Eigen::Vector3d com_;

        /** @brief The center of mass velocity. */
        Eigen::Vector3d com_d_;

        /** @brief The center of mass acceleration. */
        Eigen::Vector3d com_dd_;

        /** @brief The center of mass position
         * that calculated by current measured data. */
        Eigen::Vector3d com_mea_;

        /** @brief The center of mass velocity
         * that calculated by current measured data. */
        Eigen::Vector3d com_d_mea_;

        /** @brief The center of mass acceleration
         * that calculated by current measured data. */
        Eigen::Vector3d com_dd_mea_;

        /** @brief The previous center of mass position. */
        Eigen::Vector3d last_x_com_;

        /** @brief The previous center of mass velocity. */
        Eigen::Vector3d last_xd_com_;

        /** @brief The r_vrp_current_step. */
        Eigen::Vector3d r_vrp_current_step_;

        /** @brief The r_vrp. */
        Eigen::Vector3d r_vrp_;

        Eigen::Vector3d com_t_s_;

        Eigen::Vector3d com_d_t_s_;
};

}  // namespace reactive_planners
