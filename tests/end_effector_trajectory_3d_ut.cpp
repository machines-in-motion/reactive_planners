/**
 * @file
 * @license BSD 3-Clause.
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft.
 *
 * @brief Unittest for the end-effector generation.
 */

#include <fstream>
#include <memory>
#include <gtest/gtest.h>
#include <stdlib.h> /* srand, rand */
#include <time.h>   /* time */
#include "reactive_planners/end_effector_trajectory_3d.hpp"

class Disabled_TestEndEffectorTrajectory3d : public ::testing::Test
{
};

/**
 * Unit tests of the DcmVrpPlanner
 */

class TestEndEffectorTrajectory3d : public ::testing::Test
{
protected:
    /*
     * Methods
     */
    virtual void SetUp()
    {
        /* initialize random seed: */
        srand(time(NULL));

        start_pose_ = 10 * Eigen::Vector3d::Random();
        current_pose_ = start_pose_;
        current_velocity_ = Eigen::Vector3d::Zero();
        current_acceleration_ = Eigen::Vector3d::Zero();
        target_pose_ = 10 * Eigen::Vector3d::Random();
        start_time_ = 3.0;  // 10 * rand_double();
        end_time_ = 4.0;    // start_time_ + 10 * rand_double();
        current_time_ = start_time_;

        end_effector_trajectory_3d_.reset(
            new reactive_planners::EndEffectorTrajectory3D());
    }

    virtual void TearDown()
    {
    }

    double rand_double()
    {
        return (double)rand() / (double)(RAND_MAX);
    }

    double rand_abs_double()
    {
        return std::abs((double)rand() / (double)(RAND_MAX));
    }

    void print_test_parameters()
    {
        std::cout << "start_pose_ = " << start_pose_.transpose() << std::endl;
        std::cout << "current_pose_ = " << current_pose_.transpose()
                  << std::endl;
        std::cout << "current_velocity_ = " << current_velocity_.transpose()
                  << std::endl;
        std::cout << "current_acceleration_ = "
                  << current_acceleration_.transpose() << std::endl;
        std::cout << "target_pose_ = " << target_pose_.transpose() << std::endl;
        std::cout << "start_time_ = " << start_time_ << std::endl;
        std::cout << "end_time_ = " << end_time_ << std::endl;
        std::cout << "current_time_ = " << current_time_ << std::endl;
    }

    /*
     * Attributes
     */
    std::shared_ptr<reactive_planners::EndEffectorTrajectory3D>
        end_effector_trajectory_3d_;
    Eigen::Vector3d start_pose_;
    Eigen::Vector3d current_pose_;
    Eigen::Vector3d current_velocity_;
    Eigen::Vector3d current_acceleration_;
    Eigen::Vector3d target_pose_;
    double start_time_;
    double current_time_;
    double end_time_;
};

typedef TestEndEffectorTrajectory3d TestEFFTraj3D;

/*---------------------------------------------------------------------------*/

TEST_F(TestEndEffectorTrajectory3d, test_constructor)
{
}

/*---------------------------------------------------------------------------*/

TEST_F(TestEndEffectorTrajectory3d, test_compute)
{
}

/*---------------------------------------------------------------------------*/

TEST_F(TestEndEffectorTrajectory3d, test_get_next_state)
{
    // ASSERT_EQ(dcm_vrp_planner_.get_t_nom(), 2.0);
    // ASSERT_EQ(dcm_vrp_planner_.get_tau_nom(), exp(2.0));
}

/*---------------------------------------------------------------------------*/

TEST_F(TestEndEffectorTrajectory3d, test_mid_air_height)
{
    double mid_air_height = rand_double() * 10 * rand_double();
    end_effector_trajectory_3d_->set_mid_air_height(mid_air_height);
    ASSERT_EQ(end_effector_trajectory_3d_->get_mid_air_height(),
              mid_air_height);
}

/*---------------------------------------------------------------------------*/

TEST_F(TestEndEffectorTrajectory3d, test_mid_air_height_compute)
{
    unsigned int N = 1000;
    for (unsigned int i = 0; i < N; ++i)
    {
        double Nd = (double)N;
        double id = (double)i;
        double alpha = (id / Nd - 0.5) * 2.0;
        double mid_air_height = 0.05 + alpha * 0.05;
        end_effector_trajectory_3d_->set_mid_air_height(mid_air_height);
        start_pose_.setZero();
        current_pose_.setZero();
        current_velocity_.setZero();
        current_acceleration_.setZero();
        target_pose_.setZero();
        end_effector_trajectory_3d_->compute(start_pose_,
                                             current_pose_,
                                             current_velocity_,
                                             current_acceleration_,
                                             target_pose_,
                                             0.0,
                                             1e-4,
                                             1.0);
        end_effector_trajectory_3d_->get_next_state(
            0.5, current_pose_, current_velocity_, current_acceleration_);

        ASSERT_NEAR(mid_air_height, current_pose_(2), 1e-3);
    }
}

/*---------------------------------------------------------------------------*/

TEST_F(TestEndEffectorTrajectory3d, test_compute_derivatives)
{
    std::ofstream myfile;
    myfile.open(
        "/tmp/TestEndEffectorTrajectory3d_test_compute_derivatives.dat");

    Eigen::Vector3d vel_num_diff = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_num_diff = Eigen::Vector3d::Zero();
    Eigen::Vector3d pos_prev = current_pose_;
    Eigen::Vector3d vel_prev = Eigen::Vector3d::Zero();

    unsigned int N = 10000;
    double sampling_period = (end_time_ - start_time_) / (double)N;
    for (unsigned int i = 0; i < N; ++i)
    {
        current_time_ = start_time_ + i * sampling_period;
        end_effector_trajectory_3d_->compute(start_pose_,
                                             current_pose_,
                                             current_velocity_,
                                             current_acceleration_,
                                             target_pose_,
                                             start_time_,
                                             current_time_,
                                             end_time_);
        end_effector_trajectory_3d_->get_next_state(
            current_time_ + sampling_period,
            current_pose_,
            current_velocity_,
            current_acceleration_);

        vel_num_diff = (current_pose_ - pos_prev) / sampling_period;
        acc_num_diff = (current_velocity_ - vel_prev) / sampling_period;

        bool are_vel_same =
            (vel_num_diff - current_velocity_).isMuchSmallerThan(1.0, 1e-2);
        if (!are_vel_same)
        {
            std::cout << "vel_num_diff = " << vel_num_diff.transpose()
                      << " ; current_velocity_ = "
                      << current_velocity_.transpose() << " ; Norm = "
                      << (vel_num_diff - current_velocity_).norm() << std::endl;
        }
        ASSERT_TRUE(are_vel_same);

        bool are_acc_same =
            (acc_num_diff - current_acceleration_).isMuchSmallerThan(1.0, 2e-1);
        if (!are_acc_same)
        {
            std::cout << "acc_num_diff = " << acc_num_diff.transpose()
                      << " ; current_acceleration_ = "
                      << current_acceleration_.transpose() << " ; Norm = "
                      << (acc_num_diff - current_acceleration_).norm()
                      << std::endl;
        }
        ASSERT_TRUE(are_acc_same);

        // clang-format off
        myfile << current_time_ << " "
               << current_pose_.transpose() << " "
               << current_velocity_.transpose() << " "
               << current_acceleration_.transpose() << " "
               << vel_num_diff.transpose() << " "
               << acc_num_diff.transpose() << std::endl;
        // clang-format on

        pos_prev = current_pose_;
        vel_prev = current_velocity_;
    }

    // print_test_parameters();
    myfile.close();
}
