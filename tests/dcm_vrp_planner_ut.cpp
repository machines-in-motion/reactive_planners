#include <gtest/gtest.h>
#include <stdlib.h> /* srand, rand */
#include <time.h>   /* time */
#include "reactive_planners/dcm_vrp_planner.hpp"

class TestEigenQuadProg : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
    }
    virtual void TearDown()
    {
    }
};

/**
 * Quick test on the eigen-quapgrog API
 */
struct QP1
{
    QP1()
    {
        nrvar = 6;
        nreq = 3;
        nrineq = 2;

        Q.resize(nrvar, nrvar);
        Aeq.resize(nreq, nrvar);
        Aineq.resize(nrineq, nrvar);

        C.resize(nrvar);
        Beq.resize(nreq);
        Bineq.resize(nrineq);
        XL.resize(nrvar);
        XU.resize(nrvar);
        X.resize(nrvar);

        // clang-format off
    Aeq << 1., -1.,  1.,  0., 3., 1.,
          -1.,  0., -3., -4., 5., 6.,
           2.,  5.,  3.,  0., 1., 0.;
    Beq << 1., 2., 3.;

    Aineq <<  0., 1., 0., 1., 2., -1.,
             -1., 0., 2., 1., 1.,  0.;
    Bineq << -1., 2.5;
        // clang-format on

        // with  x between ci and cs:
        XL << -1000., -10000., 0., -1000., -1000., -1000.;
        XU << 10000., 100., 1.5, 100., 100., 1000.;

        // and minimize 0.5*x'*Q*x + p'*x with
        C << 1., 2., 3., 4., 5., 6.;
        Q.setIdentity();

        X << 1.7975426, -0.3381487, 0.1633880, -4.9884023, 0.6054943,
            -3.1155623;
    }

    int nrvar, nreq, nrineq;
    Eigen::MatrixXd Q, Aeq, Aineq;
    Eigen::VectorXd C, Beq, Bineq, XL, XU, X;
};

void ineqWithXBounds(Eigen::MatrixXd& Aineq,
                     Eigen::VectorXd& Bineq,
                     const Eigen::VectorXd& XL,
                     const Eigen::VectorXd& XU)
{
    double inf = std::numeric_limits<double>::infinity();

    std::vector<std::pair<int, double> > lbounds, ubounds;

    for (int i = 0; i < XL.rows(); ++i)
    {
        if (XL[i] != -inf) lbounds.emplace_back(i, XL[i]);
        if (XU[i] != inf) ubounds.emplace_back(i, XU[i]);
    }

    long int nrconstr = Bineq.rows() + static_cast<long int>(lbounds.size()) +
                        static_cast<long int>(ubounds.size());

    Eigen::MatrixXd A(Eigen::MatrixXd::Zero(nrconstr, Aineq.cols()));
    Eigen::VectorXd B(Eigen::VectorXd::Zero(nrconstr));

    A.block(0, 0, Aineq.rows(), Aineq.cols()) = Aineq;
    B.segment(0, Bineq.rows()) = Bineq;

    int start = static_cast<int>(Aineq.rows());

    for (int i = 0; i < static_cast<int>(lbounds.size()); ++i)
    {
        const auto& b = lbounds[i];
        A(start, b.first) = -1.;
        B(start) = -b.second;
        ++start;
    }

    for (int i = 0; i < static_cast<int>(ubounds.size()); ++i)
    {
        const auto& b = ubounds[i];
        A(start, b.first) = 1.;
        B(start) = b.second;
        ++start;
    }

    Aineq = A;
    Bineq = B;
}

TEST_F(TestEigenQuadProg, test_QuadProgDense)
{
    QP1 qp1;
    ineqWithXBounds(qp1.Aineq, qp1.Bineq, qp1.XL, qp1.XU);

    int nrineq = static_cast<int>(qp1.Aineq.rows());
    Eigen::QuadProgDense qp(qp1.nrvar, qp1.nreq, nrineq);

    qp.solve(qp1.Q, qp1.C, qp1.Aeq, qp1.Beq, qp1.Aineq, qp1.Bineq);

    ASSERT_LE((qp.result() - qp1.X).norm(), 1e-6);

    // give the decomposition to quad prog
    // ok that's not realy clever with QP1 because Q is identity.
    Eigen::MatrixXd Linv = qp1.Q.llt().matrixU();
    qp.solve(
        Linv.inverse(), qp1.C, qp1.Aeq, qp1.Beq, qp1.Aineq, qp1.Bineq, true);

    ASSERT_LE((qp.result() - qp1.X).norm(), 1e-6);
}

TEST_F(TestEigenQuadProg, test_QuadProgSparse)
{
    QP1 qp1;
    ineqWithXBounds(qp1.Aineq, qp1.Bineq, qp1.XL, qp1.XU);

    int nrineq = static_cast<int>(qp1.Aineq.rows());
    Eigen::QuadProgSparse qp(qp1.nrvar, qp1.nreq, nrineq);

    Eigen::SparseMatrix<double> SAeq(qp1.Aeq.sparseView());
    Eigen::SparseMatrix<double> SAineq(qp1.Aineq.sparseView());
    SAeq.makeCompressed();
    SAineq.makeCompressed();

    qp.solve(qp1.Q, qp1.C, SAeq, qp1.Beq, SAineq, qp1.Bineq);

    ASSERT_LE((qp.result() - qp1.X).norm(), 1e-6);

    // give the decomposition to quad prog
    // ok that's not realy clever with QP1 because Q is identity.
    Eigen::MatrixXd Linv = qp1.Q.llt().matrixU();
    qp.solve(Linv.inverse(), qp1.C, SAeq, qp1.Beq, SAineq, qp1.Bineq, true);

    ASSERT_LE((qp.result() - qp1.X).norm(), 1e-6);
}

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/**
 * Unit tests of the DcmVrpPlanner
 */

class TestDcmVrpPlanner : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        /* initialize random seed: */
        srand(time(NULL));

        l_min_ = 0.0;
        l_max_ = 0.0;
        w_min_ = 0.0;
        w_max_ = 0.0;
        t_min_ = 0.0;
        t_max_ = 0.0;
        l_p_ = 0.0;
        ht_ = 0.0;
        cost_weights_local_.setZero();

        current_step_location_.setZero();
        time_from_last_step_touchdown_ = 0.0;
        is_left_leg_in_contact_ = true;
        v_des_.setZero();
        com_.setZero();
        com_vel_.setZero();
        world_M_base_ = pinocchio::SE3::Identity();
    }

    virtual void TearDown()
    {
    }

    // Planner
    reactive_planners::DcmVrpPlanner dcm_vrp_planner_;

    // Planner constant parameters
    double l_min_;
    double l_max_;
    double w_min_;
    double w_max_;
    double t_min_;
    double t_max_;
    double l_p_;
    double ht_;
    Eigen::Vector9d cost_weights_local_;

    // Time varying parameters
    Eigen::Vector3d current_step_location_;
    double time_from_last_step_touchdown_;
    bool is_left_leg_in_contact_;
    Eigen::Vector3d v_des_;
    Eigen::Vector3d com_;
    Eigen::Vector3d com_vel_;
    pinocchio::SE3 world_M_base_;
};

/*---------------------------------------------------------------------------*/

TEST_F(TestDcmVrpPlanner, test_constructor)
{
}

/*---------------------------------------------------------------------------*/

TEST_F(TestDcmVrpPlanner, test_t_nom)
{
    v_des_.setZero();
    t_min_ = (double)rand() / (double)(RAND_MAX)*0.1 + 0.1;
    t_max_ = (double)rand() / (double)(RAND_MAX)*0.1 + 3 * t_min_;
    dcm_vrp_planner_.initialize(l_min_,
                                l_max_,
                                w_min_,
                                w_max_,
                                t_min_,
                                t_max_,
                                l_p_,
                                ht_,
                                cost_weights_local_);
    dcm_vrp_planner_.update(current_step_location_,
                            time_from_last_step_touchdown_,
                            is_left_leg_in_contact_,
                            v_des_,
                            com_,
                            com_vel_,
                            world_M_base_);
    ASSERT_NEAR(dcm_vrp_planner_.get_t_nom(), (t_min_ + t_max_) * 0.5, 1e-8);

    v_des_ << 0.0, 1.0, 0.0;
    dcm_vrp_planner_.initialize(l_min_,
                                l_max_,
                                w_min_,
                                w_max_,
                                t_min_,
                                t_max_,
                                l_p_,
                                ht_,
                                cost_weights_local_);
    dcm_vrp_planner_.update(current_step_location_,
                            time_from_last_step_touchdown_,
                            is_left_leg_in_contact_,
                            v_des_,
                            com_,
                            com_vel_,
                            world_M_base_);
    ASSERT_NEAR(dcm_vrp_planner_.get_t_nom(), (t_min_ + t_max_) * 0.5, 1e-8);

    v_des_ << 1.0, 0.0, 0.0;
    dcm_vrp_planner_.initialize(l_min_,
                                l_max_,
                                w_min_,
                                w_max_,
                                t_min_,
                                t_max_,
                                l_p_,
                                ht_,
                                cost_weights_local_);
    dcm_vrp_planner_.update(current_step_location_,
                            time_from_last_step_touchdown_,
                            is_left_leg_in_contact_,
                            v_des_,
                            com_,
                            com_vel_,
                            world_M_base_);
    ASSERT_NEAR(dcm_vrp_planner_.get_t_nom(), (t_min_ + t_max_) * 0.5, 1e-8);

    v_des_ << 1.0, 1.0, 0.0;
    t_min_ = 0.1;
    l_min_ = 0.101;
    w_min_ = 0.102;
    t_max_ = 0.2;
    l_max_ = 0.201;
    w_max_ = 0.202;
    dcm_vrp_planner_.initialize(l_min_,
                                l_max_,
                                w_min_,
                                w_max_,
                                t_min_,
                                t_max_,
                                l_p_,
                                ht_,
                                cost_weights_local_);
    dcm_vrp_planner_.update(current_step_location_,
                            time_from_last_step_touchdown_,
                            is_left_leg_in_contact_,
                            v_des_,
                            com_,
                            com_vel_,
                            world_M_base_);
    ASSERT_NEAR(dcm_vrp_planner_.get_t_nom(), (w_min_ + t_max_) * 0.5, 1e-8);

    v_des_ << 1.0, 1.0, 0.0;
    t_min_ = 0.102;
    l_min_ = 0.101;
    w_min_ = 0.100;
    t_max_ = 0.202;
    l_max_ = 0.201;
    w_max_ = 0.200;
    dcm_vrp_planner_.initialize(l_min_,
                                l_max_,
                                w_min_,
                                w_max_,
                                t_min_,
                                t_max_,
                                l_p_,
                                ht_,
                                cost_weights_local_);
    dcm_vrp_planner_.update(current_step_location_,
                            time_from_last_step_touchdown_,
                            is_left_leg_in_contact_,
                            v_des_,
                            com_,
                            com_vel_,
                            world_M_base_);
    ASSERT_NEAR(dcm_vrp_planner_.get_t_nom(), (t_min_ + w_max_) * 0.5, 1e-8);

    v_des_ << 1.0, 1.0, 0.0;
    t_min_ = 0.101;
    l_min_ = 0.102;
    w_min_ = 0.100;
    t_max_ = 0.201;
    l_max_ = 0.200;
    w_max_ = 0.202;
    dcm_vrp_planner_.initialize(l_min_,
                                l_max_,
                                w_min_,
                                w_max_,
                                t_min_,
                                t_max_,
                                l_p_,
                                ht_,
                                cost_weights_local_);
    dcm_vrp_planner_.update(current_step_location_,
                            time_from_last_step_touchdown_,
                            is_left_leg_in_contact_,
                            v_des_,
                            com_,
                            com_vel_,
                            world_M_base_);
    ASSERT_NEAR(dcm_vrp_planner_.get_t_nom(), (l_min_ + l_max_) * 0.5, 1e-8);
}

/*---------------------------------------------------------------------------*/

TEST_F(TestDcmVrpPlanner, test_tau_nom)
{
    dcm_vrp_planner_.initialize(l_min_,
                                l_max_,
                                w_min_,
                                w_max_,
                                t_min_,
                                t_max_,
                                l_p_,
                                ht_,
                                cost_weights_local_);
    dcm_vrp_planner_.update(current_step_location_,
                            time_from_last_step_touchdown_,
                            is_left_leg_in_contact_,
                            v_des_,
                            com_,
                            com_vel_,
                            world_M_base_);

    ASSERT_FALSE(dcm_vrp_planner_.get_tau_nom() ==
                 dcm_vrp_planner_.get_tau_nom());

    ht_ = 9.81;
    t_min_ = 1.0;
    t_max_ = 3.0;
    dcm_vrp_planner_.initialize(l_min_,
                                l_max_,
                                w_min_,
                                w_max_,
                                t_min_,
                                t_max_,
                                l_p_,
                                ht_,
                                cost_weights_local_);
    dcm_vrp_planner_.update(current_step_location_,
                            time_from_last_step_touchdown_,
                            is_left_leg_in_contact_,
                            v_des_,
                            com_,
                            com_vel_,
                            world_M_base_);
    ASSERT_EQ(dcm_vrp_planner_.get_t_nom(), 2.0);
    ASSERT_EQ(dcm_vrp_planner_.get_tau_nom(), exp(2.0));
}

/*---------------------------------------------------------------------------*/

TEST_F(TestDcmVrpPlanner, test_lw_nom)
{
    dcm_vrp_planner_.initialize(l_min_,
                                l_max_,
                                w_min_,
                                w_max_,
                                t_min_,
                                t_max_,
                                l_p_,
                                ht_,
                                cost_weights_local_);
    dcm_vrp_planner_.update(current_step_location_,
                            time_from_last_step_touchdown_,
                            is_left_leg_in_contact_,
                            v_des_,
                            com_,
                            com_vel_,
                            world_M_base_);

    ASSERT_EQ(dcm_vrp_planner_.get_l_nom(), 0.0);

    t_min_ = 1.0;
    t_max_ = 3.0;
    dcm_vrp_planner_.initialize(l_min_,
                                l_max_,
                                w_min_,
                                w_max_,
                                t_min_,
                                t_max_,
                                l_p_,
                                ht_,
                                cost_weights_local_);
    dcm_vrp_planner_.update(current_step_location_,
                            time_from_last_step_touchdown_,
                            is_left_leg_in_contact_,
                            v_des_,
                            com_,
                            com_vel_,
                            world_M_base_);
    ASSERT_EQ(dcm_vrp_planner_.get_l_nom(), 0.0);

    double a(0.1), b(0.2);
    v_des_ << a, b, 0.0;
    t_min_ = a;
    l_min_ = a - 0.1;
    w_min_ = a - 0.1;
    t_max_ = b;
    l_max_ = b + 0.1;
    w_max_ = b + 0.1;
    dcm_vrp_planner_.initialize(l_min_,
                                l_max_,
                                w_min_,
                                w_max_,
                                t_min_,
                                t_max_,
                                l_p_,
                                ht_,
                                cost_weights_local_);
    dcm_vrp_planner_.update(current_step_location_,
                            time_from_last_step_touchdown_,
                            is_left_leg_in_contact_,
                            v_des_,
                            com_,
                            com_vel_,
                            world_M_base_);
    ASSERT_EQ(dcm_vrp_planner_.get_l_nom(), a * (0.5 * (a + b)));
    ASSERT_EQ(dcm_vrp_planner_.get_w_nom(), b * (0.5 * (a + b)));
}

/*---------------------------------------------------------------------------*/

TEST_F(TestDcmVrpPlanner, test_solver_rest_pose)
{
    l_min_ = -0.15;
    l_max_ = -l_min_;
    w_min_ = -0.15;
    w_max_ = -w_min_;
    t_min_ = 0.1;
    t_max_ = t_min_ + 0.2;
    v_des_ << 0.0, 0.0, 0.0;
    l_p_ = 0;
    ht_ = 0.20;
    // clang-format off
  cost_weights_local_ <<
      /*u_T_x*/ 10,
      /*u_T_y*/ 10,
      /*tau*/ 1.0,
      /*b_x*/ 1000,
      /*b_x*/ 1000,
      /*psi_0*/ 1e6,
      /*psi_1*/ 1e6,
      /*psi_2*/ 1e6,
      /*psi_3*/ 1e6;
    // clang-format on
    dcm_vrp_planner_.initialize(l_min_,
                                l_max_,
                                w_min_,
                                w_max_,
                                t_min_,
                                t_max_,
                                l_p_,
                                ht_,
                                cost_weights_local_);

    current_step_location_ << 0.0, 0.0, 0.0;
    time_from_last_step_touchdown_ = 0.0;
    is_left_leg_in_contact_ = 0.0;
    v_des_ << 0.0, 0.0, 0.0;
    com_ << 0.0, 0.0, ht_;
    com_vel_ << 0.0, 0.0, 0.0;
    world_M_base_ = pinocchio::SE3::Identity();
    dcm_vrp_planner_.update(current_step_location_,
                            time_from_last_step_touchdown_,
                            is_left_leg_in_contact_,
                            v_des_,
                            com_,
                            com_vel_,
                            world_M_base_);

    dcm_vrp_planner_.solve();

    ASSERT_TRUE(dcm_vrp_planner_.internal_checks());

    Eigen::Vector3d next_step;
    next_step << 0, 0, 0;

    ASSERT_TRUE((next_step - dcm_vrp_planner_.get_next_step_location())
                    .isMuchSmallerThan(1.0, 1e-6));
    ASSERT_EQ(dcm_vrp_planner_.get_t_nom(),
              dcm_vrp_planner_.get_duration_before_step_landing());
}

/*---------------------------------------------------------------------------*/

TEST_F(TestDcmVrpPlanner, test_solver_com_forward)
{
    l_min_ = -0.15;
    l_max_ = -l_min_;
    w_min_ = -0.15;
    w_max_ = -w_min_;
    t_min_ = 0.1;
    t_max_ = t_min_ + 0.2;
    v_des_ << 0.0, 0.0, 0.0;
    l_p_ = 0;
    ht_ = 0.20;
    // clang-format off
  cost_weights_local_ <<
      /*u_T_x*/ 10,
      /*u_T_y*/ 10,
      /*tau*/ 1.0,
      /*b_x*/ 1000,
      /*b_x*/ 1000,
      /*psi_0*/ 1e6,
      /*psi_1*/ 1e6,
      /*psi_2*/ 1e6,
      /*psi_3*/ 1e6;
    // clang-format on
    dcm_vrp_planner_.initialize(l_min_,
                                l_max_,
                                w_min_,
                                w_max_,
                                t_min_,
                                t_max_,
                                l_p_,
                                ht_,
                                cost_weights_local_);

    current_step_location_ << 0.0, 0.0, 0.0;
    time_from_last_step_touchdown_ = 0.0;
    is_left_leg_in_contact_ = 0.0;
    v_des_ << 0.0, 0.0, 0.0;
    com_ << 0.02, 0.0, ht_;
    com_vel_ << 0.0, 0.0, 0.0;
    world_M_base_ = pinocchio::SE3::Identity();
    dcm_vrp_planner_.update(current_step_location_,
                            time_from_last_step_touchdown_,
                            is_left_leg_in_contact_,
                            v_des_,
                            com_,
                            com_vel_,
                            world_M_base_);

    dcm_vrp_planner_.solve();

    ASSERT_TRUE(dcm_vrp_planner_.internal_checks());

    Eigen::Vector3d next_step;
    next_step << 0.0800414, 0, 0;

    ASSERT_TRUE((next_step - dcm_vrp_planner_.get_next_step_location())
                    .isMuchSmallerThan(1.0, 1e-5));
    ASSERT_NEAR(
        0.199436, dcm_vrp_planner_.get_duration_before_step_landing(), 1e-5);
}

/*---------------------------------------------------------------------------*/

TEST_F(TestDcmVrpPlanner, test_solver_com_backward)
{
    l_min_ = -0.15;
    l_max_ = -l_min_;
    w_min_ = -0.15;
    w_max_ = -w_min_;
    t_min_ = 0.1;
    t_max_ = t_min_ + 0.2;
    v_des_ << 0.0, 0.0, 0.0;
    l_p_ = 0;
    ht_ = 0.20;
    // clang-format off
  cost_weights_local_ <<
      /*u_T_x*/ 10,
      /*u_T_y*/ 10,
      /*tau*/ 1.0,
      /*b_x*/ 1000,
      /*b_x*/ 1000,
      /*psi_0*/ 1e6,
      /*psi_1*/ 1e6,
      /*psi_2*/ 1e6,
      /*psi_3*/ 1e6;
    // clang-format on
    dcm_vrp_planner_.initialize(l_min_,
                                l_max_,
                                w_min_,
                                w_max_,
                                t_min_,
                                t_max_,
                                l_p_,
                                ht_,
                                cost_weights_local_);

    current_step_location_ << 0.0, 0.0, 0.0;
    time_from_last_step_touchdown_ = 0.0;
    is_left_leg_in_contact_ = 0.0;
    v_des_ << 0.0, 0.0, 0.0;
    com_ << -0.02, 0.0, ht_;
    com_vel_ << 0.0, 0.0, 0.0;
    world_M_base_ = pinocchio::SE3::Identity();
    dcm_vrp_planner_.update(current_step_location_,
                            time_from_last_step_touchdown_,
                            is_left_leg_in_contact_,
                            v_des_,
                            com_,
                            com_vel_,
                            world_M_base_);

    dcm_vrp_planner_.solve();

    ASSERT_TRUE(dcm_vrp_planner_.internal_checks());

    Eigen::Vector3d next_step;
    next_step << -0.0800414, 0, 0;

    ASSERT_TRUE((next_step - dcm_vrp_planner_.get_next_step_location())
                    .isMuchSmallerThan(1.0, 1e-5));
    ASSERT_NEAR(
        0.199436, dcm_vrp_planner_.get_duration_before_step_landing(), 1e-5);
}

/*---------------------------------------------------------------------------*/

TEST_F(TestDcmVrpPlanner, test_solver_com_left)
{
    l_min_ = -0.15;
    l_max_ = -l_min_;
    w_min_ = -0.15;
    w_max_ = -w_min_;
    t_min_ = 0.1;
    t_max_ = t_min_ + 0.2;
    v_des_ << 0.0, 0.0, 0.0;
    l_p_ = 0;
    ht_ = 0.20;
    // clang-format off
  cost_weights_local_ <<
      /*u_T_x*/ 10,
      /*u_T_y*/ 10,
      /*tau*/ 1.0,
      /*b_x*/ 1000,
      /*b_x*/ 1000,
      /*psi_0*/ 1e6,
      /*psi_1*/ 1e6,
      /*psi_2*/ 1e6,
      /*psi_3*/ 1e6;
    // clang-format on
    dcm_vrp_planner_.initialize(l_min_,
                                l_max_,
                                w_min_,
                                w_max_,
                                t_min_,
                                t_max_,
                                l_p_,
                                ht_,
                                cost_weights_local_);

    current_step_location_ << 0.0, 0.0, 0.0;
    time_from_last_step_touchdown_ = 0.0;
    is_left_leg_in_contact_ = 0.0;
    v_des_ << 0.0, 0.0, 0.0;
    com_ << 0.0, -0.02, ht_;
    com_vel_ << 0.0, 0.0, 0.0;
    world_M_base_ = pinocchio::SE3::Identity();
    dcm_vrp_planner_.update(current_step_location_,
                            time_from_last_step_touchdown_,
                            is_left_leg_in_contact_,
                            v_des_,
                            com_,
                            com_vel_,
                            world_M_base_);

    dcm_vrp_planner_.solve();

    ASSERT_TRUE(dcm_vrp_planner_.internal_checks());

    Eigen::Vector3d next_step;
    next_step << 0, -0.0800414, 0;

    ASSERT_TRUE((next_step - dcm_vrp_planner_.get_next_step_location())
                    .isMuchSmallerThan(1.0, 1e-5));
    ASSERT_NEAR(
        0.199436, dcm_vrp_planner_.get_duration_before_step_landing(), 1e-5);
}

/*---------------------------------------------------------------------------*/

TEST_F(TestDcmVrpPlanner, test_solver_com_right)
{
    l_min_ = -0.15;
    l_max_ = -l_min_;
    w_min_ = -0.15;
    w_max_ = -w_min_;
    t_min_ = 0.1;
    t_max_ = t_min_ + 0.2;
    v_des_ << 0.0, 0.0, 0.0;
    l_p_ = 0;
    ht_ = 0.20;
    // clang-format off
  cost_weights_local_ <<
      /*u_T_x*/ 10,
      /*u_T_y*/ 10,
      /*tau*/ 1.0,
      /*b_x*/ 1000,
      /*b_x*/ 1000,
      /*psi_0*/ 1e6,
      /*psi_1*/ 1e6,
      /*psi_2*/ 1e6,
      /*psi_3*/ 1e6;
    // clang-format on
    dcm_vrp_planner_.initialize(l_min_,
                                l_max_,
                                w_min_,
                                w_max_,
                                t_min_,
                                t_max_,
                                l_p_,
                                ht_,
                                cost_weights_local_);

    current_step_location_ << 0.0, 0.0, 0.0;
    time_from_last_step_touchdown_ = 0.0;
    is_left_leg_in_contact_ = 0.0;
    v_des_ << 0.0, 0.0, 0.0;
    com_ << 0.0, 0.02, ht_;
    com_vel_ << 0.0, 0.0, 0.0;
    world_M_base_ = pinocchio::SE3::Identity();
    dcm_vrp_planner_.update(current_step_location_,
                            time_from_last_step_touchdown_,
                            is_left_leg_in_contact_,
                            v_des_,
                            com_,
                            com_vel_,
                            world_M_base_);

    dcm_vrp_planner_.solve();

    ASSERT_TRUE(dcm_vrp_planner_.internal_checks());

    Eigen::Vector3d next_step;
    next_step << 0, 0.0800414, 0;

    ASSERT_TRUE((next_step - dcm_vrp_planner_.get_next_step_location())
                    .isMuchSmallerThan(1.0, 1e-5));
    ASSERT_NEAR(
        0.199436, dcm_vrp_planner_.get_duration_before_step_landing(), 1e-5);
}
