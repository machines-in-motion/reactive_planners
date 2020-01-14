#include <gtest/gtest.h>
#include "reactive_planners/dcm_vrp_planner.hpp"

class TestReactivePlanners : public ::testing::Test {
 protected:
  virtual void SetUp() {}
  virtual void TearDown() {}
};

struct QP1 {
  QP1() {
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

    X << 1.7975426, -0.3381487, 0.1633880, -4.9884023, 0.6054943, -3.1155623;
  }

  int nrvar, nreq, nrineq;
  Eigen::MatrixXd Q, Aeq, Aineq;
  Eigen::VectorXd C, Beq, Bineq, XL, XU, X;
};

void ineqWithXBounds(Eigen::MatrixXd& Aineq, Eigen::VectorXd& Bineq,
                     const Eigen::VectorXd& XL, const Eigen::VectorXd& XU) {
  double inf = std::numeric_limits<double>::infinity();

  std::vector<std::pair<int, double> > lbounds, ubounds;

  for (int i = 0; i < XL.rows(); ++i) {
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

  for (int i = 0; i < static_cast<int>(lbounds.size()); ++i) {
    const auto& b = lbounds[i];
    A(start, b.first) = -1.;
    B(start) = -b.second;
    ++start;
  }

  for (int i = 0; i < static_cast<int>(ubounds.size()); ++i) {
    const auto& b = ubounds[i];
    A(start, b.first) = 1.;
    B(start) = b.second;
    ++start;
  }

  Aineq = A;
  Bineq = B;
}

TEST_F(TestReactivePlanners, test_QuadProgDense) {
  QP1 qp1;
  ineqWithXBounds(qp1.Aineq, qp1.Bineq, qp1.XL, qp1.XU);

  int nrineq = static_cast<int>(qp1.Aineq.rows());
  Eigen::QuadProgDense qp(qp1.nrvar, qp1.nreq, nrineq);

  qp.solve(qp1.Q, qp1.C, qp1.Aeq, qp1.Beq, qp1.Aineq, qp1.Bineq);

  ASSERT_LE((qp.result() - qp1.X).norm(), 1e-6);

  // give the decomposition to quad prog
  // ok that's not realy clever with QP1 because Q is identity.
  Eigen::MatrixXd Linv = qp1.Q.llt().matrixU();
  qp.solve(Linv.inverse(), qp1.C, qp1.Aeq, qp1.Beq, qp1.Aineq, qp1.Bineq, true);

  ASSERT_LE((qp.result() - qp1.X).norm(), 1e-6);
}

TEST_F(TestReactivePlanners, test_QuadProgSparse) {
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
