#include "drake/solvers/cost.h"

#include <memory>

<<<<<<< HEAD
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::make_shared;
using std::shared_ptr;

namespace drake {
namespace solvers {
template <typename DerivedX, typename U>
void LinearCost::DoEvalGeneric(const Eigen::MatrixBase<DerivedX>& x,
                               VectorX<U>* y) const {
  y->resize(1);
  (*y)(0) = a_.dot(x) + b_;
}

void LinearCost::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                        Eigen::VectorXd* y) const {
  DoEvalGeneric(x, y);
}
void LinearCost::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                        AutoDiffVecXd* y) const {
  DoEvalGeneric(x, y);
}

void LinearCost::DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
                        VectorX<symbolic::Expression>* y) const {
  DoEvalGeneric(x, y);
}

template <typename DerivedX, typename U>
void QuadraticCost::DoEvalGeneric(const Eigen::MatrixBase<DerivedX>& x,
                                  VectorX<U>* y) const {
  y->resize(1);
  *y = .5 * x.transpose() * Q_ * x + b_.transpose() * x;
  (*y)(0) += c_;
}

void QuadraticCost::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                           Eigen::VectorXd* y) const {
  DoEvalGeneric(x, y);
}

void QuadraticCost::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                           AutoDiffVecXd* y) const {
  DoEvalGeneric(x, y);
}

void QuadraticCost::DoEval(
    const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
    VectorX<symbolic::Expression>* y) const {
  DoEvalGeneric(x, y);
=======
using std::make_shared;
using std::shared_ptr;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace solvers {

void LinearCost::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                        Eigen::VectorXd& y) const {
  y.resize(1);
  y(0) = a_.dot(x) + b_;
}
void LinearCost::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                        AutoDiffVecXd& y) const {
  y.resize(1);
  y(0) = a_.cast<AutoDiffXd>().dot(x) + b_;
}

void QuadraticCost::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                           Eigen::VectorXd& y) const {
  y.resize(1);
  y = .5 * x.transpose() * Q_ * x + b_.transpose() * x;
  y(0) += c_;
}

void QuadraticCost::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                           AutoDiffVecXd& y) const {
  y.resize(1);
  y = .5 * x.transpose() * Q_.cast<AutoDiffXd>() * x +
      b_.cast<AutoDiffXd>().transpose() * x;
  y(0) += c_;
>>>>>>> intial
}

shared_ptr<QuadraticCost> MakeQuadraticErrorCost(
    const Eigen::Ref<const MatrixXd>& Q,
    const Eigen::Ref<const VectorXd>& x_desired) {
  const double c = x_desired.dot(Q * x_desired);
  return make_shared<QuadraticCost>(2 * Q, -2 * Q * x_desired, c);
}

shared_ptr<QuadraticCost> MakeL2NormCost(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::VectorXd>& b) {
  const double c = b.dot(b);
  return make_shared<QuadraticCost>(2 * A.transpose() * A,
                                    -2 * A.transpose() * b, c);
}

}  // namespace solvers
}  // namespace drake
