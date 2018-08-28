#include "drake/solvers/evaluator_base.h"

using std::make_shared;
using std::shared_ptr;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace solvers {

<<<<<<< HEAD
void PolynomialEvaluator::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                 Eigen::VectorXd* y) const {
=======
void PolynomialEvaluator::DoEval(const Eigen::Ref<const Eigen::VectorXd> &x,
                                 Eigen::VectorXd &y) const {
>>>>>>> intial
  double_evaluation_point_temp_.clear();
  for (size_t i = 0; i < poly_vars_.size(); i++) {
    double_evaluation_point_temp_[poly_vars_[i]] = x[i];
  }
<<<<<<< HEAD
  y->resize(num_outputs());
  for (int i = 0; i < num_outputs(); i++) {
    (*y)[i] =
        polynomials_[i].EvaluateMultivariate(double_evaluation_point_temp_);
  }
}

void PolynomialEvaluator::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                 AutoDiffVecXd* y) const {
=======
  y.resize(num_outputs());
  for (int i = 0; i < num_outputs(); i++) {
    y[i] = polynomials_[i].EvaluateMultivariate(double_evaluation_point_temp_);
  }
}

void PolynomialEvaluator::DoEval(const Eigen::Ref<const AutoDiffVecXd> &x,
                                 AutoDiffVecXd &y) const {
>>>>>>> intial
  taylor_evaluation_point_temp_.clear();
  for (size_t i = 0; i < poly_vars_.size(); i++) {
    taylor_evaluation_point_temp_[poly_vars_[i]] = x[i];
  }
<<<<<<< HEAD
  y->resize(num_outputs());
  for (int i = 0; i < num_outputs(); i++) {
    (*y)[i] =
        polynomials_[i].EvaluateMultivariate(taylor_evaluation_point_temp_);
=======
  y.resize(num_outputs());
  for (int i = 0; i < num_outputs(); i++) {
    y[i] = polynomials_[i].EvaluateMultivariate(taylor_evaluation_point_temp_);
>>>>>>> intial
  }
}

}  // namespace solvers
}  // namespace drake
