#include "drake/systems/controllers/linear_quadratic_regulator.h"

#include "drake/math/discrete_algebraic_riccati_equation.h"

namespace drake {
namespace systems {
namespace controllers {

LinearQuadraticRegulatorResult DiscreteTimeLinearQuadraticRegulator(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::MatrixXd>& B,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R) {
  Eigen::Index n = A.rows(), m = B.cols();

  LinearQuadraticRegulatorResult ret;

  ret.S = math::DiscreteAlgebraicRiccatiEquation(A, B, Q, R);

  Eigen::MatrixXd tmp = B.transpose() * ret.S * B + R;
  ret.K = tmp.llt().solve(B.transpose() * ret.S * A);

  return ret;
}

}  // namespace controllers
}  // namespace systems
}  // namespace drake
