#include "ddbot_controller/linear_quadratic_regulator.hpp"
#include "ddbot_controller/discrete_algebraic_riccati_equation.hpp"

namespace controllers {

LinearQuadraticRegulatorResult DiscreteTimeLinearQuadraticRegulator(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::MatrixXd>& B,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R) {
  LinearQuadraticRegulatorResult ret;

  ret.S = math::DiscreteAlgebraicRiccatiEquation(A, B, Q, R);

  Eigen::MatrixXd tmp = B.transpose() * ret.S * B + R;
  ret.K = tmp.llt().solve(B.transpose() * ret.S * A);

  return ret;
}

} // namespace controller

