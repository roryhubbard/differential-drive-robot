#pragma once

#include <memory>

namespace drake {
namespace systems {
namespace controllers {

struct LinearQuadraticRegulatorResult {
  Eigen::MatrixXd K;
  Eigen::MatrixXd S;
};

// TODO(russt): Consider implementing the optional N argument as in the
// continuous-time formulation.
/// Computes the optimal feedback controller, u=-Kx, and the optimal
/// cost-to-go J = x'Sx for the problem:
///
///   @f[ x[n+1] = Ax[n] + Bu[n] @f]
///   @f[ \min_u \sum_0^\infty x'Qx + u'Ru @f]
///
/// @param A The state-space dynamics matrix of size num_states x num_states.
/// @param B The state-space input matrix of size num_states x num_inputs.
/// @param Q A symmetric positive semi-definite cost matrix of size num_states x
/// num_states.
/// @param R A symmetric positive definite cost matrix of size num_inputs x
/// num_inputs.
/// @returns A structure that contains the optimal feedback gain K and the
/// quadratic cost term S. The optimal feedback control is u = -Kx;
///
/// @throws std::exception if R is not positive definite.
/// @ingroup control
LinearQuadraticRegulatorResult DiscreteTimeLinearQuadraticRegulator(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::MatrixXd>& B,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R);

}  // namespace controllers
}  // namespace systems
}  // namespace drake
