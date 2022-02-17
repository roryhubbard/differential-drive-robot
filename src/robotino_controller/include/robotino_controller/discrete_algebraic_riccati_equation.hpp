#pragma once

#include <cmath>
#include <cstdlib>

#include <Eigen/Dense>

namespace math {

/**
Computes the unique stabilizing solution X to the discrete-time algebraic
Riccati equation:

AᵀXA − X − AᵀXB(BᵀXB + R)⁻¹BᵀXA + Q = 0

Based on the Schur Vector approach outlined in this paper:
"On the Numerical Solution of the Discrete-Time Algebraic Riccati Equation"
by Thrasyvoulos Pappas, Alan J. Laub, and Nils R. Sandell
*/
Eigen::MatrixXd DiscreteAlgebraicRiccatiEquation(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::MatrixXd>& B,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R);

}  // namespace math

