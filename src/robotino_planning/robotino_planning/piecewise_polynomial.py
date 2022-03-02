import numpy as np
from math import factorial
import cvxpy as cp


class PiecewisePolynomial:

  def __init__(self, ts, nflats, poly_degree, smoothness_degree):
    self.ts = ts
    self.ns = self.ts.size - 1 # number of splines
    self.nflats = nflats
    self.poly_degree = poly_degree
    self.smoothness_degree = smoothness_degree
    self.spline_coeffs = [
      cp.Variable((nflats, poly_degree+1))
      for _ in range(self.ns)
    ]
    self.cost = 0
    self.constraints = []
    self._add_continuity_constraints()

  def add_cost(self):
    """
    Minimize highest order coefficients
    """
    for s in range(self.ns):
      self.cost += cp.sum_squares(self.spline_coeffs[s][:, -1])

  def _add_continuity_constraints(self):
    for s in range(self.ns-1):
      h = self.ts[s+1] - self.ts[s]
      for z in range(self.nflats):
        for sd in range(self.smoothness_degree+1):
          spline_end = self._eval_spline(h, sd, self.spline_coeffs[s][z])
          next_spline_start = self._eval_spline(0, sd, self.spline_coeffs[s+1][z])
          self.constraints += [spline_end == next_spline_start]

  def add_constraint(self, t, derivative_order, bounds, equality=False):
    """
    Add constraint to all flat outputs at derivative order
    """
    bounds = np.asarray(bounds).reshape(-1, 1)
    flats = cp.vstack(self.eval(t, derivative_order))
    self.constraints += [flats == bounds] if equality else [flats <= bounds]

  def _eval_spline_simple(self, t, derivative_order, coefficients):
    if derivative_order == 0:
      equation = np.array([1, t, t**2, t**3, t**4, t**5])
    elif derivative_order == 1:
      equation = np.array([0, 1, 2*t, 3*t**2, 4*t**3, 5*t**4])
    elif derivative_order == 2:
      equation = np.array([0, 0, 2, 6*t, 12*t**2, 20*t**3])
    elif derivative_order == 3:
      equation = np.array([0, 0, 0, 6, 24*t, 60*t**2])
    elif derivative_order == 4:
      equation = np.array([0, 0, 0, 0, 24, 120*t])
    elif derivative_order == 5:
      equation = np.array([0, 0, 0, 0, 0, 120])
    elif derivative_order == 6:
      equation = np.array([0, 0, 0, 0, 0, 0])
    else:
      print("derivative order is too high")
      return
    return equation[:self.poly_degree+1] @ coefficients

  def _eval_spline(self, h, d, c):
    """
    h: float = time relative to start of spline
    d: int = derivative order
    c: cp.Variable = spline coefficients for a flat output
      - could be solved or unsolved depending on when this function is called
    """
    result = 0
    for pd in range(d, self.poly_degree+1):
      result += c[pd] * np.power(h, pd - d) * factorial(pd) / factorial(pd - d)
    return result

  def eval(self, t, derivative_order):
    """
    Evaluate flat outputs at a derivative order and time t
      - coefficients could be solved or unsolved depending on when this function is called
    """
    s = self.ts[self.ts <= t].argmax()
    if s >= self.ns:
      s = self.ns - 1
    h = t - self.ts[s]

    c = self.spline_coeffs[s] \
      if self.spline_coeffs[s].value is None \
      else self.spline_coeffs[s].value

    flats = []
    for z in range(self.nflats):
      flats.append(self._eval_spline(h, derivative_order, c[z]))

    return flats

  def solve(self, verbose=False):
    self.problem = cp.Problem(cp.Minimize(self.cost), self.constraints)
    self.problem.solve(verbose=verbose)

