import numpy as np
import cvxpy as cp
from .piecewise_polynomial import PiecewisePolynomial


class DiffDrivePlanner(PiecewisePolynomial):

  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)

  def add_cost(self):
    """
    Minimize jerk
    """
    for s in range(self.ns):
      h = self.ts[s+1] - self.ts[s]
      P = get_jerk_matrix(h, self.poly_degree)
      for z in range(self.nflats):
        x = self.spline_coeffs[s][z]
        self.cost += (1/2) * cp.quad_form(x, P)

  def add_obstacle(self, vertices, checkpoints, bigM=10, buffer=0):
    """
    vertices: list(tuple(float)) = coordinates specifying vertices of obstacle
      - counterclockwise ordering and closed (first element == last elements)
    checkpoints: np.ndarray(float) = sample times to enforce as collision free
    """
    z = cp.Variable((checkpoints.size, len(vertices)-1), boolean=True)
    A = []
    b = []

    for i in range(len(vertices)-1):
      v1 = vertices[i]
      v2 = vertices[i+1]
      a = get_orthoganal_vector(v2 - v1)
      A.append(a)
      b.append(a @ v1)

    A = np.asarray(A)
    b = np.asarray(b)

    for t in checkpoints:
      flats = cp.vstack(self.eval(t, 0))
      bigM_rhs = cp.vstack(b + z[t] * bigM - buffer)
      self.constraints += [A @ flats <= bigM_rhs,
                           cp.sum(z[t]) <= len(vertices) - 2]

  def recover_yaw(self, t, derivative_order):
    if derivative_order > 1:
      raise NotImplementedError
    xdot, ydot = self.eval(t, 1)
    if derivative_order == 0:
      return np.arctan2(ydot, xdot)
    xddot, yddot = self.eval(t, 2)
    if derivative_order == 1:
      return (xdot * yddot - xddot * ydot) / (xdot**2 + ydot**2)

  def recover_longitudinal_velocity(self, t):
    xdot, ydot = self.eval(t, 1)
    return np.sqrt(xdot**2 + ydot**2)

  def recover_longitudinal_acceleration(self, t):
    xdot, ydot = self.eval(t, 1)
    xddot, yddot = self.eval(t, 2)
    return (xdot * xddot + ydot * yddot) / (np.sqrt(xdot**2 + ydot**2))

  def recover_angular_velocity(self, t):
    xdot, ydot = self.eval(t, 1)
    xddot, yddot = self.eval(t, 2)
    return (xdot * yddot - xddot * ydot) / (xdot**2 + ydot**2)


def get_jerk_matrix(t, po=5):
  """
  Integral of squared jerk over the interval: [0, t]
  Assumes polynomial order <= 5
  """
  P = np.array([
    [0, 0, 0,        0,         0,        0],
    [0, 0, 0,        0,         0,        0],
    [0, 0, 0,        0,         0,        0],
    [0, 0, 0,     36*t,   72*t**2, 120*t**3],
    [0, 0, 0,  72*t**2,  192*t**3, 360*t**4],
    [0, 0, 0, 120*t**3,  360*t**4, 720*t**5],
  ])
  return P[:po+1, :po+1]


def get_orthoganal_vector(v):
  R = np.array([
    [0, -1],
    [1, 0],
  ])
  return R @ v

