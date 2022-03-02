import matplotlib.pyplot as plt
import numpy as np
from diff_drive_planner import DiffDrivePlanner


def rotate(theta, vertices):
  R = np.array([
    [np.cos(theta), -np.sin(theta)],
    [np.sin(theta), np.cos(theta)],
  ])
  return vertices @ R.T


def generate_rectangle(cx, cy, width, height):
  hw = width / 2
  hh = height / 2
  top_right = [cx + hw, cy + hh]
  top_left = [cx - hw, cy + hh]
  bottom_left = [cx - hw, cy - hh]
  bottom_right = [cx + hw, cy - hh]
  return np.array([
    top_right,
    top_left,
    bottom_left,
    bottom_right,
    top_right,
  ])


def main():
  num_splines = 4
  total_travel_time = 8
  control_frequency = 10
  time_samples = np.linspace(0, total_travel_time, num_splines+1)
  n_flat_outputs = 2
  poly_degree = 5
  smoothness_degree = 2

  ddp = DiffDrivePlanner(time_samples, n_flat_outputs,
                         poly_degree, smoothness_degree)

  ddp.add_cost()
  ddp.add_constraint(t=0, derivative_order=0, bounds=[-6, 0], equality=True)
  ddp.add_constraint(t=0, derivative_order=1, bounds=[0, 0], equality=True)
  ddp.add_constraint(t=0, derivative_order=2, bounds=[0, 0], equality=True)

  ddp.add_constraint(t=2, derivative_order=0, bounds=[-3, 0], equality=True)
  ddp.add_constraint(t=4., derivative_order=0, bounds=[0, 0], equality=True)
  ddp.add_constraint(t=6, derivative_order=0, bounds=[3, 0], equality=True)

  ddp.add_constraint(t=total_travel_time, derivative_order=0, bounds=[6, 0], equality=True)
  ddp.add_constraint(t=total_travel_time, derivative_order=1, bounds=[0, 0], equality=True)
  ddp.add_constraint(t=total_travel_time, derivative_order=2, bounds=[0, 0], equality=True)

  height = 2.
  width = .5
  ob1 = generate_rectangle(-4.5, 0., width, height);
  ob2 = generate_rectangle(-1.5, 0., width, height);
  ob3 = generate_rectangle(1.5, 0., width, height);
  ob4 = generate_rectangle(4.5, 0., width, height);
  checkpoints = np.linspace(0, total_travel_time, total_travel_time*control_frequency)
  bigM = 32
  ddp.add_obstacle(ob1, checkpoints, bigM)
  ddp.add_obstacle(ob2, checkpoints, bigM)
  ddp.add_obstacle(ob3, checkpoints, bigM)
  ddp.add_obstacle(ob4, checkpoints, bigM)

  ddp.solve(verbose=True)

  x = []
  y = []
  yaw = []
  for t in np.linspace(0, total_travel_time, total_travel_time*control_frequency):
    flats = ddp.eval(t, 0)
    x.append(flats[0])
    y.append(flats[1])
    yaw.append(ddp.recover_yaw(t))

  fig, ax = plt.subplots(ncols=2)
  ax[0].plot(x, y, '.')
  ax[0].plot(*zip(*ob1))
  ax[0].plot(*zip(*ob2))
  ax[0].plot(*zip(*ob3))
  ax[0].plot(*zip(*ob4))
  ax[1].plot(yaw)
  plt.show()
  plt.close()


if __name__ == '__main__':
  plt.rcParams['figure.figsize'] = [10, 8]
  plt.rcParams['savefig.facecolor'] = 'black'
  plt.rcParams['figure.facecolor'] = 'black'
  plt.rcParams['figure.edgecolor'] = 'white'
  plt.rcParams['axes.facecolor'] = 'black'
  plt.rcParams['axes.edgecolor'] = 'white'
  plt.rcParams['axes.labelcolor'] = 'white'
  plt.rcParams['axes.titlecolor'] = 'white'
  plt.rcParams['xtick.color'] = 'white'
  plt.rcParams['ytick.color'] = 'white'
  plt.rcParams['text.color'] = 'white'
  plt.rcParams["figure.autolayout"] = True
  # plt.rcParams['legend.facecolor'] = 'white'
  main()

