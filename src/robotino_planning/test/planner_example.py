import matplotlib.pyplot as plt
from diff_drive_planner import DiffDrivePlanner


def rotate(theta, vertices):
  R = np.array([
    [np.cos(theta), -np.sin(theta)],
    [np.sin(theta), np.cos(theta)],
  ])
  return vertices @ R.T


def main():
  N = 5
  t0 = 0
  tf = 10
  time_samples = np.linspace(t0, tf, N)
  n_flat_outputs = 2
  poly_degree = 3
  smoothness_degree = 2

  ddp = DiffDrivePlanner(time_samples, n_flat_outputs,
                         poly_degree, smoothness_degree)

  ddp.add_cost()
  ddp.add_constraint(t=0, derivative_order=0, bounds=[-2, -2], equality=True)
  ddp.add_constraint(t=0, derivative_order=1, bounds=[0, 0], equality=True)
  ddp.add_constraint(t=0, derivative_order=2, bounds=[0, 0], equality=True)
  ddp.add_constraint(t=tf, derivative_order=0, bounds=[2, 2], equality=True)
  ddp.add_constraint(t=tf, derivative_order=1, bounds=[0, 0], equality=True)
  ddp.add_constraint(t=tf, derivative_order=2, bounds=[0, 0], equality=True)

  square = np.array([
    [1, 1],
    [-1, 1],
    [-1, -1],
    [1, -1],
    [1, 1],
  ])
  theta = np.pi / 4
  square = rotate(theta, square)

  checkpoints = np.linspace(t0, tf, 22)
  ddp.add_obstacle(square, checkpoints)

  ddp.solve(verbose=True)

  x = []
  y = []
  yaw = []
  t_result = np.linspace(t0, tf, 100)
  for t in t_result:
    flats = ddp.eval(t, 0)
    x.append(flats[0])
    y.append(flats[1])
    yaw.append(ddp.recover_yaw(t))

  fig, ax = plt.subplots(ncols=2)
  ax[0].plot(x, y, '.')
  ax[0].plot(*zip(*square))
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

