import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import pandas as pd
from mpl_toolkits.mplot3d import axes3d, Axes3D



fig = plt.figure()
ax = Axes3D(fig)

#df = pd.read_csv("/home/justin/circle_point.csv")
#ax.scatter(df['x'], df['y'], df['alt'])

df = pd.read_csv("/home/justin/state_valid_grid.csv")
#plt.scatter(df['x'], df['y'])
ax.scatter(df['x'], df['y'], df['occupancy'])

plt.show()
