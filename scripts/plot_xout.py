import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import pandas as pd
from mpl_toolkits.mplot3d import axes3d, Axes3D



df = pd.read_csv("/home/justin/circle_point.csv")


fig = plt.figure()
ax = Axes3D(fig)
ax.scatter(df['x'], df['y'], df['alt'])
#plt.plot(df['x'], df['alt'])


plt.show()
