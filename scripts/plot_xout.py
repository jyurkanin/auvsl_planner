import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import pandas as pd
from mpl_toolkits.mplot3d import axes3d, Axes3D




fig = plt.figure()
ax = Axes3D(fig)

df = pd.read_csv("/home/justin/raw_pcl.csv")
skip = 8
ax.scatter(df['x'][::skip], df['y'][::skip], df['alt'][::skip], alpha=.1)# .01*occ[occ[:] > cutoff])

df = pd.read_csv("/home/justin/xout_file.csv")
ax.scatter(df['x'], df['y'], df['z'])
#plt.plot(df['x'], df['alt'])

plt.show()
