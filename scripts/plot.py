import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import pandas as pd
from mpl_toolkits.mplot3d import axes3d, Axes3D



fig = plt.figure()
ax = Axes3D(fig)

df = pd.read_csv("/home/justin/occ_grid.csv")
occ = df['occupancy']
cutoff = 10    
ax.scatter(df['x'][occ[:] > cutoff], df['y'][occ[:] > cutoff])# .01*occ[occ[:] > cutoff])
#ax.scatter(df['x'], df['y'], df['inflated'])# .01*occ[occ[:] > cutoff])
#ax.scatter(df['x'], df['y'], occ)
#ax.scatter(df['x'], df['y'], df['cost'])

#df = pd.read_csv("/home/justin/raw_occ.csv")
#ax.scatter(df['x'][::8], df['y'][::8], df['z'][::8])

plt.show()
