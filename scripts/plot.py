import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import pandas as pd
from mpl_toolkits.mplot3d import axes3d, Axes3D



#fig = plt.figure()
#ax = Axes3D(fig)

df = pd.read_csv("/home/justin/occ_grid.csv")
occ = df['occupancy']
cutoff = 12    
plt.scatter(df['x'][occ[:] > cutoff], df['y'][occ[:] > cutoff])# .01*occ[occ[:] > cutoff])

df = pd.read_csv("/home/justin/xout_file.csv")
plt.scatter(df['x'], df['y'])

plt.show()
