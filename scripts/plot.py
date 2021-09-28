import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import pandas as pd
import numpy as np
from mpl_toolkits.mplot3d import axes3d, Axes3D
from sklearn.preprocessing import MinMaxScaler


fig = plt.figure()
ax = Axes3D(fig)

skip = 10
idx = 1
stop_idx = 2969


df = pd.read_csv("/home/justin/elev.csv")
print(df.shape)
df = df[df['y'] < 5]
df = df[df['x'] > -5]
print(df.shape)
ax.scatter(df['x'][::skip], df['y'][::skip], df['alt'][::skip], alpha=.2)

df = pd.read_csv("/home/justin/xout_file.csv")
df = df[::100]
plt.plot(df['x'], df['y'], df['z'], alpha=1, color='r')



plt.show()



#df = pd.read_csv("/home/justin/occ_grid.csv")
#occ = df['occupancy']
#cutoff = 8
#plt.scatter(df['x'][occ[:] > cutoff], df['y'][occ[:] > cutoff])# .01*occ[occ[:] > cutoff])
#plt.show()
#df = pd.read_csv("/home/justin/xout_file.csv")
#plt.scatter(df['x'], df['y'])

#plt.show()


#df = pd.read_csv("/home/justin/code/AUVSL_ROS/pose.csv")
#plt.scatter(df['x'], df['y'])
#plt.show()
