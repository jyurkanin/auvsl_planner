import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import pandas as pd
import numpy as np
from mpl_toolkits.mplot3d import axes3d, Axes3D
from sklearn.preprocessing import MinMaxScaler


#fig = plt.figure()
#ax = Axes3D(fig)

#skip = 10
#idx = 1
#stop_idx = 2969



df = pd.read_csv("/home/justin/sinkages.csv")
plt.plot(df['zr1'])
plt.plot(df['zr2'])
plt.plot(df['zr3'])
plt.plot(df['zr4'])
plt.legend(['1','2','3','4'])
plt.show()



#df = pd.read_csv("/home/justin/occ_grid.csv")
#occ = df['occ']
#cutoff = 0
#plt.scatter(df['x'][occ[:] > cutoff], df['y'][occ[:] > cutoff])# .01*occ[occ[:] > cutoff])
#plt.show()
#df = pd.read_csv("/home/justin/xout_file.csv")
#plt.scatter(df['x'], df['y'])

#plt.show()


#df = pd.read_csv("/home/justin/code/AUVSL_ROS/pose.csv")
#plt.scatter(df['x'], df['y'])
#plt.show()
