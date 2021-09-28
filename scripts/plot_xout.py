import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import pandas as pd
from mpl_toolkits.mplot3d import axes3d, Axes3D



#fig = plt.figure()
#ax = Axes3D(fig)

#df = pd.read_csv("/home/justin/elev.csv")
#ax.scatter(df['x'][::10], df['y'][::10], df['alt'][::10])

#idx = 1000
#stop_idx = -1
#df = pd.read_csv("/home/justin/xout_file.csv")
#ax.scatter(df['x'][idx:stop_idx], df['y'][idx:stop_idx], df['z'][idx:stop_idx])

#df = pd.read_csv("/home/justin/code/AUVSL_ROS/bags/odom_10hz.csv")
#ax.scatter(df['field.pose.pose.position.x'][idx::stop_idx], df['field.pose.pose.position.y'][idx::stop_idx], df['field.pose.pose.position.z'][idx::stop_idx])

idx = 1
stop_idx = 2969


df = pd.read_csv("/home/justin/xout_file.csv")
df = df[::20]
print(df.shape)
plt.plot(df['x'][idx:stop_idx], df['y'][idx:stop_idx])


df = pd.read_csv("/home/justin/kinematic_xout.csv")
df = df[::20]
print(df.shape)
plt.plot(df['x'][idx:stop_idx], df['y'][idx:stop_idx])



df = pd.read_csv("/home/justin/code/AUVSL_ROS/bags/rantoul/bags/rantoul_odom_2.csv")
print(df.shape)
plt.plot(df['field.pose.pose.position.x'][idx:stop_idx], df['field.pose.pose.position.y'][idx:stop_idx])

plt.xlabel("m");
plt.ylabel("m");
plt.title("Comparison of Dynamic Model, Kinematic Model, and Odometry Trajectories at 1m/s")
plt.legend(["Dynamic Model","Kinematic Model","Odometry"])
plt.show()
