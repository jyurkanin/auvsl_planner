#Goal of this script is to process x,y,rad data from Train3 dataset
#and output approx vx,vy,rads and left/right velocities.



import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import pandas as pd
import numpy as np
from scipy import interpolate



#Time,x,y,rad
df = pd.read_csv("/home/justin/Downloads/Train3/localization_ground_truth/0001_Tr_grass_GT.txt", header=None)
df = df.to_numpy()

df = pd.read_csv("/home/justin/Downloads/Train3/extracted_data/odometry/0001_odom_data.txt", header=None)
odom_data = df.to_numpy()

vx = (odom_data[1:,1] - odom_data[:-1,1]) / (odom_data[1:,0] - odom_data[:-1,0]) #body frame velocity.
vy = (odom_data[1:,2] - odom_data[:-1,2]) / (odom_data[1:,0] - odom_data[:-1,0])
yaw_rate = (np.mod((odom_data[1:,3] - odom_data[:-1,3] + np.pi), 2*np.pi) - np.pi) / (odom_data[1:,0] - odom_data[:-1,0])

data_x = np.stack([vx,vy,yaw_rate], axis=0)

#rotate velocities into body frame
for i in range(data_x.shape[1]):
    #rotation that transforms vectors in world frame to vectors in vehicle frame
    rot = np.array([[np.cos(odom_data[i,3]), -np.sin(odom_data[i,3])], [np.sin(odom_data[i,3]), np.cos(odom_data[i,3])]]).T
    data_x[0:2,i] = rot@data_x[0:2,i]


f_interp = interpolate.interp1d(df[:,0], df[:,1:3], axis=0, fill_value='extrapolate') #interpolate so that data is down sampled to odom rate.
down_sampled_pos = f_interp(odom_data[:,5])
odom_rate_pos = np.concatenate([odom_data[:,5].reshape(-1,1), down_sampled_pos], axis=1)

    

#plt.plot(odom_data[:-1,5], data_x[0,:])
plt.show()
