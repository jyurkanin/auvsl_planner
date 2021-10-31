import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import pandas as pd
import numpy as np
from mpl_toolkits.mplot3d import axes3d, Axes3D

df = pd.read_csv("/home/justin/Downloads/CV3/localization_ground_truth/0050_CV_grass_GT.txt", header=None)
plt.plot(df[1], df[2], color='r')

df = pd.read_csv("/home/justin/xout_file.csv")
plt.scatter(df['x'], df['y'], color='b', s=1)

plt.xlabel('m')
plt.ylabel('m')
plt.legend(['Odometry', 'Simulation'])
plt.title('Rantoul Long Test Simulation vs Odometry')
plt.show()




df = pd.read_csv("/home/justin/Downloads/CV3/extracted_data/odometry/0050_odom_data.txt", header=None)
plt.plot()
plt.show()
