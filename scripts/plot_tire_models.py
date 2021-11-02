import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import pandas as pd
from mpl_toolkits.mplot3d import axes3d, Axes3D

df = pd.read_csv("/home/justin/force_log.csv")
plt.plot(df['slip_angle'], df['bk'])
plt.plot(df['slip_angle'], df['nn'])
plt.show()
