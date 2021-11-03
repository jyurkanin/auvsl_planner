import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import pandas as pd
from mpl_toolkits.mplot3d import axes3d, Axes3D

df = pd.read_csv("/home/justin/force_log.csv")
plt.plot(df['slip_ratio'], df['bk1'])
plt.plot(df['slip_ratio'], df['nn1'])
plt.show()

plt.plot(df['slip_ratio'], df['bk2'])
plt.plot(df['slip_ratio'], df['nn2'])
plt.show()
