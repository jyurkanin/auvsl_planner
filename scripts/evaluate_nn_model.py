import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import torch.autograd as autograd


import pandas as pd
from sklearn.preprocessing import MinMaxScaler, StandardScaler
from sklearn.metrics import mean_squared_error
from sklearn.utils import shuffle
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt

from pickle import load
from pickle import dump

#import pdb; pdb.set_trace() #this is completely overpowered. Too useful.

input_scaler = StandardScaler()
output_scaler = StandardScaler()

#load test and training data
def load_training_data():
    in_features = ['qd1_0','qd3_0','qd1_1','qd3_1','qd1_2','qd3_2','qd1_3','qd3_3','qd1_4','qd3_4','qd1_5','qd3_5','qd1_6','qd3_6','qd1_7','qd3_7']
    out_features = ["vx","vy","wz"]
    
    df_x = pd.read_csv("../data/joydeep_data/training_x.csv")
    df_y = pd.read_csv("../data/joydeep_data/training_y.csv")
    
    skip_rows = 0
    data_x = np.array(df_x[in_features])    
    data_x = data_x[skip_rows:]
    
    data_y = np.array(df_y[out_features])
    data_y = data_y[skip_rows:]
    
    data_x = input_scaler.fit_transform(data_x)
    data_y = output_scaler.fit_transform(data_y)
    
    data_len = data_x.shape[0]
    
    train_data = data_x[:int(data_len),:]
    label_data = data_y[:int(data_len),:]
    
    train_data, label_data = shuffle(train_data, label_data, random_state=1)    
    
    return train_data, label_data
    
    
    
class VehicleNet(nn.Module):
  def __init__(self, mn):
    super(VehicleNet, self).__init__()
    
    self.in_size = 16
    self.hidden_size = 10
    self.out_size = 3
    
    self.tanh_block1 = nn.Sequential(
      nn.Linear(self.in_size, self.hidden_size),
      nn.Tanh(),
      nn.Linear(self.hidden_size, self.hidden_size),
      nn.Tanh(),
      nn.Linear(self.hidden_size, self.out_size)
    )
        
    self.loss_fn = torch.nn.MSELoss()
    self.opt = torch.optim.Adam(self.tanh_block1.parameters(), lr=1e-1)
    self.count = 0
    self.model_name = mn
    
  def save(self):
    md = self.tanh_block1.state_dict()
    torch.save(md, self.model_name)
    return

  def load(self, name):
    md = torch.load(name)
    self.tanh_block1.load_state_dict(md)
    return
    
  def forward(self, x):
    return self.tanh_block1.forward(x)

  def get_validation_loss(self):
    x = test_data
    y = test_labels
    
    self.opt.zero_grad()
    y_hat = self.forward(x)
    loss = self.loss_fn(y_hat, y)
    plt.scatter(self.count, loss.item(), color='r')
      
  def fit(self, lr, batch_size, epochs):
    for param_group in self.opt.param_groups:
      param_group['lr'] = lr
        
    for j in range(epochs):
      for i in range(0, train_data.shape[0]-batch_size, batch_size):
        x = train_data[i:i+batch_size, :]
        y = label_data[i:i+batch_size, :]
        
        self.opt.zero_grad()
        y_hat = self.forward(x)
        loss = self.loss_fn(y_hat, y)
        loss.backward()
        self.opt.step()

      self.get_validation_loss()
      plt.scatter(self.count, loss.item(), color='b')
      self.count += 1
      print("LOSS", loss)


    

print("going to load data")
train_data, label_data = load_training_data()
print("done loading")

train_data = torch.from_numpy(train_data).float()
label_data = torch.from_numpy(label_data).float()

print("train data size ", train_data.size())
print("label data size ", label_data.size())
print("has gpu? ", torch.cuda.is_available())

def network_forward(model, x):
  x_scaled = input_scaler.transform(x)
  x_torch= torch.from_numpy(x_scaled).float()
  yhat = model.forward(x_torch).detach().numpy()
  yhat = output_scaler.inverse_transform(yhat)
  return yhat

#returns linear and angular displacement
#expects gt_pos to be [[x,y,yaw], ...]
def get_path_displacement(gt_pos):
  diff = gt_pos[1:,:] - gt_pos[:-1,:]
  diff_sq = np.multiply(diff[:,0:2], diff[:,0:2]) #dx^2, dy^2
  dist = np.sqrt(np.sum(diff_sq, axis=1))         #sqrt(dx^2 + dy^2)
  path_len = np.sum(dist)                         #sum(length of segments)
  
  angle_diff = np.abs(np.mod((diff[:,2] + np.pi), 2*np.pi) - np.pi)
  yaw_displacement = np.sum(angle_diff)
  
  return path_len, yaw_displacement
  


def test_vehicle_network(model, gt_filename, features_filename):
  in_features = ['ts','qd1_0','qd3_0','qd1_1','qd3_1','qd1_2','qd3_2','qd1_3','qd3_3','qd1_4','qd3_4','qd1_5','qd3_5','qd1_6','qd3_6','qd1_7','qd3_7']
  
  df_x = pd.read_csv(features_filename)
  data_x = np.array(df_x[in_features])
  test_features = data_x

  sim_len = 6
  
  idx = 0
  start_time = test_features[0,0]
  for i in range(test_features.shape[0]):
    if((test_features[i,0] - start_time) > sim_len):
      idx = i
      break
  
  print("time diff", test_features[i,0] - start_time)
  
  test_features = test_features[:idx+1,:]
  test_features = test_features[:,1:]
  
  
  df = pd.read_csv(gt_filename, header=None)
  gt_pos = df.to_numpy()
  
  idx = 0
  start_time = gt_pos[0,0]
  for i in range(gt_pos.shape[0]):
    if((gt_pos[i,0] - start_time) > sim_len):
      idx = i
      break
  
  print("time diff", gt_pos[i,0] - start_time)
  
  gt_pos = gt_pos[:idx+1,:]  
  gt_pos = gt_pos[:,1:4]
  
  init_x = gt_pos[0,0]
  init_y = gt_pos[0,1]
  plt.plot(gt_pos[:,0] - init_x, gt_pos[:,1] - init_y, color='blue')
  
  pos = np.array([0.0,0.0])
  features = np.zeros([16], dtype=float)
  
  yhat = np.array([0.0, 0, 0])
  predicted_path = np.zeros([test_features.shape[0]+1, 3])
  predicted_path[0][2] = 0
  
  ts = .05
  
  for i in range(test_features.shape[0]):
    for j in range(6,-1,-1): #6,5,4,3,2,1,0
      features[((j+1)*2)] = features[(j*2)]
      features[((j+1)*2)+1] = features[(j*2)+1]
    
    features[0] = test_features[i,0]
    features[1] = test_features[i,1]
    
    yhat = network_forward(model, features.reshape(1, -1)).reshape((3,))
        
    theta = predicted_path[i][2]
    rot = np.array([[np.cos(theta), -np.sin(theta)],[np.sin(theta), np.cos(theta)]])
    vel = np.dot(rot, yhat[0:2])  #transform to world coordinates so we can integrate
    
    predicted_path[i+1][0] = predicted_path[i][0] + vel[0]*ts
    predicted_path[i+1][1] = predicted_path[i][1] + vel[1]*ts
    predicted_path[i+1][2] = predicted_path[i][2] + yhat[2]*ts

  plt.plot(predicted_path[:,0], predicted_path[:,1], color='red')
  #plt.show()
  
  total_lin_disp, total_ang_disp = get_path_displacement(gt_pos)
  
  dx = predicted_path[-1,0] - gt_pos[-1,0]
  dy = predicted_path[-1,1] - gt_pos[-1,1]
  dyaw = predicted_path[-1,2] - gt_pos[-1,2]
  dyaw = np.abs(np.mod((dyaw + np.pi), 2*np.pi) - np.pi)
  return (np.sqrt((dx*dx) + (dy*dy))) / total_lin_disp, dyaw / total_ang_disp


def evaluate_cv3_paths(model):
  gt_filename = "/home/justin/JoydeepDataset/CV3/localization_ground_truth/{:04d}_CV_grass_GT.txt"
  features_filename = "../data/joydeep_data/cv3/test_x_CV3_{:04d}.csv"
  
  total_lin_err = 0
  total_ang_err = 0

  count = 0
  
  for i in range(1, 145):
    lin_err, ang_err = test_vehicle_network(model, gt_filename.format(i), features_filename.format(i))
    total_lin_err += lin_err
    total_ang_err += ang_err
    count += 1

  print(float(total_lin_err) / count, float(total_ang_err) / count)
    



def evaluate_ld3_paths(model):
  gt_filename = "/home/justin/JoydeepDataset/LD3/localization_ground_truth/0001_LD_grass_GT.txt"
  features_filename = "../data/joydeep_data/ld3/test_x_LD3_0001.csv"
  
  total_lin_err = 0
  total_ang_err = 0
  
  lin_err, ang_err = test_vehicle_network(model, gt_filename, features_filename)
  total_lin_err += lin_err
  total_ang_err += ang_err

  print(float(total_lin_err), float(total_ang_err))






  
model_name = "../data/vehicle2.net"
model = VehicleNet(model_name)
model.load(model_name)
 
evaluate_cv3_paths(model)


#md = model.state_dict()

#import pdb; pdb.set_trace() #this is completely overpowered. Too useful.
#print_c_network(md, input_scaler, output_scaler)

