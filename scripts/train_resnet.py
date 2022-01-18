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


def print_c_network(md, input_scaler, output_scaler):
  output = """#include \"solver_res_nn.h\"
//Auto Generated by train_resnet.py
Eigen::Matrix<float,NNResModel::num_hidden_nodes,NNResModel::num_in_features> NNResModel::weight0;
Eigen::Matrix<float,NNResModel::num_hidden_nodes,1> NNResModel::bias0;
Eigen::Matrix<float,NNResModel::num_hidden_nodes,NNResModel::num_hidden_nodes> NNResModel::weight2;
Eigen::Matrix<float,NNResModel::num_hidden_nodes,1> NNResModel::bias2;
Eigen::Matrix<float,NNResModel::num_out_features,NNResModel::num_hidden_nodes> NNResModel::weight4;
Eigen::Matrix<float,NNResModel::num_out_features,1> NNResModel::bias4;
Eigen::Matrix<float,NNResModel::num_out_features,1> NNResModel::out_mean;
Eigen::Matrix<float,NNResModel::num_out_features,1> NNResModel::out_std;
Eigen::Matrix<float,NNResModel::num_in_features,1> NNResModel::in_mean;
Eigen::Matrix<float,NNResModel::num_in_features,1> NNResModel::in_std;

void NNResModel::load_nn_model(){
"""
  for c in md.keys():
    temp = str(c)
    name = temp[2:] + temp[0]
    layer = md[c].flatten()
    output += name
    output += " << "
    for i in range(layer.size()[0] - 1):
      output += (str(float(layer[i])) + "f, ")
      
    output += (str(float(layer[-1])) + "f;\n")
  
  output += "out_mean " + str(output_scaler.mean_.tolist()).replace("[","<< ").replace("]", ";") + "\n"
  output += "out_std " + str(np.sqrt(output_scaler.var_).tolist()).replace("[","<< ").replace("]", ";") + "\n"
  
  output += "in_mean " + str(input_scaler.mean_.tolist()).replace("[","<< ").replace("]", ";") + "\n"
  output += "in_std " + str(np.sqrt(input_scaler.var_).tolist()).replace("[","<< ").replace("]", ";") + "\n"
  
  output += "}"

  output += """

Eigen::Matrix<float,NNResModel::num_out_features,1> NNResModel::forward(Eigen::Matrix<float,NNResModel::num_in_features,1> features){
  Eigen::Matrix<float,NNResModel::num_hidden_nodes,1> layer0_out;
  Eigen::Matrix<float,NNResModel::num_hidden_nodes,1> layer2_out;
  Eigen::Matrix<float,NNResModel::num_out_features,1> layer4_out;
  Eigen::Matrix<float,NNResModel::num_out_features,1> labels;
  Eigen::Matrix<float,NNResModel::num_in_features,1> scaled_features;
  
  scaled_features = (features - in_mean).cwiseQuotient(in_std);
  layer0_out = (weight0*scaled_features) + bias0;
  layer0_out = layer0_out.unaryExpr(&tanhf);
  layer2_out = (weight2*layer0_out) + bias2;
  layer2_out = layer2_out.unaryExpr(&tanhf);
  layer4_out = (weight4*layer2_out) + bias4;        
  labels = layer4_out.cwiseProduct(out_std) + out_mean;

  return labels;
}
"""
  with open('solver_nn_gc.cpp','w') as f:
    f.write(output)
  return


#load test and training data
def load_data():
    in_features = ['vx','vy','az','qd1','qd3']
    out_features = ['e_vx','e_vy','e_wz']
    
    df_all = pd.read_csv("../data/residual_data/features.csv")
    
    #import pdb; pdb.set_trace()
    
    skip_rows = 0
    data_x = np.array(df_all[in_features])    
    data_y = np.array(df_all[out_features])
    
    for i in range(data_x.shape[1]):
      print("nans in data_x", i, "=", np.any(np.isnan(data_x[:,i])))

    num_features = len(in_features)
    data_x_0 = np.concatenate([np.zeros([0,num_features]), data_x[:,:]], axis=0)   # data at t=0
    data_x_1 = np.concatenate([np.zeros([1,num_features]), data_x[:-1,:]], axis=0) # data at t=-1
    data_x_2 = np.concatenate([np.zeros([2,num_features]), data_x[:-2,:]], axis=0) # data at t=-2
    data_x_3 = np.concatenate([np.zeros([3,num_features]), data_x[:-3,:]], axis=0) # data at t=-3
    data_x_4 = np.concatenate([np.zeros([4,num_features]), data_x[:-4,:]], axis=0) # data at t=-4
    data_x_5 = np.concatenate([np.zeros([5,num_features]), data_x[:-5,:]], axis=0) # data at t=-5
    data_x_6 = np.concatenate([np.zeros([6,num_features]), data_x[:-6,:]], axis=0) # data at t=-6
    data_x_7 = np.concatenate([np.zeros([7,num_features]), data_x[:-7,:]], axis=0) # data at t=-7
    
    data_x = np.concatenate((data_x_0, data_x_1, data_x_2, data_x_3, data_x_4, data_x_5, data_x_6, data_x_7), axis=1)

      
    #plt.plot(data_x[:,2])
    #plt.show()
    
    data_x = input_scaler.fit_transform(data_x)
    data_y = output_scaler.fit_transform(data_y)
    
    print(input_scaler.mean_)
    print(input_scaler.var_)
    
    data_len = data_x.shape[0]*.95
    
    train_data = data_x[:int(data_len),:]
    label_data = data_y[:int(data_len),:]
    
    train_data, label_data = shuffle(train_data, label_data, random_state=1)
    
    test_data = data_x[int(data_len):,:]
    test_labels = data_y[int(data_len):,:]
    
    #plt.plot(np.linspace(0,1,data_x.shape[0]), data_x[:,4])
    #plt.show()

    #plt.scatter(data_y[:,0], data_y[:,1])
    #plt.show()
    
    return train_data, label_data, test_data, test_labels
    
    
    
class VehicleResNet(nn.Module):
  def __init__(self, mn):
    super(VehicleResNet, self).__init__()
    
    self.in_size = test_data.shape[1]
    self.hidden_size = 10
    self.out_size = test_labels.shape[1]
    
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
    
  def forward_partial(self, x):
    return self.tanh_block1.forward(x)

  def get_validation_loss(self):
    x = test_data
    y = test_labels
    
    self.opt.zero_grad()
    y_hat = self.forward_partial(x)
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
        y_hat = self.forward_partial(x)
        loss = self.loss_fn(y_hat, y)
        loss.backward()
        self.opt.step()

      self.get_validation_loss()
      plt.scatter(self.count, loss.item(), color='b')
      self.count += 1
      print("LOSS", loss)


    

print("going to load data")
train_data, label_data, test_data, test_labels = load_data()
print("done loading")

train_data = torch.from_numpy(train_data).float()
label_data = torch.from_numpy(label_data).float()

test_data = torch.from_numpy(test_data).float()
test_labels = torch.from_numpy(test_labels).float()

print("train data size ", train_data.size())
print("label data size ", label_data.size())
print("has gpu? ", torch.cuda.is_available())

#import pdb; pdb.set_trace() #this is completely overpowered. Too useful.

def network_forward(model, x):
  x_scaled = input_scaler.transform(x)
  x_torch= torch.from_numpy(x_scaled).float()
  yhat = model.forward_partial(x_torch).detach().numpy()
  yhat = output_scaler.inverse_transform(yhat)
  return yhat

def test_network(model):
    yhat = model.forward(test_data).detach().numpy()
    
    yhat = output_scaler.inverse_transform(yhat)
    cpu_test_labels = output_scaler.inverse_transform(test_labels.detach().numpy())
    
    print("Evaluation Loss", np.mean(np.abs(yhat.flatten() - cpu_test_labels.flatten())))
    
    idx = 2
    plt.plot(yhat[0:1000,idx])
    plt.plot(cpu_test_labels[0:1000,idx])
    plt.show()



    
model_name = "../data/vehicle_res1.net"
model = VehicleResNet(model_name)
model.load(model_name)

print("Training first block")
model.fit(1e-4, 10, 1000)
print("Done learning")
plt.show()

model.save()

md = model.tanh_block1.state_dict()
print_c_network(md, input_scaler, output_scaler)

